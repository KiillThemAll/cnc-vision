#include "automator.h"
#include <math.h>

Automator::Automator(QObject *parent) : QObject(parent)
{
    m_working = false;
    m_enabled = false;
    m_mcConnected = false;
    m_cameraConnected = false;
    m_lastdzValid = false;
    m_lastCoordsValid = false;
    m_message = "Waiting for pause";
    m_cutModeEnabled = false;
    m_autosendPower = false;
    m_minPower = 1.0;
    m_maxPower = 0.8;
    m_lastSentPower = 0.0;
    m_powerTimer.setInterval(1000); // maximum power update rate [ms]
    m_powerTimer.setSingleShot(true);

    m_mcs_x_check_state = 0;
    m_mcs_y_check_state = 0;
    m_compensatorOneShot = false;
    m_cutCompensatorOneShot = false;
    m_answerFromMCReceived = true;
    m_scanOneShot = false;

    m_surfaceModel = new SurfaceModel(this);

    m_state = Disabled;

    m_scanComplited = false;
    m_scanApprooved = false;
    m_entryMissing = false;

    //m_surfaceSpline = nullptr;
}

Automator::~Automator()
{
    //delete m_surfaceSpline;
}

bool Automator::working() const
{
    return m_working;
}

void Automator::ondzChanged(float dz)
{
    m_lastdz = dz;
}

void Automator::ondzValidChanged(bool valid)
{
    m_lastdzValid = valid;
    m_cameraConnected = valid;
    checkWorkingState();
}

void Automator::onMcConnectionStateChanged(bool connected)
{
    m_mcConnected = connected;
    checkWorkingState();
}

void Automator::onRayConnectionStateChanged(bool connected)
{
    m_lastCoordsValid = connected;
}

void Automator::onCoordsChanged(float x, float y, float z, float b)
{
    Q_UNUSED(z);
    m_mcs_x = x;
    m_mcs_y = y;
    m_mcs_b = b;

    if (!m_autosendPower)
        return;

    if (m_maxPower < m_minPower)
        return;

    if (m_powerTimer.isActive())
        return;

    const float maxX = 2200;
    const float minX = 600;
    const float maxY = 1500;
    const float minY = 0;
    const float dPower = 0.01;
    float airPathLength = (maxX - m_mcs_x) + (maxY - m_mcs_y);
    const float maximumAirPathLength = (maxX - minX) + (maxY - minY);
    float powerSpan = m_maxPower - m_minPower;
    float dist = airPathLength / maximumAirPathLength;
    float targetPower = m_minPower + powerSpan * dist;

    if (fabs(targetPower - m_lastSentPower) < dPower)
        return;

    m_lastSentPower = targetPower;
    emit changePower(m_lastSentPower / 5.0);

    m_powerTimer.start();
}

void Automator::onMcStateChanged(RayReceiver::State s)
{
    m_lastMCState = s;


    if (m_mcConnected && m_lastCoordsValid && m_enabled && m_cutModeEnabled && s == RayReceiver::Playing) {
        if (!m_cutCompensatorOneShot && m_answerFromMCReceived) {
            QTimer::singleShot(1000, this, SLOT(compensateFromScan()));
            m_cutCompensatorOneShot = true;
        }
        return;
    }

    if (!m_working) return;

    if (!m_cutModeEnabled && s == RayReceiver::Paused) {
        if(m_mcs_x_check_state != m_mcs_x || m_mcs_y_check_state != m_mcs_y)
        {
            m_mcs_x_check_state = m_mcs_x;
            m_mcs_y_check_state = m_mcs_y;
            return;
        }

        if (!m_compensatorOneShot)
        {
            QTimer::singleShot(2000, this, SLOT(compensate()));
            m_compensatorOneShot = true;
        }
    }
    if (!m_cutModeEnabled && s == RayReceiver::Playing)
            m_compensatorOneShot = false;
}

void Automator::onCameraFail()
{
    m_cameraConnected = false;
    checkWorkingState();
}

void Automator::scanSnapshot(GcodePlayer::State s)
{
    if (s == GcodePlayer::State::PausedM25)
    {
        QTimer::singleShot(1000, this, SLOT(m_scanSnapshot()));
    }
}

void Automator::scanFinished(GcodePlayer::State s)
{
    if (s == GcodePlayer::State::Stopped && m_state == Scanning)
    {
            /*
            delete m_surfaceSpline;

            SPLINTER::DenseVector x(2);
            float y;

            SurfacePoint point;

            foreach (point, m_surfaceModel->m_surface) {
                x(0) = point.x;
                x(1) = point.y;
                y = point.z;

                m_samples.addSample(x,y);
            }

            m_surfaceSpline = new SPLINTER::BSpline(SPLINTER::BSpline::Builder(m_samples).degree(3).build());


        QVector<SurfacePoint> points;

        points.reserve((m_scanWidth+1)*(m_scanHeight+1));

        for(int i = 0; i <= m_scanHeight; i++)
        {
            for(int j = 0; j <= m_scanWidth; j++)
            {
                x(0) = i;
                point.x = i;
                x(1) = j;
                point.y = j;
                point.z = m_surfaceSpline->eval(x);

                points.append(point);
            }
        }

        m_surfaceModel->updatePoints(points);
        */

        m_surfaceModel->sortPoints();

        m_scanComplited = true;
        emit scanStateChanged();

        bool working = m_lastdzValid && m_mcConnected && m_lastCoordsValid && m_enabled && m_cameraConnected;
        if (working) {
            m_working = true;
            m_state = m_cutModeEnabled ? AutoCutting : AutoEngraving;
            emit stateChanged(m_state);
        } else {
            m_working = false;
            m_state = Disabled;
            emit stateChanged(m_state);
        }
    }
}

void Automator::compensate()
{
    if (!m_working || m_cutModeEnabled)
        return;
    float compensated = compensate(m_lastdz);
    if (compensated > 10) {
        m_message = "No entry in comp table";
        emit messageChanged();
    } else {
        QString correction = QString("G90 G0 B%1\n").arg(compensated);
        m_message = correction;
        emit messageChanged();
        qDebug() << correction;
        emit sendToMC(correction);
        emit sendToMC("M24\n");
    }
}

float Automator::compensate(float dz) const
{
    //                   CAM     ERROR
    const float map[] = {30,	1.98499458592347,
                         29.9,	1.96805560374944,
                         29.8,	1.95113530682373,
                         29.7,	1.93423424491316,
                         29.6,	1.917352925779,
                         29.5,	1.90049181563091,
                         29.4,	1.88365133958086,
                         29.3,	1.86683188209708,
                         29.2,	1.85003378745796,
                         29.1,	1.83325736020603,
                         29,	1.81650286560186,
                         28.9,	1.79977053007801,
                         28.8,	1.78306054169295,
                         28.7,	1.76637305058502,
                         28.6,	1.74970816942632,
                         28.5,	1.73306597387668,
                         28.4,	1.7164465030376,
                         28.3,	1.69984975990613,
                         28.2,	1.68327571182889,
                         28.1,	1.6667242909559,
                         28,	1.65019539469462,
                         27.9,	1.63368888616378,
                         27.8,	1.61720459464741,
                         27.7,	1.60074231604871,
                         27.6,	1.58430181334399,
                         27.5,	1.56788281703664,
                         27.4,	1.55148502561104,
                         27.3,	1.53510810598647,
                         27.2,	1.51875169397108,
                         27.1,	1.50241539471582,
                         27,	1.48609878316837,
                         26.9,	1.46980140452703,
                         26.8,	1.45352277469475,
                         26.7,	1.43726238073295,
                         26.6,	1.42101968131556,
                         26.5,	1.40479410718286,
                         26.4,	1.38858506159549,
                         26.3,	1.37239192078833,
                         26.2,	1.35621403442447,
                         26.1,	1.34005072604912,
                         26,	1.32390129354356,
                         25.9,	1.30776500957905,
                         25.8,	1.29164112207079,
                         25.7,	1.27552885463186,
                         25.6,	1.2594274070271,
                         25.5,	1.24333595562712,
                         25.4,	1.22725365386216,
                         25.3,	1.21117963267609,
                         25.2,	1.19511300098029,
                         25.1,	1.17905284610761,
                         25,	1.1629982342663,
                         24.9,	1.14694821099396,
                         24.8,	1.13090180161142,
                         24.7,	1.11485801167676,
                         24.6,	1.09881582743914,
                         24.5,	1.08277421629283,
                         24.4,	1.06673212723109,
                         24.3,	1.0506884913001,
                         24.2,	1.03464222205294,
                         24.1,	1.01859221600346,
                         24,	1.00253735308027,
                         23.9,	0.986476497080633,
                         23.8,	0.970408496124434,
                         23.7,	0.954332183108084,
                         23.6,	0.93824637615847,
                         23.5,	0.92214987908688,
                         23.4,	0.906041481842947,
                         23.3,	0.889919960968572,
                         23.2,	0.873784080051864,
                         23.1,	0.857632590181074,
                         23,	0.841464230398522,
                         22.9,	0.825277728154542,
                         22.8,	0.809071799761404,
                         22.7,	0.792845150847256,
                         22.6,	0.776596476810054,
                         22.5,	0.760324463271495,
                         22.4,	0.744027786530954,
                         22.3,	0.727705114019418,
                         22.2,	0.711355104753411,
                         22.1,	0.694976409788941,
                         22,	0.678567672675423,
                         21.9,	0.662127529909619,
                         21.8,	0.645654611389569,
                         21.7,	0.629147540868524,
                         21.6,	0.612604936408883,
                         21.5,	0.596025410836122,
                         21.4,	0.579407572192733,
                         21.3,	0.562750024192155,
                         21.2,	0.546051366672705,
                         21.1,	0.529310196051517,
                         21,	0.512525105778474,
                         20.9,	0.495694686790137,
                         20.8,	0.478817527963687,
                         20.7,	0.461892216570852,
                         20.6,	0.444917338731844,
                         20.5,	0.427891479869293,
                         20.4,	0.410813225162175,
                         20.3,	0.393681159999757,
                         20.2,	0.376493870435519,
                         20.1,	0.359249943641093,
                         20,	0.3419479683602,
                         19.9,	0.324586535362575,
                         19.8,	0.307164237897912,
                         19.7,	0.289679672149786,
                         19.6,	0.272131437689595,
                         19.5,	0.254518137930491,
                         19.4,	0.236838380581314,
                         19.3,	0.219090778100523,
                         19.2,	0.201273948150136,
                         19.1,	0.183386514049656,
                         19,	0.165427105230013,
                         18.9,	0.147394357687488,
                         18.8,	0.129286914437658,
                         18.7,	0.111103425969318,
                         18.6,	0.0928425506984253,
                         18.5,	0.0745029554220258,
                         18.4,	0.0560833157721901,
                         18.3,	0.0375823166699496,
                         18.2,	0.0189986527792254,
                         18.1,	0.000331028960767342,
                         18,	-0.0184218392739176,
                         17.9,	-0.0372612253086268,
                         17.8,	-0.0561883909685305,
                         17.7,	-0.0752045860662399,
                         17.6,	-0.094311047947869,
                         17.5,	-0.113509001039106,
                         17.4,	-0.132799656391274,
                         17.3,	-0.152184211227403,
                         17.2,	-0.171663848488292,
                         17.1,	-0.191239736378576,
                         17,	-0.210913027912796,
                         16.9,	-0.230684860461459,
                         16.8,	-0.250556355297108,
                         16.7,	-0.270528617140391,
                         16.6,	-0.29060273370612,
                         16.5,	-0.310779775249344,
                         16.4,	-0.331060794111412,
                         16.3,	-0.351446824266038,
                         16.2,	-0.371938880865374,
                         16.1,	-0.392537959786067,
                         16,	-0.413245037175332,
                         15.9,	-0.434061068997016,
                         15.8,	-0.454986990577662,
                         15.7,	-0.476023716152582,
                         15.6,	-0.497172138411915,
                         15.5,	-0.5184331280467,
                         15.4,	-0.539807533294937,
                         15.3,	-0.561296179487658,
                         15.2,	-0.58289986859499,
                         15.1,	-0.604619378772222,
                         15,	-0.626455463905872,
                         14.9,	-0.648408853159754,
                         14.8,	-0.670480250521041,
                         14.7,	-0.692670334346336,
                         14.6,	-0.714979756907733,
                         14.5,	-0.737409143938888,
                         14.4,	-0.759959094181083,
                         14.3,	-0.782630178929293,
                         14.2,	-0.805422941578249,
                         14.1,	-0.828337897168512,
                         14,	-0.85137553193253,
                         13.9,	-0.874536302840712,
                         13.8,	-0.897820637147488,
                         13.7,	-0.921228931937381,
                         13.6,	-0.944761553671069,
                         13.5,	-0.968418837731454,
                         13.4,	-0.992201087969726,
                         13.3,	-1.01610857625143,
                         13.2,	-1.04014154200253,
                         13.1,	-1.06430019175549,
                         13,	-1.08858469869532,
                         12.9,	-1.11299520220564,
                         12.8,	-1.13753180741477,
                         12.7,	-1.16219458474179,
                         12.6,	-1.18698356944257,
                         12.5,	-1.21189876115591,
                         12.4,	-1.23694012344952,
                         12.3,	-1.26210758336615,
                         12.2,	-1.28740103096963,
                         12.1,	-1.31282031889094,
                         12,	-1.33836526187427,
                         11.9,	-1.36403563632312,
                         11.8,	-1.38983117984631,
                         11.7,	-1.41575159080408,
                         11.6,	-1.44179652785417,
                         11.5,	-1.46796560949786,
                         11.4,	-1.49425841362602,
                         11.3,	-1.52067447706525,
                         11.2,	-1.54721329512385,
                         11.1,	-1.57387432113796,
                         11,	-1.60065696601761,
                         10.9,	-1.62756059779273,
                         10.8,	-1.65458454115932,
                         10.7,	-1.68172807702541,
                         10.6,	-1.70899044205721,
                         10.5,	-1.73637082822513,
                         10.4,	-1.76386838234984,
                         10.3,	-1.79148220564838,
                         10.2,	-1.81921135328019,
                         10.1,	-1.84705483389317,
                         10,	-1.8750116091698,
                         9.9,	-1.90308059337314,
                         9.8,	-1.93126065289292,
                         9.7,	-1.95955060579164,
                         9.6,	-1.98794922135058,
                         9.5,	-2.01645521961591,
                         9.4,	-2.04506727094474,
                         9.3,	-2.07378399555118,
                         9.2,	-2.10260396305242,
                         9.1,	-2.13152569201478,
                         9,	-2.16054764949978,
                         8.9,	-2.18966825061024,
                         8.8,	-2.2188858580363,
                         8.7,	-2.24819878160149,
                         8.6,	-2.27760527780884,
                         8.5,	-2.30710354938689,
                         8.4,	-2.33669174483581,
                         8.3,	-2.36636795797341,
                         8.2,	-2.39613022748126,
                         8.1,	-2.42597653645072,
                         8,	-2.45590481192902,
                         7.9,	-2.48591292446532,
                         7.8,	-2.5159986876568,
                         7.7,	-2.54615985769469,
                         7.6,	-2.57639413291035,
                         7.5,	-2.60669915332136,
                         7.4,	-2.63707250017753,
                         7.3,	-2.66751169550705,
                         7.2,	-2.69801420166247,
                         7.1,	-2.72857742086682,
                         7,	-2.75919869475966,
                         6.9,	-2.78987530394315,
                         6.8,	-2.82060446752811,
                         6.7,	-2.85138334268009,
                         6.6,	-2.88220902416543,
                         6.5,	-2.91307854389735,
                         6.4,	-2.94398887048199,
                         6.3,	-2.97493690876448,
                         6.2,	-3.00591949937501,
                         6.1,	-3.0369334182749,
                         6,	-3.06797537630268,
                         5.9,	-3.09904201872012,
                         5.8,	-3.13012992475832,
                         5.7,	-3.16123560716378,
                         5.6,	-3.19235551174445,
                         5.5,	-3.22348601691582,
                         5.4,	-3.25462343324695,
                         5.3,	-3.28576400300658,
                         5.2,	-3.31690389970916,
                         5.1,	-3.34803922766093,
                         5,	-3.379166021506,
                         4.9,	-3.4102802457724,
                         4.8,	-3.44137779441812,
                         4.7,	-3.47245449037726,
                         4.6,	-3.50350608510598,
                         4.5,	-3.53452825812868,
                         4.4,	-3.56551661658399,
                         4.3,	-3.59646669477087,
                         4.2,	-3.62737395369465,
                         4.1,	-3.65823378061313,
                         4,	-3.68904148858262,
                         3.9,	-3.71979231600403,
                         3.8,	-3.75048142616891,
                         3.7,	-3.78110390680552,
                         3.6,	-3.81165476962493,
                         3.5,	-3.84212894986704,
                         3.4,	-3.87252130584667,
                         3.3,	-3.90282661849964,
                         3.2,	-3.93303959092879,
                         3.1,	-3.96315484795011,
                         3,	-3.99316693563874,
                         2.9,	-4.0230703208751,
                         2.8,	-4.0528593908909,
                         2.7,	-4.08252845281524,
                         2.6,	-4.11207173322067,
                         2.5,	-4.14148337766925,
                         2.4,	-4.17075745025862
                        };
    const int mapSize = sizeof(map) / sizeof(map[0]);
    int range = -1;
    for (int i = 0; i < mapSize - 2; i += 2) {
        if (dz >= map[i + 2] && dz < map[i]) {
            range = i;
            break;
        }
    }
    if (range == -1)
        return 1000;
    float rangeSpan = map[range] - map[range + 2];
    float dist = (dz - map[range + 2]) / rangeSpan;
    float valueSpan = map[range + 1] - map[range + 3];
    return map[range + 3] + valueSpan * dist;
}

void Automator::compensateFromScan()
{
    if (m_working || m_cutModeEnabled || m_scanApprooved){
        SurfacePoint point;
        point.x = m_mcs_x;
        point.y = m_mcs_y;
        float compensated = interpolateFromSurfaceScan(point);
        if (compensated > 10) {
            m_message = "Out of scan range";
            emit messageChanged();
            m_cutCompensatorOneShot = false;
        } else {
            QString correction = QString("G90 G1 B%1\n").arg(compensated);
            m_message = correction;
            emit messageChanged();
            qDebug() << correction;
            emit sendToMCWithAnswer(correction);
            m_answerFromMCReceived = false;
            m_cutCompensatorOneShot = false;
        }
    }
}

void Automator::answerFromMCReceived()
{
    m_answerFromMCReceived = true;
}

float Automator::interpolateFromSurfaceScan(const SurfacePoint &point)
{
    const int mapSizeX = m_surfaceModel->surfaceColNum;
    const int mapSizeY = m_surfaceModel->surfaceRowNum;

    float x1;
    float x2;
    int rangeX = -1;
    for (int i = 0; i < mapSizeX-1; i++) {
        if (point.x-605 >= m_surfaceModel->m_surfaceSorted.at(i).x && point.x-605 < m_surfaceModel->m_surfaceSorted.at(i+1).x) {
            x1 = m_surfaceModel->m_surfaceSorted.at(i).x;
            x2 = m_surfaceModel->m_surfaceSorted.at(i+1).x;
            rangeX = i;
            break;
        }
    }
    if (rangeX == -1)
        return 1000;

    float y1;
    float y2;
    int rangeY = -1;
    for (int i = 0; i < mapSizeY-1; i++) {
        if (point.y >= m_surfaceModel->m_surfaceSorted.at(i*mapSizeX).y && point.y < m_surfaceModel->m_surfaceSorted.at(i*mapSizeX+mapSizeX).y) {
            y1 = m_surfaceModel->m_surfaceSorted.at(i*mapSizeX).y;
            y2 = m_surfaceModel->m_surfaceSorted.at(i*mapSizeX+mapSizeX).y;
            rangeY = i;
            break;
        }
    }
    if (rangeY == -1)
        return 1000;

    float z11 = m_surfaceModel->m_surfaceSorted.at(rangeY*mapSizeX+rangeX).z;
    float z12 = m_surfaceModel->m_surfaceSorted.at(rangeY*mapSizeX+rangeX+1).z;
    float z21 = m_surfaceModel->m_surfaceSorted.at(rangeY*mapSizeX+mapSizeX+rangeX).z;
    float z22 = m_surfaceModel->m_surfaceSorted.at(rangeY*mapSizeX+mapSizeX+rangeX+1).z;


    float rangeSpanX = x2-x1;
    float rangeSpanY = y2-y1;
    float distX = (point.x-605 - x1) / rangeSpanX;
    float distY = (point.y - y1) / rangeSpanY;

    float valueSpanX1 = z12 - z11;
    float interpolatedX1 = z11 + valueSpanX1 * distX;

    float valueSpanX2 = z22 - z21;
    float interpolatedX2 = z21 + valueSpanX2 * distX;

    float valueSpanY = interpolatedX2 - interpolatedX1;
    return interpolatedX1 + valueSpanY * distY;
}

void Automator::checkWorkingState()
{
    if (m_state == Scanning)
        return;

    bool working = m_lastdzValid && m_mcConnected && m_lastCoordsValid && m_enabled && m_cameraConnected;
    if (working) {
        m_working = true;
        m_state = m_cutModeEnabled ? AutoCutting : AutoEngraving;
        emit stateChanged(m_state);
    } else {
        m_working = false;
        m_state = Disabled;
        emit stateChanged(m_state);
    }
}

void Automator::m_scanSnapshot()
{
    float compensated = compensate(m_lastdz);
    if (compensated > 10) {
        m_message = "No entry in comp table";
        emit messageChanged();
        m_entryMissing = true;
        emit requestMissingEntry();
    } else {
        SurfacePoint point;
        point.x = qRound(m_mcs_x-605);
        point.y = qRound(m_mcs_y);
        point.z = compensated;
        m_surfaceModel->updatePoint(point, scanSnapshotNumber);
        scanSnapshotNumber++;
        m_message = QString("Z: %1").arg(compensated);
        emit messageChanged();
        qDebug() << m_message;

        /*SPLINTER::DenseVector x(2);
        float y;

        x(0) = qRound(m_mcs_x-605);
        x(1) = qRound(m_mcs_y);
        y = compensated;

        m_samples.addSample(x,y);*/

        emit continueScan();
    }
}

float Automator::minPower() const
{
    return m_minPower;
}

void Automator::setMinPower(float minPower)
{
    m_minPower = minPower;
}

float Automator::lastSentPower() const
{
    return m_lastSentPower;
}

void Automator::scanSurface(int width, int height, int step, int leftShit)
{

    /*SurfacePoint point;

    for (int i=0; i<=height/step; i++)
    {
        for (int j=1; j<=width/step; j++)
            if (i%2)
            {
                point.x = width-step*j;
                point.y = i;
                point.z = i+width-step*j;
                m_surfaceModel->addPointWithoutNotify(point);
            }
            else
            {
                point.x = step*j;
                point.y = i;
                point.z = i+step*j;
                m_surfaceModel->addPointWithoutNotify(point);
            }
    }*/
    m_scanWidth = width-leftShit;
    m_scanHeight = height;

    if (m_scanComplited) m_surfaceModel->removeAll();
    m_surfaceModel->createZeroSurface(width,height,step,leftShit);

    scanSnapshotNumber = 0;

    QFile scanGCode("scanGCode.ngc");
    if (scanGCode.open(QFile::WriteOnly | QFile::Truncate)) {
        QTextStream out(&scanGCode);

        out << QString("M220 S100\nM204 S600\nG90 G0 X%1 Y0 F10000\n").arg(leftShit);

        for (int i=0; i<=height/step; i++)
        {
            out << QString("G0 Y%1\nM25\n").arg(step*i);

            for (int j=1; j<=width/step; j++)
                if (i%2)
                    out << QString("G0 X%1\nM25\n").arg(width/step*step-step*j+leftShit);
                else
                    out << QString("G0 X%1\nM25\n").arg(step*j+leftShit);
        }

        out << "G0 X0 Y0\n";
    }

    emit startScan(QUrl("file:scanGCode.ngc"));


    m_state = Scanning;
    emit stateChanged(m_state);

    m_scanComplited = false;
    m_scanApprooved = false;
}

void Automator::approveScan()
{
    m_scanComplited = false;
    emit scanStateChanged();

    m_scanApprooved = true;
}

void Automator::addMissingEntry(float entry)
{
    SurfacePoint point;
    point.x = qRound(m_mcs_x-605);
    point.y = qRound(m_mcs_y);
    point.z = entry;
    m_surfaceModel->updatePoint(point, scanSnapshotNumber);
    scanSnapshotNumber++;
    m_message = QString("Z: %1").arg(entry);
    emit messageChanged();
    qDebug() << m_message;

    /*SPLINTER::DenseVector x(2);
    float y;

    x(0) = qRound(m_mcs_x-605);
    x(1) = qRound(m_mcs_y);
    y = compensated;

    m_samples.addSample(x,y);*/

    m_entryMissing = false;

    emit continueScan();
}

SurfaceModel *Automator::surfaceModel() const
{
    return m_surfaceModel;
}

void Automator::clearSurface() const
{
    m_surfaceModel->removeAll();
}

Automator::State Automator::state() const
{
    return m_state;
}

float Automator::maxPower() const
{
    return m_maxPower;
}

void Automator::setMaxPower(float maxPower)
{
    m_maxPower = maxPower;
}

bool Automator::autosendPower() const
{
    return m_autosendPower;
}

void Automator::setAutosendPower(bool autosendPower)
{
    m_autosendPower = autosendPower;
}

bool Automator::cutModeEnabled() const
{
    return m_cutModeEnabled;
}

void Automator::setCutModeEnabled(bool cutMode)
{
    m_cutModeEnabled = cutMode;
    checkWorkingState();
}

bool Automator::scanComplited() const
{
    return m_scanComplited;
}

bool Automator::enabled() const
{
    return m_enabled;
}

void Automator::setEnabled(bool enabled)
{
    m_enabled = enabled;
    emit enabledChanged();
    checkWorkingState();
}

QString Automator::message() const
{
    return m_message;
}
