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
    const float map[] = {33,	2.40810520218361,
                         32.9,	2.41050073531094,
                         32.8,	2.41243249558688,
                         32.7,	2.41389957538154,
                         32.6,	2.41490139561771,
                         32.5,	2.41543769742353,
                         32.4,	2.41550853386105,
                         32.3,	2.41511426173082,
                         32.2,	2.41425553345237,
                         32.1,	2.41293328902072,
                         32,	2.41114874803885,
                         31.9,	2.40890340182609,
                         31.8,	2.40619900560257,
                         31.7,	2.40303757074949,
                         31.6,	2.39942135714553,
                         31.5,	2.39535286557909,
                         31.4,	2.39083483023658,
                         31.3,	2.38587021126662,
                         31.2,	2.38046218742024,
                         31.1,	2.37461414876706,
                         31,	2.36832968948738,
                         30.9,	2.3616126007403,
                         30.8,	2.3544668636078,
                         30.7,	2.34689664211471,
                         30.6,	2.33890627632475,
                         30.5,	2.33050027551248,
                         30.4,	2.32168331141123,
                         30.3,	2.312460211537,
                         30.2,	2.3028359525883,
                         30.1,	2.29281565392201,
                         30,	2.28240457110517,
                         29.9,	2.27160808954272,
                         29.8,	2.26043171818127,
                         29.7,	2.24888108328874,
                         29.6,	2.2369619223101,
                         29.5,	2.22468007779895,
                         29.4,	2.21204149142512,
                         29.3,	2.19905219805825,
                         29.2,	2.18571831992735,
                         29.1,	2.17204606085622,
                         29,	2.158041700575,
                         28.9,	2.14371158910757,
                         28.8,	2.12906214123494,
                         28.7,	2.11409983103464,
                         28.6,	2.09883118649602,
                         28.5,	2.0832627842116,
                         28.4,	2.06740124414432,
                         28.3,	2.05125322447073,
                         28.2,	2.03482541650028,
                         28.1,	2.01812453967042,
                         28,	2.00115733661775,
                         27.9,	1.98393056832515,
                         27.8,	1.96645100934483,
                         27.7,	1.94872544309737,
                         27.6,	1.93076065724672,
                         27.5,	1.91256343915118,
                         27.4,	1.89414057139035,
                         27.3,	1.875498827368,
                         27.2,	1.85664496699098,
                         27.1,	1.83758573242406,
                         27,	1.81832784392067,
                         26.9,	1.79887799572979,
                         26.8,	1.77924285207858,
                         26.7,	1.75942904323115,
                         26.6,	1.73944316162321,
                         26.5,	1.71929175807274,
                         26.4,	1.69898133806655,
                         26.3,	1.67851835812291,
                         26.2,	1.65790922223005,
                         26.1,	1.6371602783607,
                         26,	1.61627781506255,
                         25.9,	1.59526805812468,
                         25.8,	1.57413716732001,
                         25.7,	1.55289123322365,
                         25.6,	1.53153627410724,
                         25.5,	1.51007823290928,
                         25.4,	1.48852297428141,
                         25.3,	1.46687628171063,
                         25.2,	1.44514385471753,
                         25.1,	1.42333130613047,
                         25,	1.40144415943574,
                         24.9,	1.37948784620365,
                         24.8,	1.3574677035906,
                         24.7,	1.33538897191717,
                         24.6,	1.31325679232211,
                         24.5,	1.29107620449231,
                         24.4,	1.26885214446878,
                         24.3,	1.24658944252853,
                         24.2,	1.22429282114248,
                         24.1,	1.2019668930093,
                         24,	1.17961615916523,
                         23.9,	1.15724500716984,
                         23.8,	1.13485770936782,
                         23.7,	1.11245842122669,
                         23.6,	1.09005117975043,
                         23.5,	1.06763990196922,
                         23.4,	1.04522838350498,
                         23.3,	1.02282029721302,
                         23.2,	1.00041919189953,
                         23.1,	0.97802849111515,
                         23,	0.95565149202443,
                         22.9,	0.933291364351294,
                         22.8,	0.910951149400459,
                         22.7,	0.888633759154817,
                         22.6,	0.866341975448798,
                         22.5,	0.844078449217683,
                         22.4,	0.821845699822902,
                         22.3,	0.799646114453281,
                         22.2,	0.777481947602268,
                         22.1,	0.755355320621124,
                         22,	0.733268221348079,
                         21.9,	0.711222503813453,
                         21.8,	0.68921988802075,
                         21.7,	0.667261959803711,
                         21.6,	0.645350170759343,
                         21.5,	0.623485838256911,
                         21.4,	0.601670145522892,
                         21.3,	0.579904141801904,
                         21.2,	0.558188742593597,
                         21.1,	0.536524729965512,
                         21,	0.514912752941912,
                         20.9,	0.49335332796857,
                         20.8,	0.471846839453531,
                         20.7,	0.45039354038384,
                         20.6,	0.42899355301824,
                         20.5,	0.407646869655827,
                         20.4,	0.386353353480682,
                         20.3,	0.365112739482468,
                         20.2,	0.343924635452988,
                         20.1,	0.322788523058721,
                         20,	0.301703758989311,
                         19.9,	0.280669576182036,
                         19.8,	0.259685085122238,
                         19.7,	0.238749275219715,
                         19.6,	0.217861016261094,
                         19.5,	0.197019059938158,
                         19.4,	0.176222041452141,
                         19.3,	0.155468481194001,
                         19.2,	0.134756786500645,
                         19.1,	0.114085253487136,
                         19,	0.0934520689548516,
                         18.9,	0.0728553123756247,
                         18.8,	0.0522929579518399,
                         18.7,	0.0317628767524995,
                         18.6,	0.0112628389252633,
                         18.5,	-0.00920948401555621,
                         18.4,	-0.0296565168250197,
                         18.3,	-0.0500807780876285,
                         18.2,	-0.0704848777014241,
                         18.1,	-0.0908715142861128,
                         18,	-0.111243472515236,
                         17.9,	-0.131603620372363,
                         17.8,	-0.151954906331319,
                         17.7,	-0.172300356460453,
                         17.6,	-0.192643071450927,
                         17.5,	-0.212986223569052,
                         17.4,	-0.233333053532642,
                         17.3,	-0.253686867311412,
                         17.2,	-0.274051032851409,
                         17.1,	-0.294428976723464,
                         17,	-0.314824180695697,
                         16.9,	-0.335240178230033,
                         16.8,	-0.355680550902765,
                         16.7,	-0.376148924749152,
                         16.6,	-0.396648966532037,
                         16.5,	-0.417184379934511,
                         16.4,	-0.4377589016766,
                         16.3,	-0.458376297555992,
                         16.2,	-0.479040358412799,
                         16.1,	-0.499754896018338,
                         16,	-0.520523738887963,
                         15.9,	-0.541350728017917,
                         15.8,	-0.562239712546225,
                         15.7,	-0.583194545337613,
                         15.6,	-0.604219078492465,
                         15.5,	-0.625317158779816,
                         15.4,	-0.646492622994365,
                         15.3,	-0.667749293237537,
                         15.2,	-0.689090972122567,
                         15.1,	-0.710521437903623,
                         15,	-0.732044439528956,
                         14.9,	-0.753663691618093,
                         14.8,	-0.775382869363048,
                         14.7,	-0.797205603353583,
                         14.6,	-0.819135474326488,
                         14.5,	-0.841176007838902,
                         14.4,	-0.863330668865665,
                         14.3,	-0.885602856320702,
                         14.2,	-0.90799589750244,
                         14.1,	-0.93051304246326,
                         14,	-0.95315745830298,
                         13.9,	-0.975932223386373,
                         13.8,	-0.998840321484712,
                         13.7,	-1.02188463584136,
                         13.6,	-1.04506794316138,
                         13.5,	-1.06839290752518,
                         13.4,	-1.09186207422622,
                         13.3,	-1.11547786353268,
                         13.2,	-1.13924256437326,
                         13.1,	-1.16315832794691,
                         13,	-1.18722716125669,
                         12.9,	-1.21145092056759,
                         12.8,	-1.23583130478841,
                         12.7,	-1.26036984877769,
                         12.6,	-1.28506791657363,
                         12.5,	-1.30992669454808,
                         12.4,	-1.33494718448458,
                         12.3,	-1.36013019658036,
                         12.2,	-1.38547634237243,
                         12.1,	-1.41098602758772,
                         12,	-1.43665944491718,
                         11.9,	-1.46249656671399,
                         11.8,	-1.48849713761576,
                         11.7,	-1.51466066709076,
                         11.6,	-1.54098642190822,
                         11.5,	-1.56747341853262,
                         11.4,	-1.59412041544203,
                         11.3,	-1.62092590537049,
                         11.2,	-1.64788810747444,
                         11.1,	-1.67500495942313,
                         11,	-1.70227410941308,
                         10.9,	-1.72969290810665,
                         10.8,	-1.75725840049452,
                         10.7,	-1.78496731768229,
                         10.6,	-1.81281606860107,
                         10.5,	-1.84080073164214,
                         10.4,	-1.8689170462156,
                         10.3,	-1.89716040423309,
                         10.2,	-1.92552584151452,
                         10.1,	-1.95400802911884,
                         10,	-1.98260126459887,
                         9.9,	-2.01129946318008,
                         9.8,	-2.04009614886351,
                         9.7,	-2.06898444545267,
                         9.6,	-2.09795706750443,
                         9.5,	-2.12700631120406,
                         9.4,	-2.15612404516416,
                         9.3,	-2.18530170114774,
                         9.2,	-2.21453026471529,
                         9.1,	-2.24380026579584,
                         9,	-2.27310176918214,
                         8.9,	-2.3024243649498,
                         8.8,	-2.3317571588005,
                         8.7,	-2.36108876232922,
                         8.6,	-2.39040728321551,
                         8.5,	-2.41970031533876,
                         8.4,	-2.44895492881759,
                         8.3,	-2.47815765997317,
                         8.2,	-2.50729450121662,
                         8.1,	-2.53635089086045,
                         8,	-2.56531170285405,
                         7.9,	-2.59416123644312,
                         7.8,	-2.62288320575328,
                         7.7,	-2.65146072929757,
                         7.6,	-2.67987631940807,
                         7.5,	-2.70811187159154,
                         7.4,	-2.73614865380907,
                         7.3,	-2.76396729567976,
                         7.2,	-2.79154777760847,
                         7.1,	-2.81886941983759,
                         7,	-2.8459108714228,
                         6.9,	-2.87265009913291,
                         6.8,	-2.89906437627375,
                         6.7,	-2.92513027143602,
                         6.6,	-2.95082363716722,
                         6.5,	-2.97611959856766,
                         6.4,	-3.00099254181038,
                         6.3,	-3.02541610258523,
                         6.2,	-3.04936315446689,
                         6.1,	-3.07280579720699,
                         6,	-3.09571534495023,
                         5.9,	-3.11806231437451,
                         5.8,	-3.13981641275515,
                         5.7,	-3.1609465259531,
                         5.6,	-3.1814207063272,
                         5.5,	-3.20120616057047,
                         5.4,	-3.22026923747041,
                         5.3,	-3.2385754155934,
                         5.2,	-3.25608929089305,
                         5.1,	-3.27277456424262,
                         5,	-3.28859402889151,
                         4.9,	-3.30350955784571,
                         4.8,	-3.31748209117233,
                         4.7,	-3.33047162322818,
                         4.6,	-3.3424371898123,
                         4.5,	-3.35333685524262,
                         4.4,	-3.36312769935664,
                         4.3,	-3.37176580443602,
                         4.2,	-3.3792062420554,
                         4.1,	-3.38540305985509,
                         4,	-3.39030926823787,
                         3.9,	-3.39387682698983,
                         3.8,	-3.39605663182516,
                         3.7,	-3.39679850085511,
                         3.6,	-3.39605116098084,
                         3.5,	-3.39376223421041,
                         3.4,	-3.38987822389975
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

        out << "M220 S100\nM204 S600\nG90 G0 X0 Y0 F10000\n";

        for (int i=0; i<=height/step; i++)
        {
            out << QString("G0 Y%1\nM25\n").arg(step*i);

            for (int j=1; j<=width/step; j++)
                if (i%2)
                    out << QString("G0 X%1\nM25\n").arg(width/step*step-step*j);
                else
                    out << QString("G0 X%1\nM25\n").arg(step*j);
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
