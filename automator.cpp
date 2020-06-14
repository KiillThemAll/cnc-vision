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
        m_surfaceModel->saveSurfaceToJsonFile();

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
    const float map[] = {45.1,	1.55833954654446,
                         45,	1.54818771021748,
                         44.9,	1.53803066653259,
                         44.8,	1.52786831862347,
                         44.7,	1.51770056962379,
                         44.6,	1.50752732266724,
                         44.5,	1.4973484808875,
                         44.4,	1.48716394741823,
                         44.3,	1.47697362539312,
                         44.2,	1.46677741794585,
                         44.1,	1.4565752282101,
                         44,	1.44636695931953,
                         43.9,	1.43615251440784,
                         43.8,	1.42593179660869,
                         43.7,	1.41570470905577,
                         43.6,	1.40547115488275,
                         43.5,	1.39523103722331,
                         43.4,	1.38498425921113,
                         43.3,	1.37473072397989,
                         43.2,	1.36447033466326,
                         43.1,	1.35420299439492,
                         43,	1.34392860630855,
                         42.9,	1.33364707353783,
                         42.8,	1.32335829921644,
                         42.7,	1.31306218647805,
                         42.6,	1.30275863845634,
                         42.5,	1.29244755828499,
                         42.4,	1.28212884909768,
                         42.3,	1.27180241402808,
                         42.2,	1.26146815620987,
                         42.1,	1.25112597877673,
                         42,	1.24077578486234,
                         41.9,	1.23041747760038,
                         41.8,	1.22005096012451,
                         41.7,	1.20967613556844,
                         41.6,	1.19929290706581,
                         41.5,	1.18890117775033,
                         41.4,	1.17850085075566,
                         41.3,	1.16809182921548,
                         41.2,	1.15767401626348,
                         41.1,	1.14724731503332,
                         41,	1.13681162865868,
                         40.9,	1.12636686027325,
                         40.8,	1.1159129130107,
                         40.7,	1.10544969000471,
                         40.6,	1.09497709438896,
                         40.5,	1.08449502929712,
                         40.4,	1.07400339786288,
                         40.3,	1.0635021032199,
                         40.2,	1.05299104850187,
                         40.1,	1.04247013684247,
                         40,	1.03193927137537,
                         39.9,	1.02139835523426,
                         39.8,	1.0108472915528,
                         39.7,	1.00028598346468,
                         39.6,	0.989714334103573,
                         39.5,	0.97913224660316,
                         39.4,	0.968539624097117,
                         39.3,	0.957936369719122,
                         39.2,	0.947322386602854,
                         39.1,	0.936697577881989,
                         39,	0.926061846690205,
                         38.9,	0.915415096161181,
                         38.8,	0.904757229428594,
                         38.7,	0.894088149626122,
                         38.6,	0.883407759887443,
                         38.5,	0.872715963346234,
                         38.4,	0.862012663136173,
                         38.3,	0.851297762390939,
                         38.2,	0.840571164244209,
                         38.1,	0.82983277182966,
                         38,	0.81908248828097,
                         37.9,	0.808320216731818,
                         37.8,	0.797545860315881,
                         37.7,	0.786759322166838,
                         37.6,	0.775960505418364,
                         37.5,	0.765149313204139,
                         37.4,	0.75432564865784,
                         37.3,	0.743489414913145,
                         37.2,	0.732640515103732,
                         37.1,	0.721778852363278,
                         37,	0.710904329825462,
                         36.9,	0.700016850623961,
                         36.8,	0.689116317892454,
                         36.7,	0.678202634764616,
                         36.6,	0.667275704374128,
                         36.5,	0.656335429854665,
                         36.4,	0.645381714339907,
                         36.3,	0.634414460963531,
                         36.2,	0.623433572859215,
                         36.1,	0.612438953160636,
                         36,	0.601430505001472,
                         35.9,	0.590408131515401,
                         35.8,	0.579371735836102,
                         35.7,	0.568321221097251,
                         35.6,	0.557256490432527,
                         35.5,	0.546177446975606,
                         35.4,	0.535083993860168,
                         35.3,	0.523976034219891,
                         35.2,	0.51285347118845,
                         35.1,	0.501716207899525,
                         35,	0.490564147486793,
                         34.9,	0.479397193083932,
                         34.8,	0.468215247824621,
                         34.7,	0.457018214842535,
                         34.6,	0.445805997271354,
                         34.5,	0.434578498244755,
                         34.4,	0.423335620896416,
                         34.3,	0.412077268360016,
                         34.2,	0.400803343769231,
                         34.1,	0.389513750257738,
                         34,	0.378208390959217,
                         33.9,	0.366887169007345,
                         33.8,	0.355549987535801,
                         33.7,	0.34419674967826,
                         33.6,	0.332827358568402,
                         33.5,	0.321441717339904,
                         33.4,	0.310039729126443,
                         33.3,	0.298621297061699,
                         33.2,	0.287186324279348,
                         33.1,	0.275734713913069,
                         33,	0.264266369096538,
                         32.9,	0.252781192963434,
                         32.8,	0.241279088647436,
                         32.7,	0.229759959282219,
                         32.6,	0.218223708001463,
                         32.5,	0.206670237938844,
                         32.4,	0.195099452228042,
                         32.3,	0.183511254002734,
                         32.2,	0.171905546396596,
                         32.1,	0.160282232543308,
                         32,	0.148641215576547,
                         31.9,	0.136982398629991,
                         31.8,	0.125305684837318,
                         31.7,	0.113610977332204,
                         31.6,	0.101898179248329,
                         31.5,	0.0901671937193698,
                         31.4,	0.0784179238790043,
                         31.3,	0.0666502728609114,
                         31.2,	0.0548641437987669,
                         31.1,	0.0430594398262495,
                         31,	0.0312360640770371,
                         30.9,	0.0193939196848072,
                         30.8,	0.00753290978323902,
                         30.7,	-0.00434706249399183,
                         30.6,	-0.0162460940132066,
                         30.5,	-0.0281642816407274,
                         30.4,	-0.0401017222428765,
                         30.3,	-0.0520585126859752,
                         30.2,	-0.0640347498363475,
                         30.1,	-0.0760305305603148,
                         30,	-0.0880459517241992,
                         29.9,	-0.100081110194323,
                         29.8,	-0.112136102837007,
                         29.7,	-0.124211026518577,
                         29.6,	-0.136305978105351,
                         29.5,	-0.148421054463655,
                         29.4,	-0.160556352459808,
                         29.3,	-0.172711968960134,
                         29.2,	-0.184888000830954,
                         29.1,	-0.197084544938592,
                         29,	-0.20930169814937,
                         28.9,	-0.221539557329608,
                         28.8,	-0.233798219345629,
                         28.7,	-0.246077781063757,
                         28.6,	-0.258378339350313,
                         28.5,	-0.27069999107162,
                         28.4,	-0.283042833093998,
                         28.3,	-0.295406962283771,
                         28.2,	-0.307792475507261,
                         28.1,	-0.320199469630791,
                         28,	-0.332628041520681,
                         27.9,	-0.345078288043255,
                         27.8,	-0.357550306064834,
                         27.7,	-0.370044192451742,
                         27.6,	-0.3825600440703,
                         27.5,	-0.39509795778683,
                         27.4,	-0.407658030467655,
                         27.3,	-0.420240358979095,
                         27.2,	-0.432845040187476,
                         27.1,	-0.445472170959117,
                         27,	-0.458121848160342,
                         26.9,	-0.470794168657472,
                         26.8,	-0.48348922931683,
                         26.7,	-0.496207127004738,
                         26.6,	-0.508947958587518,
                         26.5,	-0.521711820931493,
                         26.4,	-0.534498810902984,
                         26.3,	-0.547309025368313,
                         26.2,	-0.560142561193805,
                         26.1,	-0.572999515245779,
                         26,	-0.585879984390559,
                         25.9,	-0.598784065494466,
                         25.8,	-0.611711855423823,
                         25.7,	-0.624663451044952,
                         25.6,	-0.637638949224176,
                         25.5,	-0.650638446827816,
                         25.4,	-0.663662040722195,
                         25.3,	-0.676709827773634,
                         25.2,	-0.689781904848456,
                         25.1,	-0.702878368812984,
                         25,	-0.71599931653354,
                         24.9,	-0.729144844876445,
                         24.8,	-0.742315050708022,
                         24.7,	-0.755510030894593,
                         24.6,	-0.768729882302481,
                         24.5,	-0.781974701798007,
                         24.4,	-0.795244586247494,
                         24.3,	-0.808539632517263,
                         24.2,	-0.821859937473638,
                         24.1,	-0.83520559798294,
                         24,	-0.848576710911492,
                         23.9,	-0.861973373125616,
                         23.8,	-0.875395681491633,
                         23.7,	-0.888843732875867,
                         23.6,	-0.90231762414464,
                         23.5,	-0.915817452164273,
                         23.4,	-0.929343313801089,
                         23.3,	-0.942895305921409,
                         23.2,	-0.956473525391557,
                         23.1,	-0.970078069077854,
                         23,	-0.983709033846624,
                         22.9,	-0.997366516564187,
                         22.8,	-1.01105061409687,
                         22.7,	-1.02476142331098,
                         22.6,	-1.03849904107286,
                         22.5,	-1.05226356424882,
                         22.4,	-1.06605508970519,
                         22.3,	-1.07987371430828,
                         22.2,	-1.09371953492442,
                         22.1,	-1.10759264841993,
                         22,	-1.12149315166114,
                         21.9,	-1.13542114151436,
                         21.8,	-1.14937671484593,
                         21.7,	-1.16335996852215,
                         21.6,	-1.17737099940935,
                         21.5,	-1.19140990437386,
                         21.4,	-1.205476780282,
                         21.3,	-1.21957172400008,
                         21.2,	-1.23369483239444,
                         21.1,	-1.24784620233139,
                         21,	-1.26202593067725,
                         20.9,	-1.27623411429836,
                         20.8,	-1.29047085006102,
                         20.7,	-1.30473623483157,
                         20.6,	-1.31903036547632,
                         20.5,	-1.3333533388616,
                         20.4,	-1.34770525185372,
                         20.3,	-1.36208620131902,
                         20.2,	-1.37649628412381,
                         20.1,	-1.39093559713442,
                         20,	-1.40540423721717,
                         19.9,	-1.41990230123837,
                         19.8,	-1.43442988606436,
                         19.7,	-1.44898708856145,
                         19.6,	-1.46357400559597,
                         19.5,	-1.47819073403424,
                         19.4,	-1.49283737074258,
                         19.3,	-1.50751401258731,
                         19.2,	-1.52222075643476,
                         19.1,	-1.53695769915124,
                         19,	-1.55172493760309,
                         18.9,	-1.56652256865662,
                         18.8,	-1.58135068917815,
                         18.7,	-1.59620939603401,
                         18.6,	-1.61109878609051,
                         18.5,	-1.62601895621399,
                         18.4,	-1.64097000327076,
                         18.3,	-1.65595202412715,
                         18.2,	-1.67096511564948,
                         18.1,	-1.68600937470406,
                         18,	-1.70108489815723,
                         17.9,	-1.7161917828753,
                         17.8,	-1.73133012572459,
                         17.7,	-1.74650002357144,
                         17.6,	-1.76170157328216,
                         17.5,	-1.77693487172307,
                         17.4,	-1.79220001576049,
                         17.3,	-1.80749710226075,
                         17.2,	-1.82282622809018,
                         17.1,	-1.83818749011508,
                         17,	-1.85358098520179,
                         16.9,	-1.86900681021662,
                         16.8,	-1.88446506202591,
                         16.7,	-1.89995583749596,
                         16.6,	-1.91547923349311,
                         16.5,	-1.93103534688367,
                         16.4,	-1.94662427453397,
                         16.3,	-1.96224611331033,
                         16.2,	-1.97790096007907,
                         16.1,	-1.99358891170651,
                         16,	-2.00931006505898,
                         15.9,	-2.0250645170028,
                         15.8,	-2.04085236440429,
                         15.7,	-2.05667370412977,
                         15.6,	-2.07252863304557,
                         15.5,	-2.08841724801801,
                         15.4,	-2.1043396459134,
                         15.3,	-2.12029592359808,
                         15.2,	-2.13628617793836,
                         15.1,	-2.15231050580056,
                         15,	-2.16836900405102,
                         14.9,	-2.18446176955604,
                         14.8,	-2.20058889918196,
                         14.7,	-2.21675048979509,
                         14.6,	-2.23294663826176,
                         14.5,	-2.24917744144829,
                         14.4,	-2.265442996221,
                         14.3,	-2.28174339944621,
                         14.2,	-2.29807874799025,
                         14.1,	-2.31444913871944,
                         14,	-2.33085466850009,
                         13.9,	-2.34729543419854,
                         13.8,	-2.36377153268111,
                         13.7,	-2.38028306081411,
                         13.6,	-2.39683011546387,
                         13.5,	-2.41341279349672,
                         13.4,	-2.43003119177896,
                         13.3,	-2.44668540717693,
                         13.2,	-2.46337553655695,
                         13.1,	-2.48010167678534,
                         13,	-2.49686392472843,
                         12.9,	-2.51366237725252,
                         12.8,	-2.53049713122396,
                         12.7,	-2.54736828350905,
                         12.6,	-2.56427593097413,
                         12.5,	-2.5812201704855,
                         12.4,	-2.59820109890951,
                         12.3,	-2.61521881311246,
                         12.2,	-2.63227340996068,
                         12.1,	-2.64936498632049,
                         12,	-2.66649363905822,
                         11.9,	-2.68365946504019,
                         11.8,	-2.70086256113271,
                         11.7,	-2.71810302420212,
                         11.6,	-2.73538095111472,
                         11.5,	-2.75269643873686,
                         11.4,	-2.77004958393484,
                         11.3,	-2.78744048357499,
                         11.2,	-2.80486923452363,
                         11.1,	-2.82233593364709,
                         11,	-2.83984067781169,
                         10.9,	-2.85738356388374,
                         10.8,	-2.87496468872957,
                         10.7,	-2.89258414921551,
                         10.6,	-2.91024204220787,
                         10.5,	-2.92793846457298,
                         10.4,	-2.94567351317716,
                         10.3,	-2.96344728488673,
                         10.2,	-2.98125987656802,
                         10.1,	-2.99911138508734,
                         10,	-3.01700190731102,
                         9.9,	-3.03493154010539,
                         9.8,	-3.05290038033675,
                         9.7,	-3.07090852487144,
                         9.6,	-3.08895607057578,
                         9.5,	-3.10704311431609,
                         9.4,	-3.12516975295869,
                         9.3,	-3.1433360833699,
                         9.2,	-3.16154220241605,
                         9.1,	-3.17978820696346,
                         9,	-3.19807419387845,
                         8.9,	-3.21640026002734,
                         8.8,	-3.23476650227646,
                         8.7,	-3.25317301749212,
                         8.6,	-3.27161990254065,
                         8.5,	-3.29010725428838,
                         8.4,	-3.30863516960162,
                         8.3,	-3.3272037453467,
                         8.2,	-3.34581307838993,
                         8.1,	-3.36446326559764,
                         8,	-3.38315440383616,
                         7.9,	-3.4018865899718,
                         7.8,	-3.42065992087089,
                         7.7,	-3.43947449339975,
                         7.6,	-3.4583304044247,
                         7.5,	-3.47722775081207,
                         7.4,	-3.49616662942817,
                         7.3,	-3.51514713713933,
                         7.2,	-3.53416937081187,
                         7.1,	-3.55323342731211,
                         7,	-3.57233940350638,
                         6.9,	-3.59148739626099,
                         6.8,	-3.61067750244227,
                         6.7,	-3.62990981891655,
                         6.6,	-3.64918444255014,
                         6.5,	-3.66850147020936,
                         6.4,	-3.68786099876054,
                         6.3,	-3.70726312507,
                         6.2,	-3.72670794600407,
                         6.1,	-3.74619555842906,
                         6,	-3.7657260592113,
                         5.9,	-3.7852995452171,
                         5.8,	-3.8049161133128,
                         5.7,	-3.82457586036471,
                         5.6,	-3.84427888323916,
                         5.5,	-3.86402527880246,
                         5.4,	-3.88381514392095,
                         5.3,	-3.90364857546093,
                         5.2,	-3.92352567028875,
                         5.1,	-3.9434465252707,
                         5,	-3.96341123727313,
                         4.9,	-3.98341990316235,
                         4.1,   -4
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
        if (!scanSnapshotNumber) {
            emit messageChanged();
            m_entryMissing = true;
            emit requestMissingEntry();
        } else {
            m_surfaceModel->updatePoint(m_surfaceModel->m_surface.at(scanSnapshotNumber-1), scanSnapshotNumber);
            scanSnapshotNumber++;
            emit continueScan();
        }

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
    //m_scanWidth = width-leftShit;
    //m_scanHeight = height;

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

            for (int j=1; j<=(width-leftShit)/step; j++)
                if (i%2)
                    out << QString("G0 X%1\nM25\n").arg((width-leftShit)/step*step-step*j+leftShit);
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

void Automator::loadLastScan()
{
    if (m_surfaceModel->loadSurfaceFromJsonFile()) {
        m_message = QString("Scan loaded");
        emit messageChanged();
        qDebug() << m_message;
        m_scanComplited = true;
        m_scanApprooved = false;
        emit scanStateChanged();
    }
    else {
        m_message = QString("Scan load fail");
        emit messageChanged();
        qDebug() << m_message;
    }
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
