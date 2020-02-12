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
    m_autosendB = false;
    m_autosendPower = false;
    m_minPower = 1.0;
    m_maxPower = 0.8;
    m_lastSentPower = 0.0;
    m_powerTimer.setInterval(1000); // maximum power update rate [ms]
    m_powerTimer.setSingleShot(true);

    m_mcs_b_initial = 0;
    m_mcs_x_check_state = 0;
    m_mcs_y_check_state = 0;
    m_compensatorOneShot = false;
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
    if (!m_working)
        return;
    if (s == RayReceiver::Paused) {
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
    if (s == RayReceiver::Playing)
        m_compensatorOneShot = false;
}

void Automator::onCameraStateChanged(CaptureController::Status s)
{
    if (s == CaptureController::Status::Stopped)
    {
        m_cameraConnected = false;
        checkWorkingState();
    }

}

void Automator::compensate()
{
    if (!m_working)
        return;
    float compensated = compensate(m_lastdz);
    if (compensated > 10) {
        m_message = "No entry in comp table";
        emit messageChanged();
    } else {
        float targetB = m_mcs_b_initial + compensated;
        QString correction = QString("G90 G0 B%1\n").arg(targetB);
        m_message = correction;
        emit messageChanged();
        qDebug() << "Command: " << correction;
        if (m_autosendB) {
            emit sendToMC(correction);
            emit sendToMC("M24\n");
        }
    }
}

float Automator::compensate(float dz) const
{
    //                   CAM     ERROR
    const float map[] = {33,	2.52564934482776,
                         32.9,	2.51293396378311,
                         32.8,	2.50015530870311,
                         32.7,	2.48731341987319,
                         32.6,	2.47440833757877,
                         32.5,	2.4614401021053,
                         32.4,	2.4484087537382,
                         32.3,	2.43531433276289,
                         32.2,	2.42215687946481,
                         32.1,	2.40893643412939,
                         32,	2.39565303704206,
                         31.9,	2.38230672848825,
                         31.8,	2.3688975487534,
                         31.7,	2.35542553812291,
                         31.6,	2.34189073688224,
                         31.5,	2.32829318531681,
                         31.4,	2.31463292371205,
                         31.3,	2.30090999235339,
                         31.2,	2.28712443152626,
                         31.1,	2.27327628151609,
                         31,	2.25936558260831,
                         30.9,	2.24539237508834,
                         30.8,	2.23135669924163,
                         30.7,	2.2172585953536,
                         30.6,	2.20309810370968,
                         30.5,	2.1888752645953,
                         30.4,	2.17459011829589,
                         30.3,	2.16024270509688,
                         30.2,	2.1458330652837,
                         30.1,	2.13136123914178,
                         30,	2.11682726695655,
                         29.9,	2.10223118901345,
                         29.8,	2.08757304559789,
                         29.7,	2.07285287699532,
                         29.6,	2.05807072349115,
                         29.5,	2.04322662537083,
                         29.4,	2.02832062291978,
                         29.3,	2.01335275642343,
                         29.2,	1.99832306616721,
                         29.1,	1.98323159243655,
                         29,	1.96807837551688,
                         28.9,	1.95286345569364,
                         28.8,	1.93758687325224,
                         28.7,	1.92224866847813,
                         28.6,	1.90684888165673,
                         28.5,	1.89138755307347,
                         28.4,	1.87586472301379,
                         28.3,	1.8602804317631,
                         28.2,	1.84463471960685,
                         28.1,	1.82892762683046,
                         28,	1.81315919371936,
                         27.9,	1.79732946055898,
                         27.8,	1.78143846763476,
                         27.7,	1.76548625523212,
                         27.6,	1.74947286363649,
                         27.5,	1.7333983331333,
                         27.4,	1.71726270400799,
                         27.3,	1.70106601654598,
                         27.2,	1.6848083110327,
                         27.1,	1.66848962775358,
                         27,	1.65211000699406,
                         26.9,	1.63566948903956,
                         26.8,	1.61916811417551,
                         26.7,	1.60260592268735,
                         26.6,	1.58598295486049,
                         26.5,	1.56929925098039,
                         26.4,	1.55255485133245,
                         26.3,	1.53574979620212,
                         26.2,	1.51888412587482,
                         26.1,	1.50195788063599,
                         26,	1.48497110077105,
                         25.9,	1.46792382656543,
                         25.8,	1.45081609830457,
                         25.7,	1.43364795627389,
                         25.6,	1.41641944075882,
                         25.5,	1.3991305920448,
                         25.4,	1.38178145041725,
                         25.3,	1.36437205616161,
                         25.2,	1.3469024495633,
                         25.1,	1.32937267090775,
                         25,	1.3117827604804,
                         24.9,	1.29413275856667,
                         24.8,	1.276422705452,
                         24.7,	1.25865264142181,
                         24.6,	1.24082260676154,
                         24.5,	1.22293264175661,
                         24.4,	1.20498278669246,
                         24.3,	1.18697308185451,
                         24.2,	1.16890356752819,
                         24.1,	1.15077428399894,
                         24,	1.13258527155219,
                         23.9,	1.11433657047336,
                         23.8,	1.09602822104788,
                         23.7,	1.0776602635612,
                         23.6,	1.05923273829872,
                         23.5,	1.04074568554589,
                         23.4,	1.02219914558814,
                         23.3,	1.00359315871089,
                         23.2,	0.984927765199579,
                         23.1,	0.966203005339633,
                         23,	0.947418919416482,
                         22.9,	0.928575547715556,
                         22.8,	0.909672930522287,
                         22.7,	0.890711108122104,
                         22.6,	0.871690120800437,
                         22.5,	0.852610008842715,
                         22.4,	0.833470812534369,
                         22.3,	0.81427257216083,
                         22.2,	0.795015328007527,
                         22.1,	0.775699120359891,
                         22,	0.756323989503351,
                         21.9,	0.736889975723337,
                         21.8,	0.71739711930528,
                         21.7,	0.697845460534609,
                         21.6,	0.678235039696756,
                         21.5,	0.658565897077149,
                         21.4,	0.638838072961218,
                         21.3,	0.619051607634395,
                         21.2,	0.599206541382109,
                         21.1,	0.57930291448979,
                         21,	0.559340767242867,
                         20.9,	0.539320139926772,
                         20.8,	0.519241072826935,
                         20.7,	0.499103606228784,
                         20.6,	0.478907780417751,
                         20.5,	0.458653635679266,
                         20.4,	0.438341212298758,
                         20.3,	0.417970550561657,
                         20.2,	0.397541690753394,
                         20.1,	0.3770546731594,
                         20,	0.356509538065102,
                         19.9,	0.335906325755933,
                         19.8,	0.315245076517322,
                         19.7,	0.294525830634698,
                         19.6,	0.273748628393494,
                         19.5,	0.252913510079136,
                         19.4,	0.232020515977058,
                         19.3,	0.211069686372687,
                         19.2,	0.190061061551455,
                         19.1,	0.168994681798792,
                         19,	0.147870587400127,
                         18.9,	0.126688818640891,
                         18.8,	0.105449415806513,
                         18.7,	0.0841524191824236,
                         18.6,	0.0627978690540538,
                         18.5,	0.0413858057068322,
                         18.4,	0.0199162694261904,
                         18.3,	-0.00161069950244318,
                         18.2,	-0.0231950607936376,
                         18.1,	-0.0448367741619621,
                         18,	-0.0665357993219882,
                         17.9,	-0.0882920959882841,
                         17.8,	-0.110105623875421,
                         17.7,	-0.131976342697969,
                         17.6,	-0.153904212170497,
                         17.5,	-0.175889192007576,
                         17.4,	-0.197931241923774,
                         17.3,	-0.220030321633663,
                         17.2,	-0.242186390851813,
                         17.1,	-0.264399409292792,
                         17,	-0.286669336671171,
                         16.9,	-0.30899613270152,
                         16.8,	-0.33137975709841,
                         16.7,	-0.353820169576409,
                         16.6,	-0.376317329850088,
                         16.5,	-0.398871197634016,
                         16.4,	-0.421481732642764,
                         16.3,	-0.444148894590902,
                         16.2,	-0.466872643193,
                         16.1,	-0.489652938163626,
                         16,	-0.512489739217352,
                         15.9,	-0.535383006068747,
                         15.8,	-0.558332698432381,
                         15.7,	-0.581338776022825,
                         15.6,	-0.604401198554647,
                         15.5,	-0.627519925742419,
                         15.4,	-0.650694917300709,
                         15.3,	-0.673926132944088,
                         15.2,	-0.697213532387127,
                         15.1,	-0.720557075344393,
                         15,	-0.743956721530459,
                         14.9,	-0.767412430659892,
                         14.8,	-0.790924162447265,
                         14.7,	-0.814491876607146,
                         14.6,	-0.838115532854104,
                         14.5,	-0.861795090902712,
                         14.4,	-0.885530510467537,
                         14.3,	-0.909321751263151,
                         14.2,	-0.933168773004123,
                         14.1,	-0.957071535405022,
                         14,	-0.98102999818042,
                         13.9,	-1.00504412104489,
                         13.8,	-1.02911386371299,
                         13.7,	-1.0532391858993,
                         13.6,	-1.07742004731839,
                         13.5,	-1.10165640768482,
                         13.4,	-1.12594822671318,
                         13.3,	-1.15029546411802,
                         13.2,	-1.17469807961392,
                         13.1,	-1.19915603291544,
                         13,	-1.22366928373717,
                         12.9,	-1.24823779179366,
                         12.8,	-1.27286151679948,
                         12.7,	-1.29754041846922,
                         12.6,	-1.32227445651743,
                         12.5,	-1.34706359065869,
                         12.4,	-1.37190778060756,
                         12.3,	-1.39680698607862,
                         12.2,	-1.42176116678644,
                         12.1,	-1.44677028244558,
                         12,	-1.47183429277062,
                         11.9,	-1.49695315747613,
                         11.8,	-1.52212683627668,
                         11.7,	-1.54735528888683,
                         11.6,	-1.57263847502115,
                         11.5,	-1.59797635439422,
                         11.4,	-1.62336888672061,
                         11.3,	-1.64881603171489,
                         11.2,	-1.67431774909162,
                         11.1,	-1.69987399856537,
                         11,	-1.72548473985073,
                         10.9,	-1.75114993266224,
                         10.8,	-1.7768695367145,
                         10.7,	-1.80264351172206,
                         10.6,	-1.82847181739949,
                         10.5,	-1.85435441346137,
                         10.4,	-1.88029125962226,
                         10.3,	-1.90628231559674,
                         10.2,	-1.93232754109938,
                         10.1,	-1.95842689584474,
                         10,	-1.9845803395474,
                         9.9,	-2.01078783192192,
                         9.8,	-2.03704933268287,
                         9.7,	-2.06336480154483,
                         9.6,	-2.08973419822237,
                         9.5,	-2.11615748243005,
                         9.4,	-2.14263461388244,
                         9.3,	-2.16916555229412,
                         9.2,	-2.19575025737966,
                         9.1,	-2.22238868885362,
                         9,	-2.24908080643057,
                         8.9,	-2.27582656982508,
                         8.8,	-2.30262593875174,
                         8.7,	-2.32947887292509,
                         8.6,	-2.35638533205972,
                         8.5,	-2.38334527587019,
                         8.4,	-2.41035866407108,
                         8.3,	-2.43742545637695,
                         8.2,	-2.46454561250238,
                         8.1,	-2.49171909216192,
                         8,	-2.51894585507017,
                         7.9,	-2.54622586094167,
                         7.8,	-2.57355906949101,
                         7.7,	-2.60094544043276,
                         7.6,	-2.62838493348147,
                         7.5,	-2.65587750835173,
                         7.4,	-2.6834231247581,
                         7.3,	-2.71102174241516,
                         7.2,	-2.73867332103747,
                         7.1,	-2.7663778203396,
                         7,	-2.79413520003613,
                         6.9,	-2.82194541984161,
                         6.8,	-2.84980843947063,
                         6.7,	-2.87772421863776,
                         6.6,	-2.90569271705755,
                         6.5,	-2.93371389444459,
                         6.4,	-2.96178771051344,
                         6.3,	-2.98991412497867,
                         6.2,	-3.01809309755486,
                         6.1,	-3.04632458795657,
                         6,	-3.07460855589837,
                         5.9,	-3.10294496109483,
                         5.8,	-3.13133376326053,
                         5.7,	-3.15977492211002,
                         5.6,	-3.18826839735789,
                         5.5,	-3.2168141487187,
                         5.4,	-3.24541213590702,
                         5.3,	-3.27406231863743,
                         5.2,	-3.30276465662448,
                         5.1,	-3.33151910958276,
                         5,	-3.36032563722683,
                         4.9,	-3.38918419927126,
                         4.8,	-3.41809475543062,
                         4.7,	-3.44705726541948
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

void Automator::checkWorkingState()
{
    bool working = m_lastdzValid && m_mcConnected && m_lastCoordsValid && m_enabled && m_cameraConnected;
    if (working != m_working) {
        m_working = working;
        emit workingChanged();
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

bool Automator::autosendB() const
{
    return m_autosendB;
}

void Automator::setAutosendB(bool autosendB)
{
    m_autosendB = autosendB;
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
