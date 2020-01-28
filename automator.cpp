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
    const float map[] = {33,	1.90909157534572,
                         32.9,	1.8938785383023,
                         32.8,	1.87862486533457,
                         32.7,	1.86333055644253,
                         32.6,	1.84799561162618,
                         32.5,	1.83262003088552,
                         32.4,	1.81720381422055,
                         32.3,	1.80174696163127,
                         32.2,	1.78624947311768,
                         32.1,	1.77071134867978,
                         32,	1.75513258831757,
                         31.9,	1.73951319203105,
                         31.8,	1.72385315982022,
                         31.7,	1.70815249168508,
                         31.6,	1.69241118762564,
                         31.5,	1.67662924764188,
                         31.4,	1.66080667173381,
                         31.3,	1.64494345990144,
                         31.2,	1.62903961214475,
                         31.1,	1.61309512846375,
                         31,	1.59711000885845,
                         30.9,	1.58108425332883,
                         30.8,	1.56501786187491,
                         30.7,	1.54891083449667,
                         30.6,	1.53276317119413,
                         30.5,	1.51657487196727,
                         30.4,	1.50034593681611,
                         30.3,	1.48407636574064,
                         30.2,	1.46776615874085,
                         30.1,	1.45141531581676,
                         30,	1.43502383696836,
                         29.9,	1.41859172219565,
                         29.8,	1.40211897149863,
                         29.7,	1.38560558487729,
                         29.6,	1.36905156233165,
                         29.5,	1.3524569038617,
                         29.4,	1.33582160946744,
                         29.3,	1.31914567914887,
                         29.2,	1.30242911290599,
                         29.1,	1.2856719107388,
                         29,	1.2688740726473,
                         28.9,	1.25203559863149,
                         28.8,	1.23515648869137,
                         28.7,	1.21823674282695,
                         28.6,	1.20127636103821,
                         28.5,	1.18427534332516,
                         28.4,	1.1672336896878,
                         28.3,	1.15015140012614,
                         28.2,	1.13302847464016,
                         28.1,	1.11586491322987,
                         28,	1.09866071589528,
                         27.9,	1.08141588263637,
                         27.8,	1.06413041345316,
                         27.7,	1.04680430834563,
                         27.6,	1.0294375673138,
                         27.5,	1.01203019035765,
                         27.4,	0.994582177477198,
                         27.3,	0.977093528672434,
                         27.2,	0.959564243943361,
                         27.1,	0.941994323289977,
                         27,	0.924383766712286,
                         26.9,	0.906732574210283,
                         26.8,	0.88904074578397,
                         26.7,	0.871308281433348,
                         26.6,	0.853535181158418,
                         26.5,	0.835721444959176,
                         26.4,	0.817867072835625,
                         26.3,	0.799972064787764,
                         26.2,	0.782036420815594,
                         26.1,	0.764060140919115,
                         26,	0.746043225098325,
                         25.9,	0.727985673353226,
                         25.8,	0.709887485683817,
                         25.7,	0.691748662090098,
                         25.6,	0.673569202572071,
                         25.5,	0.655349107129732,
                         25.4,	0.637088375763084,
                         25.3,	0.618787008472127,
                         25.2,	0.60044500525686,
                         25.1,	0.582062366117284,
                         25,	0.563639091053397,
                         24.9,	0.545175180065201,
                         24.8,	0.526670633152695,
                         24.7,	0.50812545031588,
                         24.6,	0.489539631554755,
                         24.5,	0.47091317686932,
                         24.4,	0.452246086259576,
                         24.3,	0.433538359725521,
                         24.2,	0.414789997267157,
                         24.1,	0.396000998884485,
                         24,	0.377171364577502,
                         23.9,	0.358301094346208,
                         23.8,	0.339390188190606,
                         23.7,	0.320438646110693,
                         23.6,	0.301446468106472,
                         23.5,	0.282413654177941,
                         23.4,	0.263340204325099,
                         23.3,	0.244226118547948,
                         23.2,	0.225071396846487,
                         23.1,	0.205876039220718,
                         23,	0.186640045670638,
                         22.9,	0.167363416196248,
                         22.8,	0.148046150797549,
                         22.7,	0.128688249474539,
                         22.6,	0.109289712227222,
                         22.5,	0.0898505390555931,
                         22.4,	0.0703707299596548,
                         22.3,	0.0508502849394069,
                         22.2,	0.0312892039948493,
                         22.1,	0.0116874871259834,
                         22,	-0.00795486566719359,
                         21.9,	-0.0276378543846802,
                         21.8,	-0.0473614790264766,
                         21.7,	-0.0671257395925826,
                         21.6,	-0.0869306360829976,
                         21.5,	-0.106776168497722,
                         21.4,	-0.126662336836757,
                         21.3,	-0.146589141100102,
                         21.2,	-0.166556581287756,
                         21.1,	-0.18656465739972,
                         21,	-0.206613369435993,
                         20.9,	-0.226702717396576,
                         20.8,	-0.246832701281469,
                         20.7,	-0.267003321090672,
                         20.6,	-0.287214576824184,
                         20.5,	-0.307466468482005,
                         20.4,	-0.327758996064137,
                         20.3,	-0.348092159570579,
                         20.2,	-0.368465959001329,
                         20.1,	-0.38888039435639,
                         20,	-0.40933546563576,
                         19.9,	-0.42983117283944,
                         19.8,	-0.45036751596743,
                         19.7,	-0.470944495019729,
                         19.6,	-0.491562109996338,
                         19.5,	-0.512220360897256,
                         19.4,	-0.532919247722485,
                         19.3,	-0.553658770472023,
                         19.2,	-0.574438929145871,
                         19.1,	-0.595259723744028,
                         19,	-0.616121154266495,
                         18.9,	-0.637023220713272,
                         18.8,	-0.657965923084359,
                         18.7,	-0.678949261379754,
                         18.6,	-0.69997323559946,
                         18.5,	-0.721037845743475,
                         18.4,	-0.742143091811801,
                         18.3,	-0.763288973804436,
                         18.2,	-0.78447549172138,
                         18.1,	-0.805702645562634,
                         18,	-0.826970435328198,
                         17.9,	-0.848278861018071,
                         17.8,	-0.869627922632255,
                         17.7,	-0.891017620170747,
                         17.6,	-0.91244795363355,
                         17.5,	-0.933918923020662,
                         17.4,	-0.955430528332084,
                         17.3,	-0.976982769567816,
                         17.2,	-0.998575646727857,
                         17.1,	-1.02020915981221,
                         17,	-1.04188330882087,
                         16.9,	-1.06359809375384,
                         16.8,	-1.08535351461112,
                         16.7,	-1.10714957139271,
                         16.6,	-1.12898626409861,
                         16.5,	-1.15086359272882,
                         16.4,	-1.17278155728334,
                         16.3,	-1.19474015776216,
                         16.2,	-1.2167393941653,
                         16.1,	-1.23877926649275,
                         16,	-1.26085977474451,
                         15.9,	-1.28298091892057,
                         15.8,	-1.30514269902095,
                         15.7,	-1.32734511504564,
                         15.6,	-1.34958816699463,
                         15.5,	-1.37187185486794,
                         15.4,	-1.39419617866555,
                         15.3,	-1.41656113838748,
                         15.2,	-1.43896673403371,
                         15.1,	-1.46141296560426,
                         15,	-1.48389983309911,
                         14.9,	-1.50642733651828,
                         14.8,	-1.52899547586175,
                         14.7,	-1.55160425112953,
                         14.6,	-1.57425366232163,
                         14.5,	-1.59694370943803,
                         14.4,	-1.61967439247874,
                         14.3,	-1.64244571144376,
                         14.2,	-1.66525766633309,
                         14.1,	-1.68811025714674,
                         14,	-1.71100348388469,
                         13.9,	-1.73393734654695,
                         13.8,	-1.75691184513352,
                         13.7,	-1.7799269796444,
                         13.6,	-1.80298275007959,
                         13.5,	-1.82607915643909,
                         13.4,	-1.8492161987229,
                         13.3,	-1.87239387693101,
                         13.2,	-1.89561219106344,
                         13.1,	-1.91887114112018,
                         13,	-1.94217072710123,
                         12.9,	-1.96551094900659,
                         12.8,	-1.98889180683625,
                         12.7,	-2.01231330059023,
                         12.6,	-2.03577543026852,
                         12.5,	-2.05927819587111,
                         12.4,	-2.08282159739802,
                         12.3,	-2.10640563484923,
                         12.2,	-2.13003030822476,
                         12.1,	-2.15369561752459,
                         12,	-2.17740156274874,
                         11.9,	-2.20114814389719,
                         11.8,	-2.22493536096996,
                         11.7,	-2.24876321396703,
                         11.6,	-2.27263170288841,
                         11.5,	-2.29654082773411,
                         11.4,	-2.32049058850411,
                         11.3,	-2.34448098519842,
                         11.2,	-2.36851201781704,
                         11.1,	-2.39258368635997,
                         11,	-2.41669599082722,
                         10.9,	-2.44084893121877,
                         10.8,	-2.46504250753463,
                         10.7,	-2.4892767197748,
                         10.6,	-2.51355156793928,
                         10.5,	-2.53786705202807,
                         10.4,	-2.56222317204117,
                         10.3,	-2.58661992797858,
                         10.2,	-2.61105731984029,
                         10.1,	-2.63553534762632,
                         10,	-2.66005401133666,
                         9.9,	-2.68461331097131,
                         9.8,	-2.70921324653027,
                         9.7,	-2.73385381801353,
                         9.6,	-2.75853502542111,
                         9.5,	-2.783256868753,
                         9.4,	-2.80801934800919,
                         9.3,	-2.8328224631897,
                         9.2,	-2.85766621429451,
                         9.1,	-2.88255060132364,
                         9,	-2.90747562427707,
                         8.9,	-2.93244128315482,
                         8.8,	-2.95744757795687,
                         8.7,	-2.98249450868324,
                         8.6,	-3.00758207533391,
                         8.5,	-3.03271027790889,
                         8.4,	-3.05787911640819,
                         8.3,	-3.08308859083179,
                         8.2,	-3.1083387011797,
                         8.1,	-3.13362944745192,
                         8,	-3.15896082964845,
                         7.9,	-3.1843328477693,
                         7.8,	-3.20974550181445,
                         7.7,	-3.23519879178391,
                         7.6,	-3.26069271767768,
                         7.5,	-3.28622727949576,
                         7.4,	-3.31180247723815,
                         7.3,	-3.33741831090485,
                         7.2,	-3.36307478049586,
                         7.1,	-3.38877188601117,
                         7,	-3.4145096274508,
                         6.9,	-3.44028800481474,
                         6.8,	-3.46610701810299,
                         6.7,	-3.49196666731555,
                         6.6,	-3.51786695245241,
                         6.5,	-3.54380787351359,
                         6.4,	-3.56978943049908,
                         6.3,	-3.59581162340887,
                         6.2,	-3.62187445224298,
                         6.1,	-3.64797791700139,
                         6,	-3.67412201768412,
                         5.9,	-3.70030675429115,
                         5.8,	-3.7265321268225,
                         5.7,	-3.75279813527815,
                         5.6,	-3.77910477965812,
                         5.5,   -3.78
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
