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
    const float map[] = {33,	2.53121591118932,
                         32.9,	2.51879749258041,
                         32.8,	2.50631458493228,
                         32.7,	2.49376718824494,
                         32.6,	2.48115530251838,
                         32.5,	2.4684789277526,
                         32.4,	2.4557380639476,
                         32.3,	2.4429327111034,
                         32.2,	2.43006286921997,
                         32.1,	2.41712853829733,
                         32,	2.40412971833547,
                         31.9,	2.39106640933439,
                         31.8,	2.3779386112941,
                         31.7,	2.36474632421459,
                         31.6,	2.35148954809586,
                         31.5,	2.33816828293792,
                         31.4,	2.32478252874076,
                         31.3,	2.31133228550439,
                         31.2,	2.2978175532288,
                         31.1,	2.28423833191399,
                         31,	2.27059462155997,
                         30.9,	2.25688642216672,
                         30.8,	2.24311373373427,
                         30.7,	2.22927655626259,
                         30.6,	2.2153748897517,
                         30.5,	2.2014087342016,
                         30.4,	2.18737808961228,
                         30.3,	2.17328295598374,
                         30.2,	2.15912333331598,
                         30.1,	2.14489922160901,
                         30,	2.13061062086282,
                         29.9,	2.11625753107741,
                         29.8,	2.10183995225279,
                         29.7,	2.08735788438895,
                         29.6,	2.0728113274859,
                         29.5,	2.05820028154363,
                         29.4,	2.04352474656214,
                         29.3,	2.02878472254144,
                         29.2,	2.01398020948151,
                         29.1,	1.99911120738238,
                         29,	1.98417771624402,
                         28.9,	1.96917973606645,
                         28.8,	1.95411726684967,
                         28.7,	1.93899030859366,
                         28.6,	1.92379886129845,
                         28.5,	1.90854292496401,
                         28.4,	1.89322249959036,
                         28.3,	1.87783758517749,
                         28.2,	1.8623881817254,
                         28.1,	1.8468742892341,
                         28,	1.83129590770358,
                         27.9,	1.81565303713385,
                         27.8,	1.7999456775249,
                         27.7,	1.78417382887673,
                         27.6,	1.76833749118935,
                         27.5,	1.75243666446275,
                         27.4,	1.73647134869693,
                         27.3,	1.72044154389189,
                         27.2,	1.70434725004764,
                         27.1,	1.68818846716418,
                         27,	1.67196519524149,
                         26.9,	1.6556774342796,
                         26.8,	1.63932518427848,
                         26.7,	1.62290844523815,
                         26.6,	1.6064272171586,
                         26.5,	1.58988150003983,
                         26.4,	1.57327129388185,
                         26.3,	1.55659659868465,
                         26.2,	1.53985741444824,
                         26.1,	1.52305374117261,
                         26,	1.50618557885776,
                         25.9,	1.4892529275037,
                         25.8,	1.47225578711042,
                         25.7,	1.45519415767792,
                         25.6,	1.43806803920621,
                         25.5,	1.42087743169528,
                         25.4,	1.40362233514513,
                         25.3,	1.38630274955577,
                         25.2,	1.36891867492719,
                         25.1,	1.35147011125939,
                         25,	1.33395705855238,
                         24.9,	1.31637951680615,
                         24.8,	1.2987374860207,
                         24.7,	1.28103096619604,
                         24.6,	1.26325995733216,
                         24.5,	1.24542445942907,
                         24.4,	1.22752447248676,
                         24.3,	1.20955999650523,
                         24.2,	1.19153103148449,
                         24.1,	1.17343757742453,
                         24,	1.15527963432535,
                         23.9,	1.13705720218696,
                         23.8,	1.11877028100935,
                         23.7,	1.10041887079252,
                         23.6,	1.08200297153648,
                         23.5,	1.06352258324122,
                         23.4,	1.04497770590674,
                         23.3,	1.02636833953305,
                         23.2,	1.00769448412014,
                         23.1,	0.988956139668015,
                         23,	0.970153306176674,
                         22.9,	0.951285983646115,
                         22.8,	0.932354172076341,
                         22.7,	0.913357871467349,
                         22.6,	0.894297081819142,
                         22.5,	0.875171803131718,
                         22.4,	0.855982035405077,
                         22.3,	0.836727778639221,
                         22.2,	0.817409032834147,
                         22.1,	0.798025797989858,
                         22,	0.778578074106351,
                         21.9,	0.759065861183628,
                         21.8,	0.739489159221689,
                         21.7,	0.719847968220533,
                         21.6,	0.700142288180161,
                         21.5,	0.680372119100572,
                         21.4,	0.660537460981766,
                         21.3,	0.640638313823745,
                         21.2,	0.620674677626507,
                         21.1,	0.600646552390052,
                         21,	0.580553938114381,
                         20.9,	0.560396834799494,
                         20.8,	0.54017524244539,
                         20.7,	0.519889161052069,
                         20.6,	0.499538590619532,
                         20.5,	0.479123531147779,
                         20.4,	0.458643982636809,
                         20.3,	0.438099945086622,
                         20.2,	0.417491418497219,
                         20.1,	0.3968184028686,
                         20,	0.376080898200764,
                         19.9,	0.355278904493713,
                         19.8,	0.334412421747444,
                         19.7,	0.313481449961958,
                         19.6,	0.292485989137257,
                         19.5,	0.271426039273339,
                         19.4,	0.250301600370204,
                         19.3,	0.229112672427853,
                         19.2,	0.207859255446285,
                         19.1,	0.186541349425502,
                         19,	0.165158954365501,
                         18.9,	0.143712070266284,
                         18.8,	0.122200697127851,
                         18.7,	0.1006248349502,
                         18.6,	0.0789844837333345,
                         18.5,	0.0572796434772513,
                         18.4,	0.0355103141819525,
                         18.3,	0.0136764958474364,
                         18.2,	-0.00822181152629609,
                         18.1,	-0.0301846079392443,
                         18,	-0.0522118933914098,
                         17.9,	-0.074303667882791,
                         17.8,	-0.0964599314133894,
                         17.7,	-0.118680683983204,
                         17.6,	-0.140965925592235,
                         17.5,	-0.163315656240483,
                         17.4,	-0.185729875927946,
                         17.3,	-0.208208584654627,
                         17.2,	-0.230751782420524,
                         17.1,	-0.253359469225637,
                         17,	-0.276031645069967,
                         16.9,	-0.298768309953513,
                         16.8,	-0.321569463876276,
                         16.7,	-0.344435106838256,
                         16.6,	-0.367365238839451,
                         16.5,	-0.390359859879864,
                         16.4,	-0.413418969959492,
                         16.3,	-0.436542569078337,
                         16.2,	-0.459730657236399,
                         16.1,	-0.482983234433677,
                         16,	-0.506300300670172,
                         15.9,	-0.529681855945882,
                         15.8,	-0.55312790026081,
                         15.7,	-0.576638433614955,
                         15.6,	-0.600213456008315,
                         15.5,	-0.623852967440892,
                         15.4,	-0.647556967912685,
                         15.3,	-0.671325457423695,
                         15.2,	-0.695158435973921,
                         15.1,	-0.719055903563364,
                         15,	-0.743017860192023,
                         14.9,	-0.767044305859899,
                         14.8,	-0.791135240566991,
                         14.7,	-0.8152906643133,
                         14.6,	-0.839510577098825,
                         14.5,	-0.863794978923567,
                         14.4,	-0.888143869787524,
                         14.3,	-0.912557249690699,
                         14.2,	-0.93703511863309,
                         14.1,	-0.961577476614698,
                         14,	-0.986184323635522,
                         13.9,	-1.01085565969556,
                         13.8,	-1.03559148479482,
                         13.7,	-1.06039179893329,
                         13.6,	-1.08525660211098,
                         13.5,	-1.11018589432789,
                         13.4,	-1.13517967558401,
                         13.3,	-1.16023794587935,
                         13.2,	-1.18536070521391,
                         13.1,	-1.21054795358768,
                         13,	-1.23579969100067,
                         12.9,	-1.26111591745287,
                         12.8,	-1.28649663294429,
                         12.7,	-1.31194183747493,
                         12.6,	-1.33745153104479,
                         12.5,	-1.36302571365386,
                         12.4,	-1.38866438530214,
                         12.3,	-1.41436754598965,
                         12.2,	-1.44013519571637,
                         12.1,	-1.46596733448231,
                         12,	-1.49186396228746,
                         11.9,	-1.51782507913183,
                         11.8,	-1.54385068501542,
                         11.7,	-1.56994077993822,
                         11.6,	-1.59609536390024,
                         11.5,	-1.62231443690147,
                         11.4,	-1.64859799894193,
                         11.3,	-1.67494605002159,
                         11.2,	-1.70135859014048,
                         11.1,	-1.72783561929858,
                         11,	-1.7543771374959,
                         10.9,	-1.78098314473243,
                         10.8,	-1.80765364100818,
                         10.7,	-1.83438862632315,
                         10.6,	-1.86118810067734,
                         10.5,	-1.88805206407074,
                         10.4,	-1.91498051650335,
                         10.3,	-1.94197345797519,
                         10.2,	-1.96903088848624,
                         10.1,	-1.9961528080365,
                         10,	-2.02333921662598,
                         9.9,	-2.05059011425468,
                         9.8,	-2.0779055009226,
                         9.7,	-2.10528537662973,
                         9.6,	-2.13272974137608,
                         9.5,	-2.16023859516165,
                         9.4,	-2.18781193798643,
                         9.3,	-2.21544976985042,
                         9.2,	-2.24315209075364,
                         9.1,	-2.27091890069607,
                         9,	-2.29875019967772,
                         8.9,	-2.32664598769858,
                         8.8,	-2.35460626475866,
                         8.7,	-2.38263103085796,
                         8.6,	-2.41072028599647,
                         8.5,	-2.4388740301742,
                         8.4,	-2.46709226339115,
                         8.3,	-2.49537498564731,
                         8.2,	-2.52372219694269,
                         8.1,	-2.55213389727729,
                         8,	-2.5806100866511,
                         7.9,	-2.60915076506413,
                         7.8,	-2.63775593251637,
                         7.7,	-2.66642558900783,
                         7.6,	-2.69515973453851,
                         7.5,	-2.72395836910841,
                         7.4,	-2.75282149271752,
                         7.3,	-2.78174910536584,
                         7.2,	-2.81074120705339,
                         7.1,	-2.83979779778015,
                         7,	-2.86891887754613,
                         6.9,	-2.89810444635132,
                         6.8,	-2.92735450419573,
                         6.7,	-2.95666905107935,
                         6.6,	-2.9860480870022,
                         6.5,	-3.01549161196426,
                         6.4,	-3.04499962596553,
                         6.3,	-3.07457212900602,
                         6.2,	-3.10420912108573,
                         6.1,	-3.13391060220466,
                         6,	-3.1636765723628,
                         5.9,	-3.19350703156016,
                         5.8,	-3.22340197979673,
                         5.7,	-3.25336141707252,
                         5.6,	-3.28338534338753,
                         5.5,	-3.31347375874175,
                         5.4,	-3.34362666313519,
                         5.3,	-3.37384405656785,
                         5.2,	-3.40412593903972
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
