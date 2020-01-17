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
    const float map[] = {33,	1.9138565642294,
                         32.9,	1.8986898592164,
                         32.8,	1.88348148080862,
                         32.7,	1.86823142900607,
                         32.6,	1.85293970380875,
                         32.5,	1.83760630521665,
                         32.4,	1.82223123322979,
                         32.3,	1.80681448784814,
                         32.2,	1.79135606907173,
                         32.1,	1.77585597690054,
                         32,	1.76031421133457,
                         31.9,	1.74473077237383,
                         31.8,	1.72910566001832,
                         31.7,	1.71343887426804,
                         31.6,	1.69773041512298,
                         31.5,	1.68198028258314,
                         31.4,	1.66618847664854,
                         31.3,	1.65035499731916,
                         31.2,	1.63447984459501,
                         31.1,	1.61856301847608,
                         31,	1.60260451896238,
                         30.9,	1.5866043460539,
                         30.8,	1.57056249975065,
                         30.7,	1.55447898005263,
                         30.6,	1.53835378695984,
                         30.5,	1.52218692047227,
                         30.4,	1.50597838058992,
                         30.3,	1.48972816731281,
                         30.2,	1.47343628064092,
                         30.1,	1.45710272057425,
                         30,	1.44072748711281,
                         29.9,	1.4243105802566,
                         29.8,	1.40785200000562,
                         29.7,	1.39135174635986,
                         29.6,	1.37480981931933,
                         29.5,	1.35822621888402,
                         29.4,	1.34160094505394,
                         29.3,	1.32493399782909,
                         29.2,	1.30822537720946,
                         29.1,	1.29147508319506,
                         29,	1.27468311578588,
                         28.9,	1.25784947498193,
                         28.8,	1.24097416078321,
                         28.7,	1.22405717318972,
                         28.6,	1.20709851220145,
                         28.5,	1.1900981778184,
                         28.4,	1.17305617004059,
                         28.3,	1.155972488868,
                         28.2,	1.13884713430063,
                         28.1,	1.12168010633849,
                         28,	1.10447140498158,
                         27.9,	1.0872210302299,
                         27.8,	1.06992898208344,
                         27.7,	1.05259526054221,
                         27.6,	1.0352198656062,
                         27.5,	1.01780279727542,
                         27.4,	1.00034405554987,
                         27.3,	0.982843640429538,
                         27.2,	0.965301551914437,
                         27.1,	0.947717790004562,
                         27,	0.930092354699913,
                         26.9,	0.912425246000491,
                         26.8,	0.894716463906295,
                         26.7,	0.876966008417326,
                         26.6,	0.859173879533583,
                         26.5,	0.841340077255065,
                         26.4,	0.823464601581775,
                         26.3,	0.805547452513711,
                         26.2,	0.787588630050873,
                         26.1,	0.769588134193261,
                         26,	0.751545964940876,
                         25.9,	0.733462122293717,
                         25.8,	0.715336606251784,
                         25.7,	0.697169416815078,
                         25.6,	0.678960553983598,
                         25.5,	0.660710017757344,
                         25.4,	0.642417808136317,
                         25.3,	0.624083925120515,
                         25.2,	0.605708368709941,
                         25.1,	0.587291138904592,
                         25,	0.56883223570447,
                         24.9,	0.550331659109574,
                         24.8,	0.531789409119904,
                         24.7,	0.513205485735461,
                         24.6,	0.494579888956244,
                         24.5,	0.475912618782254,
                         24.4,	0.45720367521349,
                         24.3,	0.438453058249952,
                         24.2,	0.41966076789164,
                         24.1,	0.400826804138555,
                         24,	0.381951166990695,
                         23.9,	0.363033856448063,
                         23.8,	0.344074872510656,
                         23.7,	0.325074215178476,
                         23.6,	0.306031884451522,
                         23.5,	0.286947880329795,
                         23.4,	0.267822202813294,
                         23.3,	0.248654851902019,
                         23.2,	0.22944582759597,
                         23.1,	0.210195129895148,
                         23,	0.190902758799552,
                         22.9,	0.171568714309183,
                         22.8,	0.152192996424039,
                         22.7,	0.132775605144122,
                         22.6,	0.113316540469432,
                         22.5,	0.0938158023999673,
                         22.4,	0.0742733909357297,
                         22.3,	0.0546893060767177,
                         22.2,	0.035063547822932,
                         22.1,	0.0153961161743734,
                         22,	-0.00431298886895964,
                         21.9,	-0.0240637673070664,
                         21.8,	-0.0438562191399461,
                         21.7,	-0.0636903443675995,
                         21.6,	-0.0835661429900273,
                         21.5,	-0.103483615007229,
                         21.4,	-0.123442760419204,
                         21.3,	-0.143443579225952,
                         21.2,	-0.163486071427474,
                         21.1,	-0.18357023702377,
                         21,	-0.20369607601484,
                         20.9,	-0.223863588400684,
                         20.8,	-0.2440727741813,
                         20.7,	-0.26432363335669,
                         20.6,	-0.284616165926855,
                         20.5,	-0.304950371891793,
                         20.4,	-0.325326251251505,
                         20.3,	-0.34574380400599,
                         20.2,	-0.366203030155249,
                         20.1,	-0.386703929699282,
                         20,	-0.407246502638089,
                         19.9,	-0.427830748971669,
                         19.8,	-0.448456668700023,
                         19.7,	-0.46912426182315,
                         19.6,	-0.489833528341051,
                         19.5,	-0.510584468253727,
                         19.4,	-0.531377081561175,
                         19.3,	-0.552211368263397,
                         19.2,	-0.573087328360393,
                         19.1,	-0.594004961852163,
                         19,	-0.614964268738706,
                         18.9,	-0.635965249020024,
                         18.8,	-0.657007902696114,
                         18.7,	-0.678092229766978,
                         18.6,	-0.699218230232616,
                         18.5,	-0.720385904093028,
                         18.4,	-0.741595251348214,
                         18.3,	-0.762846271998173,
                         18.2,	-0.784138966042905,
                         18.1,	-0.805473333482412,
                         18,	-0.826849374316693,
                         17.9,	-0.848267088545747,
                         17.8,	-0.869726476169574,
                         17.7,	-0.891227537188175,
                         17.6,	-0.91277027160155,
                         17.5,	-0.934354679409699,
                         17.4,	-0.955980760612621,
                         17.3,	-0.977648515210317,
                         17.2,	-0.999357943202786,
                         17.1,	-1.02110904459003,
                         17,	-1.04290181937205,
                         16.9,	-1.06473626754884,
                         16.8,	-1.0866123891204,
                         16.7,	-1.10853018408674,
                         16.6,	-1.13048965244785,
                         16.5,	-1.15249079420374,
                         16.4,	-1.1745336093544,
                         16.3,	-1.19661809789983,
                         16.2,	-1.21874425984004,
                         16.1,	-1.24091209517502,
                         16,	-1.26312160390477,
                         15.9,	-1.2853727860293,
                         15.8,	-1.3076656415486,
                         15.7,	-1.33000017046267,
                         15.6,	-1.35237637277152,
                         15.5,	-1.37479424847515,
                         15.4,	-1.39725379757354,
                         15.3,	-1.41975502006671,
                         15.2,	-1.44229791595465,
                         15.1,	-1.46488248523737,
                         15,	-1.48750872791486,
                         14.9,	-1.51017664398713,
                         14.8,	-1.53288623345417,
                         14.7,	-1.55563749631598,
                         14.6,	-1.57843043257256,
                         14.5,	-1.60126504222392,
                         14.4,	-1.62414132527005,
                         14.3,	-1.64705928171096,
                         14.2,	-1.67001891154664,
                         14.1,	-1.6930202147771,
                         14,	-1.71606319140232,
                         13.9,	-1.73914784142232,
                         13.8,	-1.7622741648371,
                         13.7,	-1.78544216164665,
                         13.6,	-1.80865183185097,
                         13.5,	-1.83190317545007,
                         13.4,	-1.85519619244394,
                         13.3,	-1.87853088283258,
                         13.2,	-1.901907246616,
                         13.1,	-1.92532528379419,
                         13,	-1.94878499436715,
                         12.9,	-1.97228637833489,
                         12.8,	-1.9958294356974,
                         12.7,	-2.01941416645469,
                         12.6,	-2.04304057060675,
                         12.5,	-2.06670864815358,
                         12.4,	-2.09041839909519,
                         12.3,	-2.11416982343157,
                         12.2,	-2.13796292116272,
                         12.1,	-2.16179769228865,
                         12,	-2.18567413680935,
                         11.9,	-2.20959225472483,
                         11.8,	-2.23355204603507,
                         11.7,	-2.2575535107401,
                         11.6,	-2.28159664883989,
                         11.5,	-2.30568146033446,
                         11.4,	-2.32980794522381,
                         11.3,	-2.35397610350792,
                         11.2,	-2.37818593518681,
                         11.1,	-2.40243744026048,
                         11,	-2.42673061872892,
                         10.9,	-2.45106547059213,
                         10.8,	-2.47544199585011,
                         10.7,	-2.49986019450287,
                         10.6,	-2.52432006655041,
                         10.5,	-2.54882161199271,
                         10.4,	-2.57336483082979,
                         10.3,	-2.59794972306165,
                         10.2,	-2.62257628868827,
                         10.1,	-2.64724452770968,
                         10,	-2.67195444012585,
                         9.9,	-2.6967060259368,
                         9.8,	-2.72149928514252,
                         9.7,	-2.74633421774302,
                         9.6,	-2.77121082373829,
                         9.5,	-2.79612910312833,
                         9.4,	-2.82108905591315,
                         9.3,	-2.84609068209274,
                         9.2,	-2.8711339816671,
                         9.1,	-2.89621895463624,
                         9,	-2.92134560100015,
                         8.9,	-2.94651392075884,
                         8.8,	-2.9717239139123,
                         8.7,	-2.99697558046053,
                         8.6,	-3.02226892040354,
                         8.5,	-3.04760393374132,
                         8.4,	-3.07298062047387,
                         8.3,	-3.0983989806012,
                         8.2,	-3.1238590141233,
                         8.1,	-3.14936072104018,
                         8,	-3.17490410135183,
                         7.9,	-3.20048915505825,
                         7.8,	-3.22611588215945,
                         7.7,	-3.25178428265542,
                         7.6,	-3.27749435654616,
                         7.5,	-3.30324610383168,
                         7.4,	-3.32903952451197,
                         7.3,	-3.35487461858703,
                         7.2,	-3.38075138605687,
                         7.1,	-3.40666982692148,
                         7,	-3.43262994118087,
                         6.9,	-3.45863172883503,
                         6.8,	-3.48467518988396,
                         6.7,	-3.51076032432767,
                         6.6,	-3.53688713216615,
                         6.5,	-3.5630556133994,
                         6.4,	-3.58926576802743,
                         6.3,	-3.61551759605023,
                         6.2,	-3.64181109746781,
                         6.1,	-3.66814627228015,
                         6,	-3.69452312048728
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
