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
    const float map[] = {45.1,	2.01271093616442,
                         45,	1.99769696209912,
                         44.9,	1.98279532587391,
                         44.8,	1.96800420333968,
                         44.7,	1.95332178557384,
                         44.6,	1.93874627881751,
                         44.5,	1.92427590441275,
                         44.4,	1.90990889873979,
                         44.3,	1.8956435131542,
                         44.2,	1.88147801392411,
                         44.1,	1.86741068216747,
                         44,	1.8534398137892,
                         43.9,	1.83956371941844,
                         43.8,	1.82578072434575,
                         43.7,	1.81208916846034,
                         43.6,	1.79848740618725,
                         43.5,	1.78497380642458,
                         43.4,	1.77154675248072,
                         43.3,	1.75820464201152,
                         43.2,	1.74494588695756,
                         43.1,	1.73176891348129,
                         43,	1.71867216190432,
                         42.9,	1.70565408664457,
                         42.8,	1.69271315615352,
                         42.7,	1.6798478528534,
                         42.6,	1.66705667307443,
                         42.5,	1.65433812699199,
                         42.4,	1.64169073856387,
                         42.3,	1.62911304546748,
                         42.2,	1.61660359903703,
                         42.1,	1.60416096420078,
                         42,	1.59178371941824,
                         41.9,	1.57947045661736,
                         41.8,	1.56721978113177,
                         41.7,	1.55503031163801,
                         41.6,	1.54290068009267,
                         41.5,	1.53082953166969,
                         41.4,	1.5188155246975,
                         41.3,	1.5068573305963,
                         41.2,	1.49495363381519,
                         41.1,	1.48310313176948,
                         41,	1.47130453477781,
                         40.9,	1.45955656599943,
                         40.8,	1.44785796137136,
                         40.7,	1.43620746954566,
                         40.6,	1.42460385182659,
                         40.5,	1.41304588210785,
                         40.4,	1.40153234680979,
                         40.3,	1.39006204481661,
                         40.2,	1.37863378741358,
                         40.1,	1.36724639822426,
                         40,	1.35589871314771,
                         39.9,	1.34458958029569,
                         39.8,	1.33331785992988,
                         39.7,	1.3220824243991,
                         39.6,	1.31088215807652,
                         39.5,	1.29971595729684,
                         39.4,	1.28858273029356,
                         39.3,	1.27748139713616,
                         39.2,	1.2664108896673,
                         39.1,	1.25537015144006,
                         39,	1.24435813765514,
                         38.9,	1.23337381509806,
                         38.8,	1.22241616207641,
                         38.7,	1.21148416835701,
                         38.6,	1.20057683510317,
                         38.5,	1.18969317481187,
                         38.4,	1.17883221125099,
                         38.3,	1.16799297939652,
                         38.2,	1.15717452536977,
                         38.1,	1.14637590637458,
                         38,	1.13559619063452,
                         37.9,	1.12483445733016,
                         37.8,	1.11408979653618,
                         37.7,	1.1033613091587,
                         37.6,	1.09264810687239,
                         37.5,	1.08194931205775,
                         37.4,	1.0712640577383,
                         37.3,	1.06059148751778,
                         37.2,	1.04993075551739,
                         37.1,	1.03928102631296,
                         37,	1.02864147487223,
                         36.9,	1.01801128649198,
                         36.8,	1.00738965673531,
                         36.7,	0.996775791368821,
                         36.6,	0.986168906299827,
                         36.5,	0.97556822751358,
                         36.4,	0.964972991010474,
                         36.3,	0.954382442743257,
                         36.2,	0.94379583855425,
                         36.1,	0.933212444112552,
                         36,	0.922631534851257,
                         35.9,	0.912052395904666,
                         35.8,	0.901474322045497,
                         35.7,	0.890896617622099,
                         35.6,	0.880318596495665,
                         35.5,	0.869739581977446,
                         35.4,	0.859158906765959,
                         35.3,	0.8485759128842,
                         35.2,	0.837989951616863,
                         35.1,	0.827400383447544,
                         35,	0.81680657799596,
                         34.9,	0.806207913955157,
                         34.8,	0.795603779028723,
                         34.7,	0.784993569868003,
                         34.6,	0.774376692009311,
                         34.5,	0.763752559811139,
                         34.4,	0.753120596391374,
                         34.3,	0.742480233564506,
                         34.2,	0.731830911778845,
                         34.1,	0.721172080053729,
                         34,	0.710503195916743,
                         33.9,	0.699823725340922,
                         33.8,	0.68913314268197,
                         33.7,	0.678430930615472,
                         33.6,	0.667716580074106,
                         33.5,	0.656989590184853,
                         33.4,	0.646249468206212,
                         33.3,	0.635495729465414,
                         33.2,	0.624727897295628,
                         33.1,	0.613945502973181,
                         33,	0.603148085654766,
                         32.9,	0.592335192314655,
                         32.8,	0.581506377681915,
                         32.7,	0.570661204177612,
                         32.6,	0.559799241852033,
                         32.5,	0.548920068321894,
                         32.4,	0.538023268707552,
                         32.3,	0.527108435570219,
                         32.2,	0.516175168849171,
                         32.1,	0.505223075798968,
                         32,	0.494251770926657,
                         31.9,	0.483260875928992,
                         31.8,	0.472250019629643,
                         31.7,	0.461218837916406,
                         31.6,	0.450166973678423,
                         31.5,	0.439094076743386,
                         31.4,	0.427999803814755,
                         31.3,	0.41688381840897,
                         31.2,	0.40574579079266,
                         31.1,	0.394585397919857,
                         31,	0.383402323369213,
                         30.9,	0.372196257281203,
                         30.8,	0.360966896295349,
                         30.7,	0.34971394348742,
                         30.6,	0.338437108306656,
                         30.5,	0.327136106512974,
                         30.4,	0.31581066011418,
                         30.3,	0.304460497303185,
                         30.2,	0.293085352395214,
                         30.1,	0.281684965765022,
                         30,	0.270259083784104,
                         29.9,	0.258807458757907,
                         29.8,	0.247329848863045,
                         29.7,	0.235826018084507,
                         29.6,	0.224295736152875,
                         29.5,	0.212738778481534,
                         29.4,	0.201154926103883,
                         29.3,	0.189543965610548,
                         29.2,	0.177905689086596,
                         29.1,	0.166239894048748,
                         29,	0.154546383382587,
                         28.9,	0.142824965279776,
                         28.8,	0.131075453175266,
                         28.7,	0.119297665684512,
                         28.6,	0.107491426540683,
                         28.5,	0.0956565645318761,
                         28.4,	0.0837929134383254,
                         28.3,	0.0719003119696207,
                         28.2,	0.0599786037019135,
                         28.1,	0.048027637015134,
                         28,	0.036047265030202,
                         27.9,	0.0240373455462378,
                         27.8,	0.0119977409777775,
                         27.7,	-7.16817080171612E-05,
                         27.6,	-0.0121710510541444,
                         27.5,	-0.0243004911765503,
                         27.4,	-0.0364601218269183,
                         27.3,	-0.0486500584554536,
                         27.2,	-0.0608704122736739,
                         27.1,	-0.073121290317194,
                         27,	-0.0854027955085151,
                         26.9,	-0.097715026719813,
                         26.8,	-0.110058078835723,
                         26.7,	-0.122432042816131,
                         26.6,	-0.134837005758958,
                         26.5,	-0.147273050962948,
                         26.4,	-0.159740257990459,
                         26.3,	-0.172238702730244,
                         26.2,	-0.184768457460248,
                         26.1,	-0.197329590910384,
                         26,	-0.20992216832533,
                         25.9,	-0.222546251527314,
                         25.8,	-0.235201898978897,
                         25.7,	-0.247889165845768,
                         25.6,	-0.260608104059526,
                         25.5,	-0.273358762380469,
                         25.4,	-0.286141186460383,
                         25.3,	-0.298955418905328,
                         25.2,	-0.311801499338426,
                         25.1,	-0.324679464462649,
                         25,	-0.337589348123604,
                         24.9,	-0.350531181372325,
                         24.8,	-0.363504992528058,
                         24.7,	-0.376510807241049,
                         24.6,	-0.389548648555329,
                         24.5,	-0.402618536971506,
                         24.4,	-0.415720490509551,
                         24.3,	-0.428854524771582,
                         24.2,	-0.442020653004658,
                         24.1,	-0.455218886163561,
                         24,	-0.468449232973585,
                         23.9,	-0.481711699993328,
                         23.8,	-0.49500629167747,
                         23.7,	-0.508333010439573,
                         23.6,	-0.521691856714855,
                         23.5,	-0.535082829022989,
                         23.4,	-0.548505924030885,
                         23.3,	-0.561961136615477,
                         23.2,	-0.575448459926515,
                         23.1,	-0.588967885449346,
                         23,	-0.602519403067708,
                         22.9,	-0.616103001126514,
                         22.8,	-0.629718666494638,
                         22.7,	-0.643366384627709,
                         22.6,	-0.657046139630892,
                         22.5,	-0.670757914321677,
                         22.4,	-0.68450169029267,
                         22.3,	-0.698277447974376,
                         22.2,	-0.712085166697989,
                         22.1,	-0.72592482475818,
                         22,	-0.739796399475882,
                         21.9,	-0.753699867261082,
                         21.8,	-0.767635203675602,
                         21.7,	-0.781602383495895,
                         21.6,	-0.795601380775825,
                         21.5,	-0.809632168909456,
                         21.4,	-0.823694720693845,
                         21.3,	-0.837789008391822,
                         21.2,	-0.851915003794784,
                         21.1,	-0.866072678285478,
                         21,	-0.880262002900789,
                         20.9,	-0.894482948394533,
                         20.8,	-0.908735485300235,
                         20.7,	-0.923019583993927,
                         20.6,	-0.937335214756926,
                         20.5,	-0.951682347838628,
                         20.4,	-0.966060953519296,
                         20.3,	-0.98047100217284,
                         20.2,	-0.994912464329612,
                         20.1,	-1.00938531073919,
                         20,	-1.02388951243318,
                         19.9,	-1.03842504078796,
                         19.8,	-1.05299186758753,
                         19.7,	-1.06758996508625,
                         19.6,	-1.08221930607165,
                         19.5,	-1.09687986392721,
                         19.4,	-1.11157161269515,
                         19.3,	-1.12629452713922,
                         19.2,	-1.14104858280749,
                         19.1,	-1.15583375609511,
                         19,	-1.17065002430716,
                         18.9,	-1.18549736572136,
                         18.8,	-1.20037575965091,
                         18.7,	-1.21528518650727,
                         18.6,	-1.23022562786294,
                         18.5,	-1.24519706651422,
                         18.4,	-1.26019948654407,
                         18.3,	-1.27523287338483,
                         18.2,	-1.29029721388101,
                         18.1,	-1.30539249635214,
                         18,	-1.32051871065549,
                         17.9,	-1.33567584824888,
                         17.8,	-1.3508639022535,
                         17.7,	-1.36608286751663,
                         17.6,	-1.38133274067449,
                         17.5,	-1.396613520215,
                         17.4,	-1.41192520654056,
                         17.3,	-1.42726780203087,
                         17.2,	-1.44264131110569,
                         17.1,	-1.4580457402876,
                         17,	-1.47348109826488,
                         16.9,	-1.48894739595419,
                         16.8,	-1.50444464656343,
                         16.7,	-1.51997286565449,
                         16.6,	-1.53553207120608,
                         16.5,	-1.55112228367644,
                         16.4,	-1.56674352606624,
                         16.3,	-1.58239582398124,
                         16.2,	-1.59807920569519,
                         16.1,	-1.61379370221255,
                         16,	-1.62953934733129,
                         15.9,	-1.64531617770569,
                         15.8,	-1.66112423290914,
                         15.7,	-1.67696355549688,
                         15.6,	-1.69283419106883,
                         15.5,	-1.70873618833238,
                         15.4,	-1.72466959916514,
                         15.3,	-1.74063447867775,
                         15.2,	-1.75663088527669,
                         15.1,	-1.77265888072701,
                         15,	-1.7887185302152,
                         14.9,	-1.80480990241187,
                         14.8,	-1.82093306953466,
                         14.7,	-1.83708810741091,
                         14.6,	-1.85327509554055,
                         14.5,	-1.86949411715879,
                         14.4,	-1.88574525929901,
                         14.3,	-1.90202861285546,
                         14.2,	-1.9183442726461,
                         14.1,	-1.93469233747536,
                         14,	-1.95107291019693,
                         13.9,	-1.96748609777658,
                         13.8,	-1.98393201135491,
                         13.7,	-2.00041076631015,
                         13.6,	-2.01692248232095,
                         13.5,	-2.03346728342916,
                         13.4,	-2.05004529810263,
                         13.3,	-2.06665665929799,
                         13.2,	-2.08330150452343,
                         13.1,	-2.09997997590152,
                         13,	-2.11669222023193,
                         12.9,	-2.13343838905432,
                         12.8,	-2.15021863871101,
                         12.7,	-2.16703313040986,
                         12.6,	-2.18388203028702,
                         12.5,	-2.20076550946972,
                         12.4,	-2.21768374413904,
                         12.3,	-2.23463691559275,
                         12.2,	-2.25162521030804,
                         12.1,	-2.26864882000433,
                         12,	-2.28570794170607,
                         11.9,	-2.30280277780553,
                         11.8,	-2.31993353612553,
                         11.7,	-2.33710042998232,
                         11.6,	-2.35430367824828,
                         11.5,	-2.37154350541478,
                         11.4,	-2.38882014165492,
                         11.3,	-2.40613382288631,
                         11.2,	-2.42348479083393,
                         11.1,	-2.44087329309281,
                         11,	-2.45829958319093,
                         10.9,	-2.4757639206519,
                         10.8,	-2.49326657105783,
                         10.7,	-2.51080780611208,
                         10.6,	-2.52838790370207,
                         10.5,	-2.54600714796202,
                         10.4,	-2.56366582933579,
                         10.3,	-2.58136424463965,
                         10.2,	-2.59910269712507,
                         10.1,	-2.61688149654147,
                         10,	-2.63470095919908,
                         9.9,	-2.65256140803166,
                         9.8,	-2.67046317265933,
                         9.7,	-2.68840658945134,
                         9.6,	-2.70639200158885,
                         9.5,	-2.72441975912775,
                         9.4,	-2.7424902190614,
                         9.3,	-2.76060374538346,
                         9.2,	-2.77876070915065,
                         9.1,	-2.79696148854556,
                         9,	-2.81520646893941,
                         8.9,	-2.83349604295487,
                         8.8,	-2.85183061052882,
                         8.7,	-2.87021057897514,
                         8.6,	-2.88863636304753,
                         8.5,	-2.90710838500226,
                         8.4,	-2.92562707466096,
                         8.3,	-2.94419286947343,
                         8.2,	-2.96280621458043,
                         8.1,	-2.98146756287642,
                         8,	-3.00017737507241,
                         7.9,	-3.01893611975871,
                         7.8,	-3.03774427346772,
                         7.7,	-3.05660232073673,
                         7.6,	-3.0755107541707,
                         7.5,	-3.09447007450505,
                         7.4,	-3.11348079066844,
                         7.3,	-3.13254341984557,
                         7.2,	-3.15165848753997,
                         7.1,	-3.17082652763675,
                         7,	-3.19004808246546,
                         6.9,	-3.20932370286279,
                         6.8,	-3.22865394823544,
                         6.7,	-3.24803938662284,
                         6.6,	-3.26748059475998,
                         6.5,	-3.28697815814018,
                         6.4,	-3.3065326710779,
                         6.3,	-3.32614473677147,
                         6.2,	-3.34581496736596,
                         6.1,	-3.3655439840159,
                         6,	-3.3853324169481,
                         5.9,	-3.40518090552443,
                         5.8,	-3.4250900983046,
                         5.7,	-3.44506065310895,
                         5.6,	-3.46509323708128,
                         5.5,	-3.48518852675154,
                         5.4,	-3.50534720809874,
                         5.3,	-3.52556997661362,
                         5.2,	-3.54585753736153,
                         5.1,	-3.56621060504517,
                         5,	-3.58662990406738,
                         4.9,	-3.60711616859394,
                         4.8,	-3.62767014261636,
                         4.7,	-3.64829258001464,
                         4.6,	-3.6689842446201,
                         4.5,	-3.68974591027814,
                         4.4,	-3.71057836091103,
                         4.3,	-3.73148239058069,
                         4.2,	-3.75245880355151,
                         4.1,	-3.7735084143531,
                         4,	-3.7946320478431,
                         3.9,	-3.81583053926996,
                         3.8,	-3.83710473433572,
                         3.7,	-3.85845548925882,
                         3.6,	-3.87988367083685,
                         3.5,	-3.9013901565094,
                         3.4,	-3.92297583442078,
                         3.3,	-3.94464160348283,
                         3.2,	-3.96638837343773,
                         3.1,	-3.98821706492078,
                         3,	-4.01012860952314,
                         2.9,	-4.03212394985471,
                         2.8,	-4.05420403960681,
                         2.7,	-4.07636984361506,
                         2.6,	-4.09862233792212,
                         2.5,	-4.12096250984048,
                         2.4,	-4.14339135801525,
                         2.3,	-4.16590989248696,
                         2.2,	-4.18851913475434,
                         2.1,	-4.2112201178371,
                         2,	-4.23401388633873,
                         1.9,	-4.25690149650929,
                         1.8,	-4.27988401630817,
                         1.7,	-4.30296252546691,
                         1.6,	-4.32613811555196
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
