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
    const float map[] = {43.3,	0.794508851141795,
                         43.2,	0.788161365638901,
                         43.1,	0.781719747764988,
                         43,	0.775184929886432,
                         42.9,	0.768557839528926,
                         42.8,	0.761839399377481,
                         42.7,	0.755030527276422,
                         42.6,	0.748132136229393,
                         42.5,	0.741145134399353,
                         42.4,	0.734070425108575,
                         42.3,	0.726908906838655,
                         42.2,	0.719661473230498,
                         42.1,	0.712329013084332,
                         42,	0.704912410359696,
                         41.9,	0.697412544175449,
                         41.8,	0.689830288809766,
                         41.7,	0.682166513700137,
                         41.6,	0.674422083443369,
                         41.5,	0.666597857795586,
                         41.4,	0.658694691672228,
                         41.3,	0.650713435148052,
                         41.2,	0.642654933457131,
                         41.1,	0.634520026992855,
                         41,	0.626309551307929,
                         40.9,	0.618024337114376,
                         40.8,	0.609665210283535,
                         40.7,	0.601232991846061,
                         40.6,	0.592728497991928,
                         40.5,	0.584152540070421,
                         40.4,	0.575505924590146,
                         40.3,	0.566789453219025,
                         40.2,	0.558003922784295,
                         40.1,	0.549150125272511,
                         40,	0.540228847829542,
                         39.9,	0.531240872760577,
                         39.8,	0.522186977530117,
                         39.7,	0.513067934761985,
                         39.6,	0.503884512239315,
                         39.5,	0.494637472904561,
                         39.4,	0.485327574859492,
                         39.3,	0.475955571365194,
                         39.2,	0.466522210842069,
                         39.1,	0.457028236869837,
                         39,	0.447474388187532,
                         38.9,	0.437861398693504,
                         38.8,	0.428189997445424,
                         38.7,	0.418460908660275,
                         38.6,	0.40867485171436,
                         38.5,	0.398832541143293,
                         38.4,	0.38893468664201,
                         38.3,	0.378981993064762,
                         38.2,	0.368975160425114,
                         38.1,	0.358914883895952,
                         38,	0.348801853809474,
                         37.9,	0.338636755657195,
                         37.8,	0.328420270089949,
                         37.7,	0.318153072917886,
                         37.6,	0.307835835110471,
                         37.5,	0.297469222796485,
                         37.4,	0.287053897264028,
                         37.3,	0.276590514960513,
                         37.2,	0.266079727492674,
                         37.1,	0.255522181626557,
                         37,	0.244918519287528,
                         36.9,	0.234269377560266,
                         36.8,	0.223575388688769,
                         36.7,	0.21283718007635,
                         36.6,	0.202055374285641,
                         36.5,	0.191230589038588,
                         36.4,	0.180363437216452,
                         36.3,	0.169454526859815,
                         36.2,	0.158504461168573,
                         36.1,	0.147513838501936,
                         36,	0.136483252378437,
                         35.9,	0.125413291475917,
                         35.8,	0.11430453963154,
                         35.7,	0.103157575841785,
                         35.6,	0.0919729742624454,
                         35.5,	0.0807513042086335,
                         35.4,	0.0694931301547759,
                         35.3,	0.0581990117346172,
                         35.2,	0.0468695037412178,
                         35.1,	0.0355051561269553,
                         35,	0.0241065140035236,
                         34.9,	0.0126741176419312,
                         34.8,	0.00120850247250507,
                         34.7,	-0.0102898009151115,
                         34.6,	-0.0218202667719593,
                         34.5,	-0.0333823741897618,
                         34.4,	-0.0449756071009283,
                         34.3,	-0.0565994542785495,
                         34.2,	-0.0682534093363999,
                         34.1,	-0.0799369707289379,
                         34,	-0.0916496417513047,
                         33.9,	-0.103390930539328,
                         33.8,	-0.115160350069514,
                         33.7,	-0.126957418159058,
                         33.6,	-0.138781657465834,
                         33.5,	-0.150632595488401,
                         33.4,	-0.162509764566005,
                         33.3,	-0.174412701878571,
                         33.2,	-0.186340949446709,
                         33.1,	-0.198294054131713,
                         33,	-0.21027156763556,
                         32.9,	-0.222273046500912,
                         32.8,	-0.234298052111113,
                         32.7,	-0.246346150690191,
                         32.6,	-0.258416913302857,
                         32.5,	-0.270509915854505,
                         32.4,	-0.282624739091216,
                         32.3,	-0.29476096859975,
                         32.2,	-0.306918194807555,
                         32.1,	-0.319096012982758,
                         32,	-0.331294023234172,
                         31.9,	-0.343511830511294,
                         31.8,	-0.355749044604304,
                         31.7,	-0.368005280144065,
                         31.6,	-0.380280156602124,
                         31.5,	-0.392573298290711,
                         31.4,	-0.40488433436274,
                         31.3,	-0.41721289881181,
                         31.2,	-0.429558630472201,
                         31.1,	-0.441921173018877,
                         31,	-0.454300174967486,
                         30.9,	-0.466695289674361,
                         30.8,	-0.479106175336517,
                         30.7,	-0.491532494991653,
                         30.6,	-0.50397391651815,
                         30.5,	-0.516430112635075,
                         30.4,	-0.528900760902177,
                         30.3,	-0.541385543719889,
                         30.2,	-0.553884148329328,
                         30.1,	-0.566396266812293,
                         30,	-0.578921596091268,
                         29.9,	-0.59145983792942,
                         29.8,	-0.604010698930601,
                         29.7,	-0.616573890539344,
                         29.6,	-0.629149129040867,
                         29.5,	-0.64173613556107,
                         29.4,	-0.65433463606654,
                         29.3,	-0.666944361364544,
                         29.2,	-0.679565047103035,
                         29.1,	-0.692196433770647,
                         29,	-0.704838266696699,
                         28.9,	-0.717490296051196,
                         28.8,	-0.730152276844822,
                         28.7,	-0.742823968928946,
                         28.6,	-0.755505136995623,
                         28.5,	-0.768195550577588,
                         28.4,	-0.780894984048263,
                         28.3,	-0.793603216621751,
                         28.2,	-0.806320032352839,
                         28.1,	-0.819045220136998,
                         28,	-0.831778573710381,
                         27.9,	-0.844519891649829,
                         27.8,	-0.857268977372862,
                         27.7,	-0.870025639137685,
                         27.6,	-0.882789690043187,
                         27.5,	-0.895560948028938,
                         27.4,	-0.908339235875197,
                         27.3,	-0.921124381202901,
                         27.2,	-0.933916216473673,
                         27.1,	-0.94671457898982,
                         27,	-0.959519310894331,
                         26.9,	-0.97233025917088,
                         26.8,	-0.985147275643824,
                         26.7,	-0.997970216978203,
                         26.6,	-1.01079894467974,
                         26.5,	-1.02363332509485,
                         26.4,	-1.03647322941061,
                         26.3,	-1.0493185336548,
                         26.2,	-1.06216911869589,
                         26.1,	-1.07502487024301,
                         26,	-1.08788567884598,
                         25.9,	-1.10075143989533,
                         25.8,	-1.11362205362223,
                         25.7,	-1.12649742509856,
                         25.6,	-1.13937746423689,
                         25.5,	-1.15226208579046,
                         25.4,	-1.16515120935319,
                         25.3,	-1.17804475935969,
                         25.2,	-1.19094266508526,
                         25.1,	-1.20384486064587,
                         25,	-1.21675128499819,
                         24.9,	-1.22966188193956,
                         24.8,	-1.24257660010801,
                         24.7,	-1.25549539298224,
                         24.6,	-1.26841821888166,
                         24.5,	-1.28134504096633,
                         24.4,	-1.29427582723703,
                         24.3,	-1.3072105505352,
                         24.2,	-1.32014918854297,
                         24.1,	-1.33309172378315,
                         24,	-1.34603814361923,
                         23.9,	-1.3589884402554,
                         23.8,	-1.37194261073652,
                         23.7,	-1.38490065694813,
                         23.6,	-1.39786258561647,
                         23.5,	-1.41082840830845,
                         23.4,	-1.42379814143167,
                         23.3,	-1.43677180623441,
                         23.2,	-1.44974942880563,
                         23.1,	-1.46273104007497,
                         23,	-1.47571667581278,
                         22.9,	-1.48870637663007,
                         22.8,	-1.50170018797854,
                         22.7,	-1.51469816015056,
                         22.6,	-1.5277003482792,
                         22.5,	-1.54070681233822,
                         22.4,	-1.55371761714204,
                         22.3,	-1.56673283234578,
                         22.2,	-1.57975253244525,
                         22.1,	-1.59277679677692,
                         22,	-1.60580570951796,
                         21.9,	-1.61883935968623,
                         21.8,	-1.63187784114025,
                         21.7,	-1.64492125257924,
                         21.6,	-1.65796969754311,
                         21.5,	-1.67102328441244,
                         21.4,	-1.6840821264085,
                         21.3,	-1.69714634159323,
                         21.2,	-1.71021605286928,
                         21.1,	-1.72329138797997,
                         21,	-1.73637247950929,
                         20.9,	-1.74945946488193,
                         20.8,	-1.76255248636326,
                         20.7,	-1.77565169105934,
                         20.6,	-1.78875723091689,
                         20.5,	-1.80186926272335,
                         20.4,	-1.81498794810681,
                         20.3,	-1.82811345353607,
                         20.2,	-1.84124595032058,
                         20.1,	-1.85438561461051,
                         20,	-1.8675326273967,
                         19.9,	-1.88068717451066,
                         19.8,	-1.8938494466246,
                         19.7,	-1.90701963925141,
                         19.6,	-1.92019795274466,
                         19.5,	-1.93338459229861,
                         19.4,	-1.94657976794819,
                         19.3,	-1.95978369456902,
                         19.2,	-1.97299659187743,
                         19.1,	-1.98621868443038,
                         19,	-1.99945020162556,
                         18.9,	-2.01269137770132,
                         18.8,	-2.02594245173671,
                         18.7,	-2.03920366765145,
                         18.6,	-2.05247527420594,
                         18.5,	-2.06575752500127,
                         18.4,	-2.07905067847922,
                         18.3,	-2.09235499792226,
                         18.2,	-2.10567075145351,
                         18.1,	-2.1189982120368,
                         18,	-2.13233765747665,
                         17.9,	-2.14568937041824,
                         17.8,	-2.15905363834746,
                         17.7,	-2.17243075359085,
                         17.6,	-2.18582101331567,
                         17.5,	-2.19922471952983,
                         17.4,	-2.21264217908195,
                         17.3,	-2.22607370366133,
                         17.2,	-2.23951960979794,
                         17.1,	-2.25298021886243,
                         17,	-2.26645585706616,
                         16.9,	-2.27994685546115,
                         16.8,	-2.29345354994011,
                         16.7,	-2.30697628123644,
                         16.6,	-2.32051539492421,
                         16.5,	-2.33407124141819,
                         16.4,	-2.34764417597382,
                         16.3,	-2.36123455868724,
                         16.2,	-2.37484275449524,
                         16.1,	-2.38846913317534,
                         16,	-2.4021140693457,
                         15.9,	-2.4157779424652,
                         15.8,	-2.42946113683337,
                         15.7,	-2.44316404159045,
                         15.6,	-2.45688705071735,
                         15.5,	-2.47063056303568,
                         15.4,	-2.4843949822077,
                         15.3,	-2.49818071673638,
                         15.2,	-2.51198817996538,
                         15.1,	-2.52581779007902,
                         15,	-2.53966997010231,
                         14.9,	-2.55354514790096,
                         14.8,	-2.56744375618135,
                         14.7,	-2.58136623249055,
                         14.6,	-2.59531301921629,
                         14.5,	-2.60928456358702,
                         14.4,	-2.62328131767185,
                         14.3,	-2.63730373838059,
                         14.2,	-2.6513522874637,
                         14.1,	-2.66542743151237,
                         14,	-2.67952964195843,
                         13.9,	-2.69365939507444,
                         13.8,	-2.70781717197359,
                         13.7,	-2.72200345860979,
                         13.6,	-2.73621874577764,
                         13.5,	-2.75046352911239,
                         13.4,	-2.76473830908999,
                         13.3,	-2.77904359102709,
                         13.2,	-2.793379885081,
                         13.1,	-2.80774770624972,
                         13,	-2.82214757437193,
                         12.9,	-2.83658001412702,
                         12.8,	-2.85104555503503,
                         12.7,	-2.86554473145669,
                         12.6,	-2.88007808259343,
                         12.5,	-2.89464615248735,
                         12.4,	-2.90924949002123,
                         12.3,	-2.92388864891855,
                         12.2,	-2.93856418774347,
                         12.1,	-2.95327666990081,
                         12,	-2.9680266636361,
                         11.9,	-2.98281474203555,
                         11.8,	-2.99764148302603,
                         11.7,	-3.01250746937514,
                         11.6,	-3.02741328869111,
                         11.5,	-3.04235953342289,
                         11.4,	-3.0573468008601,
                         11.3,	-3.07237569313305,
                         11.2,	-3.08744681721273,
                         11.1,	-3.1025607849108,
                         11,	-3.11771821287963,
                         10.9,	-3.13291972261225,
                         10.8,	-3.1481659404424,
                         10.7,	-3.16345749754447,
                         10.6,	-3.17879502993356,
                         10.5,	-3.19417917846543,
                         10.4,	-3.20961058883656,
                         10.3,	-3.22508991158408,
                         10.2,	-3.24061780208581,
                         10.1,	-3.25619492056027,
                         10,	-3.27182193206664,
                         9.9,	-3.28749950650481,
                         9.8,	-3.30322831861532,
                         9.7,	-3.31900904797942,
                         9.6,	-3.33484237901905,
                         9.5,	-3.3507290009968,
                         9.4,	-3.36666960801598,
                         9.3,	-3.38266489902055,
                         9.2,	-3.39871557779518,
                         9.1,	-3.41482235296521,
                         9,	-3.43098593799668,
                         8.9,	-3.44720705119628,
                         8.8,	-3.46348641571142,
                         8.7,	-3.47982475953017,
                         8.6,	-3.4962228154813,
                         8.5,	-3.51268132123424,
                         8.4,	-3.52920101929914,
                         8.3,	-3.54578265702679,
                         8.2,	-3.5624269866087,
                         8.1,	-3.57913476507704,
                         8,	-3.59590675430469,
                         7.9,	-3.61274372100517,
                         7.8,	-3.62964643673274,
                         7.7,	-3.64661567788228,
                         7.6,	-3.66365222568942,
                         7.5,	-3.68075686623042,
                         7.4,	-3.69793039042224,
                         7.3,	-3.71517359402255,
                         7.2,	-3.73248727762966,
                         7.1,	-3.74987224668259,
                         7,	-3.76732931146105,
                         6.9,	-3.7848592870854,
                         6.8,	-3.80246299351673,
                         6.7,	-3.82014125555677,
                         6.6,	-3.83789490284795,
                         6.5,	-3.85572476987341,
                         6.4,	-3.87363169595692,
                         6.3,	-3.89161652526299,
                         6.2,	-3.90968010679676,
                         6.1,	-3.9278232944041,
                         6,	-3.94604694677154,
                         5.9,	-3.96435192742629,
                         5.8,	-3.98273910473626,
                         5.7,	-4.00120935191003,
                         5.6,	-4.01976354699686,
                         5.5,	-4.03840257288671,
                         5.4,	-4.05712731731022,
                         5.3,	-4.07593867283869,
                         5.2,	-4.09483753688414,
                         5.1,	-4.11382481169925
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
