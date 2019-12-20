#include "automator.h"
#include <math.h>

Automator::Automator(QObject *parent) : QObject(parent)
{
    m_working = false;
    m_enabled = false;
    m_mcConnected = false;
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
        float compensated = compensate(m_lastdz);
        if (compensated > 10) {
            m_message = "No entry in comp table";
            emit messageChanged();
        } else {
            float targetB = m_mcs_b - compensated;
            QString correction = QString("G90 G0 B%1\n").arg(targetB);
            m_message = correction;
            if (m_autosendB) {
                emit sendToMC(correction);
                emit sendToMC("M24\n");
            }
        }
    }
}

float Automator::compensate(float dz) const
{
    //                   CAM     ERROR
    const float map[] = {-16.4,	-0.802602002722413,
                         -16.3,	-0.79696950431019,
                         -16.2,	-0.791344519761904,
                         -16.1,	-0.785727049077553,
                         -16,	-0.780117092257137,
                         -15.9,	-0.774514649300657,
                         -15.8,	-0.768919720208112,
                         -15.7,	-0.763332304979503,
                         -15.6,	-0.75775240361483,
                         -15.5,	-0.752180016114092,
                         -15.4,	-0.746615142477289,
                         -15.3,	-0.741057782704422,
                         -15.2,	-0.735507936795491,
                         -15.1,	-0.729965604750495,
                         -15,	-0.724430786569435,
                         -14.9,	-0.71890348225231,
                         -14.8,	-0.713383691799121,
                         -14.7,	-0.707871415209867,
                         -14.6,	-0.702366652484549,
                         -14.5,	-0.696869403623166,
                         -14.4,	-0.691379668625719,
                         -14.3,	-0.685897447492208,
                         -14.2,	-0.680422740222632,
                         -14.1,	-0.674955546816991,
                         -14,	-0.669495867275286,
                         -13.9,	-0.664043701597517,
                         -13.8,	-0.658599049783683,
                         -13.7,	-0.653161911833784,
                         -13.6,	-0.647732287747821,
                         -13.5,	-0.642310177525794,
                         -13.4,	-0.636895581167702,
                         -13.3,	-0.631488498673546,
                         -13.2,	-0.626088930043325,
                         -13.1,	-0.62069687527704,
                         -13,	-0.61531233437469,
                         -12.9,	-0.609935307336276,
                         -12.8,	-0.604565794161798,
                         -12.7,	-0.599203794851255,
                         -12.6,	-0.593849309404647,
                         -12.5,	-0.588502337821975,
                         -12.4,	-0.583162880103239,
                         -12.3,	-0.577830936248438,
                         -12.2,	-0.572506506257572,
                         -12.1,	-0.567189590130642,
                         -12,	-0.561880187867648,
                         -11.9,	-0.556578299468589,
                         -11.8,	-0.551283924933466,
                         -11.7,	-0.545997064262278,
                         -11.6,	-0.540717717455026,
                         -11.5,	-0.53544588451171,
                         -11.4,	-0.530181565432328,
                         -11.3,	-0.524924760216883,
                         -11.2,	-0.519675468865373,
                         -11.1,	-0.514433691377798,
                         -11,	-0.509199427754159,
                         -10.9,	-0.503972677994456,
                         -10.8,	-0.498753442098688,
                         -10.7,	-0.493541720066855,
                         -10.6,	-0.488337511898958,
                         -10.5,	-0.483140817594997,
                         -10.4,	-0.477951637154971,
                         -10.3,	-0.472769970578881,
                         -10.2,	-0.467595817866726,
                         -10.1,	-0.462429179018507,
                         -10,	-0.457270054034223,
                         -9.9,	-0.452118442913875,
                         -9.8,	-0.446974345657462,
                         -9.7,	-0.441837762264985,
                         -9.6,	-0.436708692736444,
                         -9.5,	-0.431587137071838,
                         -9.4,	-0.426473095271167,
                         -9.3,	-0.421366567334432,
                         -9.2,	-0.416267553261633,
                         -9.1,	-0.411176053052769,
                         -9,	-0.40609206670784,
                         -8.9,	-0.401015594226848,
                         -8.8,	-0.39594663560979,
                         -8.7,	-0.390885190856669,
                         -8.6,	-0.385831259967482,
                         -8.5,	-0.380784842942232,
                         -8.4,	-0.375745939780916,
                         -8.3,	-0.370714550483537,
                         -8.2,	-0.365690675050093,
                         -8.1,	-0.360674313480584,
                         -8,	-0.355665465775011,
                         -7.9,	-0.350664131933373,
                         -7.8,	-0.345670311955672,
                         -7.7,	-0.340684005841905,
                         -7.6,	-0.335705213592074,
                         -7.5,	-0.330733935206179,
                         -7.4,	-0.325770170684219,
                         -7.3,	-0.320813920026195,
                         -7.2,	-0.315865183232106,
                         -7.1,	-0.310923960301953,
                         -7,	-0.305990251235735,
                         -6.9,	-0.301064056033453,
                         -6.8,	-0.296145374695106,
                         -6.7,	-0.291234207220695,
                         -6.6,	-0.286330553610219,
                         -6.5,	-0.281434413863679,
                         -6.4,	-0.276545787981075,
                         -6.3,	-0.271664675962406,
                         -6.2,	-0.266791077807672,
                         -6.1,	-0.261924993516874,
                         -6,	-0.257066423090012,
                         -5.9,	-0.252215366527085,
                         -5.8,	-0.247371823828094,
                         -5.7,	-0.242535794993038,
                         -5.6,	-0.237707280021918,
                         -5.5,	-0.232886278914733,
                         -5.4,	-0.228072791671484,
                         -5.3,	-0.22326681829217,
                         -5.2,	-0.218468358776792,
                         -5.1,	-0.213677413125349,
                         -5,	-0.208893981337842,
                         -4.9,	-0.204118063414271,
                         -4.8,	-0.199349659354634,
                         -4.7,	-0.194588769158934,
                         -4.6,	-0.189835392827169,
                         -4.5,	-0.18508953035934,
                         -4.4,	-0.180351181755446,
                         -4.3,	-0.175620347015487,
                         -4.2,	-0.170897026139464,
                         -4.1,	-0.166181219127377,
                         -4,	-0.161472925979225,
                         -3.9,	-0.156772146695009,
                         -3.8,	-0.152078881274729,
                         -3.7,	-0.147393129718383,
                         -3.6,	-0.142714892025974,
                         -3.5,	-0.1380441681975,
                         -3.4,	-0.133380958232961,
                         -3.3,	-0.128725262132358,
                         -3.2,	-0.124077079895691,
                         -3.1,	-0.119436411522959,
                         -3,	-0.114803257014162,
                         -2.9,	-0.110177616369301,
                         -2.8,	-0.105559489588376,
                         -2.7,	-0.100948876671386,
                         -2.6,	-0.0963457776183316,
                         -2.5,	-0.0917501924292129,
                         -2.4,	-0.0871621211040297,
                         -2.3,	-0.0825815636427819,
                         -2.2,	-0.0780085200454697,
                         -2.1,	-0.0734429903120931,
                         -2,	-0.068884974442652,
                         -1.9,	-0.0643344724371464,
                         -1.8,	-0.0597914842955763,
                         -1.7,	-0.0552560100179418,
                         -1.6,	-0.0507280496042428,
                         -1.5,	-0.0462076030544794,
                         -1.4,	-0.0416946703686515,
                         -1.3,	-0.0371892515467591,
                         -1.2,	-0.0326913465888022,
                         -1.1,	-0.0282009554947809,
                         -1,	-0.0237180782646951,
                         -0.9,	-0.0192427148985448,
                         -0.800000000000001,	-0.0147748653963301,
                         -0.699999999999999,	-0.0103145297580509,
                         -0.6,	-0.00586170798370722,
                         -0.5,	-0.00141640007329909,
                         -0.4,	0.00302139397317349,
                         -0.300000000000001,	0.00745167415571055,
                         -0.199999999999999,	0.0118744404743121,
                         -0.100000000000001,	0.016289692928978,
                         0,	0.0206974315197086,
                         0.100000000000001,	0.0250976562465036,
                         0.199999999999999,	0.0294903671093629,
                         0.300000000000001,	0.0338755641082868,
                         0.399999999999999,	0.0382532472432751,
                         0.5,	0.042623416514328,
                         0.600000000000001,	0.0469860719214453,
                         0.699999999999999,	0.051341213464627,
                         0.800000000000001,	0.0556888411438732,
                         0.899999999999999,	0.0600289549591838,
                         1,	0.064361554910559,
                         1.1,	0.0686866409979987,
                         1.2,	0.0730042132215027,
                         1.3,	0.0773142715810713,
                         1.4,	0.0816168160767042,
                         1.5,	0.0859118467084018,
                         1.6,	0.0901993634761638,
                         1.7,	0.0944793663799902,
                         1.8,	0.0987518554198811,
                         1.9,	0.103016830595836,
                         2,	0.107274291907856,
                         2.1,	0.111524239355941,
                         2.2,	0.115766672940089,
                         2.3,	0.120001592660303,
                         2.4,	0.12422899851658,
                         2.5,	0.128448890508922,
                         2.6,	0.132661268637329,
                         2.7,	0.1368661329018,
                         2.8,	0.141063483302336,
                         2.9,	0.145253319838936,
                         3,	0.1494356425116,
                         3.1,	0.153610451320329,
                         3.2,	0.157777746265123,
                         3.3,	0.161937527345981,
                         3.4,	0.166089794562903,
                         3.5,	0.17023454791589,
                         3.6,	0.174371787404941,
                         3.7,	0.178501513030057,
                         3.8,	0.182623724791237,
                         3.9,	0.186738422688482,
                         4,	0.190845606721791,
                         4.1,	0.194945276891165,
                         4.2,	0.199037433196603,
                         4.3,	0.203122075638105,
                         4.4,	0.207199204215672,
                         4.5,	0.211268818929304,
                         4.6,	0.215330919779,
                         4.7,	0.21938550676476,
                         4.8,	0.223432579886585,
                         4.9,	0.227472139144475,
                         5,	0.231504184538429,
                         5.1,	0.235528716068447,
                         5.2,	0.23954573373453,
                         5.3,	0.243555237536677,
                         5.4,	0.247557227474889,
                         5.5,	0.251551703549165,
                         5.6,	0.255538665759506,
                         5.7,	0.259518114105911,
                         5.8,	0.26349004858838,
                         5.9,	0.267454469206914,
                         6,	0.271411375961513,
                         6.1,	0.275360768852176,
                         6.2,	0.279302647878903,
                         6.3,	0.283237013041695,
                         6.4,	0.287163864340552,
                         6.5,	0.291083201775473,
                         6.6,	0.294995025346458,
                         6.7,	0.298899335053508,
                         6.8,	0.302796130896622,
                         6.9,	0.306685412875801,
                         7,	0.310567180991044,
                         7.1,	0.314441435242352,
                         7.2,	0.318308175629724,
                         7.3,	0.322167402153161,
                         7.4,	0.326019114812662,
                         7.5,	0.329863313608227,
                         7.6,	0.333699998539857,
                         7.7,	0.337529169607552,
                         7.8,	0.341350826811311,
                         7.9,	0.345164970151134,
                         8,	0.348971599627022,
                         8.1,	0.352770715238974,
                         8.2,	0.356562316986991,
                         8.3,	0.360346404871072,
                         8.4,	0.364122978891218,
                         8.5,	0.367892039047428,
                         8.6,	0.371653585339703,
                         8.7,	0.375407617768042,
                         8.8,	0.379154136332446,
                         8.9,	0.382893141032914,
                         9,	0.386624631869447,
                         9.1,	0.390348608842044,
                         9.2,	0.394065071950705,
                         9.3,	0.397774021195431,
                         9.4,	0.401475456576222,
                         9.5,	0.405169378093076,
                         9.6,	0.408855785745996,
                         9.7,	0.41253467953498,
                         9.8,	0.416206059460028,
                         9.9,	0.419869925521141,
                         10,	0.423526277718318,
                         10.1,	0.42717511605156,
                         10.2,	0.430816440520866,
                         10.3,	0.434450251126237,
                         10.4,	0.438076547867672,
                         10.5,	0.441695330745171,
                         10.6,	0.445306599758735,
                         10.7,	0.448910354908364,
                         10.8,	0.452506596194057,
                         10.9,	0.456095323615814,
                         11,	0.459676537173636,
                         11.1,	0.463250236867523,
                         11.2,	0.466816422697474,
                         11.3,	0.470375094663489,
                         11.4,	0.473926252765569,
                         11.5,	0.477469897003713,
                         11.6,	0.481006027377922,
                         11.7,	0.484534643888195,
                         11.8,	0.488055746534533,
                         11.9,	0.491569335316935,
                         12,	0.495075410235401,
                         12.1,	0.498573971289932,
                         12.2,	0.502065018480528,
                         12.3,	0.505548551807188,
                         12.4,	0.509024571269912,
                         12.5,	0.512493076868701,
                         12.6,	0.515954068603555,
                         12.7,	0.519407546474473,
                         12.8,	0.522853510481455,
                         12.9,	0.526291960624502,
                         13,	0.529722896903613,
                         13.1,	0.533146319318789,
                         13.2,	0.536562227870029,
                         13.3,	0.539970622557334,
                         13.4,	0.543371503380703,
                         13.5,	0.546764870340137,
                         13.6,	0.550150723435635,
                         13.7,	0.553529062667197,
                         13.8,	0.556899888034824,
                         13.9,	0.560263199538516,
                         14,	0.563618997178272,
                         14.1,	0.566967280954092,
                         14.2,	0.570308050865977,
                         14.3,	0.573641306913926,
                         14.4,	0.57696704909794,
                         14.5,	0.580285277418018,
                         14.6,	0.583595991874161,
                         14.7,	0.586899192466368,
                         14.8,	0.59019487919464,
                         14.9,	0.593483052058976,
                         15,	0.596763711059377,
                         15.1,	0.600036856195842,
                         15.2,	0.603302487468372,
                         15.3,	0.606560604876966,
                         15.4,	0.609811208421624,
                         15.5,	0.613054298102347,
                         15.6,	0.616289873919135,
                         15.7,	0.619517935871986,
                         15.8,	0.622738483960903,
                         15.9,	0.625951518185884,
                         16,	0.629157038546929,
                         16.1,	0.632355045044039,
                         16.2,	0.635545537677213,
                         16.3,	0.638728516446452,
                         16.4,	0.641903981351755,
                         16.5,	0.645071932393123,
                         16.6,	0.648232369570555,
                         16.7,	0.651385292884051,
                         16.8,	0.654530702333612,
                         16.9,	0.657668597919238,
                         17,	0.660798979640928,
                         17.1,	0.663921847498682,
                         17.2,	0.667037201492501,
                         17.3,	0.670145041622385,
                         17.4,	0.673245367888333,
                         17.5,	0.676338180290345,
                         17.6,	0.679423478828422,
                         17.7,	0.682501263502563,
                         17.8,	0.685571534312769,
                         17.9,	0.688634291259039,
                         18,	0.691689534341374,
                         18.1,	0.694737263559773,
                         18.2,	0.697777478914236,
                         18.3,	0.700810180404764,
                         18.4,	0.703835368031357,
                         18.5,	0.706853041794014,
                         18.6,	0.709863201692735,
                         18.7,	0.712865847727521,
                         18.8,	0.715860979898372,
                         18.9,	0.718848598205287,
                         19,	0.721828702648266,
                         19.1,	0.72480129322731,
                         19.2,	0.727766369942418,
                         19.3,	0.730723932793591,
                         19.4,	0.733673981780828,
                         19.5,	0.73661651690413,
                         19.6,	0.739551538163496,
                         19.7,	0.742479045558926,
                         19.8,	0.745399039090422,
                         19.9,	0.748311518757981,
                         20,	0.751216484561605,
                         20.1,	0.754113936501294,
                         20.2,	0.757003874577047,
                         20.3,	0.759886298788864,
                         20.4,	0.762761209136746,
                         20.5,	0.765628605620692,
                         20.6,	0.768488488240703,
                         20.7,	0.771340856996778,
                         20.8,	0.774185711888918,
                         20.9,	0.777023052917122,
                         21,	0.779852880081391,
                         21.1,	0.782675193381724,
                         21.2,	0.785489992818122,
                         21.3,	0.788297278390584,
                         21.4,	0.791097050099111,
                         21.5,	0.793889307943702,
                         21.6,	0.796674051924357,
                         21.7,	0.799451282041077,
                         21.8,	0.802220998293862,
                         21.9,	0.804983200682711,
                         22,	0.807737889207624,
                         22.1,	0.810485063868602,
                         22.2,	0.813224724665644,
                         22.3,	0.815956871598751,
                         22.4,	0.818681504667922,
                         22.5,	0.821398623873158,
                         22.6,	0.824108229214458,
                         22.7,	0.826810320691822,
                         22.8,	0.829504898305252,
                         22.9,	0.832191962054746,
                         23,	0.834871511940304,
                         23.1,	0.837543547961926,
                         23.2,	0.840208070119613,
                         23.3,	0.842865078413364,
                         23.4,	0.84551457284318,
                         23.5,	0.848156553409061,
                         23.6,	0.850791020111006,
                         23.7,	0.853417972949015,
                         23.8,	0.856037411923089,
                         23.9,	0.858649337033227,
                         24,	0.86125374827943,
                         24.1,	0.863850645661697,
                         24.2,	0.866440029180028,
                         24.3,	0.869021898834425,
                         24.4,	0.871596254624885,
                         24.5,	0.87416309655141,
                         24.6,	0.876722424614,
                         24.7,	0.879274238812654,
                         24.8,	0.881818539147372,
                         24.9,	0.884355325618155,
                         25,	0.886884598225003,
                         25.1,	0.889406356967915,
                         25.2,	0.891920601846891,
                         25.3,	0.894427332861932,
                         25.4,	0.896926550013037,
                         25.5,	0.899418253300207,
                         25.6,	0.901902442723441,
                         25.7,	0.90437911828274,
                         25.8,	0.906848279978103,
                         25.9,	0.90930992780953,
                         26,	0.911764061777023,
                         26.1,	0.914210681880579,
                         26.2,	0.9166497881202,
                         26.3,	0.919081380495886,
                         26.4,	0.921505459007636,
                         26.5,	0.92392202365545,
                         26.6,	0.926331074439329,
                         26.7,	0.928732611359272,
                         26.8,	0.93112663441528,
                         26.9,	0.933513143607352,
                         27,	0.935892138935489,
                         27.1,	0.93826362039969,
                         27.2,	0.940627587999956,
                         27.3,	0.942984041736286,
                         27.4,	0.945332981608681,
                         27.5,	0.94767440761714,
                         27.6,	0.950008319761664,
                         27.7,	0.952334718042252,
                         27.8,	0.954653602458904,
                         27.9,	0.956964973011621,
                         28,	0.959268829700403,
                         28.1,	0.961565172525249,
                         28.2,	0.963854001486159,
                         28.3,	0.966135316583134,
                         28.4,	0.968409117816173,
                         28.5,	0.970675405185277,
                         28.6,	0.972934178690445,
                         28.7,	0.975185438331678,
                         28.8,	0.977429184108975,
                         28.9,	0.979665416022337,
                         29,	0.981894134071763,
                         29.1,	0.984115338257253,
                         29.2,	0.986329028578808,
                         29.3,	0.988535205036428,
                         29.4,	0.990733867630112,
                         29.5,	0.992925016359861,
                         29.6,	0.995108651225673,
                         29.7,	0.997284772227551,
                         29.8,	0.999453379365493,
                         29.9,	1.0016144726395,
                         30,	1.00376805204957,
                         30.1,	1.00591411759571,
                         30.2,	1.0080526692779,
                         30.3,	1.01018370709617,
                         30.4,	1.0123072310505,
                         30.5,	1.01442324114089,
                         30.6,	1.01653173736735,
                         30.7,	1.01863271972987,
                         30.8,	1.02072618822846,
                         30.9,	1.02281214286311,
                         31,	1.02489058363382,
                         31.1,	1.0269615105406,
                         31.2,	1.02902492358345,
                         31.3,	1.03108082276236,
                         31.4,	1.03312920807733,
                         31.5,	1.03517007952837,
                         31.6,	1.03720343711547,
                         31.7,	1.03922928083864,
                         31.8,	1.04124761069787,
                         31.9,	1.04325842669316,
                         32,	1.04526172882452,
                         32.1,	1.04725751709195,
                         32.2,	1.04924579149544,
                         32.3,	1.05122655203499,
                         32.4,	1.05319979871061,
                         32.5,	1.05516553152229,
                         32.6,	1.05712375047004,
                         32.7,	1.05907445555385,
                         32.8,	1.06101764677373,
                         32.9,	1.06295332412967,
                         33,	1.06488148762167,
                         33.1,	1.06680213724974,
                         33.2,	1.06871527301387,
                         33.3,	1.07062089491407,
                         33.4,	1.07251900295034,
                         33.5,	1.07440959712266,
                         33.6,	1.07629267743105,
                         33.7,	1.07816824387551,
                         33.8,	1.08003629645603,
                         33.9,	1.08189683517262,
                         34,	1.08374986002527,
                         34.1,	1.08559537101398,
                         34.2,	1.08743336813876,
                         34.3,	1.0892638513996,
                         34.4,	1.09108682079651,
                         34.5,	1.09290227632948,
                         34.6,	1.09471021799852,
                         34.7,	1.09651064580362,
                         34.8,	1.09830355974478,
                         34.9,	1.10008895982201,
                         35,	1.10186684603531,
                         35.1,	1.10363721838466,
                         35.2,	1.10540007687009,
                         35.3,	1.10715542149158,
                         35.4,	1.10890325224913,
                         35.5,	1.11064356914274,
                         35.6,	1.11237637217243,
                         35.7,	1.11410166133817,
                         35.8,	1.11581943663998,
                         35.9,	1.11752969807786,
                         36,	1.11923244565179,
                         36.1,	1.1209276793618,
                         36.2,	1.12261539920787,
                         36.3,	1.12429560519,
                         36.4,	1.12596829730819,
                         36.5,	1.12763347556246,
                         36.6,	1.12929113995278,
                         36.7,	1.13094129047917,
                         36.8,	1.13258392714163,
                         36.9,	1.13421904994014,
                         37,	1.13584665887473,
                         37.1,	1.13746675394538,
                         37.2,	1.13907933515209,
                         37.3,	1.14068440249487,
                         37.4,	1.14228195597371,
                         37.5,	1.14387199558861,
                         37.6,	1.14545452133958,
                         37.7,	1.14702953322662,
                         37.8,	1.14859703124972,
                         37.9,	1.15015701540888,
                         38,	1.15170948570411,
                         38.1,	1.1532544421354,
                         38.2,	1.15479188470276,
                         38.3,	1.15632181340618,
                         38.4,	1.15784422824567,
                         38.5,	1.15935912922122,
                         38.6,	1.16086651633283,
                         38.7,	1.16236638958051,
                         38.8,	1.16385874896426,
                         38.9,	1.16534359448406,
                         39,	1.16682092613994,
                         39.1,	1.16829074393188,
                         39.2,	1.16975304785988,
                         39.3,	1.17120783792394,
                         39.4,	1.17265511412407,
                         39.5,	1.17409487646027,
                         39.6,	1.17552712493253,
                         39.7,	1.17695185954085,
                         39.8,	1.17836908028524,
                         39.9,	1.17977878716569,
                         40,	1.18118098018221,
                         40.1,	1.18257565933479,
                         40.2,	1.18396282462344,
                         40.3,	1.18534247604815,
                         40.4,	1.18671461360893,
                         40.5,	1.18807923730577,
                         40.6,	1.18943634713867,
                         40.7,	1.19078594310764,
                         40.8,	1.19212802521267,
                         40.9,	1.19346259345377,
                         41,	1.19478964783093,
                         41.1,	1.19610918834416,
                         41.2,	1.19742121499345,
                         41.3,	1.19872572777881,
                         41.4,	1.20002272670023,
                         41.5,	1.20131221175771,
                         41.6,	1.20259418295126,
                         41.7,	1.20386864028087,
                         41.8,	1.20513558374655,
                         41.9,	1.2063950133483,
                         42,	1.2076469290861,
                         42.1,	1.20889133095997,
                         42.2,	1.21012821896991,
                         42.3,	1.21135759311591,
                         42.4,	1.21257945339797,
                         42.5,	1.2137937998161,
                         42.6,	1.2150006323703,
                         42.7,	1.21619995106056,
                         42.8,	1.21739175588688,
                         42.9,	1.21857604684927,
                         43,	1.21975282394772,
                         43.1,	1.22092208718223,
                         43.2,	1.22208383655281,
                         43.3,	1.22323807205946,
                         43.4,	1.22438479370217,
                         43.5,	1.22552400148094,
                         43.6,	1.22665569539578,
                         43.7,	1.22777987544668,
                         43.8,	1.22889654163365,
                         43.9,	1.23000569395668,
                         44,	1.23110733241578,
                         44.1,	1.23220145701094,
                         44.2,	1.23328806774217,
                         44.3,	1.23436716460945,
                         44.4,	1.23543874761281,
                         44.5,	1.23650281675223,
                         44.6,	1.23755937202771,
                         44.7,	1.23860841343926,
                         44.8,	1.23964994098687,
                         44.9,	1.24068395467055,
                         45,	1.24171045449029,
                         45.1,	1.24272944044609,
                         45.2,	1.24374091253796,
                         45.3,	1.2447448707659,
                         45.4,	1.2457413151299,
                         45.5,	1.24673024562996,
                         45.6,	1.24771166226609,
                         45.7,	1.24868556503828,
                         45.8,	1.24965195394654,
                         45.9,	1.25061082899086,
                         46,	1.25156219017124,
                         46.1,	1.25250603748769,
                         46.2,	1.25344237094021,
                         46.3,	1.25437119052879,
                         46.4,	1.25529249625343,
                         46.5,	1.25620628811414,
                         46.6,	1.25711256611091,
                         46.7,	1.25801133024375,
                         46.8,	1.25890258051265,
                         46.9,	1.25978631691762,
                         47,	1.26066253945865
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
    bool working = m_lastdzValid && m_mcConnected && m_lastCoordsValid && m_enabled;
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
}

QString Automator::message() const
{
    return m_message;
}
