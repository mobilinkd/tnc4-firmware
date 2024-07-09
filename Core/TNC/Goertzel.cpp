// Copyright 2017 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "Goertzel.h"

namespace mobilinkd { namespace tnc {

#if 0
const float WINDOW[] = {
  0.006878,
  0.00687800535762,
  0.00687802143048,
  0.00687804821858,
  0.00687808572193,
  0.00687813394055,
  0.00687819287442,
  0.00687826252358,
  0.00687834288804,
  0.00687843396781,
  0.00687853576292,
  0.00687864827338,
  0.00687877149922,
  0.00687890544047,
  0.00687905009716,
  0.00687920546932,
  0.00687937155698,
  0.00687954836018,
  0.00687973587896,
  0.00687993411336,
  0.00688014306342,
  0.0068803627292,
  0.00688059311073,
  0.00688083420806,
  0.00688108602126,
  0.00688134855038,
  0.00688162179547,
  0.00688190575659,
  0.00688220043381,
  0.00688250582719,
  0.0068828219368,
  0.0068831487627,
  0.00688348630498,
  0.0068838345637,
  0.00688419353893,
  0.00688456323077,
  0.00688494363928,
  0.00688533476456,
  0.00688573660669,
  0.00688614916575,
  0.00688657244184,
  0.00688700643505,
  0.00688745114548,
  0.00688790657321,
  0.00688837271837,
  0.00688884958103,
  0.00688933716132,
  0.00688983545934,
  0.00689034447519,
  0.00689086420899,
  0.00689139466085,
  0.00689193583089,
  0.00689248771923,
  0.00689305032598,
  0.00689362365128,
  0.00689420769525,
  0.00689480245801,
  0.00689540793969,
  0.00689602414044,
  0.00689665106038,
  0.00689728869965,
  0.0068979370584,
  0.00689859613676,
  0.00689926593487,
  0.0068999464529,
  0.00690063769098,
  0.00690133964926,
  0.00690205232791,
  0.00690277572707,
  0.00690350984691,
  0.00690425468758,
  0.00690501024926,
  0.0069057765321,
  0.00690655353627,
  0.00690734126195,
  0.00690813970931,
  0.00690894887852,
  0.00690976876976,
  0.0069105993832,
  0.00691144071905,
  0.00691229277746,
  0.00691315555864,
  0.00691402906278,
  0.00691491329006,
  0.00691580824067,
  0.00691671391482,
  0.00691763031271,
  0.00691855743453,
  0.00691949528048,
  0.00692044385078,
  0.00692140314564,
  0.00692237316525,
  0.00692335390984,
  0.00692434537961,
  0.0069253475748,
  0.00692636049561,
  0.00692738414227,
  0.006928418515,
  0.00692946361403,
  0.0069305194396,
  0.00693158599192,
  0.00693266327124,
  0.00693375127779,
  0.00693485001181,
  0.00693595947354,
  0.00693707966322,
  0.00693821058111,
  0.00693935222744,
  0.00694050460247,
  0.00694166770645,
  0.00694284153963,
  0.00694402610228,
  0.00694522139465,
  0.006946427417,
  0.00694764416961,
  0.00694887165273,
  0.00695010986663,
  0.00695135881159,
  0.00695261848788,
  0.00695388889578,
  0.00695517003556,
  0.0069564619075,
  0.0069577645119,
  0.00695907784903,
  0.00696040191918,
  0.00696173672264,
  0.00696308225971,
  0.00696443853067,
  0.00696580553584,
  0.0069671832755,
  0.00696857174996,
  0.00696997095952,
  0.00697138090449,
  0.00697280158518,
  0.00697423300189,
  0.00697567515496,
  0.00697712804468,
  0.00697859167137,
  0.00698006603537,
  0.00698155113699,
  0.00698304697655,
  0.00698455355439,
  0.00698607087083,
  0.00698759892621,
  0.00698913772086,
  0.00699068725512,
  0.00699224752933,
  0.00699381854382,
  0.00699540029895,
  0.00699699279507,
  0.00699859603251,
  0.00700021001163,
  0.00700183473278,
  0.00700347019633,
  0.00700511640262,
  0.00700677335202,
  0.0070084410449,
  0.00701011948161,
  0.00701180866252,
  0.00701350858801,
  0.00701521925844,
  0.00701694067419,
  0.00701867283565,
  0.00702041574318,
  0.00702216939716,
  0.00702393379799,
  0.00702570894605,
  0.00702749484173,
  0.00702929148541,
  0.0070310988775,
  0.00703291701839,
  0.00703474590847,
  0.00703658554814,
  0.00703843593781,
  0.00704029707789,
  0.00704216896878,
  0.00704405161089,
  0.00704594500463,
  0.00704784915041,
  0.00704976404867,
  0.0070516896998,
  0.00705362610424,
  0.0070555732624,
  0.00705753117472,
  0.00705949984162,
  0.00706147926353,
  0.00706346944089,
  0.00706547037414,
  0.0070674820637,
  0.00706950451002,
  0.00707153771354,
  0.0070735816747,
  0.00707563639396,
  0.00707770187177,
  0.00707977810857,
  0.00708186510481,
  0.00708396286097,
  0.00708607137748,
  0.00708819065483,
  0.00709032069346,
  0.00709246149384,
  0.00709461305645,
  0.00709677538175,
  0.00709894847022,
  0.00710113232233,
  0.00710332693855,
  0.00710553231938,
  0.00710774846528,
  0.00710997537675,
  0.00711221305427,
  0.00711446149834,
  0.00711672070943,
  0.00711899068805,
  0.0071212714347,
  0.00712356294986,
  0.00712586523405,
  0.00712817828776,
  0.0071305021115,
  0.00713283670578,
  0.00713518207111,
  0.007137538208,
  0.00713990511696,
  0.00714228279852,
  0.00714467125318,
  0.00714707048149,
  0.00714948048395,
  0.00715190126109,
  0.00715433281345,
  0.00715677514156,
  0.00715922824594,
  0.00716169212714,
  0.00716416678568,
  0.00716665222212,
  0.007169148437,
  0.00717165543085,
  0.00717417320424,
  0.0071767017577,
  0.00717924109179,
  0.00718179120707,
  0.00718435210408,
  0.0071869237834,
  0.00718950624558,
  0.00719209949118,
  0.00719470352078,
  0.00719731833493,
  0.00719994393421,
  0.0072025803192,
  0.00720522749047,
  0.00720788544859,
  0.00721055419416,
  0.00721323372774,
  0.00721592404992,
  0.0072186251613,
  0.00722133706245,
  0.00722405975398,
  0.00722679323647,
  0.00722953751053,
  0.00723229257674,
  0.00723505843572,
  0.00723783508807,
  0.00724062253438,
  0.00724342077527,
  0.00724622981136,
  0.00724904964324,
  0.00725188027154,
  0.00725472169688,
  0.00725757391987,
  0.00726043694113,
  0.0072633107613,
  0.00726619538099,
  0.00726909080084,
  0.00727199702147,
  0.00727491404352,
  0.00727784186763,
  0.00728078049443,
  0.00728372992457,
  0.00728669015868,
  0.00728966119742,
  0.00729264304143,
  0.00729563569136,
  0.00729863914785,
  0.00730165341158,
  0.00730467848319,
  0.00730771436334,
  0.00731076105269,
  0.00731381855191,
  0.00731688686166,
  0.00731996598262,
  0.00732305591544,
  0.00732615666082,
  0.00732926821941,
  0.0073323905919,
  0.00733552377897,
  0.0073386677813,
  0.00734182259957,
  0.00734498823448,
  0.0073481646867,
  0.00735135195694,
  0.00735455004589,
  0.00735775895424,
  0.00736097868268,
  0.00736420923193,
  0.00736745060269,
  0.00737070279565,
  0.00737396581154,
  0.00737723965105,
  0.0073805243149,
  0.00738381980381,
  0.00738712611849,
  0.00739044325966,
  0.00739377122804,
  0.00739711002436,
  0.00740045964934,
  0.00740382010372,
  0.00740719138821,
  0.00741057350357,
  0.00741396645051,
  0.00741737022979,
  0.00742078484213,
  0.00742421028828,
  0.00742764656899,
  0.00743109368501,
  0.00743455163707,
  0.00743802042595,
  0.00744150005238,
  0.00744499051712,
  0.00744849182094,
  0.00745200396459,
  0.00745552694884,
  0.00745906077446,
  0.0074626054422,
  0.00746616095284,
  0.00746972730716,
  0.00747330450593,
  0.00747689254992,
  0.00748049143992,
  0.0074841011767,
  0.00748772176106,
  0.00749135319377,
  0.00749499547563,
  0.00749864860742,
  0.00750231258994,
  0.00750598742399,
  0.00750967311036,
  0.00751336964985,
  0.00751707704327,
  0.00752079529142,
  0.0075245243951,
  0.00752826435514,
  0.00753201517233,
  0.00753577684749,
  0.00753954938144,
  0.007543332775,
  0.00754712702899,
  0.00755093214423,
  0.00755474812154,
  0.00755857496176,
  0.00756241266571,
  0.00756626123423,
  0.00757012066815,
  0.00757399096831,
  0.00757787213555,
  0.00758176417071,
  0.00758566707463,
  0.00758958084816,
  0.00759350549215,
  0.00759744100745,
  0.0076013873949,
  0.00760534465538,
  0.00760931278973,
  0.00761329179881,
  0.0076172816835,
  0.00762128244464,
  0.00762529408311,
  0.00762931659978,
  0.00763334999552,
  0.0076373942712,
  0.0076414494277,
  0.0076455154659,
  0.00764959238668,
  0.00765368019092,
  0.0076577788795,
  0.00766188845331,
  0.00766600891325,
  0.00767014026021,
  0.00767428249507,
  0.00767843561874,
  0.00768259963212,
  0.00768677453611,
  0.0076909603316,
  0.00769515701951,
  0.00769936460074,
  0.00770358307621,
  0.00770781244683,
  0.00771205271351,
  0.00771630387717,
  0.00772056593873,
  0.00772483889911,
  0.00772912275923,
  0.00773341752003,
  0.00773772318243,
  0.00774203974736,
  0.00774636721576,
  0.00775070558855,
  0.00775505486669,
  0.0077594150511,
  0.00776378614274,
  0.00776816814254,
  0.00777256105145,
  0.00777696487043,
  0.00778137960042,
  0.00778580524238,
  0.00779024179727,
  0.00779468926603,
  0.00779914764964,
  0.00780361694906,
  0.00780809716525,
  0.00781258829918,
  0.00781709035181,
  0.00782160332413,
  0.00782612721711,
  0.00783066203172,
  0.00783520776893,
  0.00783976442975,
  0.00784433201514,
  0.00784891052609,
  0.00785349996359,
  0.00785810032863,
  0.00786271162221,
  0.00786733384532,
  0.00787196699895,
  0.00787661108411,
  0.00788126610179,
  0.00788593205301,
  0.00789060893876,
  0.00789529676006,
  0.00789999551792,
  0.00790470521335,
  0.00790942584736,
  0.00791415742098,
  0.00791889993523,
  0.00792365339112,
  0.00792841778967,
  0.00793319313193,
  0.00793797941892,
  0.00794277665166,
  0.00794758483119,
  0.00795240395855,
  0.00795723403478,
  0.00796207506091,
  0.007966927038,
  0.00797178996707,
  0.00797666384919,
  0.0079815486854,
  0.00798644447675,
  0.0079913512243,
  0.00799626892911,
  0.00800119759222,
  0.00800613721471,
  0.00801108779763,
  0.00801604934206,
  0.00802102184906,
  0.00802600531969,
  0.00803099975504,
  0.00803600515617,
  0.00804102152416,
  0.0080460488601,
  0.00805108716506,
  0.00805613644013,
  0.00806119668639,
  0.00806626790493,
  0.00807135009684,
  0.00807644326321,
  0.00808154740515,
  0.00808666252374,
  0.00809178862008,
  0.00809692569528,
  0.00810207375045,
  0.00810723278668,
  0.00811240280508,
  0.00811758380677,
  0.00812277579286,
  0.00812797876447,
  0.00813319272271,
  0.0081384176687,
  0.00814365360356,
  0.00814890052842,
  0.00815415844441,
  0.00815942735265,
  0.00816470725428,
  0.00816999815042,
  0.00817530004221,
  0.0081806129308,
  0.00818593681732,
  0.00819127170291,
  0.00819661758873,
  0.0082019744759,
  0.00820734236559,
  0.00821272125895,
  0.00821811115712,
  0.00822351206127,
  0.00822892397256,
  0.00823434689214,
  0.00823978082117,
  0.00824522576082,
  0.00825068171227,
  0.00825614867667,
  0.0082616266552,
  0.00826711564903,
  0.00827261565934,
  0.00827812668731,
  0.00828364873412,
  0.00828918180095,
  0.00829472588898,
  0.00830028099941,
  0.00830584713342,
  0.00831142429221,
  0.00831701247697,
  0.00832261168889,
  0.00832822192917,
  0.00833384319902,
  0.00833947549964,
  0.00834511883223,
  0.008350773198,
  0.00835643859816,
  0.00836211503393,
  0.00836780250651,
  0.00837350101713,
  0.00837921056699,
  0.00838493115734,
  0.00839066278937,
  0.00839640546433,
  0.00840215918344,
  0.00840792394793,
  0.00841369975903,
  0.00841948661798,
  0.00842528452601,
  0.00843109348436,
  0.00843691349428,
  0.008442744557,
  0.00844858667377,
  0.00845443984585,
  0.00846030407447,
  0.0084661793609,
  0.00847206570638,
  0.00847796311218,
  0.00848387157955,
  0.00848979110975,
  0.00849572170406,
  0.00850166336372,
  0.00850761609002,
  0.00851357988422,
  0.00851955474759,
  0.00852554068142,
  0.00853153768697,
  0.00853754576552,
  0.00854356491836,
  0.00854959514677,
  0.00855563645203,
  0.00856168883544,
  0.00856775229828,
  0.00857382684185,
  0.00857991246744,
  0.00858600917635,
  0.00859211696987,
  0.00859823584931,
  0.00860436581598,
  0.00861050687117,
  0.0086166590162,
  0.00862282225238,
  0.00862899658102,
  0.00863518200343,
  0.00864137852093,
  0.00864758613484,
  0.00865380484648,
  0.00866003465717,
  0.00866627556825,
  0.00867252758104,
  0.00867879069686,
  0.00868506491706,
  0.00869135024296,
  0.00869764667591,
  0.00870395421724,
  0.00871027286829,
  0.00871660263041,
  0.00872294350495,
  0.00872929549324,
  0.00873565859665,
  0.00874203281652,
  0.00874841815421,
  0.00875481461108,
  0.00876122218848,
  0.00876764088777,
  0.00877407071033,
  0.00878051165751,
  0.00878696373068,
  0.00879342693122,
  0.0087999012605,
  0.00880638671988,
  0.00881288331075,
  0.00881939103449,
  0.00882590989247,
  0.00883243988609,
  0.00883898101672,
  0.00884553328576,
  0.00885209669459,
  0.00885867124461,
  0.0088652569372,
  0.00887185377378,
  0.00887846175573,
  0.00888508088446,
  0.00889171116137,
  0.00889835258787,
  0.00890500516536,
  0.00891166889526,
  0.00891834377897,
  0.00892502981791,
  0.0089317270135,
  0.00893843536715,
  0.00894515488029,
  0.00895188555434,
  0.00895862739073,
  0.00896538039087,
  0.00897214455621,
  0.00897891988817,
  0.00898570638819,
  0.00899250405771,
  0.00899931289815,
  0.00900613291097,
  0.00901296409761,
  0.0090198064595,
  0.00902665999811,
  0.00903352471487,
  0.00904040061125,
  0.00904728768868,
  0.00905418594864,
  0.00906109539257,
  0.00906801602194,
  0.00907494783821,
  0.00908189084285,
  0.00908884503732,
  0.00909581042309,
  0.00910278700163,
  0.00910977477442,
  0.00911677374293,
  0.00912378390864,
  0.00913080527304,
  0.00913783783759,
  0.00914488160379,
  0.00915193657313,
  0.00915900274708,
  0.00916608012715,
  0.00917316871483,
  0.00918026851161,
  0.00918737951899,
  0.00919450173846,
  0.00920163517154,
  0.00920877981973,
  0.00921593568452,
  0.00922310276744,
  0.00923028106999,
  0.00923747059368,
  0.00924467134003,
  0.00925188331055,
  0.00925910650677,
  0.00926634093021,
  0.00927358658239,
  0.00928084346483,
  0.00928811157907,
  0.00929539092664,
  0.00930268150906,
  0.00930998332787,
  0.00931729638461,
  0.00932462068082,
  0.00933195621804,
  0.00933930299781,
  0.00934666102167,
  0.00935403029118,
  0.00936141080789,
  0.00936880257334,
  0.0093762055891,
  0.00938361985671,
  0.00939104537774,
  0.00939848215374,
  0.00940593018628,
  0.00941338947693,
  0.00942086002725,
  0.0094283418388,
  0.00943583491318,
  0.00944333925193,
  0.00945085485665,
  0.00945838172891,
  0.00946591987028,
  0.00947346928236,
  0.00948102996673,
  0.00948860192496,
  0.00949618515866,
  0.00950377966942,
  0.00951138545881,
  0.00951900252845,
  0.00952663087993,
  0.00953427051484,
  0.0095419214348,
  0.0095495836414,
  0.00955725713624,
  0.00956494192095,
  0.00957263799712,
  0.00958034536637,
  0.00958806403032,
  0.00959579399058,
  0.00960353524878,
  0.00961128780652,
  0.00961905166544,
  0.00962682682716,
  0.00963461329331,
  0.00964241106551,
  0.00965022014541,
  0.00965804053463,
  0.00966587223481,
  0.00967371524759,
  0.0096815695746,
  0.0096894352175,
  0.00969731217793,
  0.00970520045752,
  0.00971310005794,
  0.00972101098083,
  0.00972893322785,
  0.00973686680065,
  0.00974481170089,
  0.00975276793023,
  0.00976073549033,
  0.00976871438286,
  0.00977670460948,
  0.00978470617186,
  0.00979271907166,
  0.00980074331057,
  0.00980877889026,
  0.0098168258124,
  0.00982488407868,
  0.00983295369076,
  0.00984103465035,
  0.00984912695912,
  0.00985723061876,
  0.00986534563096,
  0.00987347199742,
  0.00988160971981,
  0.00988975879986,
  0.00989791923924,
  0.00990609103966,
  0.00991427420282,
  0.00992246873044,
  0.0099306746242,
  0.00993889188583,
  0.00994712051704,
  0.00995536051953,
  0.00996361189502,
  0.00997187464524,
  0.00998014877189,
  0.0099884342767,
  0.0099967311614,
  0.0100050394277,
  0.0100133590774,
  0.0100216901121,
  0.0100300325336,
  0.0100383863436,
  0.010046751544,
  0.0100551281363,
  0.0100635161224,
  0.010071915504,
  0.0100803262828,
  0.0100887484606,
  0.0100971820391,
  0.0101056270202,
  0.0101140834054,
  0.0101225511967,
  0.0101310303957,
  0.0101395210042,
  0.010148023024,
  0.0101565364569,
  0.0101650613045,
  0.0101735975687,
  0.0101821452513,
  0.0101907043539,
  0.0101992748785,
  0.0102078568267,
  0.0102164502004,
  0.0102250550013,
  0.0102336712312,
  0.0102422988919,
  0.0102509379852,
  0.0102595885129,
  0.0102682504767,
  0.0102769238785,
  0.01028560872,
  0.0102943050031,
  0.0103030127295,
  0.010311731901,
  0.0103204625196,
  0.0103292045869,
  0.0103379581047,
  0.010346723075,
  0.0103554994994,
  0.0103642873798,
  0.0103730867181,
  0.010381897516,
  0.0103907197753,
  0.010399553498,
  0.0104083986857,
  0.0104172553404,
  0.0104261234639,
  0.0104350030579,
  0.0104438941244,
  0.0104527966652,
  0.010461710682,
  0.0104706361768,
  0.0104795731514,
  0.0104885216076,
  0.0104974815473,
  0.0105064529723,
  0.0105154358845,
  0.0105244302857,
  0.0105334361778,
  0.0105424535626,
  0.0105514824421,
  0.0105605228179,
  0.0105695746921,
  0.0105786380665,
  0.0105877129429,
  0.0105967993233,
  0.0106058972094,
  0.0106150066032,
  0.0106241275065,
  0.0106332599212,
  0.0106424038493,
  0.0106515592925,
  0.0106607262527,
  0.0106699047319,
  0.010679094732,
  0.0106882962547,
  0.0106975093021,
  0.010706733876,
  0.0107159699782,
  0.0107252176108,
  0.0107344767756,
  0.0107437474745,
  0.0107530297093,
  0.0107623234821,
  0.0107716287947,
  0.0107809456491,
  0.0107902740471,
  0.0107996139906,
  0.0108089654816,
  0.010818328522,
  0.0108277031138,
  0.0108370892587,
  0.0108464869588,
  0.010855896216,
  0.0108653170322,
  0.0108747494094,
  0.0108841933495,
  0.0108936488543,
  0.0109031159259,
  0.0109125945662,
  0.0109220847772,
  0.0109315865607,
  0.0109410999187,
  0.0109506248532,
  0.0109601613661,
  0.0109697094594,
  0.010979269135,
  0.0109888403949,
  0.0109984232411,
  0.0110080176754,
  0.0110176236999,
  0.0110272413165,
  0.0110368705272,
  0.0110465113339,
  0.0110561637387,
  0.0110658277435,
  0.0110755033502,
  0.0110851905609,
  0.0110948893776,
  0.0111045998021,
  0.0111143218365,
  0.0111240554828,
  0.011133800743,
  0.011143557619,
  0.0111533261128,
  0.0111631062265,
  0.011172897962,
  0.0111827013213,
  0.0111925163065,
  0.0112023429194,
  0.0112121811622,
  0.0112220310368,
  0.0112318925453,
  0.0112417656896,
  0.0112516504718,
  0.0112615468938,
  0.0112714549577,
  0.0112813746655,
  0.0112913060192,
  0.0113012490209,
  0.0113112036725,
  0.0113211699762,
  0.0113311479338,
  0.0113411375475,
  0.0113511388192,
  0.0113611517511,
  0.0113711763451,
  0.0113812126033,
  0.0113912605277,
  0.0114013201204,
  0.0114113913834,
  0.0114214743187,
  0.0114315689285,
  0.0114416752147,
  0.0114517931793,
  0.0114619228246,
  0.0114720641524,
  0.0114822171649,
  0.0114923818642,
  0.0115025582522,
  0.0115127463311,
  0.0115229461028,
  0.0115331575696,
  0.0115433807334,
  0.0115536155964,
  0.0115638621605,
  0.0115741204279,
  0.0115843904007,
  0.0115946720809,
  0.0116049654706,
  0.0116152705719,
  0.0116255873869,
  0.0116359159177,
  0.0116462561663,
  0.0116566081349,
  0.0116669718255,
  0.0116773472403,
  0.0116877343812,
  0.0116981332506,
  0.0117085438503,
  0.0117189661827,
  0.0117294002496,
  0.0117398460533,
  0.0117503035959,
  0.0117607728795,
  0.0117712539061,
  0.011781746678,
  0.0117922511972,
  0.0118027674658,
  0.011813295486,
  0.0118238352599,
  0.0118343867896,
  0.0118449500773,
  0.011855525125,
  0.0118661119349,
  0.0118767105092,
  0.0118873208499,
  0.0118979429593,
  0.0119085768394,
  0.0119192224924,
  0.0119298799204,
  0.0119405491256,
  0.0119512301102,
  0.0119619228762,
  0.0119726274258,
  0.0119833437613,
  0.0119940718847,
  0.0120048117981,
  0.0120155635039,
  0.012026327004,
  0.0120371023008,
  0.0120478893962,
  0.0120586882926,
  0.0120694989921,
  0.0120803214969,
  0.012091155809,
  0.0121020019308,
  0.0121128598643,
  0.0121237296118,
  0.0121346111755,
  0.0121455045575,
  0.01215640976,
  0.0121673267852,
  0.0121782556352,
  0.0121891963124,
  0.0122001488188,
  0.0122111131567,
  0.0122220893282,
  0.0122330773356,
  0.0122440771811,
  0.0122550888669,
  0.0122661123951,
  0.012277147768,
  0.0122881949877,
  0.0122992540566,
  0.0123103249768,
  0.0123214077505,
  0.0123325023799,
  0.0123436088673,
  0.0123547272149,
  0.0123658574249,
  0.0123769994995,
  0.012388153441,
  0.0123993192515,
  0.0124104969334,
  0.0124216864888,
  0.01243288792,
  0.0124441012292,
  0.0124553264186,
  0.0124665634906,
  0.0124778124472,
  0.0124890732909,
  0.0125003460238,
  0.0125116306481,
  0.0125229271662,
  0.0125342355802,
  0.0125455558925,
  0.0125568881053,
  0.0125682322208,
  0.0125795882413,
  0.012590956169,
  0.0126023360063,
  0.0126137277554,
  0.0126251314186,
  0.012636546998,
  0.0126479744961,
  0.012659413915,
  0.0126708652571,
  0.0126823285246,
  0.0126938037198,
  0.012705290845,
  0.0127167899025,
  0.0127283008945,
  0.0127398238233,
  0.0127513586913,
  0.0127629055007,
  0.0127744642538,
  0.0127860349529,
  0.0127976176003,
  0.0128092121983,
  0.0128208187492,
  0.0128324372553,
  0.012844067719,
  0.0128557101424,
  0.0128673645279,
  0.0128790308779,
  0.0128907091946,
  0.0129023994803,
  0.0129141017375,
  0.0129258159683,
  0.0129375421751,
  0.0129492803602,
  0.012961030526,
  0.0129727926748,
  0.0129845668089,
  0.0129963529306,
  0.0130081510423,
  0.0130199611462,
  0.0130317832448,
  0.0130436173404,
  0.0130554634352,
  0.0130673215318,
  0.0130791916323,
  0.0130910737391,
  0.0131029678546,
  0.0131148739811,
  0.013126792121,
  0.0131387222767,
  0.0131506644504,
  0.0131626186445,
  0.0131745848615,
  0.0131865631036,
  0.0131985533732,
  0.0132105556727,
  0.0132225700045,
  0.0132345963708,
  0.0132466347742,
  0.0132586852168,
  0.0132707477013,
  0.0132828222298,
  0.0132949088047,
  0.0133070074286,
  0.0133191181036,
  0.0133312408323,
  0.013343375617,
  0.0133555224601,
  0.0133676813639,
  0.0133798523309,
  0.0133920353635,
  0.013404230464,
  0.0134164376349,
  0.0134286568785,
  0.0134408881972,
  0.0134531315935,
  0.0134653870697,
  0.0134776546283,
  0.0134899342716,
  0.0135022260021,
  0.0135145298222,
  0.0135268457342,
  0.0135391737407,
  0.013551513844,
  0.0135638660465,
  0.0135762303506,
  0.0135886067589,
  0.0136009952736,
  0.0136133958973,
  0.0136258086323,
  0.0136382334811,
  0.0136506704462,
  0.0136631195298,
  0.0136755807346,
  0.0136880540629,
  0.0137005395171,
  0.0137130370997,
  0.0137255468132,
  0.0137380686599,
  0.0137506026424,
  0.0137631487631,
  0.0137757070244,
  0.0137882774287,
  0.0138008599786,
  0.0138134546765,
  0.0138260615249,
  0.0138386805261,
  0.0138513116827,
  0.0138639549971,
  0.0138766104718,
  0.0138892781093,
  0.013901957912,
  0.0139146498824,
  0.013927354023,
  0.0139400703362,
  0.0139527988246,
  0.0139655394905,
  0.0139782923365,
  0.0139910573651,
  0.0140038345787,
  0.0140166239798,
  0.0140294255709,
  0.0140422393546,
  0.0140550653332,
  0.0140679035093,
  0.0140807538854,
  0.014093616464,
  0.0141064912475,
  0.0141193782385,
  0.0141322774395,
  0.0141451888529,
  0.0141581124814,
  0.0141710483273,
  0.0141839963932,
  0.0141969566816,
  0.0142099291951,
  0.014222913936,
  0.0142359109071,
  0.0142489201107,
  0.0142619415493,
  0.0142749752256,
  0.014288021142,
  0.0143010793011,
  0.0143141497054,
  0.0143272323574,
  0.0143403272596,
  0.0143534344146,
  0.0143665538249,
  0.0143796854931,
  0.0143928294216,
  0.0144059856131,
  0.01441915407,
  0.014432334795,
  0.0144455277905,
  0.0144587330591,
  0.0144719506033,
  0.0144851804258,
  0.014498422529,
  0.0145116769155,
  0.0145249435879,
  0.0145382225487,
  0.0145515138005,
  0.0145648173459,
  0.0145781331873,
  0.0145914613275,
  0.0146048017688,
  0.014618154514,
  0.0146315195656,
  0.0146448969261,
  0.0146582865981,
  0.0146716885842,
  0.014685102887,
  0.0146985295091,
  0.014711968453,
  0.0147254197214,
  0.0147388833167,
  0.0147523592417,
  0.0147658474988,
  0.0147793480907,
  0.01479286102,
  0.0148063862893,
  0.0148199239011,
  0.014833473858,
  0.0148470361627,
  0.0148606108178,
  0.0148741978258,
  0.0148877971894,
  0.0149014089111,
  0.0149150329937,
  0.0149286694396,
  0.0149423182515,
  0.014955979432,
  0.0149696529838,
  0.0149833389094,
  0.0149970372115,
  0.0150107478926,
  0.0150244709555,
  0.0150382064027,
  0.0150519542368,
  0.0150657144606,
  0.0150794870765,
  0.0150932720873,
  0.0151070694956,
  0.015120879304,
  0.0151347015151,
  0.0151485361317,
  0.0151623831562,
  0.0151762425915,
  0.01519011444,
  0.0152039987046,
  0.0152178953877,
  0.0152318044921,
  0.0152457260204,
  0.0152596599752,
  0.0152736063593,
  0.0152875651752,
  0.0153015364257,
  0.0153155201133,
  0.0153295162409,
  0.0153435248109,
  0.0153575458261,
  0.0153715792891,
  0.0153856252027,
  0.0153996835694,
  0.015413754392,
  0.0154278376731,
  0.0154419334155,
  0.0154560416217,
  0.0154701622944,
  0.0154842954365,
  0.0154984410504,
  0.0155125991389,
  0.0155267697048,
  0.0155409527506,
  0.0155551482791,
  0.0155693562929,
  0.0155835767948,
  0.0155978097874,
  0.0156120552735,
  0.0156263132558,
  0.0156405837368,
  0.0156548667195,
  0.0156691622063,
  0.0156834702001,
  0.0156977907036,
  0.0157121237195,
  0.0157264692504,
  0.0157408272991,
  0.0157551978683,
  0.0157695809607,
  0.0157839765791,
  0.0157983847261,
  0.0158128054045,
  0.015827238617,
  0.0158416843663,
  0.0158561426551,
  0.0158706134863,
  0.0158850968624,
  0.0158995927862,
  0.0159141012606,
  0.0159286222881,
  0.0159431558716,
  0.0159577020137,
  0.0159722607172,
  0.0159868319849,
  0.0160014158195,
  0.0160160122238,
  0.0160306212004,
  0.0160452427522,
  0.0160598768818,
  0.0160745235921,
  0.0160891828858,
  0.0161038547656,
  0.0161185392343,
  0.0161332362947,
  0.0161479459495,
  0.0161626682015,
  0.0161774030534,
  0.016192150508,
  0.0162069105681,
  0.0162216832365,
  0.0162364685158,
  0.016251266409,
  0.0162660769187,
  0.0162809000477,
  0.0162957357989,
  0.0163105841749,
  0.0163254451786,
  0.0163403188128,
  0.0163552050802,
  0.0163701039836,
  0.0163850155258,
  0.0163999397096,
  0.0164148765379,
  0.0164298260132,
  0.0164447881386,
  0.0164597629167,
  0.0164747503504,
  0.0164897504425,

};

#endif

}} // mobilinkd::tnc

