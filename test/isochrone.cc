#include <iostream>
#include <string>
#include <vector>

#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "loki/worker.h"
#include "thor/worker.h"

#include "gurka/gurka.h"
#include "test.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

using point_type = boost::geometry::model::d2::point_xy<double>;
using polygon_type = boost::geometry::model::polygon<point_type>;
using boost::geometry::within;

using namespace valhalla;
using namespace valhalla::thor;
using namespace valhalla::sif;
using namespace valhalla::loki;
using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::tyr;

using rp = rapidjson::Pointer;

namespace {

const auto cfg = test::make_config(VALHALLA_BUILD_DIR "test/data/utrecht_tiles");

void check_coords(const rapidjson::Value& a, const rapidjson::Value& b) {
  EXPECT_NEAR(a.GetArray()[0].GetDouble(), b.GetArray()[0].GetDouble(), 0.00002);
  EXPECT_NEAR(a.GetArray()[1].GetDouble(), b.GetArray()[1].GetDouble(), 0.00002);
}

void try_isochrone(loki_worker_t& loki_worker,
                   thor_worker_t& thor_worker,
                   const std::string& test_request,
                   const std::string& expected_json) {
  // compute the isochrone
  Api request;
  ParseApi(test_request, Options::isochrone, request);
  loki_worker.isochrones(request);
  auto response_json = thor_worker.isochrones(request);
  loki_worker.cleanup();
  thor_worker.cleanup();

  // Parse isochrone json responses
  rapidjson::Document response, expected_response;
  response.Parse(response_json);
  expected_response.Parse(expected_json);

  // Same number of features
  auto feature_count = rp("/features").Get(expected_response)->GetArray().Size();
  ASSERT_EQ(rp("/features").Get(response)->GetArray().Size(), feature_count);

  // Check features are in the right order and look roughly the same
  for (size_t i = 0; i < feature_count; ++i) {
    // same metadata
    auto actual_properties = rp("/features/" + std::to_string(i) + "/properties").Get(response);
    auto expected_properties =
        rp("/features/" + std::to_string(i) + "/properties").Get(expected_response);
    EXPECT_TRUE((actual_properties && expected_properties) ||
                (!actual_properties && !expected_properties));
    if (expected_properties) {
      ASSERT_EQ(actual_properties->GetObject(), expected_properties->GetObject());
    }

    // same geom type
    std::string actual_type =
        rp("/features/" + std::to_string(i) + "/geometry/type").Get(response)->GetString();
    std::string expected_type =
        rp("/features/" + std::to_string(i) + "/geometry/type").Get(expected_response)->GetString();
    ASSERT_EQ(actual_type, expected_type);

    // point is special
    if (expected_type == "Point") {
      check_coords(*rp("/features/" + std::to_string(i) + "/geometry/coordinates").Get(response),
                   *rp("/features/" + std::to_string(i) + "/geometry/coordinates")
                        .Get(expected_response));
    } // iteration required
    else {
      // same geom appx
      auto actual_geom = rp("/features/" + std::to_string(i) + "/geometry/coordinates" +
                            (actual_type == "Polygon" ? "/0" : ""))
                             .Get(response)
                             ->GetArray();
      auto expected_geom = rp("/features/" + std::to_string(i) + "/geometry/coordinates" +
                              (expected_type == "Polygon" ? "/0" : ""))
                               .Get(expected_response)
                               ->GetArray();

      std::vector<PointLL> actual, expected;
      for (size_t j = 0; j < std::max(expected_geom.Size(), actual_geom.Size()); ++j) {
        if (j < actual_geom.Size()) {
          auto c = actual_geom[j].GetArray();
          actual.emplace_back(c[0].GetDouble(), c[1].GetDouble());
        }
        if (j < expected_geom.Size()) {
          auto c = expected_geom[j].GetArray();
          expected.emplace_back(c[0].GetDouble(), c[1].GetDouble());
        }
      }

      // different platforms can end up having some slightly different floating point wobble
      // to avoid failing tests we measure shape similarity and fail if its too far out of whack
      ASSERT_TRUE(test::shape_equality(actual, expected, 33));
    }
  }
}

std::vector<PointLL> polygon_from_geojson(const std::string& geojson) {
  rapidjson::Document response;
  response.Parse(geojson);

  auto feature_count = rp("/features").Get(response)->GetArray().Size();
  for (size_t i = 0; i < feature_count; ++i) {
    std::string type =
        rp("/features/" + std::to_string(i) + "/geometry/type").Get(response)->GetString();

    if (type != "Point") {
      auto geom = rp("/features/" + std::to_string(i) + "/geometry/coordinates" +
                     (type == "Polygon" ? "/0" : ""))
                      .Get(response)
                      ->GetArray();
      std::vector<PointLL> res;
      res.reserve(geom.Size());
      for (size_t j = 0; j < geom.Size(); ++j) {
        auto coord = geom[j].GetArray();
        res.emplace_back(coord[0].GetDouble(), coord[1].GetDouble());
      }
      return res;
    }
  }
  return {};
}

TEST(Isochrones, Basic) {
  // Test setup
  loki_worker_t loki_worker(cfg);
  thor_worker_t thor_worker(cfg);
  GraphReader reader(cfg.get_child("mjolnir"));

  {
    const auto request =
        R"({"locations":[{"lat":52.078937,"lon":5.115321}],"costing":"auto","contours":[{"time":9.1}],"polygons":false,"generalize":55})";
    const auto expected =
        R"({"features":[{"properties":{"fill-opacity":0.33,"fillColor":"#bf4040","opacity":0.33,"fill":"#bf4040","fillOpacity":0.33,"color":"#bf4040","contour":9.1,"metric":"time"},"geometry":{"coordinates":[[5.042321,52.127328],[5.041288,52.126971],[5.041162,52.126096],[5.040250,52.126008],[5.040123,52.125135],[5.038321,52.124230],[5.038155,52.123104],[5.035757,52.121937],[5.034321,52.119336],[5.029719,52.120335],[5.027321,52.122071],[5.025528,52.122144],[5.025321,52.123106],[5.023541,52.123157],[5.022162,52.124937],[5.022291,52.123907],[5.023244,52.122860],[5.024983,52.122599],[5.025321,52.121673],[5.026949,52.121565],[5.028321,52.119350],[5.029770,52.119386],[5.030321,52.118047],[5.031738,52.118354],[5.031768,52.117491],[5.030068,52.117190],[5.029028,52.112644],[5.032109,52.116149],[5.037321,52.115662],[5.044771,52.112387],[5.047321,52.112089],[5.047520,52.111136],[5.049379,52.110995],[5.047712,52.109937],[5.053321,52.103702],[5.057006,52.103622],[5.058321,52.102515],[5.061389,52.102937],[5.060882,52.100376],[5.059321,52.099179],[5.056855,52.098937],[5.057948,52.098564],[5.057822,52.096937],[5.059321,52.096398],[5.059890,52.094937],[5.062669,52.096285],[5.063657,52.092601],[5.059321,52.093638],[5.057934,52.090324],[5.054848,52.088410],[5.051513,52.088129],[5.051321,52.089053],[5.051073,52.088185],[5.048321,52.087370],[5.047321,52.088623],[5.046687,52.086571],[5.044576,52.084937],[5.045321,52.084802],[5.047321,52.085491],[5.048713,52.085329],[5.049321,52.082083],[5.050409,52.083850],[5.053317,52.081933],[5.055321,52.082660],[5.071321,52.081546],[5.072536,52.078937],[5.070321,52.075645],[5.068321,52.076598],[5.064321,52.076524],[5.063321,52.077492],[5.060985,52.077273],[5.060321,52.074253],[5.062321,52.076406],[5.063321,52.075371],[5.066556,52.075173],[5.067628,52.072937],[5.067514,52.070744],[5.065321,52.069631],[5.061321,52.069611],[5.060321,52.070600],[5.054934,52.070937],[5.056828,52.072937],[5.057903,52.077355],[5.060418,52.078937],[5.059321,52.081790],[5.057766,52.078492],[5.054557,52.077937],[5.055847,52.077463],[5.055644,52.074614],[5.053321,52.074166],[5.053916,52.077937],[5.053321,52.078215],[5.052921,52.076337],[5.051321,52.076004],[5.049321,52.079349],[5.049113,52.078145],[5.047811,52.078427],[5.047736,52.079522],[5.048998,52.079937],[5.048420,52.082036],[5.046798,52.078460],[5.044152,52.077106],[5.044305,52.074921],[5.043205,52.075937],[5.043272,52.074888],[5.043588,52.074204],[5.043516,52.072937],[5.042321,52.072093],[5.040321,52.073230],[5.034963,52.073579],[5.034321,52.078500],[5.028415,52.075937],[5.022321,52.075363],[5.021967,52.073583],[5.026171,52.072937],[5.027090,52.069706],[5.030321,52.071554],[5.037321,52.071095],[5.043321,52.069471],[5.047584,52.069200],[5.049321,52.068151],[5.054321,52.068229],[5.056321,52.066108],[5.059321,52.067213],[5.063501,52.066117],[5.063749,52.064365],[5.066245,52.061937],[5.066622,52.058937],[5.065629,52.057937],[5.067742,52.057358],[5.067482,52.054937],[5.068412,52.054846],[5.069986,52.058272],[5.073724,52.057937],[5.070994,52.057264],[5.069724,52.051534],[5.066166,52.051937],[5.066182,52.050798],[5.067807,52.050423],[5.070321,52.046926],[5.070489,52.047769],[5.072941,52.046557],[5.074321,52.047386],[5.075321,52.046360],[5.077321,52.046293],[5.078321,52.047273],[5.079321,52.046325],[5.080321,52.047249],[5.082321,52.046259],[5.084321,52.046217],[5.085321,52.047210],[5.089321,52.046295],[5.090321,52.047254],[5.091321,52.046299],[5.093321,52.046308],[5.094023,52.048236],[5.098321,52.046241],[5.099150,52.049108],[5.101179,52.051079],[5.102321,52.050145],[5.104104,52.050154],[5.106321,52.050234],[5.107564,52.051180],[5.107902,52.048518],[5.111321,52.048247],[5.112321,52.046211],[5.113156,52.049103],[5.114082,52.049176],[5.114885,52.046501],[5.118321,52.046159],[5.120321,52.049158],[5.123321,52.049170],[5.124321,52.048185],[5.125321,52.049180],[5.137321,52.049242],[5.138321,52.050250],[5.141321,52.050274],[5.142321,52.048247],[5.145321,52.052234],[5.146321,52.051283],[5.147321,52.052271],[5.153321,52.052233],[5.155531,52.051147],[5.155730,52.048937],[5.156715,52.048543],[5.156054,52.053937],[5.159038,52.054937],[5.158321,52.055654],[5.155321,52.055673],[5.154089,52.057169],[5.157321,52.057302],[5.159942,52.058937],[5.157321,52.059570],[5.155921,52.058337],[5.152166,52.058782],[5.151716,52.063332],[5.151269,52.063989],[5.149185,52.063801],[5.149130,52.065937],[5.151202,52.068057],[5.152321,52.068114],[5.157733,52.064349],[5.159115,52.064937],[5.158940,52.066556],[5.157196,52.066812],[5.153964,52.069580],[5.152204,52.069820],[5.151829,52.070937],[5.161805,52.079453],[5.163076,52.087937],[5.161705,52.090937],[5.163080,52.091937],[5.160754,52.092937],[5.160613,52.094229],[5.156863,52.097479],[5.157778,52.099394],[5.154859,52.100475],[5.153664,52.102937],[5.154321,52.103593],[5.156321,52.103359],[5.156893,52.104365],[5.161843,52.104937],[5.157321,52.105503],[5.156749,52.104509],[5.154046,52.104662],[5.152666,52.108282],[5.148745,52.113361],[5.148560,52.117176],[5.146688,52.117304],[5.147321,52.119762],[5.149593,52.117937],[5.149479,52.119095],[5.146387,52.121003],[5.146353,52.122905],[5.147322,52.122936],[5.147321,52.124034],[5.147316,52.122942],[5.146303,52.122937],[5.145994,52.120264],[5.142638,52.119937],[5.146082,52.119698],[5.146024,52.116937],[5.147915,52.113343],[5.145043,52.113215],[5.142321,52.111476],[5.142256,52.110872],[5.144917,52.109937],[5.142059,52.108199],[5.141321,52.106359],[5.140321,52.109126],[5.134001,52.107257],[5.131323,52.107935],[5.129321,52.104613],[5.127464,52.105937],[5.127001,52.105257],[5.124321,52.105052],[5.121321,52.105478],[5.120705,52.106321],[5.114321,52.106650],[5.112106,52.105937],[5.111321,52.104541],[5.110128,52.107937],[5.109321,52.108434],[5.108009,52.107625],[5.107321,52.109550],[5.105050,52.109208],[5.103321,52.106732],[5.101321,52.106806],[5.099613,52.107937],[5.099321,52.109550],[5.094522,52.108736],[5.094321,52.107870],[5.092536,52.110152],[5.090344,52.110960],[5.079114,52.113144],[5.078884,52.111937],[5.080159,52.110937],[5.079285,52.109937],[5.081850,52.108466],[5.083025,52.106641],[5.087321,52.107423],[5.089635,52.105623],[5.085321,52.103302],[5.084321,52.104616],[5.082879,52.104495],[5.082601,52.106217],[5.079350,52.105966],[5.078321,52.107218],[5.077321,52.107047],[5.077321,52.104012],[5.078321,52.104623],[5.078929,52.103937],[5.078321,52.103167],[5.077321,52.103618],[5.077321,52.102764],[5.078312,52.101928],[5.080321,52.102151],[5.081321,52.100617],[5.082321,52.101396],[5.082798,52.099937],[5.079321,52.099443],[5.078321,52.100514],[5.077321,52.098449],[5.076321,52.099214],[5.071321,52.098889],[5.071321,52.095681],[5.070165,52.098781],[5.061847,52.106463],[5.060235,52.106851],[5.055330,52.110937],[5.059321,52.112547],[5.062321,52.109518],[5.063768,52.109490],[5.063820,52.111937],[5.061808,52.112424],[5.057767,52.116937],[5.058695,52.118311],[5.056869,52.118485],[5.056387,52.119871],[5.058321,52.119783],[5.058970,52.118586],[5.061220,52.117836],[5.064005,52.117937],[5.062427,52.118043],[5.062321,52.119101],[5.060321,52.120161],[5.056282,52.119976],[5.054321,52.118230],[5.050361,52.118977],[5.050023,52.117236],[5.047501,52.117117],[5.047321,52.117767],[5.047321,52.116171],[5.049321,52.114042],[5.049475,52.114783],[5.052321,52.114542],[5.053321,52.115561],[5.055665,52.115281],[5.055612,52.113646],[5.053321,52.111551],[5.049195,52.112811],[5.049213,52.113829],[5.046321,52.113771],[5.043321,52.115487],[5.041321,52.115474],[5.035871,52.118487],[5.038321,52.121583],[5.038623,52.119937],[5.039353,52.119905],[5.040321,52.120732],[5.041756,52.119937],[5.042051,52.120937],[5.038526,52.122142],[5.038415,52.122937],[5.038489,52.123769],[5.039657,52.123937],[5.041389,52.125870],[5.041477,52.126781],[5.042321,52.126913],[5.042321,52.127328]],"type":"LineString"},"type":"Feature"}],"type":"FeatureCollection"})";
    try_isochrone(loki_worker, thor_worker, request, expected);
  }

  {
    const auto request =
        R"({"locations":[{"lat":52.078937,"lon":5.115321}],"costing":"bicycle","costing_options":{"bicycle":{"service_penalty":0}},"contours":[{"time":15}],"polygons":true,"denoise":0.2})";
    const auto expected =
        R"({"features":[{"properties":{"fill-opacity":0.33,"fillColor":"#bf4040","opacity":0.33,"fill":"#bf4040","fillOpacity":0.33,"color":"#bf4040","contour":15,"metric":"time"},"geometry":{"coordinates":[[[5.108321,52.106133],[5.105321,52.106242],[5.103227,52.105031],[5.101321,52.105126],[5.100321,52.104612],[5.098321,52.104950],[5.094681,52.103937],[5.094321,52.101694],[5.095785,52.101401],[5.098321,52.098534],[5.100321,52.098287],[5.101321,52.097284],[5.103321,52.097293],[5.103321,52.095801],[5.103029,52.096645],[5.102321,52.096791],[5.095321,52.096753],[5.094321,52.097731],[5.088081,52.097697],[5.087983,52.098275],[5.089502,52.098756],[5.089643,52.099615],[5.090729,52.099937],[5.089321,52.100487],[5.088321,52.100213],[5.087321,52.101688],[5.085751,52.100937],[5.085321,52.099826],[5.082580,52.098937],[5.085321,52.098574],[5.085732,52.097937],[5.085321,52.097544],[5.081321,52.097320],[5.080051,52.097937],[5.080203,52.096819],[5.081875,52.095937],[5.079321,52.095542],[5.078765,52.094937],[5.079911,52.093527],[5.081638,52.092937],[5.078917,52.092342],[5.078764,52.090937],[5.079917,52.088533],[5.081712,52.087937],[5.081321,52.087147],[5.080254,52.087004],[5.079886,52.083937],[5.081540,52.082156],[5.081661,52.079937],[5.080818,52.079440],[5.078731,52.079347],[5.077896,52.079512],[5.077321,52.080426],[5.076321,52.080581],[5.076165,52.079937],[5.077321,52.078611],[5.082321,52.078291],[5.082568,52.077184],[5.084321,52.075779],[5.084576,52.074682],[5.082885,52.073937],[5.084614,52.071644],[5.082187,52.071803],[5.081899,52.072515],[5.079135,52.074751],[5.078965,52.075293],[5.079752,52.075937],[5.078542,52.077158],[5.077321,52.077268],[5.075321,52.075447],[5.070758,52.074937],[5.071321,52.074578],[5.074321,52.074450],[5.074954,52.073570],[5.076549,52.073165],[5.076575,52.071683],[5.075321,52.071450],[5.074808,52.070937],[5.074994,52.069610],[5.075984,52.068937],[5.075914,52.065530],[5.081321,52.065241],[5.084070,52.062686],[5.086663,52.062279],[5.087321,52.061429],[5.089321,52.061093],[5.089444,52.062060],[5.088578,52.063194],[5.088887,52.064371],[5.090321,52.064512],[5.091321,52.063497],[5.092724,52.063340],[5.093321,52.062497],[5.094321,52.064097],[5.094649,52.063265],[5.095460,52.063076],[5.095733,52.062349],[5.100727,52.058343],[5.101298,52.056914],[5.102338,52.056920],[5.102894,52.057364],[5.104321,52.056186],[5.106380,52.055996],[5.105321,52.054503],[5.104321,52.054945],[5.103599,52.054659],[5.103433,52.053825],[5.102665,52.053593],[5.102435,52.052823],[5.100981,52.052277],[5.100321,52.051343],[5.096519,52.051135],[5.096321,52.051628],[5.096115,52.051143],[5.097017,52.050633],[5.102321,52.050477],[5.104321,52.048435],[5.104894,52.050364],[5.106321,52.050394],[5.107706,52.051322],[5.108321,52.050495],[5.111913,52.049529],[5.112321,52.049087],[5.114310,52.050937],[5.113321,52.051958],[5.112934,52.051324],[5.111029,52.051645],[5.112002,52.052937],[5.112126,52.055132],[5.114321,52.055681],[5.115482,52.056776],[5.121321,52.057218],[5.123321,52.059146],[5.125321,52.059215],[5.126321,52.058232],[5.127519,52.059135],[5.128186,52.056802],[5.131812,52.056428],[5.132321,52.056061],[5.136323,52.056935],[5.137321,52.057547],[5.139321,52.057085],[5.139895,52.058363],[5.141624,52.059634],[5.141735,52.060937],[5.142808,52.061450],[5.144321,52.063187],[5.150021,52.057637],[5.151782,52.057398],[5.152321,52.056021],[5.152321,52.058197],[5.150823,52.058439],[5.147866,52.062482],[5.146158,52.063774],[5.145868,52.065484],[5.144321,52.065722],[5.144200,52.066058],[5.145905,52.066353],[5.146321,52.067168],[5.147530,52.067146],[5.148321,52.063899],[5.148740,52.066518],[5.151091,52.068167],[5.152321,52.068296],[5.153321,52.067337],[5.154655,52.067271],[5.156321,52.065622],[5.158078,52.065937],[5.156855,52.066471],[5.155643,52.068259],[5.154321,52.068486],[5.152914,52.069937],[5.153321,52.070437],[5.157335,52.070937],[5.152321,52.071514],[5.151321,52.070638],[5.149107,52.072937],[5.151746,52.073512],[5.152321,52.075389],[5.153321,52.073865],[5.153943,52.075315],[5.155525,52.075734],[5.155380,52.077996],[5.153321,52.076444],[5.151876,52.077937],[5.153321,52.079469],[5.154548,52.079710],[5.155321,52.080761],[5.156130,52.080937],[5.156124,52.082134],[5.156835,52.082937],[5.156094,52.083164],[5.155658,52.082601],[5.154034,52.082224],[5.153670,52.081589],[5.152321,52.081506],[5.151583,52.080675],[5.149878,52.080380],[5.149526,52.079732],[5.148963,52.079937],[5.149164,52.081094],[5.151126,52.083132],[5.153768,52.083490],[5.153716,52.085937],[5.154647,52.087611],[5.155630,52.087937],[5.154321,52.089447],[5.151321,52.089430],[5.150858,52.089937],[5.151321,52.090504],[5.154088,52.090937],[5.153601,52.091217],[5.153321,52.093413],[5.152232,52.094026],[5.150321,52.093097],[5.149321,52.093808],[5.148321,52.093005],[5.147924,52.093540],[5.148321,52.094324],[5.149321,52.094187],[5.149858,52.095400],[5.151321,52.095321],[5.151894,52.097364],[5.154121,52.097937],[5.153491,52.098107],[5.153321,52.098841],[5.152974,52.098284],[5.152321,52.098260],[5.150908,52.098524],[5.150321,52.099513],[5.147207,52.099051],[5.146821,52.098937],[5.148732,52.098348],[5.148625,52.097634],[5.145321,52.097622],[5.144793,52.097937],[5.146181,52.098937],[5.145730,52.099937],[5.146519,52.100937],[5.146321,52.101629],[5.145019,52.101240],[5.144912,52.099937],[5.144321,52.099476],[5.142828,52.100937],[5.143321,52.101667],[5.144485,52.101937],[5.142656,52.102937],[5.140321,52.102566],[5.140038,52.102220],[5.137321,52.102151],[5.136321,52.103072],[5.135895,52.102363],[5.134155,52.102103],[5.133783,52.099937],[5.134321,52.099378],[5.135598,52.099214],[5.135580,52.096937],[5.134666,52.096592],[5.134321,52.095878],[5.132462,52.096078],[5.132120,52.096736],[5.130321,52.097066],[5.129321,52.098041],[5.128321,52.098234],[5.126988,52.099604],[5.124891,52.099367],[5.124313,52.098929],[5.124334,52.102937],[5.120321,52.103242],[5.119321,52.102682],[5.116321,52.105625],[5.115321,52.104350],[5.114321,52.104814],[5.112321,52.104898],[5.111321,52.104483],[5.110652,52.105268],[5.108321,52.106133]]],"type":"Polygon"},"type":"Feature"}],"type":"FeatureCollection"})";
    try_isochrone(loki_worker, thor_worker, request, expected);
  }

  {
    const auto request =
        R"({"locations":[{"lat":52.078937,"lon":5.115321}],"costing":"bicycle","costing_options":{"bicycle":{"service_penalty":0}},"contours":[{"time":15}],"show_locations":true})";
    const auto expected =
        R"({"features":[{"properties":{"fill-opacity":0.33,"fillColor":"#bf4040","opacity":0.33,"fill":"#bf4040","fillOpacity":0.33,"color":"#bf4040","contour":15,"metric":"time"},"geometry":{"coordinates":[[5.108321,52.106133],[5.105321,52.106242],[5.103227,52.105031],[5.101321,52.105126],[5.100321,52.104612],[5.098321,52.104950],[5.094681,52.103937],[5.094321,52.101694],[5.095785,52.101401],[5.098321,52.098534],[5.100321,52.098287],[5.101321,52.097284],[5.103321,52.097293],[5.103321,52.095801],[5.103029,52.096645],[5.102321,52.096791],[5.095321,52.096753],[5.094321,52.097731],[5.088081,52.097697],[5.087983,52.098275],[5.089502,52.098756],[5.089643,52.099615],[5.090729,52.099937],[5.089321,52.100487],[5.088321,52.100213],[5.087321,52.101688],[5.085751,52.100937],[5.085321,52.099826],[5.082580,52.098937],[5.085321,52.098574],[5.085732,52.097937],[5.085321,52.097544],[5.081321,52.097320],[5.080051,52.097937],[5.080203,52.096819],[5.081875,52.095937],[5.079321,52.095542],[5.078765,52.094937],[5.079911,52.093527],[5.081638,52.092937],[5.078917,52.092342],[5.078764,52.090937],[5.079917,52.088533],[5.081712,52.087937],[5.081321,52.087147],[5.080254,52.087004],[5.079886,52.083937],[5.081540,52.082156],[5.081661,52.079937],[5.080818,52.079440],[5.078731,52.079347],[5.077896,52.079512],[5.077321,52.080426],[5.076321,52.080581],[5.076165,52.079937],[5.077321,52.078611],[5.082321,52.078291],[5.082568,52.077184],[5.084321,52.075779],[5.084576,52.074682],[5.082885,52.073937],[5.084614,52.071644],[5.082187,52.071803],[5.081899,52.072515],[5.079135,52.074751],[5.078965,52.075293],[5.079752,52.075937],[5.078542,52.077158],[5.077321,52.077268],[5.075321,52.075447],[5.070758,52.074937],[5.071321,52.074578],[5.074321,52.074450],[5.074954,52.073570],[5.076549,52.073165],[5.076575,52.071683],[5.075321,52.071450],[5.074808,52.070937],[5.074994,52.069610],[5.075984,52.068937],[5.075914,52.065530],[5.081321,52.065241],[5.084070,52.062686],[5.086663,52.062279],[5.087321,52.061429],[5.089321,52.061093],[5.089444,52.062060],[5.088578,52.063194],[5.088887,52.064371],[5.090321,52.064512],[5.091321,52.063497],[5.092724,52.063340],[5.093321,52.062497],[5.094321,52.064097],[5.094649,52.063265],[5.095460,52.063076],[5.095733,52.062349],[5.100727,52.058343],[5.101298,52.056914],[5.102338,52.056920],[5.102894,52.057364],[5.104321,52.056186],[5.106380,52.055996],[5.105321,52.054503],[5.104321,52.054945],[5.103599,52.054659],[5.103433,52.053825],[5.102665,52.053593],[5.102435,52.052823],[5.100981,52.052277],[5.100321,52.051343],[5.096519,52.051135],[5.096321,52.051628],[5.096115,52.051143],[5.097017,52.050633],[5.102321,52.050477],[5.104321,52.048435],[5.104894,52.050364],[5.106321,52.050394],[5.107706,52.051322],[5.108321,52.050495],[5.111913,52.049529],[5.112321,52.049087],[5.114310,52.050937],[5.113321,52.051958],[5.112934,52.051324],[5.111029,52.051645],[5.112002,52.052937],[5.112126,52.055132],[5.114321,52.055681],[5.115482,52.056776],[5.121321,52.057218],[5.123321,52.059146],[5.125321,52.059215],[5.126321,52.058232],[5.127519,52.059135],[5.128186,52.056802],[5.131812,52.056428],[5.132321,52.056061],[5.136323,52.056935],[5.137321,52.057547],[5.139321,52.057085],[5.139895,52.058363],[5.141624,52.059634],[5.141735,52.060937],[5.142808,52.061450],[5.144321,52.063187],[5.150021,52.057637],[5.151782,52.057398],[5.152321,52.056021],[5.152321,52.058197],[5.150823,52.058439],[5.147866,52.062482],[5.146158,52.063774],[5.145868,52.065484],[5.144321,52.065722],[5.144200,52.066058],[5.145905,52.066353],[5.146321,52.067168],[5.147530,52.067146],[5.148321,52.063899],[5.148740,52.066518],[5.151091,52.068167],[5.152321,52.068296],[5.153321,52.067337],[5.154655,52.067271],[5.156321,52.065622],[5.158078,52.065937],[5.156855,52.066471],[5.155643,52.068259],[5.154321,52.068486],[5.152914,52.069937],[5.153321,52.070437],[5.157335,52.070937],[5.152321,52.071514],[5.151321,52.070638],[5.149107,52.072937],[5.151746,52.073512],[5.152321,52.075389],[5.153321,52.073865],[5.153943,52.075315],[5.155525,52.075734],[5.155380,52.077996],[5.153321,52.076444],[5.151876,52.077937],[5.153321,52.079469],[5.154548,52.079710],[5.155321,52.080761],[5.156130,52.080937],[5.156124,52.082134],[5.156835,52.082937],[5.156094,52.083164],[5.155658,52.082601],[5.154034,52.082224],[5.153670,52.081589],[5.152321,52.081506],[5.151583,52.080675],[5.149878,52.080380],[5.149526,52.079732],[5.148963,52.079937],[5.149164,52.081094],[5.151126,52.083132],[5.153768,52.083490],[5.153716,52.085937],[5.154647,52.087611],[5.155630,52.087937],[5.154321,52.089447],[5.151321,52.089430],[5.150858,52.089937],[5.151321,52.090504],[5.154088,52.090937],[5.153601,52.091217],[5.153321,52.093413],[5.152232,52.094026],[5.150321,52.093097],[5.149321,52.093808],[5.148321,52.093005],[5.147924,52.093540],[5.148321,52.094324],[5.149321,52.094187],[5.149858,52.095400],[5.151321,52.095321],[5.151894,52.097364],[5.154121,52.097937],[5.153491,52.098107],[5.153321,52.098841],[5.152974,52.098284],[5.152321,52.098260],[5.150908,52.098524],[5.150321,52.099513],[5.147207,52.099051],[5.146821,52.098937],[5.148732,52.098348],[5.148625,52.097634],[5.145321,52.097622],[5.144793,52.097937],[5.146181,52.098937],[5.145730,52.099937],[5.146519,52.100937],[5.146321,52.101629],[5.145019,52.101240],[5.144912,52.099937],[5.144321,52.099476],[5.142828,52.100937],[5.143321,52.101667],[5.144485,52.101937],[5.142656,52.102937],[5.140321,52.102566],[5.140038,52.102220],[5.137321,52.102151],[5.136321,52.103072],[5.135895,52.102363],[5.134155,52.102103],[5.133783,52.099937],[5.134321,52.099378],[5.135598,52.099214],[5.135580,52.096937],[5.134666,52.096592],[5.134321,52.095878],[5.132462,52.096078],[5.132120,52.096736],[5.130321,52.097066],[5.129321,52.098041],[5.128321,52.098234],[5.126988,52.099604],[5.124891,52.099367],[5.124313,52.098929],[5.124334,52.102937],[5.120321,52.103242],[5.119321,52.102682],[5.116321,52.105625],[5.115321,52.104350],[5.114321,52.104814],[5.112321,52.104898],[5.111321,52.104483],[5.110652,52.105268],[5.108321,52.106133]],"type":"LineString"},"type":"Feature"},{"geometry":{"coordinates":[[5.115328,52.078940]],"type":"MultiPoint"},"properties":{"location_index":0,"type":"snapped"},"type":"Feature"},{"geometry":{"coordinates":[5.115321,52.078937],"type":"Point"},"properties":{"location_index":0,"type":"input"},"type":"Feature"}],"type":"FeatureCollection"})";
    try_isochrone(loki_worker, thor_worker, request, expected);
  }
}

TEST(Isochrones, OriginEdge) {
  const std::string ascii_map = R"(
       a-b-c
     )";

  const gurka::ways ways = {
      {"abc", {{"highway", "primary"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 2000);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/isochrones/origin_edge");

  std::string geojson;
  auto result = gurka::do_action(valhalla::Options::isochrone, map, {"b"}, "pedestrian",
                                 {{"/contours/0/time", "10"}}, {}, &geojson);
  std::vector<PointLL> iso_polygon = polygon_from_geojson(geojson);

  auto WaypointToBoostPoint = [&](std::string waypoint) {
    auto point = map.nodes[waypoint];
    return point_type(point.x(), point.y());
  };
  polygon_type polygon;
  for (const auto& p : iso_polygon) {
    boost::geometry::append(polygon.outer(), point_type(p.x(), p.y()));
  }
  EXPECT_EQ(within(WaypointToBoostPoint("b"), polygon), true);
  EXPECT_EQ(within(WaypointToBoostPoint("a"), polygon), false);
  EXPECT_EQ(within(WaypointToBoostPoint("c"), polygon), false);
}

TEST(Isochrones, LongEdge) {
  const std::string ascii_map = R"(
          c----d
         /
      a-b--------------f
    )";

  const gurka::ways ways = {
      {"ab", {{"highway", "primary"}}},
      {"bc", {{"highway", "primary"}}},
      {"cd", {{"highway", "primary"}}},
      {"bf", {{"highway", "primary"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/isochrones/long_edge");

  std::string geojson;
  auto result = gurka::do_action(valhalla::Options::isochrone, map, {"a"}, "pedestrian",
                                 {{"/contours/0/time", "15"}}, {}, &geojson);
  std::vector<PointLL> iso_polygon = polygon_from_geojson(geojson);

  auto WaypointToBoostPoint = [&](std::string waypoint) {
    auto point = map.nodes[waypoint];
    return point_type(point.x(), point.y());
  };
  polygon_type polygon;
  for (const auto& p : iso_polygon) {
    boost::geometry::append(polygon.outer(), point_type(p.x(), p.y()));
  }
  EXPECT_EQ(within(WaypointToBoostPoint("a"), polygon), true);
  EXPECT_EQ(within(WaypointToBoostPoint("b"), polygon), true);
  EXPECT_EQ(within(WaypointToBoostPoint("c"), polygon), true);
  EXPECT_EQ(within(WaypointToBoostPoint("d"), polygon), true);
  EXPECT_EQ(within(WaypointToBoostPoint("f"), polygon), false);

  // check that b-f edges is visited and is partially within the isochrone
  auto interpolated = map.nodes["b"].PointAlongSegment(map.nodes["f"], 0.4);
  EXPECT_EQ(within(point_type(interpolated.x(), interpolated.y()), polygon), true);
}

class IsochroneTest : public thor::Isochrone {
public:
  explicit IsochroneTest(const boost::property_tree::ptree& config = {}) : Isochrone(config) {
  }

  void Clear() {
    Isochrone::Clear();
    if (clear_reserved_memory_) {
      EXPECT_EQ(bdedgelabels_.capacity(), 0);
      EXPECT_EQ(mmedgelabels_.capacity(), 0);
    } else {
      EXPECT_LE(bdedgelabels_.capacity(), max_reserved_labels_count_);
      EXPECT_LE(mmedgelabels_.capacity(), max_reserved_labels_count_);
    }
  }
};

TEST(Isochrones, test_clear_reserved_memory) {
  boost::property_tree::ptree config;
  config.put("clear_reserved_memory", true);

  IsochroneTest isochrone(config);
  isochrone.Clear();
}

TEST(Isochrones, test_max_reserved_labels_count) {
  boost::property_tree::ptree config;
  config.put("max_reserved_labels_count_dijkstras", 10);

  IsochroneTest isochrone(config);
  isochrone.Clear();
}

} // namespace

int main(int argc, char* argv[]) {
  // user wants to try it
  if (argc > 1) {
    loki_worker_t loki_worker(cfg);
    thor_worker_t thor_worker(cfg);
    GraphReader reader(cfg.get_child("mjolnir"));
    Api request;
    ParseApi(argv[1], Options::isochrone, request);
    loki_worker.isochrones(request);
    std::cout << thor_worker.isochrones(request) << std::endl;
    return EXIT_SUCCESS;
  }
  // Silence logs (especially long request logging)
  logging::Configure({{"type", ""}});
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
