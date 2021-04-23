#include "gurka.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/multi/geometries/register/multi_polygon.hpp>
#include <gtest/gtest.h>

#include "baldr/graphconstants.h"
#include "baldr/graphreader.h"
#include "loki/polygon_search.h"
#include "midgard/pointll.h"
#include "mjolnir/graphtilebuilder.h"
#include "worker.h"

using namespace valhalla;
namespace bg = boost::geometry;
namespace vm = valhalla::midgard;
namespace vl = valhalla::loki;

namespace {
// register a few boost.geometry types
using ring_bg_t = std::vector<vm::PointLL>;

rapidjson::Value get_chinese_polygon(ring_bg_t ring, rapidjson::MemoryPoolAllocator<>& allocator) {
  rapidjson::Value ring_j(rapidjson::kArrayType);
  for (auto& coord : ring) {
    rapidjson::Value coords(rapidjson::kArrayType);
    coords.PushBack(coord.lng(), allocator);
    coords.PushBack(coord.lat(), allocator);
    ring_j.PushBack(coords, allocator);
  }

  return ring_j;
}

// common method can't deal with arrays of floats
std::string build_local_req(rapidjson::Document& doc,
                            rapidjson::MemoryPoolAllocator<>& allocator,
                            const std::vector<midgard::PointLL>& waypoints,
                            const std::string& costing,
                            const rapidjson::Value& geom_obj,
                            const std::string& type) {

  rapidjson::Value locations(rapidjson::kArrayType);
  for (const auto& waypoint : waypoints) {
    rapidjson::Value p(rapidjson::kObjectType);
    p.AddMember("lon", waypoint.lng(), allocator);
    p.AddMember("lat", waypoint.lat(), allocator);
    locations.PushBack(p, allocator);
  }

  doc.AddMember("locations", locations, allocator);
  doc.AddMember("costing", costing, allocator);

  if (type == "chinese_polygon") {
    rapidjson::SetValueByPointer(doc, "/chinese_polygon", geom_obj);
  } else {
    rapidjson::SetValueByPointer(doc, "/chinese_polygon", geom_obj);
  }

  rapidjson::StringBuffer sb;
  rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
  doc.Accept(writer);
  return sb.GetString();
}
} // namespace

// parameterized test class to test all costings
class ChinesePostmanTest : public ::testing::TestWithParam<std::string> {
protected:
  static gurka::map chinese_postman_map;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
        A------B------C
        |      |    / |
        |      |   /  |
        |      |  /   |
        |      | /    |
        D------E------F
    )";
    const gurka::ways ways = {{"AB", {{"highway", "residential"}, {"name", "High"}}},
                              {"BC", {{"highway", "residential"}, {"name", "Low"}}},
                              {"AA", {{"highway", "residential"}, {"name", "1st"}}},
                              {"BE", {{"highway", "residential"}, {"name", "2nd"}}},
                              {"CE", {{"highway", "residential"}, {"name", "3rd"}}},
                              {"CF", {{"highway", "residential"}, {"name", "4th"}}},
                              {"DE", {{"highway", "residential"}, {"name", "5th"}}},
                              {"EF", {{"highway", "residential"}, {"name", "6th"}}}};
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
    // Add low length limit for avoid_polygons so it throws an error
    chinese_postman_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_chinese_postman",
                                            {{"service_limits.max_avoid_polygons_length", "1000"}});
  }
};

gurka::map ChinesePostmanTest::chinese_postman_map = {};

TEST_P(ChinesePostmanTest, TestChinesePostmanSimple) {
  auto node_a = chinese_postman_map.nodes.at("A");
  auto node_b = chinese_postman_map.nodes.at("B");
  auto node_c = chinese_postman_map.nodes.at("C");
  auto node_d = chinese_postman_map.nodes.at("D");
  auto node_e = chinese_postman_map.nodes.at("E");
  auto node_f = chinese_postman_map.nodes.at("F");

  auto dx = node_c.lng() - node_b.lng();
  auto dy = node_a.lat() - node_d.lat();

  // create a polygon covering ABDE
  //   x-------------x
  //   |  A------B---|--C
  //   |  |      |   | /|
  //   |  |      |   |/ |
  //   |  |      |  /|  |
  //   |  |      | / |  |
  //   |  D------E---|--F
  //   x-------------x
  //
  auto ratio = 0.1;
  ring_bg_t ring{{node_a.lng() - ratio * dx, node_a.lat() + ratio * dy},
                 {node_b.lng() + ratio * dx, node_b.lat() + ratio * dy},
                 {node_e.lng() + ratio * dx, node_e.lat() - ratio * dy},
                 {node_d.lng() - ratio * dx, node_d.lat() - ratio * dy},
                 {node_a.lng() - ratio * dx, node_a.lat() + ratio * dy}};

  // build request manually for now
  auto lls = {chinese_postman_map.nodes["A"], chinese_postman_map.nodes["A"]};

  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  auto value = get_chinese_polygon(ring, allocator);
  auto type = "chinese_polygon";
  auto req = build_local_req(doc, allocator, lls, GetParam(), value, type);

  auto route = gurka::do_action(Options::chinese_postman, chinese_postman_map, req);
  // gurka::assert::raw::expect_path(route, {"High", "Low", "5th", "2nd"});
}

INSTANTIATE_TEST_SUITE_P(ChinesePostmanProfilesTest, ChinesePostmanTest, ::testing::Values("auto"));
