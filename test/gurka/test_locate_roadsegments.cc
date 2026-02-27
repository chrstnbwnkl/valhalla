#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "gurka.h"
#include "test.h"

#include <gtest/gtest.h>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {
void check_shape(const std::string& shape,
                 const gurka::nodelayout& layout,
                 const std::vector<std::string>& nodes) {
  std::vector<PointLL> expected_shape;

  expected_shape.reserve(nodes.size());
  for (const auto& node : nodes) {
    expected_shape.push_back(layout.at(node));
  }

  // auto decoded = decode<std::vector<PointLL>>(shape.data(), shape.size(), 6);

  auto expected_encoded = encode(expected_shape);

  EXPECT_EQ(shape, expected_encoded) << "Shapes differ";
}
} // namespace

TEST(Standalone, base_test) {
  const std::string ascii_map = R"(
      q  1                   2       o
      |  |                   |       |
      m--A----B----C-x--D----E----F--n
      |  |                   |       |
      p  3                   4       r
  )";

  // clang-format off
  const gurka::ways ways = {
    {"AB", {{"highway", "residential"}}},
    {"BC", {{"highway", "residential"}}},
    {"CD", {{"highway", "residential"}}},
    {"DE", {{"highway", "residential"}}},
    {"EF", {{"highway", "residential"}}}, 
    {"1A", {{"highway", "service"}}},
    {"3A", {{"highway", "service"}}},
    {"2E", {{"highway", "service"}}},
    {"4E", {{"highway", "service"}}},
    {"Am", {{"highway", "residential"}}},
    {"mq", {{"highway", "service"}}},
    // some bogus ones to avoid deadends
    {"mp", {{"highway", "service"}}},
    {"Fn", {{"highway", "residential"}}},
    {"no", {{"highway", "service"}}},
    {"nr", {{"highway", "service"}}},
  };
  // clang-format on

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_road_segments");
  std::string json;
  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  auto r = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  auto result = gurka::do_action(valhalla::Options::locate, map, {"x"}, "auto",
                                 {{"/verbose", "0"}, {"/road_segments", "0"}}, r, &json);
  rapidjson::Document response;
  response.Parse(json);
  if (response.HasParseError())
    throw std::runtime_error("bad json response");
  auto expected_outer_id_1 = gurka::findNode(reader, layout, "A");
  auto expected_outer_id_2 = gurka::findNode(reader, layout, "E");
  const auto* expected_outer_1 = reader.nodeinfo(expected_outer_id_1);
  const auto* expected_outer_2 = reader.nodeinfo(expected_outer_id_2);
  auto start_node_id = static_cast<uint64_t>(
      rapidjson::Pointer("/0/edges/0/full_road_segment/intersections/start_node/id/value")
          .Get(response)
          ->GetInt64());

  auto end_node_id = static_cast<uint64_t>(
      rapidjson::Pointer("/0/edges/0/full_road_segment/intersections/end_node/id/value")
          .Get(response)
          ->GetInt64());

  // edge 1
  {
    auto start_node_id = static_cast<uint64_t>(
        rapidjson::Pointer("/0/edges/0/full_road_segment/intersections/start_node/id/value")
            .Get(response)
            ->GetInt64());

    auto end_node_id = static_cast<uint64_t>(
        rapidjson::Pointer("/0/edges/0/full_road_segment/intersections/end_node/id/value")
            .Get(response)
            ->GetInt64());
    EXPECT_EQ(expected_outer_id_1.value, start_node_id);
    EXPECT_EQ(expected_outer_id_2.value, end_node_id);
    // shape
    auto shape = rapidjson::Pointer("/0/edges/0/full_road_segment/shape").Get(response)->GetString();
    check_shape(shape, layout, {"A", "B", "C", "D", "E"});
    auto deadend_fwd =
        rapidjson::Pointer("/0/edges/0/full_road_segment/intersections/end_node/deadend")
            .Get(response)
            ->GetBool();

    EXPECT_FALSE(deadend_fwd);
    auto deadend_bwd =
        rapidjson::Pointer("/0/edges/0/full_road_segment/intersections/start_node/deadend")
            .Get(response)
            ->GetBool();

    EXPECT_FALSE(deadend_bwd);
  }

  // edge 2
  {
    auto start_node_id = static_cast<uint64_t>(
        rapidjson::Pointer("/0/edges/1/full_road_segment/intersections/start_node/id/value")
            .Get(response)
            ->GetInt64());

    auto end_node_id = static_cast<uint64_t>(
        rapidjson::Pointer("/0/edges/1/full_road_segment/intersections/end_node/id/value")
            .Get(response)
            ->GetInt64());
    EXPECT_EQ(expected_outer_id_2.value, start_node_id);
    EXPECT_EQ(expected_outer_id_1.value, end_node_id);
    // shape
    auto shape = rapidjson::Pointer("/0/edges/1/full_road_segment/shape").Get(response)->GetString();
    check_shape(shape, layout, {"E", "D", "C", "B", "A"});
  }
}

TEST(Standalone, base_test_include_driveways) {
  const std::string ascii_map = R"(
      q  1    5              2       o
      |  |    |              |       |
      m--A----B----C-x--D----E----F--n
      |  |    |              |       |
      p  3    6              4       r
  )";

  // clang-format off
  const gurka::ways ways = {
    {"AB", {{"highway", "residential"}}},
    {"BC", {{"highway", "residential"}}},
    {"CD", {{"highway", "residential"}}},
    {"DE", {{"highway", "residential"}}},
    {"EF", {{"highway", "residential"}}}, 
    {"1A", {{"highway", "service"}}},
    {"3A", {{"highway", "service"}}},
    {"B5", {{"highway", "service"}, {"service", "driveway"}}},
    {"B6", {{"highway", "service"}, {"service", "driveway"}}},
    {"2E", {{"highway", "service"}}},
    {"4E", {{"highway", "service"}}},
    {"Am", {{"highway", "residential"}}},
    {"mq", {{"highway", "service"}}},
    // some bogus ones to avoid deadends
    {"mp", {{"highway", "service"}}},
    {"Fn", {{"highway", "residential"}}},
    {"no", {{"highway", "service"}}},
    {"nr", {{"highway", "service"}}},
  };
  // clang-format on

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_road_segments",
                               {{"mjolnir.include_driveways", "true"},
                                {"mjolnir.remove_all_driveways", "false"}});
  std::string json;
  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  auto r = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  auto result = gurka::do_action(valhalla::Options::locate, map, {"x"}, "auto",
                                 {{"/verbose", "0"}, {"/road_segments", "0"}}, r, &json);
  rapidjson::Document response;
  response.Parse(json);
  if (response.HasParseError())
    throw std::runtime_error("bad json response");
  auto expected_outer_id_1 = gurka::findNode(reader, layout, "B");
  auto expected_outer_id_2 = gurka::findNode(reader, layout, "E");
  const auto* expected_outer_1 = reader.nodeinfo(expected_outer_id_1);
  const auto* expected_outer_2 = reader.nodeinfo(expected_outer_id_2);
  // edge 1
  {
    auto start_node_id = static_cast<uint64_t>(
        rapidjson::Pointer("/0/edges/0/full_road_segment/intersections/start_node/id/value")
            .Get(response)
            ->GetInt64());

    auto end_node_id = static_cast<uint64_t>(
        rapidjson::Pointer("/0/edges/0/full_road_segment/intersections/end_node/id/value")
            .Get(response)
            ->GetInt64());

    EXPECT_EQ(expected_outer_id_1.value, start_node_id);
    EXPECT_EQ(expected_outer_id_2.value, end_node_id);

    // shape
    auto shape = rapidjson::Pointer("/0/edges/0/full_road_segment/shape").Get(response)->GetString();
    check_shape(shape, layout, {"B", "C", "D", "E"});
    auto deadend_fwd =
        rapidjson::Pointer("/0/edges/0/full_road_segment/intersections/end_node/deadend")
            .Get(response)
            ->GetBool();

    EXPECT_FALSE(deadend_fwd);
    auto deadend_bwd =
        rapidjson::Pointer("/0/edges/0/full_road_segment/intersections/start_node/deadend")
            .Get(response)
            ->GetBool();

    EXPECT_FALSE(deadend_bwd);
  }
  // edge 2
  {
    auto start_node_id = static_cast<uint64_t>(
        rapidjson::Pointer("/0/edges/1/full_road_segment/intersections/start_node/id/value")
            .Get(response)
            ->GetInt64());

    auto end_node_id = static_cast<uint64_t>(
        rapidjson::Pointer("/0/edges/1/full_road_segment/intersections/end_node/id/value")
            .Get(response)
            ->GetInt64());

    EXPECT_EQ(expected_outer_id_2.value, start_node_id);
    EXPECT_EQ(expected_outer_id_1.value, end_node_id);

    // shape
    auto shape = rapidjson::Pointer("/0/edges/1/full_road_segment/shape").Get(response)->GetString();
    check_shape(shape, layout, {"E", "D", "C", "B"});
    auto deadend_fwd =
        rapidjson::Pointer("/0/edges/0/full_road_segment/intersections/end_node/deadend")
            .Get(response)
            ->GetBool();

    EXPECT_FALSE(deadend_fwd);
    auto deadend_bwd =
        rapidjson::Pointer("/0/edges/0/full_road_segment/intersections/start_node/deadend")
            .Get(response)
            ->GetBool();

    EXPECT_FALSE(deadend_bwd);
  }
}

TEST(Standalone, base_test_ignore_driveways) {
  const std::string ascii_map = R"(
      q  1    5              2       o
      |  |    |              |       |
      m--A----B----C-x--D----E----F--n
      |  |    |              |       |
      p  3    6              4       r
  )";

  // clang-format off
  const gurka::ways ways = {
    {"AB", {{"highway", "residential"}}},
    {"BC", {{"highway", "residential"}}},
    {"CD", {{"highway", "residential"}}},
    {"DE", {{"highway", "residential"}}},
    {"EF", {{"highway", "residential"}}}, 
    {"1A", {{"highway", "service"}}},
    {"3A", {{"highway", "service"}}},
    {"B5", {{"highway", "service"}, {"service", "driveway"}}},
    {"B6", {{"highway", "service"}, {"service", "driveway"}}},
    {"2E", {{"highway", "service"}}},
    {"4E", {{"highway", "service"}}},
    {"Am", {{"highway", "residential"}}},
    {"mq", {{"highway", "service"}}},
    // some bogus ones to avoid deadends
    {"mp", {{"highway", "service"}}},
    {"Fn", {{"highway", "residential"}}},
    {"no", {{"highway", "service"}}},
    {"nr", {{"highway", "service"}}},
  };
  // clang-format on

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_road_segments",
                               {{"mjolnir.include_driveways", "false"},
                                {"mjolnir.remove_all_driveways", "true"}});
  std::string json;
  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  auto r = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  auto result = gurka::do_action(valhalla::Options::locate, map, {"x"}, "auto",
                                 {{"/verbose", "0"}, {"/road_segments", "0"}}, r, &json);
  rapidjson::Document response;
  response.Parse(json);
  if (response.HasParseError())
    throw std::runtime_error("bad json response");
  auto expected_outer_id_1 = gurka::findNode(reader, layout, "A");
  auto expected_outer_id_2 = gurka::findNode(reader, layout, "E");
  const auto* expected_outer_1 = reader.nodeinfo(expected_outer_id_1);
  const auto* expected_outer_2 = reader.nodeinfo(expected_outer_id_2);
  auto start_node_id = static_cast<uint64_t>(
      rapidjson::Pointer("/0/edges/0/full_road_segment/intersections/start_node/id/value")
          .Get(response)
          ->GetInt64());

  auto end_node_id = static_cast<uint64_t>(
      rapidjson::Pointer("/0/edges/0/full_road_segment/intersections/end_node/id/value")
          .Get(response)
          ->GetInt64());

  EXPECT_EQ(expected_outer_id_1.value, start_node_id);
  EXPECT_EQ(expected_outer_id_2.value, end_node_id);
  auto deadend_fwd = rapidjson::Pointer("/0/edges/0/full_road_segment/intersections/end_node/deadend")
                         .Get(response)
                         ->GetBool();

  EXPECT_FALSE(deadend_fwd);
  auto deadend_bwd =
      rapidjson::Pointer("/0/edges/0/full_road_segment/intersections/start_node/deadend")
          .Get(response)
          ->GetBool();

  EXPECT_FALSE(deadend_bwd);
}

TEST(Standalone, base_test_search_filter) {
  const std::string ascii_map = R"(
         1    5              2       o
         |    |              |       |
      m--A----B----C-x--D----E----F--n
         |    |              |       |
         3    6              4       r
  )";

  // clang-format off
  const gurka::ways ways = {
    {"AB", {{"highway", "residential"}}},
    {"BC", {{"highway", "residential"}}},
    {"CD", {{"highway", "residential"}}},
    {"DE", {{"highway", "residential"}}},
    {"EF", {{"highway", "residential"}}},
    {"1A", {{"highway", "service"}}},
    {"3A", {{"highway", "service"}}},
    {"B5", {{"highway", "service"}, {"service", "driveway"}}},
    {"B6", {{"highway", "service"}, {"service", "driveway"}}},
    {"2E", {{"highway", "service"}}},
    {"4E", {{"highway", "service"}}},
    {"Am", {{"highway", "residential"}}},
    {"Fn", {{"highway", "residential"}}},
    {"no", {{"highway", "service"}}},
    {"nr", {{"highway", "service"}}},
  };
  // clang-format on

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_road_segments",
                               {{"mjolnir.include_driveways", "false"},
                                {"mjolnir.remove_all_driveways", "true"}});
  std::string json;
  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  auto r = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  auto result = gurka::do_action(valhalla::Options::locate, map, {"x"}, "auto",
                                 {{"/verbose", "0"},
                                  {"/road_segments", "0"},
                                  {"/locations/0/search_filter/use/0", "road"}},
                                 r, &json);
  rapidjson::Document response;
  response.Parse(json);
  if (response.HasParseError())
    throw std::runtime_error("bad json response");
  auto expected_outer_id_1 = gurka::findNode(reader, layout, "m");
  auto expected_outer_id_2 = gurka::findNode(reader, layout, "n");
  const auto* expected_outer_1 = reader.nodeinfo(expected_outer_id_1);
  const auto* expected_outer_2 = reader.nodeinfo(expected_outer_id_2);
  auto start_node_id = static_cast<uint64_t>(
      rapidjson::Pointer("/0/edges/0/full_road_segment/intersections/start_node/id/value")
          .Get(response)
          ->GetInt64());

  auto end_node_id = static_cast<uint64_t>(
      rapidjson::Pointer("/0/edges/0/full_road_segment/intersections/end_node/id/value")
          .Get(response)
          ->GetInt64());

  EXPECT_EQ(expected_outer_id_1.value, start_node_id);
  EXPECT_EQ(expected_outer_id_2.value, end_node_id);
  auto deadend_fwd = rapidjson::Pointer("/0/edges/0/full_road_segment/intersections/end_node/deadend")
                         .Get(response)
                         ->GetBool();

  EXPECT_TRUE(deadend_fwd);
  auto deadend_bwd =
      rapidjson::Pointer("/0/edges/0/full_road_segment/intersections/start_node/deadend")
          .Get(response)
          ->GetBool();

  EXPECT_TRUE(deadend_bwd);
}
