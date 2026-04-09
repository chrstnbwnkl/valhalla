#include "gurka.h"

#include <gtest/gtest.h>

using namespace valhalla;

namespace {

// Low reachability so tiny test graphs aren't rejected at snap time.
const std::unordered_map<std::string, std::string> kTrainConfig = {
    {"mjolnir.concurrency", "1"},
    {"loki.service_defaults.minimum_reachability", "2"},
    {"loki.service_defaults.radius", "10"},
};

constexpr double kGridSize = 100;

} // namespace

// A simple railway=rail line should be routable with costing=train.
TEST(Train, BasicLine) {
  const std::string ascii_map = R"(
    A----B----C----D
  )";

  const gurka::ways ways = {
      {"AB", {{"railway", "rail"}}},
      {"BC", {{"railway", "rail"}}},
      {"CD", {{"railway", "rail"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, kGridSize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_train_basic", kTrainConfig);

  auto result = gurka::do_action(Options::route, map, {"A", "D"}, "train");
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD"});

  // reverse direction (default rail is bidirectional)
  result = gurka::do_action(Options::route, map, {"D", "A"}, "train");
  gurka::assert::raw::expect_path(result, {"CD", "BC", "AB"});
}

// Auto costing must not be able to route on a pure railway network.
TEST(Train, AutoCannotUseRailway) {
  const std::string ascii_map = R"(
    A----B----C
  )";

  const gurka::ways ways = {
      {"AB", {{"railway", "rail"}}},
      {"BC", {{"railway", "rail"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, kGridSize);
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_train_auto_reject", kTrainConfig);

  EXPECT_THROW(gurka::do_action(Options::route, map, {"A", "C"}, "auto"), std::runtime_error);
}

// Train costing must not be able to route on a pure highway network.
TEST(Train, TrainCannotUseHighway) {
  const std::string ascii_map = R"(
    A----B----C
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"BC", {{"highway", "primary"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, kGridSize);
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_train_highway_reject", kTrainConfig);

  EXPECT_THROW(gurka::do_action(Options::route, map, {"A", "C"}, "train"), std::runtime_error);
}

// Non-"rail" railway types (light_rail, tram, subway, ...) should all be
// routable for trains — the Lua side treats them the same.
class TrainRailTypeTest : public ::testing::TestWithParam<std::string> {};

TEST_P(TrainRailTypeTest, RoutableAsTrain) {
  const std::string railway_type = GetParam();
  const std::string ascii_map = R"(
    A----B----C
  )";

  const gurka::ways ways = {
      {"AB", {{"railway", railway_type}}},
      {"BC", {{"railway", railway_type}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, kGridSize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_train_type_" + railway_type,
                               kTrainConfig);

  auto result = gurka::do_action(Options::route, map, {"A", "C"}, "train");
  gurka::assert::raw::expect_path(result, {"AB", "BC"});
}

INSTANTIATE_TEST_SUITE_P(Train,
                         TrainRailTypeTest,
                         ::testing::Values("rail",
                                           "light_rail",
                                           "subway",
                                           "tram",
                                           "narrow_gauge",
                                           "monorail",
                                           "funicular"));

// oneway=yes on a railway way should make it traversable in the forward
// direction only.
TEST(Train, OnewayForward) {
  const std::string ascii_map = R"(
    A----B----C
  )";

  const gurka::ways ways = {
      {"AB", {{"railway", "rail"}, {"oneway", "yes"}}},
      {"BC", {{"railway", "rail"}, {"oneway", "yes"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, kGridSize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_train_oneway", kTrainConfig);

  // forward works
  auto result = gurka::do_action(Options::route, map, {"A", "C"}, "train");
  gurka::assert::raw::expect_path(result, {"AB", "BC"});

  // reverse is blocked
  EXPECT_THROW(gurka::do_action(Options::route, map, {"C", "A"}, "train"), std::runtime_error);
}

// railway:preferred_direction should take precedence over oneway and control
// train direction on its own.
TEST(Train, PreferredDirectionBackward) {
  const std::string ascii_map = R"(
    A----B----C
  )";

  const gurka::ways ways = {
      {"AB", {{"railway", "rail"}, {"railway:preferred_direction", "backward"}}},
      {"BC", {{"railway", "rail"}, {"railway:preferred_direction", "backward"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, kGridSize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_train_preferred_backward",
                               kTrainConfig);

  // backward (C->A) works because that's the preferred direction
  auto result = gurka::do_action(Options::route, map, {"C", "A"}, "train");
  gurka::assert::raw::expect_path(result, {"BC", "AB"});

  // forward (A->C) is blocked
  EXPECT_THROW(gurka::do_action(Options::route, map, {"A", "C"}, "train"), std::runtime_error);
}

// A simple Y-junction: given two rail branches from B, the train should take
// the branch whose endpoint is the requested destination.
TEST(Train, YJunction) {
  const std::string ascii_map = R"(
    A----B----C
         |
         D
  )";

  const gurka::ways ways = {
      {"AB", {{"railway", "rail"}}},
      {"BC", {{"railway", "rail"}}},
      {"BD", {{"railway", "rail"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, kGridSize);
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_train_y_junction", kTrainConfig);

  auto to_c = gurka::do_action(Options::route, map, {"A", "C"}, "train");
  gurka::assert::raw::expect_path(to_c, {"AB", "BC"});

  auto to_d = gurka::do_action(Options::route, map, {"A", "D"}, "train");
  gurka::assert::raw::expect_path(to_d, {"AB", "BD"});
}
