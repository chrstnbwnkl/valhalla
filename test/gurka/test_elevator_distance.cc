#include "gurka.h"
#include "mjolnir/osmway.h"
#include "test.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace valhalla;

class Indoor : public ::testing::Test {
protected:
  static gurka::map map;
  static gurka::ways ways;
  static gurka::nodes nodes;
  static std::string ascii_map;
  static gurka::nodelayout layout;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;

    ascii_map = R"(
    A--B---C---D--E
    |             | 
    F             J 
    |             | 
    G             K 
    |             | 
    H             L 
    |             | 
    I             M 
    )";

    ways = {
        {"AB",
         {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "0"}, {"level:ref", "Parking"}}},
        {"BC", {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "0"}}},
        {"CD", {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "1"}, {"level:ref", "Lobby"}}},
        {"DE", {{"highway", "footway"}, {"level", "1"}}},
        {"AF",
         {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "0"}, {"level:ref", "Parking"}}},
        {"FG", {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "0"}}},
        {"GH", {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "7"}, {"level:ref", "Lobby"}}},
        {"HI", {{"highway", "footway"}, {"level", "1"}}},
        {"EJ",
         {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "0"}, {"level:ref", "Parking"}}},
        {"JK", {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "0"}}},
        {"KL", {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "-3"}, {"level:ref", "Lobby"}}},
        {"LM", {{"highway", "footway"}, {"level", "-3"}}},
    };

    nodes = {
        {"C",
         {{"highway", "elevator"}, {"indoor", "yes"}, {"level", "0;1"}, {"height:level", "4.5"}}},
        {"G", {{"highway", "elevator"}, {"indoor", "yes"}, {"level", "0;7"}}},
        {"G", {{"highway", "steps"}, {"indoor", "yes"}, {"level", "-3;0"}, {"height:level", "2.5"}}},
    };

    layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
    map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/gurka_elevator", {});
  }
};
gurka::map Indoor::map = {};
gurka::ways Indoor::ways = {};
gurka::nodes Indoor::nodes = {};
std::string Indoor::ascii_map = {};
gurka::nodelayout Indoor::layout = {};

TEST_F(Indoor, LevelHeightTag) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "pedestrian", {});
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE"});

  size_t elevator_maneuver_index = 1;
  auto man = result.directions().routes(0).legs(0).maneuver(elevator_maneuver_index);
  EXPECT_EQ(man.type(), DirectionsLeg_Maneuver_Type::DirectionsLeg_Maneuver_Type_kElevatorEnter);
  EXPECT_EQ(man.length(), 4.5f);
}

TEST_F(Indoor, MultipleLevels) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, "pedestrian", {});
  gurka::assert::raw::expect_path(result, {"AF", "FG", "GH", "HI"});

  size_t elevator_maneuver_index = 1;
  auto man = result.directions().routes(0).legs(0).maneuver(elevator_maneuver_index);
  EXPECT_EQ(man.type(), DirectionsLeg_Maneuver_Type::DirectionsLeg_Maneuver_Type_kElevatorEnter);
  EXPECT_EQ(man.length(), 21.f);
}

/********************** FAILING FROM HERE ON ********************************/

TEST_F(Indoor, ShortRoute) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"F", "H"}, "pedestrian", {});
  gurka::assert::raw::expect_path(result, {"FG", "GH"});

  size_t elevator_maneuver_index = 1;
  auto man = result.directions().routes(0).legs(0).maneuver(elevator_maneuver_index);
  EXPECT_EQ(man.type(), DirectionsLeg_Maneuver_Type::DirectionsLeg_Maneuver_Type_kElevatorEnter);
  EXPECT_EQ(man.length(), 21.f);
}

TEST_F(Indoor, StartAtElevator) {
  std::unordered_map<std::string, std::string> opts = {
      {"/locations/0/search_filter/level", "0"},
      {"/locations/1/search_filter/level", "7"},
  };

  auto result = gurka::do_action(valhalla::Options::route, map, {"G", "H"}, "pedestrian", opts);
  gurka::assert::raw::expect_path(result, {"FG", "GH"});

  size_t elevator_maneuver_index = 0;
  auto man = result.directions().routes(0).legs(0).maneuver(elevator_maneuver_index);
  EXPECT_EQ(man.type(), DirectionsLeg_Maneuver_Type::DirectionsLeg_Maneuver_Type_kElevatorEnter);
  EXPECT_EQ(man.length(), 21.f);
}
TEST_F(Indoor, StartAndEndAtElevator) {
  // get name -> graphid mapping
  auto graphreader = test::make_clean_graphreader(map.config.get_child("mjolnir"));

  for (const auto& [name, _] : ways) {
    for (const auto& node : name) {
      std::string n(&node, 1);
      auto edge = gurka::findEdge(*graphreader, layout, name, n);
      std::cout << std::get<0>(edge).value << "|fw=" << std::get<1>(edge)->forward() << "|" << name
                << "\n";
      //   auto tile = graphreader->GetGraphTile(std::get<0>(edge));
      //   auto ei = tile->edgeinfo(std::get<1>(edge));
    }
  }
  std::unordered_map<std::string, std::string> opts = {
      {"/locations/0/search_filter/level", "0"},
      {"/locations/1/search_filter/level", "7"},
  };

  auto result = gurka::do_action(valhalla::Options::route, map, {"G", "G"}, "pedestrian", opts);
  gurka::assert::raw::expect_path(result, {"FG", "GH"});

  size_t elevator_maneuver_index = 1;
  auto man = result.directions().routes(0).legs(0).maneuver(elevator_maneuver_index);
  EXPECT_EQ(man.type(), DirectionsLeg_Maneuver_Type::DirectionsLeg_Maneuver_Type_kElevatorEnter);
  EXPECT_EQ(man.length(), 21.f);
}

TEST_F(Indoor, EndAtElevator) {
  std::unordered_map<std::string, std::string> opts = {
      {"/locations/0/search_filter/level", "0"},
      {"/locations/1/search_filter/level", "7"},
  };

  auto result = gurka::do_action(valhalla::Options::route, map, {"F", "G"}, "pedestrian", opts);
  gurka::assert::raw::expect_path(result, {"FG", "GH"});

  size_t elevator_maneuver_index = 1;
  auto man = result.directions().routes(0).legs(0).maneuver(elevator_maneuver_index);
  EXPECT_EQ(man.type(), DirectionsLeg_Maneuver_Type::DirectionsLeg_Maneuver_Type_kElevatorEnter);
  EXPECT_EQ(man.length(), 21.f);
}