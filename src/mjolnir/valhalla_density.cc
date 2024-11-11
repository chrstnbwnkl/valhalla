#include <cstdint>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unordered_map>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include "baldr/directededge.h"
#include "baldr/edgeinfo.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/tilehierarchy.h"
#include "filesystem.h"
#include "midgard/logging.h"
#include "midgard/tiles.h"

#include "argparse_utils.h"

using namespace valhalla::baldr;
using namespace valhalla::midgard;

int main(int argc, char** argv) {
  const auto program = filesystem::path(__FILE__).stem().string();
  // args
  boost::property_tree::ptree config;
  int divisions;
  try {
    // clang-format off
    cxxopts::Options options(
      program,
      program + " " + VALHALLA_VERSION + "\n\n"
      "a program that creates a list of edges for each auto-drivable OSM way.\n\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("i,inline-config", "Inline JSON config", cxxopts::value<std::string>())
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>());
    // clang-format on

    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, config, "mjolnir.logging"))
      return EXIT_SUCCESS;
  } catch (cxxopts::exceptions::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }
  GraphReader reader(config.get_child("mjolnir"));

  std::unordered_map<int, int> sizes;
  // Iterate through all tiles at level 2
  for (auto edge_id : reader.GetTileSet()) {
    // If tile exists add it to the queue
    if (!reader.DoesTileExist(edge_id)) {
      continue;
    }
    if (reader.OverCommitted()) {
      reader.Trim();
    }

    graph_tile_ptr tile = reader.GetGraphTile(edge_id);
    for (uint32_t n = 0; n < tile->header()->directededgecount(); n++, ++edge_id) {
      const DirectedEdge* edge = tile->directededge(edge_id);
      if (edge->IsTransitLine() || edge->use() == Use::kTransitConnection ||
          edge->use() == Use::kEgressConnection || edge->use() == Use::kPlatformConnection ||
          edge->is_shortcut()) {
        continue;
      }

      // Intersect with tile set
      auto shape = tile->edgeinfo(edge).shape();
      auto res = sizes.find(shape.size());
      if (res != sizes.end()) {
        sizes[shape.size()]++;
      } else {
        sizes[shape.size()] = 1;
      }
      // std::cout << shape.size();
    }
  }
  int max = 0;
  for (auto [k, v] : sizes) {
    max = std::max(k, max);
  }

  for (int i = 0; i <= max; i++) {
    auto res = sizes.find(i);
    if (res != sizes.end()) {
      std::cout << i << " " << sizes[i] << "\n";
    } else {
      std::cout << i << "0\n";
    }
  }
  return EXIT_SUCCESS;
}
