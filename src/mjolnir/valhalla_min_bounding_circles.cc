#include <cstdint>
#include <stdarg.h>
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
#include "geos_c.h"

using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {

struct geos_helper_t {
  static const geos_helper_t& get() {
    static geos_helper_t singleton;
    return singleton;
  }
  static GEOSGeometry* from_shape(const std::vector<PointLL>& coords) {
    // sadly we dont layout the memory in parallel arrays so we have to copy to geos
    GEOSCoordSequence* geos_coords = GEOSCoordSeq_create(coords.size(), 2);
    for (unsigned int i = 0; i < static_cast<unsigned int>(coords.size()); ++i) {
      GEOSCoordSeq_setX(geos_coords, i, coords[i].lng());
      GEOSCoordSeq_setY(geos_coords, i, coords[i].lat());
    }
    return GEOSGeom_createLineString(geos_coords);
  }
  static PointLL to_point(const GEOSGeometry* geometry) {
    // sadly we dont layout the memory in parallel arrays so we have to copy from geos
    auto* coords = GEOSGeom_getCoordSeq(geometry);
    unsigned int coords_size;

    double lat, lon;
    GEOSCoordSeq_getSize(coords, &coords_size);
    if (coords_size == 0)
      return PointLL{0, 0};
    GEOSCoordSeq_getXY(coords, 0, &lon, &lat);
    return PointLL{lon, lat};
  }

protected:
  static void message_handler(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
  }
  geos_helper_t() {
    initGEOS(message_handler, message_handler);
  }
  ~geos_helper_t() {
    finishGEOS();
  }
};
} // namespace

int main(int argc, char** argv) {
  const auto program = filesystem::path(__FILE__).stem().string();
  // args
  boost::property_tree::ptree config;
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

  geos_helper_t::get();
  size_t tile_count = 0;
  GraphReader reader(config.get_child("mjolnir"));

  std::vector<double> radii;
  // radii.reserve(2000000000);
  // keep track of the opposite edge IDs so we only look at each shape once
  std::unordered_set<GraphId> opps;
  for (auto edge_id : reader.GetTileSet()) {
    // If tile exists add it to the queue

    std::unordered_set<GraphId> local_opps;
    if (!reader.DoesTileExist(edge_id)) {
      continue;
    }
    if (reader.OverCommitted()) {
      reader.Trim();
    }

    graph_tile_ptr tile = reader.GetGraphTile(edge_id);
    for (uint32_t n = 0; n < tile->header()->directededgecount(); n++, ++edge_id) {
      const DirectedEdge* edge = tile->directededge(edge_id);
      if (opps.count(edge_id) > 0 || local_opps.count(edge_id) > 0 || edge->IsTransitLine() ||
          edge->use() == Use::kTransitConnection || edge->use() == Use::kEgressConnection ||
          edge->use() == Use::kPlatformConnection || edge->is_shortcut()) {
        continue;
      }

      // Intersect with tile set
      auto shape = tile->edgeinfo(edge).shape();
      auto line = geos_helper_t::from_shape(shape);
      double r;
      GEOSGeometry* center;
      auto circle = GEOSMinimumBoundingCircle(line, &r, &center);
      auto centerll = geos_helper_t::to_point(center);

      // DistanceApproximator<PointLL>::MetersPerLngDegree(centerll.lat());
      // auto scale = DistanceApproximator<PointLL>::LngScalePerLat(centerll.lat());
      // radii.emplace_back(r);
      // save the opposing edge ID
      std::cout << r << " " << centerll.lat() << " " << centerll.lng() << "\n";
      const auto opp = reader.GetOpposingEdgeId(edge_id);
      // if (tile->id() != opp.tileid()) {
      //   opps.insert(opp);
      // } else {
      //   local_opps.insert(opp);
      // }
      GEOSGeom_destroy(center);
      GEOSGeom_destroy(line);
      GEOSGeom_destroy(circle);
    }
    tile_count++;
    if (tile_count % 2 == 0)
      std::cerr << "Processed " << tile_count << " tiles\n";
  }

  // for (const auto radius : radii) {
  //   std::cout << radius * kMetersPerDegreeLat << "\n";
  // }

  return EXIT_SUCCESS;
}
