#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "proto/options.pb.h"
#include "proto/trip.pb.h"
#include "route_serializer_osrm.h"
#include "route_serializer_valhalla.h"
#include "tyr/fit.h"
#include "tyr/serializers.h"

#include <chrono>
#include <cstdint>
#include <sstream>
#include <vector>

using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
namespace {

/**
 * Returns GPX formatted route responses given the legs of the route
 * @param  legs  The legs of the route
 * @return the gpx string
 */
std::string pathToGPX(const google::protobuf::RepeatedPtrField<TripLeg>& legs) {
  // start the gpx, we'll use 6 digits of precision
  std::stringstream gpx;
  gpx << std::setprecision(DIGITS_PRECISION) << std::fixed;
  gpx << R"(<?xml version="1.0" encoding="UTF-8" standalone="no"?><gpx version="1.1" creator="libvalhalla"><metadata/>)";

  // for each leg
  for (const auto& leg : legs) {
    // decode the shape for this leg
    auto wpts = midgard::decode<std::vector<PointLL>>(leg.shape());

    // throw the shape points in as way points
    // TODO: add time to each, need transition time at nodes
    for (const auto& wpt : wpts) {
      gpx << R"(<wpt lon=")" << wpt.first << R"(" lat=")" << wpt.second << R"("></wpt>)";
    }

    // throw the intersections in as route points
    // TODO: add time to each, need transition time at nodes
    gpx << "<rte>";
    uint64_t last_id = -1;
    for (const auto& node : leg.node()) {
      // if this isnt the last node we want the begin shape index of the edge
      size_t shape_idx = wpts.size() - 1;
      if (node.has_edge()) {
        last_id = node.edge().way_id();
        shape_idx = node.edge().begin_shape_index();
      }

      // output this intersection (note that begin and end points may not be intersections)
      const auto& rtept = wpts[shape_idx];
      gpx << R"(<rtept lon=")" << rtept.first << R"(" lat=")" << rtept.second << R"(">)"
          << "<name>" << last_id << "</name></rtept>";
    }
    gpx << "</rte>";
  }

  // give it back as a string
  gpx << "</gpx>";
  return gpx.str();
}

fit::CoursePointType maneuver_type_to_course_point_type(DirectionsLeg_Maneuver_Type type) {
  switch (type) {
    case DirectionsLeg_Maneuver_Type_kSlightRight:
      return fit::CoursePointType::slight_right;
    case DirectionsLeg_Maneuver_Type_kRight:
      return fit::CoursePointType::right;
    case DirectionsLeg_Maneuver_Type_kSharpRight:
      return fit::CoursePointType::sharp_right;
    case DirectionsLeg_Maneuver_Type_kSlightLeft:
      return fit::CoursePointType::slight_left;
    case DirectionsLeg_Maneuver_Type_kLeft:
      return fit::CoursePointType::left;
    case DirectionsLeg_Maneuver_Type_kSharpLeft:
      return fit::CoursePointType::sharp_left;
    case DirectionsLeg_Maneuver_Type_kUturnRight:
    case DirectionsLeg_Maneuver_Type_kUturnLeft:
      return fit::CoursePointType::u_turn;
    case DirectionsLeg_Maneuver_Type_kContinue:
    case DirectionsLeg_Maneuver_Type_kStayStraight:
    case DirectionsLeg_Maneuver_Type_kRampStraight:
      return fit::CoursePointType::straight;
    case DirectionsLeg_Maneuver_Type_kRampRight:
    case DirectionsLeg_Maneuver_Type_kExitRight:
    case DirectionsLeg_Maneuver_Type_kStayRight:
    case DirectionsLeg_Maneuver_Type_kMergeRight:
      return fit::CoursePointType::right_fork;
    case DirectionsLeg_Maneuver_Type_kRampLeft:
    case DirectionsLeg_Maneuver_Type_kExitLeft:
    case DirectionsLeg_Maneuver_Type_kStayLeft:
    case DirectionsLeg_Maneuver_Type_kMergeLeft:
      return fit::CoursePointType::left_fork;
    case DirectionsLeg_Maneuver_Type_kMerge:
      return fit::CoursePointType::straight;
    case DirectionsLeg_Maneuver_Type_kRoundaboutEnter:
    case DirectionsLeg_Maneuver_Type_kRoundaboutExit:
      return fit::CoursePointType::generic;
    default:
      return fit::CoursePointType::generic;
  }
}

std::string pathToFIT(Api& request) {
  const auto& legs = request.trip().routes(0).legs();
  const auto& dir_legs = request.directions().routes(0).legs();

  // use current time as the base timestamp for the course
  auto now = std::chrono::system_clock::now();
  uint32_t base_ts = static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count());

  fit::CourseWriter writer;
  writer.set_course_name("valhalla");

  // determine sport from costing
  auto costing = request.options().costing_type();
  if (costing == Costing::bicycle) {
    writer.set_sport(fit::Sport::cycling);
  } else if (costing == Costing::pedestrian) {
    writer.set_sport(fit::Sport::walking);
  } else {
    writer.set_sport(fit::Sport::cycling);
  }

  double cumulative_distance = 0.0;
  double cumulative_time = 0.0;

  for (int leg_idx = 0; leg_idx < legs.size(); ++leg_idx) {
    const auto& leg = legs[leg_idx];
    auto shape = midgard::decode<std::vector<PointLL>>(leg.shape());

    // compute per-shape-point distances
    std::vector<double> shape_distances(shape.size(), 0.0);
    for (size_t i = 1; i < shape.size(); ++i) {
      shape_distances[i] = shape_distances[i - 1] + shape[i - 1].Distance(shape[i]);
    }
    double leg_length = shape_distances.empty() ? 0.0 : shape_distances.back();
    double leg_time = leg_idx < dir_legs.size() ? dir_legs[leg_idx].summary().time() : 0.0;

    // add trackpoints for each shape point
    for (size_t i = 0; i < shape.size(); ++i) {
      double frac = (leg_length > 0.0) ? shape_distances[i] / leg_length : 0.0;
      uint32_t ts = base_ts + static_cast<uint32_t>(cumulative_time + frac * leg_time);
      writer.add_trackpoint(
          {shape[i].second, shape[i].first, 0.0, cumulative_distance + shape_distances[i], ts});
    }

    // add course points from maneuvers (skip start/destination)
    if (leg_idx < dir_legs.size()) {
      for (const auto& maneuver : dir_legs[leg_idx].maneuver()) {
        auto mtype = maneuver.type();
        if (mtype == DirectionsLeg_Maneuver_Type_kStart ||
            mtype == DirectionsLeg_Maneuver_Type_kStartRight ||
            mtype == DirectionsLeg_Maneuver_Type_kStartLeft ||
            mtype == DirectionsLeg_Maneuver_Type_kDestination ||
            mtype == DirectionsLeg_Maneuver_Type_kDestinationRight ||
            mtype == DirectionsLeg_Maneuver_Type_kDestinationLeft) {
          continue;
        }

        auto shape_idx = maneuver.begin_shape_index();
        if (shape_idx >= shape.size())
          continue;

        double frac = (leg_length > 0.0) ? shape_distances[shape_idx] / leg_length : 0.0;
        uint32_t ts = base_ts + static_cast<uint32_t>(cumulative_time + frac * leg_time);

        // use the first street name or the instruction text, truncated for FIT
        std::string name;
        if (maneuver.street_name_size() > 0) {
          name = maneuver.street_name(0).value();
        } else {
          name = maneuver.text_instruction();
        }
        if (name.size() > 15)
          name.resize(15);

        writer.add_course_point({shape[shape_idx].second, shape[shape_idx].first, ts,
                                 cumulative_distance + shape_distances[shape_idx], name,
                                 maneuver_type_to_course_point_type(mtype)});
      }
    }

    cumulative_distance += leg_length;
    cumulative_time += leg_time;
  }

  LOG_INFO("FIT serializer: " + std::to_string(writer.num_trackpoints()) + " trackpoints, " +
           std::to_string(writer.num_course_points()) + " course points");

  return writer.to_string();
}

} // namespace

namespace valhalla {
namespace tyr {

std::string serializeDirections(Api& request) {
  // serialize them
  switch (request.options().format()) {
    case Options_Format_osrm:
      return osrm_serializers::serialize(request);
    case Options_Format_gpx:
      return pathToGPX(request.trip().routes(0).legs());
    case Options_Format_fit:
      return pathToFIT(request);
    case Options_Format_json:
      return valhalla_serializers::serialize(request);
    case Options_Format_pbf:
      return serializePbf(request);
    default:
      throw;
  }
}

} // namespace tyr
} // namespace valhalla
