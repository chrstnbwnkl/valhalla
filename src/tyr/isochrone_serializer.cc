
#include "baldr/json.h"
#include "midgard/point2.h"
#include "midgard/pointll.h"
#include "tyr/serializers.h"

#include <cmath>
#include <sstream>
#include <utility>

using namespace valhalla::baldr::json;

namespace {
using rgba_t = std::tuple<float, float, float>;

using namespace valhalla;
using namespace tyr;
using namespace midgard;
using contour_t = std::list<PointLL>;                 // single ring
using feature_t = std::list<contour_t>;               // rings per interval
using contours_t = std::vector<std::list<feature_t>>; // all rings
using contour_group_t = std::vector<const contour_t*>;
using grouped_contours_t = std::vector<contour_group_t>;
using contour_interval_t = std::tuple<size_t, float, std::string, std::string>;

grouped_contours_t GroupContours(const bool polygons, const feature_t& contours) {
  grouped_contours_t results;

  // if the user requested linestrings, we'll give them linestrings
  if (!polygons || contours.size() < 2) {
    std::for_each(contours.begin(), contours.end(),
                  [&results](const contour_t& c) { results.push_back({&c}); });
    return results;
  }

  // inner rings have negative area
  auto isExteriorRing = [](const contour_t& c) -> bool { return polygon_area(c) > 0; };

  std::vector<const contour_t*> inner_ptrs;
  std::for_each(contours.begin(), contours.end(),
                [&results, &inner_ptrs, isExteriorRing](const contour_t& c) {
                  if (isExteriorRing(c)) {
                    results.push_back({&c});
                  } else {
                    inner_ptrs.push_back(&c);
                  }
                });

  // exactly one exterior ring, so all inners go in one group
  if (results.size() == 1) {
    std::for_each(inner_ptrs.begin(), inner_ptrs.end(),
                  [&inner_ptrs, &results](const contour_t* c) { results[0].push_back(c); });
    return results;
  }

  // now we're dealing with multiple exterior rings
  // iterate over outer rings and for each inner ring check if the inner ring is within the outer ring
  for (const auto inner : inner_ptrs) {
    // construct bbox
    AABB2<PointLL> inner_bbox(std::vector(inner->rbegin(), inner->rend()));
    bool found_exterior;
    // go over exterior rings from smallest to largest
    for (size_t i = results.size(); i > 0; --i) {
      AABB2<PointLL> outer_bbox(std::vector(results[i - 1][0]->rbegin(), results[i - 1][0]->rend()));

      // contain check
      if (outer_bbox.Contains(inner_bbox)) {
        //
        results[i - 1].push_back(inner);
        found_exterior = true;
        break;
      }
    }

    if (!found_exterior) {
      LOG_WARN("No exterior ring contour found for inner contour.");
    }
  }

  return results;
}

std::stringstream getIntervalColor(std::vector<contour_interval_t>& intervals, size_t interval_idx) {
  std::stringstream hex;
  // color was supplied
  if (!std::get<3>(intervals[interval_idx]).empty()) {
    hex << "#" << std::get<3>(intervals[interval_idx]);
  } // or we computed it..
  else {
    auto h = interval_idx * (150.f / intervals.size());
    auto c = .5f;
    auto x = c * (1 - std::abs(std::fmod(h / 60.f, 2.f) - 1));
    auto m = .25f;
    rgba_t color = h < 60 ? rgba_t{m + c, m + x, m}
                          : (h < 120 ? rgba_t{m + x, m + c, m} : rgba_t{m, m + c, m + x});
    hex << "#" << std::hex << static_cast<int>(std::get<0>(color) * 255 + .5f) << std::hex
        << static_cast<int>(std::get<1>(color) * 255 + .5f) << std::hex
        << static_cast<int>(std::get<2>(color) * 255 + .5f);
  }
  return hex;
}

std::string serializeIsochroneJson(Api& request,
                                   std::vector<contour_interval_t>& intervals,
                                   contours_t& contours,
                                   bool show_locations) {
  // for each contour interval
  int i = 0;
  auto features = array({});
  assert(intervals.size() == contours.size());
  for (size_t contour_index = 0; contour_index < intervals.size(); ++contour_index) {
    const auto& interval = intervals[contour_index];
    const auto& interval_contours = contours[contour_index];

    std::stringstream hex = getIntervalColor(intervals, i);
    ++i;

    // for each feature on that interval
    for (const auto& feature : interval_contours) {
      grouped_contours_t groups = GroupContours(true, feature);
      auto geom = array({});
      // groups are a multipolygon, or polygon if just one
      for (const auto& group : groups) {
        // group is a ring
        auto poly = array({});
        // first ring is exterior, rest are inner
        for (const auto& ring : group) {
          auto ring_coords = array({});
          for (const auto& pair : *ring) {
            ring_coords->push_back(array({fixed_t{pair.lng(), 6}, fixed_t{pair.lat(), 6}}));
          }
          poly->emplace_back(ring_coords);
        }
        geom->emplace_back(poly);
      }

      // add a feature
      features->emplace_back(map({
          {"type", std::string("Feature")},
          {"geometry", map({
                           {"type", std::string(groups.size() > 1 ? "MultiPolygon" : "Polygon")},
                           {"coordinates", geom->size() > 1 ? geom : geom->at(0)},
                       })},
          {"properties", map({
                             {"metric", std::get<2>(interval)},
                             {"contour", baldr::json::float_t{std::get<1>(interval)}},
                             {"color", hex.str()},               // lines
                             {"fill", hex.str()},                // geojson.io polys
                             {"fillColor", hex.str()},           // leaflet polys
                             {"opacity", fixed_t{.33f, 2}},      // lines
                             {"fill-opacity", fixed_t{.33f, 2}}, // geojson.io polys
                             {"fillOpacity", fixed_t{.33f, 2}},  // leaflet polys
                         })},
      }));
    }
  }

  auto feature_collection = map({
      {"type", std::string("FeatureCollection")},
      {"features", features},
  });

  if (request.options().has_id_case()) {
    feature_collection->emplace("id", request.options().id());
  }

  // add warnings to json response
  if (request.info().warnings_size() >= 1) {
    feature_collection->emplace("warnings", serializeWarnings(request));
  }

  std::stringstream ss;
  ss << *feature_collection;

  return ss.str();
}

[[maybe_unused]] std::string serializeIsochroneJson_Legacy(Api& request,
                                                           std::vector<contour_interval_t>& intervals,
                                                           contours_t& contours,
                                                           bool polygons,
                                                           bool show_locations) {
  // for each contour interval
  int i = 0;
  auto features = array({});
  assert(intervals.size() == contours.size());
  for (size_t contour_index = 0; contour_index < intervals.size(); ++contour_index) {
    const auto& interval = intervals[contour_index];
    const auto& feature_collection = contours[contour_index];

    std::stringstream hex = getIntervalColor(intervals, i);
    ++i;

    // for each feature on that interval
    for (const auto& feature : feature_collection) {
      // for each contour in that feature
      auto geom = array({});
      for (const auto& contour : feature) {
        // make some geometry
        auto coords = array({});
        for (const auto& coord : contour) {
          coords->push_back(array({fixed_t{coord.first, 6}, fixed_t{coord.second, 6}}));
        }
        // its either a ring
        if (polygons) {
          geom->emplace_back(coords);
          // or a single line, if someone has more than one contour per feature they messed up
        } else {
          geom = coords;
        }
      }
      // add a feature
      features->emplace_back(map({
          {"type", std::string("Feature")},
          {"geometry", map({
                           {"type", std::string(polygons ? "Polygon" : "LineString")},
                           {"coordinates", geom},
                       })},
          {"properties", map({
                             {"metric", std::get<2>(interval)},
                             {"contour", baldr::json::float_t{std::get<1>(interval)}},
                             {"color", hex.str()},               // lines
                             {"fill", hex.str()},                // geojson.io polys
                             {"fillColor", hex.str()},           // leaflet polys
                             {"opacity", fixed_t{.33f, 2}},      // lines
                             {"fill-opacity", fixed_t{.33f, 2}}, // geojson.io polys
                             {"fillOpacity", fixed_t{.33f, 2}},  // leaflet polys
                         })},
      }));
    }
  }
  // Add input and snapped locations to the geojson
  if (show_locations) {
    int idx = 0;
    for (const auto& location : request.options().locations()) {
      // first add all snapped points as MultiPoint feature per origin point
      auto snapped_points_array = array({});
      std::unordered_set<PointLL> snapped_points;
      for (const auto& path_edge : location.correlation().edges()) {
        const PointLL& snapped_current = PointLL(path_edge.ll().lng(), path_edge.ll().lat());
        // remove duplicates of path_edges in case the snapped object is a node
        if (snapped_points.insert(snapped_current).second) {
          snapped_points_array->push_back(
              array({fixed_t{snapped_current.lng(), 6}, fixed_t{snapped_current.lat(), 6}}));
        }
      };
      features->emplace_back(map(
          {{"type", std::string("Feature")},
           {"properties",
            map({{"type", std::string("snapped")}, {"location_index", static_cast<uint64_t>(idx)}})},
           {"geometry",
            map({{"type", std::string("MultiPoint")}, {"coordinates", snapped_points_array}})}}));

      // then each user input point as separate Point feature
      const valhalla::LatLng& input_latlng = location.ll();
      const auto input_array =
          array({fixed_t{input_latlng.lng(), 6}, fixed_t{input_latlng.lat(), 6}});
      features->emplace_back(map(
          {{"type", std::string("Feature")},
           {"properties",
            map({{"type", std::string("input")}, {"location_index", static_cast<uint64_t>(idx)}})},
           {"geometry", map({{"type", std::string("Point")}, {"coordinates", input_array}})}}));
      idx++;
    }
  }

  // make the collection
  auto feature_collection = map({
      {"type", std::string("FeatureCollection")},
      {"features", features},
  });

  if (request.options().has_id_case()) {
    feature_collection->emplace("id", request.options().id());
  }

  // add warnings to json response
  if (request.info().warnings_size() >= 1) {
    feature_collection->emplace("warnings", serializeWarnings(request));
  }

  std::stringstream ss;
  ss << *feature_collection;

  return ss.str();
}
std::string serializeIsochronePbf(Api& request,
                                  std::vector<contour_interval_t>& intervals,
                                  const contours_t& contours) {
  // construct pbf output
  Isochrone& isochrone = *request.mutable_isochrone();

  // construct contours
  for (size_t isoline_index = 0; isoline_index < contours.size(); ++isoline_index) {
    const auto& contour = contours[isoline_index];
    const auto& interval = intervals[isoline_index];

    auto* interval_pbf = isochrone.mutable_intervals()->Add();
    interval_pbf->set_metric(std::get<2>(interval) == "time" ? Isochrone::time : Isochrone::distance);

    interval_pbf->set_metric_value(std::get<1>(interval));

    // for each feature
    for (const auto& feature : contour) {
      grouped_contours_t groups = GroupContours(true, feature);

      // for each group of rings (first is outer, rest is inner)
      for (std::vector<const contour_t*> group_ptr : groups) {
        auto* contour_pbf = interval_pbf->mutable_contours()->Add();

        // construct a geometry
        for (const std::list<PointLL>* ring : group_ptr) {
          std::cerr << "Rings: " << ring->size() << std::endl;

          auto* geom = contour_pbf->mutable_geometries()->Add();
          for (PointLL pair : *ring) {
            geom->add_coords(round(pair.lng() * 1e6));
            geom->add_coords(round(pair.lat() * 1e6));
          }
        }
      }
    }
  }

  return serializePbf(request);
}

} // namespace

namespace valhalla {
namespace tyr {

std::string serializeIsochrones(Api& request,
                                std::vector<midgard::GriddedData<2>::contour_interval_t>& intervals,
                                midgard::GriddedData<2>::contours_t& contours,
                                bool polygons,
                                bool show_locations) {

  switch (request.options().format()) {
    case Options_Format_pbf:
      return serializeIsochronePbf(request, intervals, contours);
    case Options_Format_json:
      if (polygons)
        return serializeIsochroneJson(request, intervals, contours, show_locations);
      return serializeIsochroneJson_Legacy(request, intervals, contours, polygons, show_locations);
    default:
      throw;
  }
}
} // namespace tyr
} // namespace valhalla
