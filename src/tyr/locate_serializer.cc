#include "baldr/graphreader.h"
#include "baldr/openlr.h"
#include "baldr/pathlocation.h"
#include "baldr/rapidjson_utils.h"
#include "sif/dynamiccost.h"
#include "baldr/time_info.h"
#include "tyr/serializers.h"

#include <cstdint>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace {

OpenLR::LocationReferencePoint::FormOfWay get_fow(const baldr::DirectedEdge* de) {
  if (de->classification() == valhalla::baldr::RoadClass::kMotorway)
    return OpenLR::LocationReferencePoint::MOTORWAY;
  else if (de->roundabout())
    return OpenLR::LocationReferencePoint::ROUNDABOUT;
  else if (de->use() == valhalla::baldr::Use::kRamp ||
           de->use() == valhalla::baldr::Use::kTurnChannel)
    return OpenLR::LocationReferencePoint::SLIPROAD;
  else if ((de->forwardaccess() & kVehicularAccess) && (de->reverseaccess() & kVehicularAccess))
    return OpenLR::LocationReferencePoint::MULTIPLE_CARRIAGEWAY;
  else if ((de->forwardaccess() & kVehicularAccess) || (de->reverseaccess() & kVehicularAccess))
    return OpenLR::LocationReferencePoint::SINGLE_CARRIAGEWAY;

  return OpenLR::LocationReferencePoint::OTHER;
}

void serialize_access_restrictions(const graph_tile_ptr& tile,
                                   rapidjson::writer_wrapper_t& writer,
                                   uint32_t edge_idx) {
  for (const auto& res : tile->GetAccessRestrictions(edge_idx)) {
    res.json(writer);
  }
}

std::string
linear_reference(const baldr::DirectedEdge* de, float percent_along, const EdgeInfo& edgeinfo) {
  const auto fow = get_fow(de);
  const auto frc = static_cast<uint8_t>(de->classification());

  auto shape = edgeinfo.shape();
  if (!de->forward())
    std::reverse(shape.begin(), shape.end());
  float forward_heading = midgard::tangent_angle(0, shape.front(), shape, 20.f, true);
  float reverse_heading = midgard::tangent_angle(shape.size() - 1, shape.back(), shape, 20.f, false);

  std::vector<OpenLR::LocationReferencePoint> lrps;
  lrps.emplace_back(shape.front().lng(), shape.front().lat(), forward_heading, frc, fow, nullptr,
                    de->length(), frc);
  lrps.emplace_back(shape.back().lng(), shape.back().lat(), reverse_heading, frc, fow, &lrps.back());

  uint8_t poff = static_cast<uint8_t>(std::min(255.f, percent_along * 255 + .5f));

  return OpenLR::OpenLr{lrps,
                        poff,
                        0,
                        true,
                        OpenLR::Orientation::FirstLrpTowardsSecond,
                        OpenLR::SideOfTheRoad::DirectlyOnRoadOrNotApplicable}
      .toBase64();
}

void serialize_traffic_speed(const volatile baldr::TrafficSpeed& traffic_speed,
                             rapidjson::writer_wrapper_t& writer) {
  if (traffic_speed.speed_valid()) {
    writer.set_precision(2);
    writer("overall_speed", static_cast<uint64_t>(traffic_speed.get_overall_speed()));
    auto speed = static_cast<uint64_t>(traffic_speed.get_speed(0));
    if (speed == baldr::UNKNOWN_TRAFFIC_SPEED_KPH)
      writer("speed_0", nullptr);
    else
      writer("speed_0", speed);
    auto congestion = (traffic_speed.congestion1 - 1.0) / 62.0;
    if (congestion < 0)
      writer("congestion_0", nullptr);
    else {
      writer("congestion_0", congestion);
    }
    writer("breakpoint_0", traffic_speed.breakpoint1 / 255.0);

    speed = static_cast<uint64_t>(traffic_speed.get_speed(1));
    if (speed == baldr::UNKNOWN_TRAFFIC_SPEED_KPH)
      writer("speed_1", nullptr);
    else
      writer("speed_1", speed);
    congestion = (traffic_speed.congestion2 - 1.0) / 62.0;
    if (congestion < 0)
      writer("congestion_1", nullptr);
    else {
      writer("congestion_1", congestion);
    }
    writer("breakpoint_1", traffic_speed.breakpoint2 / 255.0);

    speed = static_cast<uint64_t>(traffic_speed.get_speed(2));
    if (speed == baldr::UNKNOWN_TRAFFIC_SPEED_KPH)
      writer("speed_2", nullptr);
    else
      writer("speed_2", speed);
    congestion = (traffic_speed.congestion3 - 1.0) / 62.0;
    if (congestion < 0)
      writer("congestion_2", nullptr);
    else {
      writer("congestion_2", congestion);
    }
    writer.set_precision(tyr::kDefaultPrecision);
  }
}

const DirectedEdge* get_opposing_edge(const DirectedEdge* de, GraphReader& reader) {
  auto tile = reader.GetGraphTile(de->endnode());
  auto opp = de->opp_index();
  auto end_node = reader.GetEndNode(de, tile);

  return tile->directededge(end_node->edge_index() + opp);
}

void get_full_road_segment(rapidjson::writer_wrapper_t& writer,
                           const DirectedEdge* de,
                           const std::shared_ptr<sif::DynamicCost>& costing,
                           const double percent_along,
                           GraphReader& reader,
                           const valhalla::baldr::Location::SearchFilter& search_filter) {
  // first, find the beginning of the road segment
  // things we need: the opposing edge and its start node
  if (de == nullptr)
    return;
  auto opp_edge = get_opposing_edge(de, reader);

  if (de->shortcut()) {
    return;
  }
  auto node_id = opp_edge->endnode();
  auto node = reader.GetGraphTile(node_id)->node(node_id);
  auto edge = de;

  // track whether the begin/end of this segment is a deadend vs an intersection
  bool forward_deadend = false;
  bool backward_deadend = false;

  std::unordered_set<const DirectedEdge*> added_edges;
  std::vector<const DirectedEdge*> edges;
  // crawl in reverse direction until we find a "true" intersection given the costing
  while (true) {
    int allowed_cnt = 0;
    const DirectedEdge* incoming_pred = nullptr;
    const DirectedEdge* outgoing_pred = nullptr;
    auto tile = reader.GetGraphTile(node_id);
    // check the number of outgoing edges accessible with the provided costing
    for (int i = 0; i < node->edge_count(); ++i) {
      auto outgoing_edge = tile->directededge(node->edge_index() + i);
      auto ei = tile->edgeinfo(outgoing_edge);

      // except our current edge
      if (outgoing_edge == edge)
        continue;
      auto incoming_edge = get_opposing_edge(outgoing_edge, reader);
      if ((costing->Allowed(incoming_edge, reader.GetGraphTile(outgoing_edge->endnode()),
                            sif::kDisallowShortcut) ||
           costing->Allowed(outgoing_edge, reader.GetGraphTile(node_id), sif::kDisallowShortcut)) &&
          !(costing->ExcludePrivate() && ei.private_access()) &&
          (1ull << static_cast<int>(incoming_edge->use()) & search_filter.use_) &&
          (static_cast<uint32_t>(incoming_edge->classification()) <=
           static_cast<uint32_t>(search_filter.min_road_class_)) &&
          (static_cast<uint32_t>(incoming_edge->classification()) >=
           static_cast<uint32_t>(search_filter.max_road_class_))) {

        allowed_cnt++;
        outgoing_pred = outgoing_edge;
        incoming_pred = incoming_edge;
      }
    }
    std::function<void(const baldr::GraphId&, const bool, const bool)> expand_back;
    expand_back = [&](const baldr::GraphId& trans_node_id, const bool from_transition,
                      const bool up) {
      graph_tile_ptr tile = reader.GetGraphTile(trans_node_id);
      if (tile == nullptr) {
        return;
      }
      const baldr::NodeInfo* trans_node = tile->node(trans_node_id);
      // get the count of incoming edges
      for (int i = 0; i < trans_node->edge_count(); ++i) {
        auto outgoing_edge = tile->directededge(trans_node->edge_index() + i);
        auto incoming_edge = get_opposing_edge(outgoing_edge, reader);
        auto ei = tile->edgeinfo(outgoing_edge);
        // auto opp_tile = reader.GetGraphTile(incoming_edge->endnode());
        // auto name = tile->edgeinfo(outgoing_edge).GetNames()[0];
        // auto opp_name = opp_tile->edgeinfo(incoming_edge).GetNames()[0];

        if ((costing->Allowed(incoming_edge, reader.GetGraphTile(outgoing_edge->endnode()),
                              sif::kDisallowShortcut) ||
             costing->Allowed(outgoing_edge, reader.GetGraphTile(node_id), sif::kDisallowShortcut)) &&
            !(costing->ExcludePrivate() && ei.private_access()) &&
            (1ull << static_cast<int>(incoming_edge->use()) & search_filter.use_) &&
            (static_cast<uint32_t>(incoming_edge->classification()) <=
             static_cast<uint32_t>(search_filter.min_road_class_)) &&
            (static_cast<uint32_t>(incoming_edge->classification()) >=
             static_cast<uint32_t>(search_filter.max_road_class_))) {

          allowed_cnt++;
          incoming_pred = incoming_edge;
          outgoing_pred = outgoing_edge;
        }
      }

      if (!from_transition && trans_node->transition_count() > 0) {
        const baldr::NodeTransition* trans = tile->transition(trans_node->transition_index());
        for (uint32_t i = 0; i < trans_node->transition_count(); ++i, ++trans) {
          if (!up == trans->up())
            continue;
          expand_back(trans->endnode(), true, up);
        }
      }
    };
    // follow node transitions
    for (int i = 0; i < node->transition_count(); ++i) {
      const baldr::NodeTransition* trans = tile->transition(node->transition_index() + i);
      expand_back(trans->endnode(), false, trans->up());
    }

    // if there's exactly one allowed incoming edge (except for the current opposing edge)
    if (allowed_cnt == 1 && incoming_pred->classification() == de->classification() &&
        incoming_pred->use() == de->use()) {
      // keep going back
      if (!(added_edges.insert(incoming_pred)).second) {
        LOG_WARN("Duplicate edge when finding beginning of road segment for edge.");
        break;
      } else {
        edges.push_back(incoming_pred);
      }
      edge = incoming_pred;
      node_id = outgoing_pred->endnode();
      node = reader.GetGraphTile(node_id)->node(node_id);
    } else {
      // we've found the start of the road segment
      // either because this is a valid intersection given the costing
      // or because there is no other edge to inspect
      if (allowed_cnt == 0)
        backward_deadend = true;
      break;
    }
  }
  // we've found our start node
  auto start_node_id = node_id;

  // resort and get our initial edge in there
  std::reverse(edges.begin(), edges.end());
  if (!(added_edges.insert(de)).second) {
    LOG_WARN("Initial edge already inserted into road segment edges");
  } else {
    edges.push_back(de);
  }

  // now move forward
  node_id = de->endnode();
  node = reader.GetGraphTile(node_id)->node(node_id);
  edge = de;

  while (true) {
    int allowed_cnt = 0;
    auto tile = reader.GetGraphTile(node_id);
    const DirectedEdge* possible_next = nullptr;

    // check the number of outgoing edges accessible with the provided costing
    for (int i = 0; i < node->edge_count(); ++i) {
      auto candidate_edge = tile->directededge(node->edge_index() + i);
      auto opp_candidate_edge = get_opposing_edge(candidate_edge, reader);
      auto ei = tile->edgeinfo(candidate_edge);
      if (edge == opp_candidate_edge)
        continue;
      if ((costing->Allowed(candidate_edge, tile, sif::kDisallowShortcut) ||
           costing->Allowed(opp_candidate_edge, reader.GetGraphTile(candidate_edge->endnode()),
                            sif::kDisallowShortcut)) &&
          !(costing->ExcludePrivate() && ei.private_access()) &&
          (1ull << static_cast<int>(candidate_edge->use()) & search_filter.use_) &&
          (static_cast<uint32_t>(candidate_edge->classification()) <=
           static_cast<uint32_t>(search_filter.min_road_class_)) &&
          (static_cast<uint32_t>(candidate_edge->classification()) >=
           static_cast<uint32_t>(search_filter.max_road_class_))) {
        allowed_cnt++;
        possible_next = candidate_edge;
      }
    }
    std::function<void(const baldr::GraphId&, const bool, const bool)> expand_forw;
    expand_forw = [&](const baldr::GraphId& trans_node_id, const bool from_transition,
                      const bool up) {
      graph_tile_ptr tile = reader.GetGraphTile(trans_node_id);
      if (tile == nullptr) {
        return;
      }
      const baldr::NodeInfo* trans_node = tile->node(trans_node_id);
      // get the count of outgoing edges
      for (int i = 0; i < trans_node->edge_count(); ++i) {
        auto candidate_edge = tile->directededge(trans_node->edge_index() + i);
        auto opp_candidate_edge = get_opposing_edge(candidate_edge, reader);
        auto ei = tile->edgeinfo(candidate_edge);
        if ((costing->Allowed(candidate_edge, tile, sif::kDisallowShortcut) ||
             costing->Allowed(opp_candidate_edge, reader.GetGraphTile(candidate_edge->endnode()),
                              sif::kDisallowShortcut)) &&
            !(costing->ExcludePrivate() && ei.private_access()) &&
            (1ull << static_cast<int>(candidate_edge->use()) & search_filter.use_) &&
            (static_cast<uint32_t>(candidate_edge->classification()) <=
             static_cast<uint32_t>(search_filter.min_road_class_)) &&
            (static_cast<uint32_t>(candidate_edge->classification()) >=
             static_cast<uint32_t>(search_filter.max_road_class_))) {
          allowed_cnt++;
          possible_next = candidate_edge;
        }
      }

      if (!from_transition && trans_node->transition_count() > 0) {
        const baldr::NodeTransition* trans = tile->transition(trans_node->transition_index());
        for (uint32_t i = 0; i < trans_node->transition_count(); ++i, ++trans) {
          if (!up == trans->up())
            continue;
          expand_forw(trans->endnode(), true, up);
        }
      }
    };
    // follow node transitions
    for (int i = 0; i < node->transition_count(); ++i) {
      const baldr::NodeTransition* trans = tile->transition(node->transition_index() + i);
      expand_forw(trans->endnode(), false, trans->up());
    }

    if (allowed_cnt == 1 && possible_next->classification() == de->classification() &&
        possible_next->use() == de->use()) {
      // keep moving forwards
      if (!(added_edges.insert(possible_next)).second) {
        LOG_WARN("Duplicate edge when finding beginning of road segment for edge.");
        break;
      } else {
        edges.push_back(possible_next);
      }

      edge = possible_next;
      node_id = possible_next->endnode();
      node = reader.GetGraphTile(node_id)->node(node_id);
    } else {
      if (allowed_cnt == 0)
        forward_deadend = true;
      break;
    }
  }

  double length = 0;
  double percent_along_total = 0;
  std::function<double(const PointLL&, const PointLL)> segment_length;
  segment_length = [&](const PointLL& from, const PointLL to) { return from.Distance(to); };
  // assemble the shape
  std::list<midgard::PointLL> concatenated_shape;
  for (auto e : edges) {
    auto opp_de = get_opposing_edge(e, reader);
    auto tile = reader.GetGraphTile(opp_de->endnode());
    auto edge_info = tile->edgeinfo(e);
    auto shape = edge_info.shape();
    if (!e->forward())
      std::reverse(shape.begin(), shape.end());
    if (concatenated_shape.size())
      concatenated_shape.pop_back();
    // walk the edge segments
    auto shape_itr = shape.begin();
    auto next_shape_itr = ++shape.begin();

    // get the edge length
    double acc_ = 0;
    for (; next_shape_itr != shape.end(); ++shape_itr, ++next_shape_itr) {
      auto dist = segment_length(*shape_itr, *next_shape_itr);
      acc_ += dist;
    }

    // if this edge is the correlated edge, calculate
    // the percent_along along the complete road segment
    if (e == de) {
      percent_along_total = length + (acc_ * percent_along);
    }
    length += acc_;
    concatenated_shape.insert(concatenated_shape.end(), shape.begin(), shape.end());
  }

  // walk the edge segments
  auto shape_itr = concatenated_shape.begin();
  auto next_shape_itr = ++concatenated_shape.begin();

  double acc_ = 0.f;
  PointLL mid;
  for (; next_shape_itr != concatenated_shape.end(); ++shape_itr, ++next_shape_itr) {
    auto dist = shape_itr->Distance(*next_shape_itr);
    acc_ += dist;

    // if we've passed the 50% threshold, stop and get the exact mid point
    if ((acc_ / length) > 0.5f) {
      auto missing_length_m = (length * 0.5f) - (acc_ - dist);
      auto frac = missing_length_m / dist;
      // LOG_ERROR("Frac=" + std::to_string(frac) + "|Dist=" + std::to_string(dist) +
      //           "|Acc=" + std::to_string(acc_) + "|Length=" + std::to_string(length));
      mid = shape_itr->PointAlongSegment(*next_shape_itr, frac);
      break;
    }
  }

  if (next_shape_itr == concatenated_shape.end()) {
    LOG_ERROR("Unexpected end of shape.");
  }

  std::string shape = midgard::encode(concatenated_shape);
  auto start_tile = reader.GetGraphTile(start_node_id);
  auto start_node = start_tile->node(start_node_id);
  auto end_tile = reader.GetGraphTile(node_id);
  auto end_node = end_tile->node(node_id);

  writer("shape", shape);

  writer.start_object("intersections");

  writer.start_object("start_node");
  writer.start_object("id");
  start_node_id.json(writer);
  writer.end_object(); // id
  writer.start_object("node");
  start_node->json(start_tile, writer);
  writer.end_object(); // node
  writer("deadend", backward_deadend);
  writer.end_object(); // start_node

  writer.start_object("end_node");
  writer.start_object("id");
  node_id.json(writer);
  writer.end_object(); // id
  writer.start_object("node");
  end_node->json(end_tile, writer);
  writer.end_object(); // node
  writer("deadend", forward_deadend);
  writer.end_object(); // end_node

  writer.end_object(); // intersections

  writer.set_precision(6);
  writer.start_object("mid_point");
  writer("lat", mid.lat());
  writer("lon", mid.lng());
  writer.end_object();

  writer.set_precision(5);
  writer("percent_along", percent_along_total / length);

  writer.set_precision(tyr::kDefaultPrecision);
}

void serialize_edges(const PathLocation& path_location,
                     const baldr::Location& location,
                     GraphReader& reader,
                     rapidjson::writer_wrapper_t& writer,
                     bool verbose,
                     bool full_road_segments,
                     sif::cost_ptr_t& costing) {
  writer.start_array("edges");
  for (const auto& edge : path_location.edges) {
    writer.start_object();
    try {
      // get the osm way id
      auto tile = reader.GetGraphTile(edge.id);
      auto* directed_edge = tile->directededge(edge.id);
      auto edge_info = tile->edgeinfo(directed_edge);
      // they want MOAR!
      if (verbose) {
        // live traffic information
        const volatile auto& traffic = tile->trafficspeed(directed_edge);

        // incident information
        if (traffic.has_incidents) {
          // TODO: incidents
        }
        writer.start_array("access_restrictions");
        serialize_access_restrictions(tile, writer, edge.id.id());
        writer.end_array();
        // write live_speed
        writer.start_object("live_speed");
        serialize_traffic_speed(traffic, writer);
        writer.end_object();

        // basic rest of it plus edge metadata
        writer.set_precision(tyr::kCoordinatePrecision);
        writer("correlated_lat", edge.projected.lat());
        writer("correlated_lon", edge.projected.lng());
        writer("side_of_street", edge.sos == PathLocation::LEFT
                                     ? "left"
                                     : (edge.sos == PathLocation::RIGHT ? "right" : "neither"));

        writer("linear_reference", linear_reference(directed_edge, edge.percent_along, edge_info));
        writer.set_precision(5);
        writer("percent_along", edge.percent_along);
        writer.set_precision(1);
        writer("distance", edge.distance);
        writer("shoulder", directed_edge->shoulder());
        writer("heading", edge.projected_heading);
        writer.set_precision(tyr::kDefaultPrecision);
        writer("outbound_reach", static_cast<int64_t>(edge.outbound_reach));
        writer("inbound_reach", static_cast<int64_t>(edge.inbound_reach));
        if (full_road_segments) {
          writer.start_object("full_road_segment");
          get_full_road_segment(writer, directed_edge, costing, edge.percent_along,
                                reader, location.search_filter_);
          writer.end_object();
        }

        writer.start_object("edge_info");
        edge_info.json(writer);
        writer.end_object();

        writer.start_object("edge");
        directed_edge->json(writer);
        writer.end_object();

        writer.start_object("edge_id");
        edge.id.json(writer);
        writer.end_object();

        // historical traffic information

        // if there's a date time on the location and the edge has
        // a predicted speed, write that
        if (location.date_time_ && !location.date_time_->empty() &&
            directed_edge->has_predicted_speed()) {
          std::string dt = location.date_time_.value();
          auto time_info = baldr::TimeInfo::make(dt, 0); // no need to pass timezone info here
          writer("timed_predicted_speed",
                 static_cast<uint64_t>(
                     tile->GetSpeed(directed_edge, kPredictedFlowMask, time_info.second_of_week)));
        }

        // in any case, to not break existing applications using this,
        // write out the whole thing if present
        writer.start_array("predicted_speeds");
        if (directed_edge->has_predicted_speed()) {
          for (uint32_t sec = 0; sec < midgard::kSecondsPerWeek; sec += 5 * midgard::kSecPerMinute) {
            writer(static_cast<uint64_t>(tile->GetSpeed(directed_edge, kPredictedFlowMask, sec)));
          }
        }
        writer.end_array();
      } // they want it lean and mean
      else {
        writer("way_id", static_cast<uint64_t>(edge_info.wayid()));
        writer.set_precision(tyr::kCoordinatePrecision);
        writer("correlated_lat", edge.projected.lat());
        writer("correlated_lon", edge.projected.lng());
        writer("side_of_street", edge.sos == PathLocation::LEFT
                                     ? "left"
                                     : (edge.sos == PathLocation::RIGHT ? "right" : "neither"));
        writer.set_precision(5);
        writer("percent_along", edge.percent_along);
        writer.set_precision(tyr::kDefaultPrecision);

        if (full_road_segments) {
          writer.start_object("full_road_segment");
          get_full_road_segment(writer, directed_edge, costing, edge.percent_along,
                                reader, location.search_filter_);
          writer.end_object();
        }
      }
    } catch (...) {
      // this really shouldnt ever get hit
      LOG_WARN("Expected edge not found in graph but found by loki::search!");
    }
    writer.end_object();
  }
  writer.end_array();
}

void serialize_nodes(const PathLocation& location,
                     GraphReader& reader,
                     rapidjson::writer_wrapper_t& writer,
                     bool verbose) {
  // get the nodes we need
  std::unordered_set<uint64_t> nodes;
  for (const auto& e : location.edges) {
    if (e.end_node()) {
      nodes.emplace(reader.GetGraphTile(e.id)->directededge(e.id)->endnode());
    }
  }
  writer.start_array("nodes");
  for (auto node_id : nodes) {
    writer.start_object();
    GraphId n(node_id);
    graph_tile_ptr tile = reader.GetGraphTile(n);
    auto* node_info = tile->node(n);

    if (verbose) {
      node_info->json(tile, writer);

      writer.start_object("node_id");
      n.json(writer);
      writer.end_object();
    } else {
      midgard::PointLL node_ll = tile->get_node_ll(n);
      writer.set_precision(tyr::kCoordinatePrecision);
      writer("lon", node_ll.first);
      writer("lat", node_ll.second);
      writer.set_precision(tyr::kDefaultPrecision);
      // TODO: osm_id
    }
    writer.end_object();
  }
  writer.end_array();
}

void serialize(rapidjson::writer_wrapper_t& writer,
               const PathLocation& path_location,
               GraphReader& reader,
               bool verbose,
               const baldr::Location& location,
               bool road_segments,
               sif::cost_ptr_t costing) {
  // serialze all the edges
  writer.start_object();
  writer.set_precision(tyr::kCoordinatePrecision);
  writer("input_lat", path_location.latlng_.lat());
  writer("input_lon", path_location.latlng_.lng());
  writer.set_precision(tyr::kDefaultPrecision);
  serialize_edges(path_location, location, reader, writer, verbose, road_segments, costing);
  serialize_nodes(path_location, reader, writer, verbose);

  writer.end_object();
}

void serialize(rapidjson::writer_wrapper_t& writer,
               const midgard::PointLL& ll,
               const std::string& reason,
               bool verbose) {
  writer.start_object();
  writer.set_precision(tyr::kCoordinatePrecision);
  writer("input_lat", ll.lat());
  writer("input_lon", ll.lng());
  writer.set_precision(tyr::kDefaultPrecision);
  writer("edges", nullptr);
  writer("nodes", nullptr);

  if (verbose) {
    writer("reason", reason);
  }
  writer.end_object();
}
} // namespace

namespace valhalla {
namespace tyr {

std::string serializeLocate(const Api& request,
                            const std::vector<baldr::Location>& locations,
                            const std::unordered_map<baldr::Location, PathLocation>& projections,
                            GraphReader& reader,
                            sif::cost_ptr_t costing) {
  rapidjson::writer_wrapper_t writer(4096);
  writer.start_array();

  for (const auto& location : locations) {
    try {
      serialize(writer, projections.at(location), reader, request.options().verbose(), location,
                request.options().road_segments(), costing);
    } catch (const std::exception& e) {
      serialize(writer, location.latlng_, "No data found for location", request.options().verbose());
    }
  }
  writer.end_array();
  return writer.get_buffer();
}

} // namespace tyr
} // namespace valhalla
