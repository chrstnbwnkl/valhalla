#include <algorithm>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/multi_array.hpp>
#include <boost/multi_array/subarray.hpp>

#include "midgard/util.h"
#include "sif/costconstants.h"
#include "sif/recost.h"
#include "thor/Hungarian.h"
#include "thor/chinese_postman_graph.h"
#include "thor/worker.h"
#include "tyr/serializers.h"

using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::sif;

namespace valhalla {
namespace thor {

typedef boost::multi_array<double, 2> DistanceMatrix;
typedef DistanceMatrix::index DistanceMatrixIndex;

typedef boost::multi_array<std::vector<int>, 2> PathMatrix;
typedef PathMatrix::index PathMatrixIndex;

std::vector<std::pair<int, int>> getNodePairs(PathMatrix pm, int startIndex, int endIndex) {
  std::vector<std::pair<int, int>> nodePairs;
  auto path = pm[startIndex][endIndex];
  for (int i = 0; i < path.size() - 1; i++) {
    nodePairs.push_back(make_pair(path[i], path[i + 1]));
  }
  // Add the last edge
  nodePairs.push_back(make_pair(path.back(), endIndex));
  return nodePairs;
}

midgard::PointLL to_ll(const valhalla::Location& l) {
  return midgard::PointLL{l.ll().lng(), l.ll().lat()};
}
midgard::PointLL thor_worker_t::getPointLL(baldr::GraphId node) {
  const NodeInfo* ni_start = reader->nodeinfo(node);
  graph_tile_ptr tile = reader->GetGraphTile(node);
  return ni_start->latlng(tile->header()->base_ll());
}

inline float find_percent_along(const valhalla::Location& location, const GraphId& edge_id) {
  for (const auto& e : location.path_edges()) {
    if (e.graph_id() == edge_id)
      return e.percent_along();
  }
  throw std::logic_error("Could not find candidate edge for the location");
}

// Return the index of an edge compared to the path_edge from a location. Assuming the path_edge is
// ordered by the best.
int get_node_candidate_index(const valhalla::Location& location,
                             const GraphId& edge_id,
                             int* percent_along) {
  int i = 0;
  for (const auto& e : location.path_edges()) {
    if (e.graph_id() == edge_id) {
      *percent_along = e.percent_along();
      return i;
    }
    i++;
  }
  return i;
}

std::vector<PathInfo> buildPath(GraphReader& graphreader,
                                const Options& /*options*/,
                                const valhalla::Location& origin,
                                const valhalla::Location& dest,
                                const baldr::TimeInfo& time_info,
                                const bool invariant,
                                std::vector<GraphId> path_edges,
                                const std::shared_ptr<sif::DynamicCost>& costing_,
                                float source_pct,
                                float target_pct) {
  // Build a vector of path info
  // once we recovered the whole path we should construct list of PathInfo objects
  // set of edges recovered from shortcuts (excluding shortcut's start edges)
  std::unordered_set<GraphId> recovered_inner_edges;

  std::vector<PathInfo> path;
  path.reserve(path_edges.size());

  auto edge_itr = path_edges.begin();
  const auto edge_cb = [&edge_itr, &path_edges]() {
    return (edge_itr == path_edges.end()) ? GraphId{} : (*edge_itr++);
  };

  const auto label_cb = [&path, &recovered_inner_edges](const EdgeLabel& label) {
    path.emplace_back(label.mode(), label.cost(), label.edgeid(), 0, label.path_distance(),
                      label.restriction_idx(), label.transition_cost(),
                      recovered_inner_edges.count(label.edgeid()));
  };

  // float source_pct = 0;
  // try {
  //   source_pct = find_percent_along(origin, path_edges.front());
  // } catch (...) {
  //   throw std::logic_error("CP - Could not find candidate edge used for origin label");
  // }

  // float target_pct = 0;
  // try {
  //   target_pct = find_percent_along(dest, path_edges.back());
  // } catch (...) {
  //   throw std::logic_error("CP - Could not find candidate edge used for destination label");
  // }

  // recost edges in final path; ignore access restrictions
  try {
    sif::recost_forward(graphreader, *costing_, edge_cb, label_cb, source_pct, target_pct, time_info,
                        invariant, true);
  } catch (const std::exception& e) {
    LOG_ERROR(std::string("Chinese Postman failed to recost final path: ") + e.what());
  }

  return path;
}

PathMatrix computeFloydWarshall(DistanceMatrix& dm) {
  if (dm.shape()[0] == dm.shape()[1]) {
    // create path matrix
    PathMatrix pm(boost::extents[dm.shape()[0]][dm.shape()[0]]);
    // populate the path matrix
    for (int i = 0; i < pm.shape()[0]; i++) {
      for (int j = 0; j < pm.shape()[0]; j++) {
        if (dm[i][j] == valhalla::thor::NOT_CONNECTED) {
          pm[i][j] = std::vector<int>{};
        } else {
          pm[i][j] = std::vector<int>{i};
        }
      }
    }

    for (int k = 0; k < dm.shape()[0]; k++) {
      for (int i = 0; i < dm.shape()[0]; i++) {
        for (int j = 0; j < dm.shape()[0]; j++) {
          if (i == j || j == k || k == i) {
            continue;
          }
          bool is_connected = (dm[i][k] != valhalla::thor::NOT_CONNECTED &&
                               dm[k][j] != valhalla::thor::NOT_CONNECTED);
          if (!is_connected) {
            continue;
          } else {
            double alt_distance = dm[i][k] + dm[k][j];
            if (alt_distance < dm[i][j] && is_connected) {
              dm[i][j] = alt_distance;
              // Update path matrix here.
              std::vector<int> new_path;
              new_path.reserve(pm[i][k].size() + pm[k][j].size());
              new_path.insert(new_path.end(), pm[i][k].begin(), pm[i][k].end());
              new_path.insert(new_path.end(), pm[k][j].begin(), pm[k][j].end());
              pm[i][j] = new_path;
            }
          }
        }
      }
    }
    return pm;
  }
}

bool isStronglyConnectedGraph(DistanceMatrix& dm) {
  for (int i = 0; i < dm.shape()[0]; i++) {
    for (int j = 0; j < dm.shape()[0]; j++) {
      if (dm[i][j] == valhalla::thor::NOT_CONNECTED) {
        return false;
      }
    }
  }
  return true;
}

double getEdgeCost(GraphReader& reader, baldr::GraphId edge_id) {
  Cost cost{};
  // fetch the graph objects
  graph_tile_ptr tile;
  const baldr::DirectedEdge* edge = reader.directededge(edge_id, tile);
  // Notes: Use edge length for now
  // uint8_t flow_sources;
  // // Update the time information even if time is invariant to account for timezones
  // const auto seconds_offset = invariant ? 0.f : cost.secs;
  // const auto offset_time =
  //       node ? time_info.forward(seconds_offset, static_cast<int>(node->timezone())) : time_info;
  // cost = costing.EdgeCost(edge, tile, offset_time.second_of_week, flow_sources); // * edge_pct;
  return edge->length();
}

void thor_worker_t::chinese_postman(Api& request) {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request, "thor_worker_t::isochrones");

  baldr::DateTime::tz_sys_info_cache_t tz_cache_;

  auto correlated = request.options().locations();
  auto it = correlated.begin();
  auto origin = &it;
  valhalla::Location originLocation = **origin;

  it++;
  auto destination = &it;
  valhalla::Location destinationLocation = **destination;

  midgard::PointLL originPoint = to_ll(originLocation);
  midgard::PointLL destinationPoint = to_ll(destinationLocation);

  ChinesePostmanGraph G;
  // Only for auto for now
  const auto& costing_ = mode_costing[Costing::auto_];

  parse_locations(request);
  auto costing = parse_costing(request);
  auto& options = *request.mutable_options();

  auto* co = options.mutable_costing_options(options.costing());
  std::list<std::string> avoid_edge_ids;

  for (auto& avoid_edge : co->exclude_edges()) {
    avoid_edge_ids.push_back(std::to_string(GraphId(avoid_edge.id())));
  }

  int currentOriginNodeIndex = originLocation.path_edges().size();
  CPVertex originVertex;
  int candidateOriginNodeIndex;
  int originPercentAlong;

  int currentDestinationNodeIndex = destinationLocation.path_edges().size();
  CPVertex destinationVertex;
  int candidateDestinationNodeIndex;
  int destinationPercentAlong;

  // Add chinese edges to internal set
  for (auto& edge : co->chinese_edges()) {
    // Exclude the edge if the edge is in avoid_edges
    bool excluded = (std::find(avoid_edge_ids.begin(), avoid_edge_ids.end(),
                               std::to_string(GraphId(edge.id()))) != avoid_edge_ids.end());
    if (excluded)
      continue;

    GraphId start_node = reader->edge_startnode(GraphId(edge.id()));
    CPVertex start_vertex = CPVertex(start_node);
    GraphId end_node = reader->edge_endnode(GraphId(edge.id()));
    CPVertex end_vertex = CPVertex(end_node);

    // Find the vertex for the origin location
    candidateOriginNodeIndex =
        get_node_candidate_index(originLocation, GraphId(edge.id()), &originPercentAlong);
    if (candidateOriginNodeIndex < currentOriginNodeIndex) {
      if (originPercentAlong < 0.5) {
        originVertex = start_vertex;
      } else {
        originVertex = end_vertex;
      }

      currentOriginNodeIndex = candidateOriginNodeIndex;
    }
    G.addVertex(start_vertex);

    // Find the vertex for the destination
    candidateDestinationNodeIndex =
        get_node_candidate_index(destinationLocation, GraphId(edge.id()), &destinationPercentAlong);
    if (candidateDestinationNodeIndex < currentDestinationNodeIndex) {
      // Check this part of the code
      if (destinationPercentAlong < 0.5) {
        destinationVertex = start_vertex;
      } else {
        destinationVertex = end_vertex;
      }
      currentDestinationNodeIndex = candidateDestinationNodeIndex;
    }
    G.addVertex(end_vertex);

    // The cost of an edge is not relevant for the graph since we need to visit all the edges.
    // For a simplicity, I put Cost(1, 1) for it.
    // The cost is only considered when matching the unbalanced nodes.
    // TODO: probably remove this since we use edge length as the heuristic
    Cost cost(1, 1);
    CPEdge cpEdge(cost, baldr::GraphId(edge.id()));
    G.addEdge(start_vertex, end_vertex, cpEdge);
  }
  // If the node index is more than the path_edge size, that means that there is no suitable
  // node for the origin or destination location.
  if (currentOriginNodeIndex >= originLocation.path_edges().size()) {
    throw valhalla_exception_t(451);
  }
  if (currentDestinationNodeIndex >= destinationLocation.path_edges().size()) {
    throw valhalla_exception_t(451);
  }

  bool isSameOriginDestination = destinationVertex.graph_id == originVertex.graph_id;

  // Solving the Chinese Postman
  std::vector<GraphId> edgeGraphIds;

  // Check if the graph is ideal or not
  if (G.isIdealGraph(originVertex, destinationVertex)) {
    edgeGraphIds = G.computeIdealEulerCycle(originVertex);
  } else {
    DistanceMatrix distanceMatrix(boost::extents[G.numVertices()][G.numVertices()]);
    for (int i = 0; i < G.numVertices(); i++) {
      for (int j = 0; j < G.numVertices(); j++) {
        if (i == j) {
          distanceMatrix[i][j] = 0;
        } else {
          auto* cp_edge = G.getCPEdge(i, j);
          if (cp_edge) {
            distanceMatrix[i][j] = getEdgeCost(*reader, cp_edge->graph_id);
          } else {
            distanceMatrix[i][j] = valhalla::thor::NOT_CONNECTED;
          }
        }
      }
    }

    PathMatrix pm = computeFloydWarshall(distanceMatrix);

    // Check if the graph is not strongly connected
    if (!isStronglyConnectedGraph(distanceMatrix)) {
      throw valhalla_exception_t(450);
    }

    // Do matching here

    // A flag to check whether we already evaluate the origin and destination nodes
    bool originNodeChecked = false;
    bool destinationNodeChecked = false;

    // Populate the list of node which has too many incoming and too few incoming
    std::vector<baldr::GraphId> overNodes;
    std::vector<baldr::GraphId> underNodes;
    for (auto const& v : G.getUnbalancedVertices()) {
      // Calculate the number of needed edges to make it balanced
      int extraEdges = 0;
      if (!isSameOriginDestination && v.first == originVertex.vertex_id) {
        extraEdges = abs(v.second + 1);
        originNodeChecked = true;
      } else if (!isSameOriginDestination && v.first == destinationVertex.vertex_id) {
        extraEdges = abs(v.second - 1);
        destinationNodeChecked = true;
      } else {
        extraEdges = abs(v.second);
      }
      for (int i = 0; i < extraEdges; i++) {
        if (v.second > 0) {
          overNodes.push_back(GraphId(v.first));
        } else {
          underNodes.push_back(GraphId(v.first));
        }
      }
    }
    // Handle if the origin or destination nodes are not managed yet
    if (!isSameOriginDestination) {
      if (!originNodeChecked) {
        overNodes.push_back(originVertex.graph_id);
      }
      if (!destinationNodeChecked) {
        underNodes.push_back(destinationVertex.graph_id);
      }
    }
    // Populating matrix for pairing
    std::vector<std::vector<double>> pairingMatrix;
    for (int i = 0; i < overNodes.size(); i++) {
      pairingMatrix.push_back(std::vector<double>{});
      for (int j = 0; j < underNodes.size(); j++) {
        int overNodeIndex = G.getVertexIndex(overNodes[i]);
        int underNodeIndex = G.getVertexIndex(underNodes[j]);
        double distance = distanceMatrix[overNodeIndex][underNodeIndex];
        pairingMatrix[i].push_back(distance);
      }
    }
    // Calling hungarian algorithm
    HungarianAlgorithm hungarian_algorithm;
    vector<int> assignment;
    double cost = hungarian_algorithm.Solve(pairingMatrix, assignment);
    std::vector<std::pair<int, int>> extraPairs;
    for (unsigned int x = 0; x < pairingMatrix.size(); x++) {
      // Get node's index for that pair
      int overNodeIndex = G.getVertexIndex(overNodes[x]);
      int underNodeIndex = G.getVertexIndex(underNodes[assignment[x]]);
      // Expand the path between the paired nodes, using the path matrix
      auto nodePairs = getNodePairs(pm, overNodeIndex, underNodeIndex);
      // Concat with main vector
      extraPairs.insert(extraPairs.end(), nodePairs.begin(), nodePairs.end());
    }
    edgeGraphIds = G.computeIdealEulerCycle(originVertex, extraPairs);
  }
  // Start build path here
  bool invariant = options.has_date_time_type() && options.date_time_type() == Options::invariant;
  auto time_info = TimeInfo::make(originLocation, *reader, &tz_cache_);
  std::vector<PathInfo> path =
      buildPath(*reader, options, originLocation, destinationLocation, time_info, invariant,
                edgeGraphIds, costing_, originPercentAlong, destinationPercentAlong);

  std::list<valhalla::Location> throughs; // Empty
  std::vector<std::string> algorithms{"Chinese Postman"};
  TripRoute* route = nullptr;
  valhalla::Trip& trip = *request.mutable_trip();
  // Form output information based on path edges
  if (trip.routes_size() == 0 || options.alternates() > 0) {
    route = trip.mutable_routes()->Add();
    route->mutable_legs()->Reserve(options.locations_size());
  }
  auto& leg = *route->mutable_legs()->Add();
  std::unordered_map<size_t, std::pair<EdgeTrimmingInfo, EdgeTrimmingInfo>> vias; // Empty
  TripLegBuilder::Build(options, controller, *reader, mode_costing, path.begin(), path.end(),
                        originLocation, destinationLocation, throughs, leg, algorithms, interrupt,
                        &vias);
}

} // namespace thor
} // namespace valhalla
