#include "sif/traincost.h"
#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "proto_conversions.h"
#include "sif/costconstants.h"
#include "sif/dynamiccost.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

/**
 * Derived class providing minimal dynamic edge costing for trains on the
 * OSM rail network. Allowed-checks look at the kTrainAccess bit only, and
 * EdgeCost returns the edge length as both cost and time. This is a
 * placeholder costing intended to prove train edges are routable before
 * layering on speed, switch penalties, gauge/electrification filters, etc.
 */
class TrainCost : public DynamicCost {
public:
  /**
   * Construct costing.
   * @param costing specified costing type and options (unused for now).
   */
  TrainCost(const Costing& costing) : DynamicCost(costing, TravelMode::kTrain, kTrainAccess) {
  }

  virtual ~TrainCost() {
  }

  /**
   * Checks if access is allowed for the provided directed edge.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const bool,
                       const EdgeLabel&,
                       const graph_tile_ptr&,
                       const baldr::GraphId&,
                       const uint64_t,
                       const uint32_t,
                       uint8_t&,
                       uint8_t&) const override {
    if (edge->is_shortcut()) {
      return false;
    }
    return (edge->forwardaccess() & kTrainAccess) != 0;
  }

  /**
   * Checks if access is allowed for an edge on the reverse path.
   */
  virtual bool AllowedReverse(const baldr::DirectedEdge*,
                              const EdgeLabel&,
                              const baldr::DirectedEdge* opp_edge,
                              const graph_tile_ptr&,
                              const baldr::GraphId&,
                              const uint64_t,
                              const uint32_t,
                              uint8_t&,
                              uint8_t&) const override {
    if (opp_edge->is_shortcut()) {
      return false;
    }
    return (opp_edge->forwardaccess() & kTrainAccess) != 0;
  }

  /**
   * Node access check — always allowed for trains in this minimal costing.
   */
  bool Allowed(const baldr::NodeInfo*) const override {
    return true;
  }

  /**
   * Edge access check used by location search filtering.
   */
  virtual bool IsAccessible(const baldr::DirectedEdge* edge) const override {
    return (edge->forwardaccess() & kTrainAccess) != 0 ||
           (edge->reverseaccess() & kTrainAccess) != 0;
  }

  bool IsClosed(const baldr::DirectedEdge*, const graph_tile_ptr&) const override {
    return false;
  }

  /**
   * Transit EdgeCost — trains here are OSM rail edges, not GTFS transit lines.
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge*,
                        const baldr::TransitDeparture*,
                        const uint32_t) const override {
    throw std::runtime_error("TrainCost::EdgeCost does not support transit edges");
  }

  /**
   * Get the cost to traverse the specified directed edge. Minimal version:
   * cost and time are both the raw edge length in meters.
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const baldr::GraphId&,
                        const graph_tile_ptr&,
                        const baldr::TimeInfo&,
                        uint8_t&) const override {
    const float length = static_cast<float>(edge->length());
    return {length, length};
  }

  /**
   * Transition cost from predecessor edge — zero for now.
   */
  virtual Cost TransitionCost(const baldr::DirectedEdge*,
                              const baldr::NodeInfo*,
                              const EdgeLabel&,
                              const baldr::graph_tile_ptr&,
                              const std::function<baldr::LimitedGraphReader()>&) const override {
    return {};
  }

  /**
   * Reverse transition cost — zero for now.
   */
  virtual Cost TransitionCostReverse(const uint32_t,
                                     const baldr::NodeInfo*,
                                     const baldr::DirectedEdge*,
                                     const baldr::DirectedEdge*,
                                     const graph_tile_ptr&,
                                     const GraphId&,
                                     const std::function<baldr::LimitedGraphReader()>&,
                                     const bool,
                                     const InternalTurn) const override {
    return {};
  }

  /**
   * A* heuristic factor. Cost is edge length (meters) so a factor of 1.0
   * keeps the heuristic admissible against a straight-line meters estimate.
   */
  virtual float AStarCostFactor() const override {
    return 1.f;
  }

  /**
   * Location search filter — accept rail-access edges, reject shortcuts and
   * GTFS transit lines.
   */
  bool Allowed(const baldr::DirectedEdge* edge, const graph_tile_ptr&, uint16_t) const override {
    if (edge->is_shortcut() || edge->IsTransitLine()) {
      return false;
    }
    return (edge->forwardaccess() & kTrainAccess) != 0 ||
           (edge->reverseaccess() & kTrainAccess) != 0;
  }
};

void ParseTrainCostOptions(const rapidjson::Document&,
                           const std::string& /*costing_options_key*/,
                           Costing* c,
                           google::protobuf::RepeatedPtrField<CodedDescription>& /*warnings*/) {
  c->set_type(Costing::train);
  c->set_name(Costing_Enum_Name(c->type()));
}

cost_ptr_t CreateTrainCost(const Costing& costing_options) {
  return std::make_shared<TrainCost>(costing_options);
}

} // namespace sif
} // namespace valhalla
