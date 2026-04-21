#include "sif/traincost.h"
#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/graphtile.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/turn.h"
#include "proto_conversions.h"
#include "sif/costconstants.h"
#include "sif/dynamiccost.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

namespace {

// Fixed "typical" train speed used when edges have no usable speed tag.
constexpr uint32_t kDefaultTrainSpeedKph = 80;

// top speed used only for the A* heuristic. Must be an upper bound on the
// effective speed used in EdgeCost for the heuristic to remain admissible.
constexpr float kTopTrainSpeedMetersPerSec = 180.0f * kKPHtoMetersPerSec; // 120 kph
                                                                          //
constexpr uint8_t kMaxAssumedTrainSpeed = 140;

// usage factors: how cheap/expensive an edge is relative to a main line.
// Scaled later by the user's `railway_use_main` preference (0..1). At
// use_main = 1.0 these are applied in full; at 0.0 everything costs the
// same as a main line (i.e. no bias).
constexpr float kUsageFactorMain = 1.0f;
constexpr float kUsageFactorMilitary = 1.10f;
constexpr float kUsageFactorBranch = 1.10f;
constexpr float kUsageFactorIndustrial = 1.75f;
constexpr float kUsageFactorFreight = 1.50f;
constexpr float kUsageFactorYard = 3.0f;
constexpr float kUsageFactorSiding = 2.5f;
constexpr float kUsageFactorSpur = 3.0f;
constexpr float kUsageFactorCrossover = 2.0f;
constexpr float kUsageFactorTest = 4.0f;
constexpr float kUsageFactorTourism = 3.0f;
constexpr float kUsageFactorScience = 4.0f;
constexpr float kUsageFactorUnknown = 1.25f; // slight bias toward tagged mains

// Turn penalties (seconds) for rail transitions. Trains do not tolerate
// tight curves well; sharp turns should be used only when there is no
// alternative, and reverse/U-turn is essentially impossible outside of
// yards.
constexpr float kTurnStraight = 0.0f;
constexpr float kTurnSlight = 0.0f;
constexpr float kTurnNormal = 10.0f;
constexpr float kTurnSharp = 100.0f;
constexpr float kTurnReverse = 800.0f;

// Option ranges.
constexpr ranged_default_t<uint32_t> kRailwayPreferredGaugeRange{0, 0,
                                                                 static_cast<uint32_t>(
                                                                     RailGauge::kOther)};
constexpr ranged_default_t<float> kRailwayGaugePenaltyRange{1.0f, 3.0f, 20.0f};
constexpr ranged_default_t<float> kRailwayUseMainRange{0.0f, 0.7f, 1.0f};
constexpr ranged_default_t<uint32_t> kRailwayTrafficModeRange{0, 0,
                                                              static_cast<uint32_t>(
                                                                  RailTrafficMode::kMixed)};
constexpr ranged_default_t<uint32_t> kTrainSpeedRange{10, kMaxAssumedTrainSpeed, baldr::kMaxSpeedKph};

// Base cost options config. Trains don't meaningfully use most of these,
// but we still parse the base options so shared knobs like ignore_closures,
// shortest, etc. work.
const BaseCostingOptionsConfig kBaseCostOptsConfig{};

float usage_factor(RailUsage u) {
  switch (u) {
    case RailUsage::kMain:
      return kUsageFactorMain;
    case RailUsage::kBranch:
      return kUsageFactorBranch;
    case RailUsage::kIndustrial:
      return kUsageFactorIndustrial;
    case RailUsage::kFreight:
      return kUsageFactorFreight;
    case RailUsage::kYard:
      return kUsageFactorYard;
    case RailUsage::kSiding:
      return kUsageFactorSiding;
    case RailUsage::kSpur:
      return kUsageFactorSpur;
    case RailUsage::kCrossover:
      return kUsageFactorCrossover;
    case RailUsage::kMilitary:
      return kUsageFactorMilitary;
    case RailUsage::kTest:
      return kUsageFactorTest;
    case RailUsage::kTourism:
      return kUsageFactorTourism;
    case RailUsage::kScience:
      return kUsageFactorScience;
    case RailUsage::kUnknown:
    default:
      return kUsageFactorUnknown;
  }
}

float turn_penalty(Turn::Type t) {
  switch (t) {
    case Turn::Type::kStraight:
      return kTurnStraight;
    case Turn::Type::kSlightLeft:
    case Turn::Type::kSlightRight:
      return kTurnSlight;
    case Turn::Type::kLeft:
    case Turn::Type::kRight:
      return kTurnNormal;
    case Turn::Type::kSharpLeft:
    case Turn::Type::kSharpRight:
      return kTurnSharp;
    case Turn::Type::kReverse:
      return kTurnReverse;
  }
  return 0.0f;
}

} // namespace

/**
 * Derived class providing dynamic edge costing for trains on the OSM rail
 * network. Allowed-checks look at the kTrainAccess bit and the optional
 * traffic-mode / electrification filters. EdgeCost bases cost on a fixed
 * typical speed (real rail speeds aren't populated yet) biased by gauge
 * and usage factors. TransitionCost penalizes sharp turns and reversals.
 */
class TrainCost : public DynamicCost {
public:
  /**
   * Construct costing. Parses train-specific options from the pbf.
   */
  TrainCost(const Costing& costing) : DynamicCost(costing, TravelMode::kTrain, kTrainAccess) {
    const auto& co = costing.options();
    preferred_gauge_ = static_cast<RailGauge>(co.railway_preferred_gauge());
    gauge_penalty_factor_ = co.railway_gauge_penalty();
    use_main_ = co.railway_use_main();
    traffic_mode_filter_ = static_cast<RailTrafficMode>(co.railway_traffic_mode());
    require_electrified_ = co.railway_require_electrified();
    snap_to_station_ = !co.normal_snapping();
  }

  virtual ~TrainCost() {
  }

  /**
   * Checks if access is allowed for the provided directed edge.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const bool,
                       const EdgeLabel&,
                       const graph_tile_ptr& tile,
                       const baldr::GraphId& edgeid,
                       const uint64_t,
                       const uint32_t,
                       uint8_t&,
                       uint8_t&) const override {
    if (edge->is_shortcut()) {
      return false;
    }
    if ((edge->forwardaccess() & kTrainAccess) == 0) {
      return false;
    }
    return PassesRailFilters(edge, tile, edgeid);
  }

  /**
   * Checks if access is allowed for an edge on the reverse path.
   */
  virtual bool AllowedReverse(const baldr::DirectedEdge*,
                              const EdgeLabel&,
                              const baldr::DirectedEdge* opp_edge,
                              const graph_tile_ptr& tile,
                              const baldr::GraphId& opp_edgeid,
                              const uint64_t,
                              const uint32_t,
                              uint8_t&,
                              uint8_t&) const override {
    if (opp_edge->is_shortcut()) {
      return false;
    }
    if ((opp_edge->forwardaccess() & kTrainAccess) == 0) {
      return false;
    }
    return PassesRailFilters(opp_edge, tile, opp_edgeid);
  }

  /**
   * Node access check — always allowed for trains.
   */
  bool Allowed(const baldr::NodeInfo*) const override {
    return true;
  }

  /**
   * Trains should snap to railway stop nodes rather than arbitrary
   * points along the nearest rail edge, so a user point correlates to
   * the station it's closest to.
   */
  bool RequiresPreferredSnapNode() const override {
    return snap_to_station_;
  }

  bool IsPreferredSnapNode(const baldr::NodeInfo* node) const override {
    return node != nullptr && node->type() == NodeType::kRailwayStop;
  }

  /**
   * Edge access check used by location search filtering.
   */
  virtual bool IsAccessible(const baldr::DirectedEdge* edge) const override {
    return (edge->forwardaccess() & kTrainAccess) != 0 || (edge->reverseaccess() & kTrainAccess) != 0;
  }

  bool IsClosed(const baldr::DirectedEdge*, const graph_tile_ptr&) const override {
    return false;
  }

  virtual Cost EdgeCost(const baldr::DirectedEdge*,
                        const baldr::TransitDeparture*,
                        const uint32_t) const override {
    throw std::runtime_error("TrainCost::EdgeCost does not support transit edges");
  }

  /**
   * Get the cost to traverse the specified directed edge. Time comes from
   * length / typical train speed; cost is that time biased by gauge and
   * usage factors pulled from the edge's extended attributes.
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const baldr::GraphId& /*edgeid*/,
                        const graph_tile_ptr& tile,
                        const baldr::TimeInfo&,
                        uint8_t&) const override {
    const float length = static_cast<float>(edge->length());
    const auto speed = fixed_speed_ == baldr::kDisableFixedSpeed
                           ? (edge->speed() != 0 ? edge->speed() : kDefaultTrainSpeedKph)
                           : fixed_speed_;
    const float seconds = length / (std::min(speed, top_speed_) * kKPHtoMetersPerSec);

    float factor = 1.0f;
    if (tile && tile->header()->has_ext_directededge()) {
      const auto* ext = tile->ext_directededge(edge);

      // Usage preference: blend between "no bias" (use_main_ = 0) and the
      // full per-usage multiplier (use_main_ = 1).
      const float raw = usage_factor(ext->railway_usage());
      factor *= (1.0f - use_main_) + use_main_ * raw;

      // Gauge preference: if the user asked for a specific gauge and the
      // edge's gauge is known but different, apply the mismatch penalty.
      if (preferred_gauge_ != RailGauge::kUnknown && ext->railway_gauge() != RailGauge::kUnknown &&
          ext->railway_gauge() != preferred_gauge_) {
        factor *= gauge_penalty_factor_;
      }
    }

    return {seconds * factor, seconds};
  }

  /**
   * Transition cost from predecessor edge; uses the turn type stored on
   * the outbound edge (keyed by the predecessor's local index) to charge
   * a seconds-based penalty. Sharp turns and reversals are strongly
   * penalized; straight-through and slight turns are free.
   */
  virtual Cost TransitionCost(const baldr::DirectedEdge* edge,
                              const baldr::NodeInfo*,
                              const EdgeLabel& pred,
                              const baldr::graph_tile_ptr&,
                              const std::function<baldr::LimitedGraphReader()>&) const override {
    const uint32_t idx = pred.opp_local_idx();
    const float seconds = turn_penalty(edge->turntype(idx));
    return {seconds, seconds};
  }

  /**
   * Reverse transition cost: mirror of TransitionCost. In the reverse
   * search the "predecessor" is `pred` (outbound from node in the reverse
   * tree) and `edge` is what it came from. idx is the predecessor's local
   * index at the node.
   */
  virtual Cost TransitionCostReverse(const uint32_t idx,
                                     const baldr::NodeInfo*,
                                     const baldr::DirectedEdge*,
                                     const baldr::DirectedEdge* edge,
                                     const graph_tile_ptr&,
                                     const GraphId&,
                                     const std::function<baldr::LimitedGraphReader()>&,
                                     const bool,
                                     const InternalTurn) const override {
    const float seconds = turn_penalty(edge->turntype(idx));
    return {seconds, seconds};
  }

  /**
   * A* heuristic factor. Cost is seconds, so the factor converts distance
   * (meters) into a lower-bound cost (seconds). Use the top train speed to
   * keep the heuristic admissible.
   */
  virtual float AStarCostFactor() const override {
    return 0; // 1.0f / kTopTrainSpeedMetersPerSec;
  }

  /**
   * Location search filter — accept rail-access edges that also satisfy
   * the traffic-mode / electrification filters.
   */
  bool Allowed(const baldr::DirectedEdge* edge, const graph_tile_ptr& tile, uint16_t) const override {
    if (edge->is_shortcut() || edge->IsTransitLine()) {
      return false;
    }
    if ((edge->forwardaccess() & kTrainAccess) == 0 && (edge->reverseaccess() & kTrainAccess) == 0) {
      return false;
    }

    // The single-arg version doesn't know the edge id. Most locate/search
    // calls use the full Allowed() above; this one is only used by reach
    // probes where being slightly permissive is fine. Apply filters only
    // when we can derive the id from the tile's directed edge index.
    if (tile && tile->header()->has_ext_directededge()) {
      return PassesRailFiltersByExt(tile->ext_directededge(edge));
    }
    return true;
  }

private:
  RailGauge preferred_gauge_ = RailGauge::kUnknown;
  float gauge_penalty_factor_ = 1.0f;
  float use_main_ = 0.0f;
  RailTrafficMode traffic_mode_filter_ = RailTrafficMode::kUnknown;
  bool require_electrified_ = false;
  bool snap_to_station_ = true;

  // Shared rail-attribute filter: tile-qualified edge id form.
  bool PassesRailFilters(const baldr::DirectedEdge* de,
                         const graph_tile_ptr& tile,
                         const baldr::GraphId&) const {
    if (traffic_mode_filter_ == RailTrafficMode::kUnknown && !require_electrified_) {
      return true;
    }
    if (!tile || !tile->header()->has_ext_directededge()) {
      // No ext data — can't evaluate filters. Be permissive so routing
      // still works on legacy tiles that predate the ext section.
      return true;
    }
    return PassesRailFiltersByExt(tile->ext_directededge(de));
  }

  bool PassesRailFiltersByExt(const DirectedEdgeExt* ext) const {
    if (ext == nullptr) {
      return true;
    }
    if (traffic_mode_filter_ != RailTrafficMode::kUnknown) {
      const auto mode = ext->railway_traffic_mode();
      // Unknown traffic mode passes (don't filter out untagged edges).
      // Mixed traffic satisfies any request. Otherwise require exact match.
      if (mode != RailTrafficMode::kUnknown && mode != RailTrafficMode::kMixed &&
          mode != traffic_mode_filter_) {
        return false;
      }
    }
    if (require_electrified_) {
      const auto e = ext->railway_electrified();
      // Anything known to be not electrified fails. Unknown passes so we
      // don't accidentally kill routes through sparsely tagged regions.
      if (e == RailElectrified::kNo) {
        return false;
      }
    }
    return true;
  }
};

void ParseTrainCostOptions(const rapidjson::Document& doc,
                           const std::string& costing_options_key,
                           Costing* c,
                           google::protobuf::RepeatedPtrField<CodedDescription>& warnings) {
  c->set_type(Costing::train);
  c->set_name(Costing_Enum_Name(c->type()));
  auto* co = c->mutable_options();

  rapidjson::Value dummy;
  const auto& json = rapidjson::get_child(doc, costing_options_key.c_str(), dummy);

  ParseBaseCostOptions(json, c, kBaseCostOptsConfig, warnings);

  JSON_PBF_RANGED_DEFAULT(co, kRailwayPreferredGaugeRange, json, "/railway_preferred_gauge",
                          railway_preferred_gauge, warnings);
  JSON_PBF_RANGED_DEFAULT(co, kRailwayGaugePenaltyRange, json, "/railway_gauge_penalty",
                          railway_gauge_penalty, warnings);
  JSON_PBF_RANGED_DEFAULT(co, kRailwayUseMainRange, json, "/railway_use_main", railway_use_main,
                          warnings);
  JSON_PBF_RANGED_DEFAULT(co, kRailwayTrafficModeRange, json, "/railway_traffic_mode",
                          railway_traffic_mode, warnings);
  JSON_PBF_RANGED_DEFAULT(co, kTrainSpeedRange, json, "/top_speed", top_speed, warnings);

  // Plain bool — no range/default helper needed; proto3 default is false.
  co->set_railway_require_electrified(
      rapidjson::get<bool>(json, "/railway_require_electrified", co->railway_require_electrified()));

  // normal_snapping: if the user explicitly provided a value, store it in the
  // oneof so the action-dependent default (false for locate, true otherwise)
  // is only applied when the field is absent.
  auto normal_snapping = rapidjson::get_optional<bool>(json, "/normal_snapping");
  if (normal_snapping) {
    co->set_normal_snapping(*normal_snapping);
  }
}

cost_ptr_t CreateTrainCost(const Costing& costing_options) {
  return std::make_shared<TrainCost>(costing_options);
}

} // namespace sif
} // namespace valhalla
