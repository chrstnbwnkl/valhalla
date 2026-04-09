#ifndef VALHALLA_SIF_TRAINCOST_H_
#define VALHALLA_SIF_TRAINCOST_H_

#include <valhalla/baldr/rapidjson_fwd.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/sif/dynamiccost.h>

namespace valhalla {
namespace sif {

/**
 * Parses the cost options from json and stores values in pbf.
 * @param doc The json request represented as a DOM tree.
 * @param costing_options_key A string representing the location in the DOM tree where the costing
 *                            options are stored.
 * @param pbf_costing A mutable protocol buffer where the parsed json values will be stored.
 */
void ParseTrainCostOptions(const rapidjson::Document& doc,
                           const std::string& costing_options_key,
                           Costing* pbf_costing,
                           google::protobuf::RepeatedPtrField<CodedDescription>& warnings);

/**
 * Create a train cost method. Routes over OSM railway edges using the
 * kTrainAccess bit. Currently uses edge length as cost and time — no speed,
 * no turn penalties, no hierarchy tuning. Intended as a minimal starting
 * point; extend with gauge/electrification/usage filtering as needed.
 * @param  options pbf with request options.
 */
cost_ptr_t CreateTrainCost(const Costing& costing);

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_TRAINCOST_H_
