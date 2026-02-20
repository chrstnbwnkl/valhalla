#pragma once

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/thor/edgestatus.h>

#include <ankerl/unordered_dense.h>

#include <cassert>
#include <cstring>
#include <memory_resource>

// handy macro for shifting the 7bit path index value so that it can be or'd with the tile/level id
#ifndef SHIFT_path_id
#define SHIFT_path_id(x) (static_cast<uint32_t>(x) << 25u)
#endif

namespace valhalla {
namespace thor {

/**
 * Pool-backed EdgeStatus for use with CostMatrix.
 *
 * Allocates EdgeStatusInfo arrays from a pmr::memory_resource (typically an
 * unsynchronized_pool_resource owned by CostMatrix). Individual arrays are
 * never deallocated; instead the owning pool is released in bulk via
 * pool_.release() when CostMatrix::Clear() is called.
 *
 * This avoids per-tile new[]/delete[] churn during expansion and keeps all
 * edge status memory in a contiguous pool for better cache behavior.
 */
class PoolEdgeStatus {
public:
  explicit PoolEdgeStatus(std::pmr::memory_resource* mr) : mr_(mr) {
  }

  PoolEdgeStatus(const PoolEdgeStatus&) = delete;
  PoolEdgeStatus& operator=(const PoolEdgeStatus&) = delete;

  PoolEdgeStatus(PoolEdgeStatus&& other) noexcept
      : edgestatus_(std::move(other.edgestatus_)), mr_(other.mr_) {
    other.edgestatus_.clear();
  }

  PoolEdgeStatus& operator=(PoolEdgeStatus&& other) noexcept {
    if (this != &other) {
      // don't deallocate — pool owns the memory
      edgestatus_ = std::move(other.edgestatus_);
      mr_ = other.mr_;
      other.edgestatus_.clear();
    }
    return *this;
  }

  ~PoolEdgeStatus() = default; // pool owns the memory, nothing to free

  /**
   * Drop all tile references. The underlying memory is NOT freed here;
   * it is reclaimed when the owning pool_resource is released.
   */
  void clear() {
    edgestatus_.clear();
  }

  void Set(const baldr::GraphId& edgeid,
           const EdgeSet set,
           const uint32_t index,
           const baldr::graph_tile_ptr& tile,
           const uint8_t path_id = 0) {
    assert(path_id <= baldr::kMaxMultiPathId);
    auto key = edgeid.tile_value() | SHIFT_path_id(path_id);
    auto p = edgestatus_.find(key);
    if (p != edgestatus_.end()) {
      p->second[edgeid.id()] = {set, index};
    } else {
      auto* arr = allocate_tile(tile->header()->directededgecount());
      auto inserted = edgestatus_.emplace(key, arr);
      inserted.first->second[edgeid.id()] = {set, index};
    }
  }

  void Update(const baldr::GraphId& edgeid, const EdgeSet set, const uint8_t path_id = 0) {
    assert(path_id <= baldr::kMaxMultiPathId);
    const auto p = edgestatus_.find(edgeid.tile_value() | SHIFT_path_id(path_id));
    if (p != edgestatus_.end()) {
      p->second[edgeid.id()].set_ = static_cast<uint32_t>(set);
    } else {
      throw std::runtime_error("PoolEdgeStatus Update on edge not previously set");
    }
  }

  EdgeStatusInfo Get(const baldr::GraphId& edgeid, const uint8_t path_id = 0) const {
    assert(path_id <= baldr::kMaxMultiPathId);
    const auto p = edgestatus_.find(edgeid.tile_value() | SHIFT_path_id(path_id));
    return (p == edgestatus_.end()) ? EdgeStatusInfo() : p->second[edgeid.id()];
  }

  EdgeStatusInfo*
  GetPtr(const baldr::GraphId& edgeid, const baldr::graph_tile_ptr& tile, const uint8_t path_id = 0) {
    assert(path_id <= baldr::kMaxMultiPathId);
    auto key = edgeid.tile_value() | SHIFT_path_id(path_id);
    const auto p = edgestatus_.find(key);
    if (p != edgestatus_.end()) {
      return &p->second[edgeid.id()];
    } else {
      auto* arr = allocate_tile(tile->header()->directededgecount());
      auto inserted = edgestatus_.emplace(key, arr);
      return &inserted.first->second[edgeid.id()];
    }
  }

private:
  /**
   * Allocate a zero-initialized array of EdgeStatusInfo from the pool.
   * Zero-init guarantees EdgeSet::kUnreachedOrReset (0) as default.
   */
  EdgeStatusInfo* allocate_tile(uint32_t edge_count) {
    auto bytes = edge_count * sizeof(EdgeStatusInfo);
    auto* arr = static_cast<EdgeStatusInfo*>(mr_->allocate(bytes, alignof(EdgeStatusInfo)));
    std::memset(arr, 0, bytes);
    return arr;
  }

  ankerl::unordered_dense::map<uint32_t, EdgeStatusInfo*> edgestatus_;
  std::pmr::memory_resource* mr_;
};

} // namespace thor
} // namespace valhalla
