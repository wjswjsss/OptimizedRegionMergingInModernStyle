/**
 * @file vertex.hpp
 * @brief Simple vertex (region) class for basic region merging
 * 
 * This is a SIMPLIFIED version with only essential features:
 * - Position and normal
 * - Adjacency list (edge IDs)
 * - State (Active/Frozen)
 * - Union-find parent pointer for merge tracking
 * 
 * NO OPTIMIZATIONS:
 * - No direction pointer (added in optimization #3 - cycle detection)
 * - No next_in_region (added in optimization #1 - spatial partitioning)
 * - No bit_sign (added in optimization #1 - spatial partitioning)
 * - No features object (we'll compute weight directly)
 * 
 * @author Claude
 * @date 2025
 */

#pragma once

#include "types.hpp"
#include <vector>
#include <algorithm>

namespace region_merging {

/**
 * @class Vertex
 * @brief Represents a region (or initial single point) in the graph
 * 
 * Memory Layout (48 bytes + vector on 64-bit system):
 * ┌────────────────────────────────────────────┐
 * │ ID (4) | state (1) | size (4) | pad (3)   │
 * ├────────────────────────────────────────────┤
 * │ position (24 bytes)                        │
 * ├────────────────────────────────────────────┤
 * │ normal (24 bytes)                          │
 * ├────────────────────────────────────────────┤
 * │ parent (8 bytes pointer)                   │
 * ├────────────────────────────────────────────┤
 * │ edge_ids (24 bytes vector)                 │
 * └────────────────────────────────────────────┘
 * 
 * Total: ~48 bytes core + vector overhead
 */
class Vertex {
public:
    // =========================================================================
    // Core Data Members
    // =========================================================================
    
    VertexID id;           ///< Unique identifier (index in vertex array)
    State state;           ///< Active or Frozen
    int size;              ///< Number of original points in this region
    
    Position3D position;   ///< Centroid position (average of all points)
    Normal3D normal;       ///< Average normal (unit vector)
    
    Vertex* parent;        ///< Parent in union-find structure (null if root)
    
    /**
     * @brief IDs of incident edges (adjacency list)
     * 
     * Stores EdgeIDs instead of pointers because:
     * - Edge array may be reallocated during pruning
     * - IDs remain valid, pointers would be invalidated
     * 
     * Typical size: 6-8 edges for interior points
     */
    std::vector<EdgeID> edge_ids;
    
    // =========================================================================
    // Construction
    // =========================================================================
    
    /**
     * @brief Default constructor
     * 
     * Creates invalid vertex for array allocation.
     * Call init() before use.
     */
    Vertex()
        : id(INVALID_ID)
        , state(State::Frozen)  // Invalid until init()
        , size(0)
        , position{0, 0, 0}
        , normal{0, 0, 1}  // Default up
        , parent(nullptr)
    {
        edge_ids.reserve(8);  // Typical degree
    }
    
    /**
     * @brief Initialize vertex with data
     * @param vertex_id Unique ID
     * @param pos Position
     * @param norm Normal (should be unit length)
     * 
     * After init(), vertex is Active with size=1
     */
    void init(VertexID vertex_id, const Position3D& pos, const Normal3D& norm) {
        id = vertex_id;
        position = pos;
        normal = norm;
        state = State::Active;
        size = 1;
        parent = nullptr;
        edge_ids.clear();
    }
    
    // =========================================================================
    // Edge Management
    // =========================================================================
    
    /**
     * @brief Add edge to adjacency list
     * @param edge_id ID of edge to add
     * 
     * No duplicate checking for performance.
     * Caller must ensure uniqueness.
     */
    void add_edge(EdgeID edge_id) {
        edge_ids.push_back(edge_id);
    }
    
    /**
     * @brief Remove edge from adjacency list
     * @param edge_id ID of edge to remove
     * @return true if found and removed
     * 
     * Uses swap-and-pop for O(1) removal (order doesn't matter)
     */
    bool remove_edge(EdgeID edge_id) {
        auto it = std::find(edge_ids.begin(), edge_ids.end(), edge_id);
        if (it != edge_ids.end()) {
            // Swap with last element and pop
            *it = edge_ids.back();
            edge_ids.pop_back();
            return true;
        }
        return false;
    }
    
    /**
     * @brief Get vertex degree (number of incident edges)
     * @return Number of edges
     */
    size_t degree() const {
        return edge_ids.size();
    }
    
    // =========================================================================
    // Union-Find Operations
    // =========================================================================
    
    /**
     * @brief Find root vertex in union-find structure
     * @return Root vertex (parent == nullptr)
     * 
     * Implements path compression:
     * - Traverse parent pointers to root
     * - Update all intermediate parents to point directly to root
     * - Gives amortized O(α(n)) ≈ O(1) complexity
     * 
     * Example:
     *   Before: v1 → v2 → v3 → v4 (root)
     *   After:  v1 ↘
     *           v2 → v4 (root)
     *           v3 ↗
     */
    Vertex* find_root() {
        if (parent == nullptr) {
            return this;  // Already root
        }
        // Path compression: point directly to root
        parent = parent->find_root();
        return parent;
    }
    
    /**
     * @brief Check if this vertex is a root
     * @return true if root (not merged into another vertex)
     */
    bool is_root() const {
        return parent == nullptr;
    }
    
    /**
     * @brief Merge this vertex into target vertex
     * @param target Vertex to merge into (becomes new parent)
     * 
     * Updates:
     * - this->parent = target
     * - this->state = Frozen
     * - target->size += this->size
     * - target->position = weighted average
     * - target->normal = weighted average
     * 
     * Note: Does NOT update edges.
     * Caller must handle edge reactivation.
     */
    void merge_into(Vertex* target) {
        // Preconditions
        if (target == nullptr || target == this || !is_root()) {
            return;  // Invalid merge
        }
        
        // Update target with weighted averages
        double total_size = target->size + this->size;
        double w1 = target->size / total_size;  // Weight of target
        double w2 = this->size / total_size;    // Weight of this
        
        // Weighted average position
        for (int i = 0; i < 3; ++i) {
            target->position[i] = w1 * target->position[i] + w2 * this->position[i];
        }
        
        // Weighted average normal
        for (int i = 0; i < 3; ++i) {
            target->normal[i] = w1 * target->normal[i] + w2 * this->normal[i];
        }
        
        // Normalize the normal vector
        double norm_len = std::sqrt(
            target->normal[0] * target->normal[0] +
            target->normal[1] * target->normal[1] +
            target->normal[2] * target->normal[2]
        );
        if (norm_len > 1e-10) {  // Avoid division by zero
            for (int i = 0; i < 3; ++i) {
                target->normal[i] /= norm_len;
            }
        }
        
        // Update size
        target->size += this->size;
        
        // Mark this vertex as merged
        parent = target;
        state = State::Frozen;
    }
    
    // =========================================================================
    // State Queries
    // =========================================================================
    
    /// Check if vertex is active
    bool is_active() const { 
        return state == State::Active; 
    }
    
    /// Check if vertex is frozen (merged)
    bool is_frozen() const { 
        return state == State::Frozen; 
    }
};

} // namespace region_merging
