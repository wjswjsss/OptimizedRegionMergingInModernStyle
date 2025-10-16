/**
 * @file edge.hpp
 * @brief Simple edge class for basic region merging
 * 
 * Represents an adjacency relationship between two vertices.
 * 
 * SIMPLIFIED VERSION - only essential features:
 * - Two vertex endpoints
 * - Merge weight (dissimilarity)
 * - State (Active/Frozen)
 * 
 * NO OPTIMIZATIONS:
 * - No Suspend state (added in optimization #5 - lazy activation)
 * - No shortest_arc_num (added in optimization #3 - NNG/cycle detection)
 * - No length counter (added for multi-edge handling)
 * - No next_edge_in_region (added in optimization #1 - spatial partitioning)
 * 
 * @author Claude
 * @date 2025
 */

#pragma once

#include "types.hpp"
#include "vertex.hpp"

namespace region_merging {

/**
 * @class Edge
 * @brief Represents adjacency between two vertices
 * 
 * Memory Layout (32 bytes on 64-bit system):
 * ┌────────────────────────────────────────────┐
 * │ ID (4) | state (1) | padding (3)           │
 * ├────────────────────────────────────────────┤
 * │ weight (8 bytes double)                    │
 * ├────────────────────────────────────────────┤
 * │ left_vertex (8 bytes pointer)              │
 * ├────────────────────────────────────────────┤
 * │ right_vertex (8 bytes pointer)             │
 * └────────────────────────────────────────────┘
 * 
 * Total: 32 bytes (cache line friendly!)
 */
class Edge {
public:
    // =========================================================================
    // Core Data Members
    // =========================================================================
    
    EdgeID id;              ///< Unique identifier (index in edge array)
    State state;            ///< Active or Frozen
    
    double weight;          ///< Merge cost (dissimilarity measure)
    
    Vertex* left_vertex;    ///< One endpoint
    Vertex* right_vertex;   ///< Other endpoint
    
    // =========================================================================
    // Construction
    // =========================================================================
    
    /**
     * @brief Default constructor
     * 
     * Creates invalid edge for array allocation.
     * Call init() before use.
     */
    Edge()
        : id(INVALID_ID)
        , state(State::Frozen)
        , weight(0.0)
        , left_vertex(nullptr)
        , right_vertex(nullptr)
    {}
    
    /**
     * @brief Initialize edge with vertices
     * @param edge_id Unique ID
     * @param v1 First vertex
     * @param v2 Second vertex
     * @param w Initial weight (dissimilarity)
     * 
     * After init(), edge is Active and connects v1-v2
     */
    void init(EdgeID edge_id, Vertex* v1, Vertex* v2, double w) {
        id = edge_id;
        left_vertex = v1;
        right_vertex = v2;
        weight = w;
        state = State::Active;
    }
    
    // =========================================================================
    // Queries
    // =========================================================================
    
    /**
     * @brief Get the other vertex of this edge
     * @param v One vertex of the edge
     * @return The other vertex, or nullptr if v is not an endpoint
     * 
     * Usage: edge connects v1-v2, calling get_other(v1) returns v2
     */
    Vertex* get_other_vertex(const Vertex* v) const {
        if (v == left_vertex) return right_vertex;
        if (v == right_vertex) return left_vertex;
        return nullptr;  // v is not an endpoint
    }
    
    /**
     * @brief Check if edge connects two specific vertices
     * @param v1 First vertex
     * @param v2 Second vertex
     * @return true if edge connects v1 and v2 (in either order)
     */
    bool connects(const Vertex* v1, const Vertex* v2) const {
        return (left_vertex == v1 && right_vertex == v2) ||
               (left_vertex == v2 && right_vertex == v1);
    }
    
    /**
     * @brief Check if edge has vertex as an endpoint
     * @param v Vertex to check
     * @return true if v is left_vertex or right_vertex
     */
    bool has_vertex(const Vertex* v) const {
        return left_vertex == v || right_vertex == v;
    }
    
    // =========================================================================
    // State Queries
    // =========================================================================
    
    /// Check if edge is active
    bool is_active() const {
        return state == State::Active;
    }
    
    /// Check if edge is frozen (deleted)
    bool is_frozen() const {
        return state == State::Frozen;
    }
    
    // =========================================================================
    // Update Operations
    // =========================================================================
    
    /**
     * @brief Update edge after vertex merge
     * @param edges Global edge array
     * @return New state of edge
     * 
     * After merging vertices, edges need to be updated:
     * 1. Update endpoints to root vertices (union-find)
     * 2. If both endpoints merged into same vertex → Freeze (self-loop)
     * 3. Otherwise keep Active
     * 
     * Note: Weight is NOT recomputed here.
     * Caller can recompute weight if needed.
     */
    State update_after_merge() {
        // Update endpoints to root vertices
        left_vertex = left_vertex->find_root();
        right_vertex = right_vertex->find_root();
        
        // Check if it's now a self-loop
        if (left_vertex == right_vertex) {
            state = State::Frozen;
            return state;
        }
        
        // Still valid edge
        state = State::Active;
        return state;
    }
};

/**
 * @brief Comparator for priority queue (min-heap by weight)
 * 
 * Used by std::priority_queue to order edges by increasing weight.
 * We want smallest weight (most similar regions) first.
 * 
 * Note: priority_queue is a max-heap by default, so we reverse comparison.
 */
struct EdgeWeightComparator {
    bool operator()(const Edge* a, const Edge* b) const {
        // Return true if a should come AFTER b (reverse for min-heap)
        return a->weight > b->weight;
    }
};

} // namespace region_merging
