/**
 * @file types.hpp
 * @brief Basic type definitions for simple region merging
 * 
 * This is the SIMPLEST version - no optimizations, just core types.
 * Perfect as a baseline for understanding and testing.
 * 
 * @author Claude
 * @date 2025
 */

#pragma once

#include <array>
#include <cstdint>
#include <cmath>

namespace region_merging {

// =============================================================================
// Basic Types
// =============================================================================

/// Vertex ID type (index in vertex array)
using VertexID = int32_t;

/// Edge ID type (index in edge array)  
using EdgeID = int32_t;

/// Invalid ID sentinel
constexpr int32_t INVALID_ID = -1;

/// 3D position (X, Y, Z coordinates)
using Position3D = std::array<double, 3>;

/// 3D normal vector (unit length)
using Normal3D = std::array<double, 3>;

// =============================================================================
// State Enumeration
// =============================================================================

/**
 * @enum State
 * @brief Simple two-state system for vertices and edges
 * 
 * For basic algorithm, we only need:
 * - Active: Currently valid
 * - Frozen: Permanently invalid (merged or deleted)
 * 
 * Note: Original had 3 states (Active/Suspend/Freeze) for lazy activation.
 * We'll add Suspend later when implementing optimization #5.
 */
enum class State : uint8_t {
    Active = 0,   ///< Valid and participating in merging
    Frozen = 1    ///< Invalid, merged into another vertex/edge
};

// =============================================================================
// Utility Functions
// =============================================================================

/**
 * @brief Compute squared Euclidean distance between two points
 * @param p1 First point
 * @param p2 Second point
 * @return Squared distance (avoids expensive sqrt)
 * 
 * Use this for distance comparisons to avoid sqrt overhead.
 */
inline double distance_squared(const Position3D& p1, const Position3D& p2) {
    const double dx = p1[0] - p2[0];
    const double dy = p1[1] - p2[1];
    const double dz = p1[2] - p2[2];
    return dx*dx + dy*dy + dz*dz;
}

/**
 * @brief Compute Euclidean distance between two points
 * @param p1 First point
 * @param p2 Second point
 * @return Distance
 */
inline double distance(const Position3D& p1, const Position3D& p2) {
    return std::sqrt(distance_squared(p1, p2));
}

/**
 * @brief Compute dot product of two vectors
 * @param v1 First vector
 * @param v2 Second vector
 * @return Dot product
 */
inline double dot_product(const Normal3D& v1, const Normal3D& v2) {
    return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

/**
 * @brief Compute angle between two normal vectors (in radians)
 * @param n1 First normal (unit vector)
 * @param n2 Second normal (unit vector)
 * @return Angle in radians [0, π]
 */
inline double angle_between(const Normal3D& n1, const Normal3D& n2) {
    double dp = dot_product(n1, n2);
    // Clamp to handle numerical errors
    dp = std::max(-1.0, std::min(1.0, dp));
    return std::acos(dp);
}

} // namespace region_merging
