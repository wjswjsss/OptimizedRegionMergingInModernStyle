/**
 * @file region_adjacency_graph.hpp
 * @brief Region Adjacency Graph with Rabbani's criterion and PCL KD-tree
 * 
 * UPDATES FROM BASELINE:
 * - Uses PCL's KD-tree for O(n log n) neighbor search (was O(n²))
 * - Uses Rabbani's smoothness criterion (curvature-based)
 * - Adaptive threshold (98th percentile, pre-computed)
 * - Compatible with Semantic3D evaluation pipeline
 * 
 * Algorithm remains same:
 * 1. Build initial graph with KD-tree
 * 2. Compute edge weights using Rabbani's criterion
 * 3. Priority queue based merging
 * 4. Output segment IDs
 * 
 * @author Claude
 * @date 2025
 */

#pragma once

#include "../core/types.hpp"
#include "../core/vertex.hpp"
#include "../core/edge.hpp"
#include "../io/point_cloud_pcl.hpp"

#include <vector>
#include <queue>
#include <memory>
#include <iostream>

namespace region_merging {

/**
 * @class RegionAdjacencyGraph
 * @brief RAG with Rabbani's criterion and PCL KD-tree optimization
 * 
 * Key differences from baseline:
 * - Neighbor search: KD-tree (10-20× faster)
 * - Weight function: Rabbani's smoothness + curvature threshold
 * - Output: Compatible with evaluate.py
 */
class RegionAdjacencyGraph {
public:
    // =========================================================================
    // Construction
    // =========================================================================
    
    /**
     * @brief Constructor
     * @param cloud PCL point cloud with normals and curvature
     * @param search_radius Neighborhood radius for edge construction
     * 
     * Builds initial graph using KD-tree (fast!)
     */
    RegionAdjacencyGraph(const PointCloudPCL& cloud, double search_radius);
    
    /**
     * @brief Destructor
     */
    ~RegionAdjacencyGraph() = default;
    
    // =========================================================================
    // Main Algorithm
    // =========================================================================
    
    /**
     * @brief Run region merging with Rabbani's criterion
     * @param curvature_threshold Maximum allowed curvature (98th percentile)
     * @param angle_threshold Maximum allowed angle between normals (radians)
     * @return Pointer to final root vertex
     * 
     * Rabbani's criterion:
     *   Merge if:
     *     1. Curvature at merge boundary < threshold
     *     2. Angle between normals < threshold
     *     3. Smoothness constraint satisfied
     */
    Vertex* run(double curvature_threshold, double angle_threshold);
    
    // =========================================================================
    // Output
    // =========================================================================
    
    /**
     * @brief Get segment ID for each point
     * @return Vector of segment IDs (compatible with evaluate.py)
     */
    std::vector<int> get_segment_ids() const;
    
    /**
     * @brief Count number of final segments
     */
    int count_segments() const;
    
    /**
     * @brief Save segmentation to file
     * @param filename Output path
     * @param original_cloud Original point cloud
     * @return true if successful
     */
    bool save_segmentation(const std::string& filename, 
                          const PointCloudPCL& original_cloud) const;
    
    // =========================================================================
    // Statistics
    // =========================================================================
    
    int vertex_count() const { return num_vertices; }
    int edge_count() const { return num_edges; }
    void print_stats() const;
    
private:
    // =========================================================================
    // Helper Functions
    // =========================================================================
    
    /**
     * @brief Build edges using PCL KD-tree
     * @param cloud Point cloud with KD-tree
     * @param search_radius Neighborhood radius
     * 
     * Uses KD-tree radius search: O(n log n) instead of O(n²)
     */
    void build_edges_kdtree(const PointCloudPCL& cloud, double search_radius);
    
    /**
     * @brief Compute edge weight using Rabbani's criterion
     * @param v1 First vertex
     * @param v2 Second vertex
     * @return Weight (lower = more similar, should merge first)
     * 
     * Rabbani's smoothness criterion:
     *   weight = distance + k_angle * angle_diff + k_curv * max_curvature
     * 
     * Where:
     *   distance = Euclidean distance between centroids
     *   angle_diff = angle between normals
     *   max_curvature = max(v1.curvature, v2.curvature)
     */
    double compute_edge_weight_rabbani(const Vertex* v1, const Vertex* v2) const;
    
    /**
     * @brief Merge two vertices
     */
    void merge_vertices_on_edge(Edge* e);
    
    /**
     * @brief Update edges after merge
     */
    std::vector<Edge*> update_edges_after_merge(Vertex* merged_vertex,
                                                 Vertex* removed_vertex);
    
    bool is_self_loop(const Edge* e) const;
    Edge* find_duplicate_edge(const Edge* e) const;
    
    // =========================================================================
    // Data Members
    // =========================================================================
    
    std::unique_ptr<Vertex[]> vertices;
    int num_vertices;
    
    std::unique_ptr<Edge[]> edges;
    int num_edges;
    
    double search_radius;
    
    // Rabbani's criterion weights
    static constexpr double ANGLE_WEIGHT = 0.5;
    static constexpr double CURVATURE_WEIGHT = 1.0;
    
    // Store curvature for vertices (copied from point cloud)
    std::vector<double> vertex_curvatures;
};

} // namespace region_merging
