/**
 * @file point_cloud.hpp
 * @brief Simple point cloud data structure
 * 
 * Minimal point cloud representation for basic region merging.
 * Just stores positions and normals.
 * 
 * @author Claude
 * @date 2025
 */

#pragma once

#include "../core/types.hpp"
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

namespace region_merging {

/**
 * @struct Point
 * @brief Single point with position and normal
 */
struct Point {
    Position3D position;  ///< XYZ coordinates
    Normal3D normal;      ///< Surface normal (unit vector)
    
    Point() : position{0,0,0}, normal{0,0,1} {}
    Point(const Position3D& pos, const Normal3D& norm) 
        : position(pos), normal(norm) {}
};

/**
 * @class PointCloud
 * @brief Container for point cloud data
 * 
 * Simple vector-based storage.
 * Future: Can extend with KD-tree, octree, etc.
 */
class PointCloud {
public:
    std::vector<Point> points;  ///< All points in cloud
    
    /**
     * @brief Default constructor
     */
    PointCloud() = default;
    
    /**
     * @brief Get number of points
     * @return Point count
     */
    size_t size() const {
        return points.size();
    }
    
    /**
     * @brief Check if empty
     * @return true if no points
     */
    bool empty() const {
        return points.empty();
    }
    
    /**
     * @brief Reserve space for points (optimization)
     * @param capacity Number of points to reserve
     */
    void reserve(size_t capacity) {
        points.reserve(capacity);
    }
    
    /**
     * @brief Add a point
     * @param pos Position
     * @param norm Normal
     */
    void add_point(const Position3D& pos, const Normal3D& norm) {
        points.emplace_back(pos, norm);
    }
    
    /**
     * @brief Load from simple text file (CSV format)
     * @param filename Path to file
     * @return true if successful
     * 
     * File format: X,Y,Z,NX,NY,NZ (one point per line)
     * Example:
     *   1.5,2.3,0.8,0.0,0.0,1.0
     *   1.6,2.4,0.9,0.1,0.0,0.9
     */
    bool load_from_csv(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Cannot open file " << filename << std::endl;
            return false;
        }
        
        points.clear();
        std::string line;
        int line_num = 0;
        
        while (std::getline(file, line)) {
            line_num++;
            
            // Skip empty lines and comments
            if (line.empty() || line[0] == '#') continue;
            
            std::istringstream iss(line);
            Position3D pos;
            Normal3D norm;
            char comma;
            
            // Parse: X,Y,Z,NX,NY,NZ
            if (iss >> pos[0] >> comma >> pos[1] >> comma >> pos[2] >> comma
                    >> norm[0] >> comma >> norm[1] >> comma >> norm[2]) {
                points.emplace_back(pos, norm);
            } else {
                std::cerr << "Warning: Invalid format at line " << line_num << std::endl;
            }
        }
        
        std::cout << "Loaded " << points.size() << " points from " << filename << std::endl;
        return !points.empty();
    }
    
    /**
     * @brief Save to simple text file (CSV format)
     * @param filename Path to output file
     * @param segment_ids Optional segment IDs for each point
     * @return true if successful
     */
    bool save_to_csv(const std::string& filename, 
                     const std::vector<int>* segment_ids = nullptr) const {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Cannot create file " << filename << std::endl;
            return false;
        }
        
        // Write header
        if (segment_ids) {
            file << "# X,Y,Z,NX,NY,NZ,SegmentID\n";
        } else {
            file << "# X,Y,Z,NX,NY,NZ\n";
        }
        
        // Write points
        for (size_t i = 0; i < points.size(); ++i) {
            const auto& p = points[i];
            file << p.position[0] << "," << p.position[1] << "," << p.position[2] << ","
                 << p.normal[0] << "," << p.normal[1] << "," << p.normal[2];
            
            if (segment_ids && i < segment_ids->size()) {
                file << "," << (*segment_ids)[i];
            }
            file << "\n";
        }
        
        std::cout << "Saved " << points.size() << " points to " << filename << std::endl;
        return true;
    }
    
    /**
     * @brief Compute bounding box
     * @return Pair of (min_corner, max_corner)
     */
    std::pair<Position3D, Position3D> compute_bounds() const {
        if (points.empty()) {
            return {{0,0,0}, {0,0,0}};
        }
        
        Position3D min_corner = points[0].position;
        Position3D max_corner = points[0].position;
        
        for (const auto& p : points) {
            for (int i = 0; i < 3; ++i) {
                min_corner[i] = std::min(min_corner[i], p.position[i]);
                max_corner[i] = std::max(max_corner[i], p.position[i]);
            }
        }
        
        return {min_corner, max_corner};
    }
    
    /**
     * @brief Estimate average point spacing
     * @return Approximate distance between neighboring points
     * 
     * Uses simple heuristic: cube root of (volume / num_points)
     */
    double estimate_point_spacing() const {
        if (points.size() < 2) return 0.0;
        
        auto [min_corner, max_corner] = compute_bounds();
        
        double volume = 1.0;
        for (int i = 0; i < 3; ++i) {
            double span = max_corner[i] - min_corner[i];
            if (span > 0) volume *= span;
        }
        
        // Average spacing ≈ (volume / num_points)^(1/3)
        return std::cbrt(volume / points.size());
    }
};

} // namespace region_merging
