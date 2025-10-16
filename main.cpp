/**
 * @file main.cpp
 * @brief Main program for Semantic3D region merging experiments
 * 
 * Compatible with run_experiment.py and evaluate.py scripts.
 * Follows same pipeline as region growing experiments.
 * 
 * Usage:
 *   ./region_merging <input_file> <output_file> [options]
 * 
 * Example:
 *   ./region_merging data/processed/semantic3d/scene_downsampled.txt \
 *                    results/scene_predicted.labels
 * 
 * @author Claude
 * @date 2025
 */

#include "io/point_cloud_pcl.hpp"
#include "graph/region_adjacency_graph.hpp"
#include "utils/timer.hpp"

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <cmath>

using namespace region_merging;

/**
 * @brief Print usage information
 */
void print_usage(const char* program_name) {
    std::cout << "Region Merging for Semantic3D\n" << std::endl;
    std::cout << "Usage:" << std::endl;
    std::cout << "  " << program_name << " <input_file> <output_file> [options]" << std::endl;
    std::cout << "\nRequired Arguments:" << std::endl;
    std::cout << "  input_file    Path to input point cloud (.txt format: X Y Z)" << std::endl;
    std::cout << "  output_file   Path to output labels file (.labels format)" << std::endl;
    std::cout << "\nOptional Arguments:" << std::endl;
    std::cout << "  --normal_radius <value>   Radius for normal estimation (default: 0.5)" << std::endl;
    std::cout << "  --search_radius <value>   Radius for neighbor search (default: 0.5)" << std::endl;
    std::cout << "  --angle_threshold <value> Max angle between normals in radians (default: 0.52, ~30°)" << std::endl;
    std::cout << "  --curvature_percentile <value> Percentile for curvature threshold (default: 98)" << std::endl;
    std::cout << "\nExamples:" << std::endl;
    std::cout << "  # Default parameters" << std::endl;
    std::cout << "  " << program_name << " input.txt output.labels" << std::endl;
    std::cout << "\n  # Custom parameters" << std::endl;
    std::cout << "  " << program_name << " input.txt output.labels \\" << std::endl;
    std::cout << "      --normal_radius 0.3 --search_radius 0.4 --angle_threshold 0.6" << std::endl;
}

/**
 * @brief Parse command line arguments
 */
struct Parameters {
    std::string input_file;
    std::string output_file;
    double normal_radius = 0.5;      // For normal estimation
    double search_radius = 0.5;       // For neighbor search
    double angle_threshold = 0.52;    // ~30 degrees in radians
    int curvature_percentile = 98;    // 98th percentile
    
    bool parse(int argc, char** argv) {
        if (argc < 3) {
            return false;
        }
        
        input_file = argv[1];
        output_file = argv[2];
        
        // Parse optional arguments
        for (int i = 3; i < argc; i += 2) {
            std::string arg = argv[i];
            
            if (i + 1 >= argc) {
                std::cerr << "Error: Missing value for " << arg << std::endl;
                return false;
            }
            
            if (arg == "--normal_radius") {
                normal_radius = std::atof(argv[i + 1]);
            } else if (arg == "--search_radius") {
                search_radius = std::atof(argv[i + 1]);
            } else if (arg == "--angle_threshold") {
                angle_threshold = std::atof(argv[i + 1]);
            } else if (arg == "--curvature_percentile") {
                curvature_percentile = std::atoi(argv[i + 1]);
            } else {
                std::cerr << "Error: Unknown argument " << arg << std::endl;
                return false;
            }
        }
        
        return true;
    }
    
    void print() const {
        std::cout << "\nParameters:" << std::endl;
        std::cout << "  Input file: " << input_file << std::endl;
        std::cout << "  Output file: " << output_file << std::endl;
        std::cout << "  Normal radius: " << normal_radius << "m" << std::endl;
        std::cout << "  Search radius: " << search_radius << "m" << std::endl;
        std::cout << "  Angle threshold: " << angle_threshold << " rad (" 
                  << (angle_threshold * 180.0 / M_PI) << "°)" << std::endl;
        std::cout << "  Curvature percentile: " << curvature_percentile << "%" << std::endl;
    }
};

/**
 * @brief Main function
 */
int main(int argc, char** argv) {
    std::cout << "=============================================" << std::endl;
    std::cout << "  Region Merging - Semantic3D Experiments" << std::endl;
    std::cout << "  With PCL KD-tree + Rabbani's Criterion" << std::endl;
    std::cout << "=============================================" << std::endl;
    
    // Parse arguments
    Parameters params;
    if (!params.parse(argc, argv)) {
        print_usage(argv[0]);
        return 1;
    }
    
    params.print();
    
    Timer total_timer;
    total_timer.start();
    
    // =================================================================
    // STEP 1: Load point cloud and compute features
    // =================================================================
    std::cout << "\n========================================" << std::endl;
    std::cout << "  STEP 1: LOADING & FEATURE COMPUTATION" << std::endl;
    std::cout << "========================================" << std::endl;
    
    Timer timer;
    timer.start();
    
    PointCloudPCL cloud;
    if (!cloud.load_from_txt(params.input_file, params.normal_radius)) {
        std::cerr << "Error: Failed to load point cloud from " << params.input_file << std::endl;
        return 1;
    }
    
    double load_time = timer.elapsed();
    std::cout << "  Load + feature computation time: " << load_time << " seconds" << std::endl;
    
    // Get adaptive curvature threshold (98th percentile, like Rabbani)
    double curvature_threshold = cloud.get_curvature_threshold_98();
    std::cout << "  Adaptive curvature threshold (98th percentile): " << curvature_threshold << std::endl;
    
    // =================================================================
    // STEP 2: Build Region Adjacency Graph
    // =================================================================
    std::cout << "\n========================================" << std::endl;
    std::cout << "  STEP 2: GRAPH CONSTRUCTION" << std::endl;
    std::cout << "========================================" << std::endl;
    
    timer.restart();
    
    RegionAdjacencyGraph rag(cloud, params.search_radius);
    
    double build_time = timer.elapsed();
    std::cout << "\n  Graph construction time: " << build_time << " seconds" << std::endl;
    
    // =================================================================
    // STEP 3: Run Region Merging
    // =================================================================
    std::cout << "\n========================================" << std::endl;
    std::cout << "  STEP 3: REGION MERGING" << std::endl;
    std::cout << "========================================" << std::endl;
    
    timer.restart();
    
    Vertex* root = rag.run(curvature_threshold, params.angle_threshold);
    
    double merge_time = timer.elapsed();
    std::cout << "\n  Merging time: " << merge_time << " seconds" << std::endl;
    
    // =================================================================
    // STEP 4: Save Results
    // =================================================================
    std::cout << "\n========================================" << std::endl;
    std::cout << "  STEP 4: SAVING RESULTS" << std::endl;
    std::cout << "========================================" << std::endl;
    
    timer.restart();
    
    if (!rag.save_segmentation(params.output_file, cloud)) {
        std::cerr << "Error: Failed to save segmentation to " << params.output_file << std::endl;
        return 1;
    }
    
    double save_time = timer.elapsed();
    std::cout << "  Save time: " << save_time << " seconds" << std::endl;
    
    // =================================================================
    // SUMMARY
    // =================================================================
    double total_time = total_timer.elapsed();
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  SUMMARY" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Total points: " << cloud.size() << std::endl;
    std::cout << "  Total edges: " << rag.edge_count() << std::endl;
    std::cout << "  Final segments: " << rag.count_segments() << std::endl;
    std::cout << "\n  Timing Breakdown:" << std::endl;
    std::cout << "    Load + features: " << load_time << "s (" 
              << (100.0 * load_time / total_time) << "%)" << std::endl;
    std::cout << "    Graph construction: " << build_time << "s (" 
              << (100.0 * build_time / total_time) << "%)" << std::endl;
    std::cout << "    Merging: " << merge_time << "s (" 
              << (100.0 * merge_time / total_time) << "%)" << std::endl;
    std::cout << "    Save: " << save_time << "s (" 
              << (100.0 * save_time / total_time) << "%)" << std::endl;
    std::cout << "    TOTAL: " << total_time << "s" << std::endl;
    std::cout << "\n  Output saved to: " << params.output_file << std::endl;
    std::cout << "========================================" << std::endl;
    
    std::cout << "\n✓ Region merging completed successfully!" << std::endl;
    
    return 0;
}
