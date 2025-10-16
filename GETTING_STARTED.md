# Getting Started - Quick Guide

## Prerequisites

```bash
# Install PCL
sudo apt-get update
sudo apt-get install libpcl-dev

# Verify
pkg-config --modversion pcl_common
# Should show: 1.8.x or higher
```

## Build (First Time)

```bash
cd basic_region_merging
mkdir build
cd build
cmake ..
make -j4  # Use 4 cores for faster build
```

**Expected output**:
```
-- Found PCL version: 1.8.x
-- Build type: Release
...
[100%] Built target region_merging
```

## Quick Test

```bash
# Still in build/
./region_merging --help
```

Should show usage information.

## Run Your First Experiment

### Option 1: Simple Test
```bash
# Create test data (3 planes, 1221 points)
cat > test_input.txt << TESTDATA
# Simple test: 3 planes
0 0 0
0 1 0
0 2 0
1 0 0
1 1 0
1 2 0
TESTDATA

# Run segmentation
./region_merging test_input.txt test_output.labels

# Check results
cat test_output.labels
# Should see segment IDs: 0, 0, 0, 0, 0, 0
```

### Option 2: Full Semantic3D Pipeline

Assuming you have Semantic3D data:
```bash
cd ..  # Back to basic_region_merging/

# Run complete pipeline
python run_experiment.py --scene_name marketsquarefeldkirch4-reduced

# This will:
# 1. Downsample point cloud
# 2. Run region merging
# 3. Upsample results
# 4. Evaluate mIoU
# 5. Create visualizations
```

## Common Issues

### PCL Not Found
```bash
# Error: Could not find PCL
sudo apt-get install libpcl-dev

# Still not found? Set path manually:
export PCL_DIR=/usr/lib/x86_64-linux-gnu/cmake/pcl
cmake ..
```

### Missing Python Packages
```bash
pip install numpy scikit-learn open3d
```

### Compilation Errors
```bash
# C++17 not supported?
# Update compiler:
sudo apt-get install g++-7
export CXX=g++-7
cmake ..
make
```

## Parameter Tuning

### Default Parameters
```bash
./region_merging input.txt output.labels
# Uses:
#   normal_radius = 0.5m
#   search_radius = 0.5m
#   angle_threshold = 0.52 rad (~30°)
#   curvature_percentile = 98
```

### Custom Parameters
```bash
# More aggressive merging
./region_merging input.txt output.labels \
    --normal_radius 0.3 \
    --search_radius 0.4 \
    --angle_threshold 0.7 \
    --curvature_percentile 99

# More conservative
./region_merging input.txt output.labels \
    --angle_threshold 0.3 \
    --curvature_percentile 95
```

## Understanding Output

### Console Output
```
============================================
  Region Merging - Semantic3D Experiments
============================================

Parameters:
  Normal radius: 0.5m
  Search radius: 0.5m
  Angle threshold: 0.52 rad (29.8°)
  Curvature percentile: 98%

STEP 1: LOADING & FEATURE COMPUTATION
  Loaded 50000 points
  Curvature threshold (98th percentile): 0.045

STEP 2: GRAPH CONSTRUCTION
  Created 300000 edges
  Average degree: 12.0

STEP 3: REGION MERGING
  Merges performed: 45000
  Rejected by angle: 2000
  Rejected by curvature: 1500
  Final segments: 150

STEP 4: SAVING RESULTS
  Output saved to: output.labels

SUMMARY:
  Total points: 50000
  Final segments: 150
  Total time: 2.5s
```

### Output File Format
```
# output.labels (one segment ID per line)
0
0
0
1
1
2
...
```

Compatible with `evaluate.py`.

## Visualize Results

### In CloudCompare
1. Load original point cloud
2. Load labels file
3. Color by scalar field

### Using Python
```python
import numpy as np
import open3d as o3d

# Load
points = np.loadtxt('input.txt')
labels = np.loadtxt('output.labels')

# Create colored cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# Color by segment
colors = plt.cm.tab20(labels % 20)[:, :3]
pcd.colors = o3d.utility.Vector3dVector(colors)

# Visualize
o3d.visualization.draw_geometries([pcd])
```

## Next Steps

1. ✅ Build successfully
2. ✅ Run quick test
3. ✅ Run on real data
4. ✅ Evaluate results
5. → Compare with region growing
6. → Tune parameters
7. → Add optimizations!

## Getting Help

- Check `README.md` for overview
- Check `CHANGES.md` for technical details
- Check `COMPARISON.md` for vs. region growing
- Ask Claude! 😊

**You're ready to go!** 🚀
