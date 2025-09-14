# Robotino SLAM and Path Planning Project

A comprehensive MATLAB-based project for Simultaneous Localization and Mapping (SLAM) using 2D LiDAR data and path planning algorithms for the Robotino mobile robot platform.

## 🚀 Overview

This project provides a complete solution for:
- **Map Generation**: Building occupancy grid maps from 2D LiDAR scans using SLAM
- **Path Planning**: Generating optimal navigation paths using A* and RRT* algorithms
- **Robotino Integration**: Real-time data collection and visualization tools

## 📁 Project Structure

```
robotino_project/
├── data/
│   ├── lidar_data/          # LiDAR scan data and preprocessing
│   │   ├── sample_data/     # CSV files with scan data
│   │   └── *.mat           # Processed LiDAR data files
│   └── maps/               # Generated occupancy maps
│       ├── *.mat           # MATLAB occupancy grid maps
│       ├── *.pgm           # Portable Gray Map format
│       ├── *.yaml          # Map metadata
│       └── *.png           # Visualized maps
├── src/
│   ├── OccupancyMapGeneration/  # SLAM and map building
│   ├── PathNetworkGeneration/   # Path planning algorithms
│   └── Utils/                   # Data collection and visualization
└── docs/                   # Documentation and resources
```

## 🛠️ Main Tasks

### 1. Generate Occupancy Map
**File**: `src/OccupancyMapGeneration/BuildMapFrom2DLidarScansForIOTFactory.m`

This script performs SLAM to build an occupancy grid map from 2D LiDAR scans:
- Processes LiDAR scan data from CSV files
- Implements SLAM algorithm with loop closure detection
- Generates occupancy grid maps in multiple formats (.mat, .pgm, .png)
- Creates animated GIF showing map building process
- Supports different environments (IOT Factory, Warehouse, Indoor areas)

**Usage**:
```matlab
% Run from MATLAB
cd src/OccupancyMapGeneration/
BuildMapFrom2DLidarScansForIOTFactory
```

### 2. Generate Path Network
Choose between two path planning algorithms:

#### Option A: A* Algorithm
**File**: `src/PathNetworkGeneration/PathPlanningAStarGrid.m`

- Implements A* pathfinding on occupancy grid
- Defines docking points, parking points, and stations
- Generates optimal paths between predefined waypoints
- Visualizes path network on occupancy map

#### Option B: RRT* Algorithm  
**File**: `src/PathNetworkGeneration/PathPlanningRRTStar.m`

- Implements Rapidly-exploring Random Tree Star (RRT*)
- Provides probabilistic path planning
- Better for complex environments with dynamic obstacles
- Generates smooth, optimal paths

**Usage**:
```matlab
% Run from MATLAB
cd src/PathNetworkGeneration/
PathPlanningAStarGrid    % or PathPlanningRRTStar
```

## 🔧 Prerequisites

- **MATLAB** with Lidar Toolbox and Navigation Toolbox
- **Python 3.x** (for data collection utilities)
- **Robotino API** (for real-time data collection)

## 📊 Data Collection

The project includes Python utilities for real-time data collection from Robotino:

- `get_lidar_data.py`: Collects LiDAR scan data from Robotino API
- `rest_connection.py`: Handles API communication
- `vis_mat_data.py`: Visualizes MATLAB data files

## 🗺️ Generated Outputs

### Maps
- **Occupancy Grid Maps**: Binary occupancy representation
- **Visual Maps**: PNG images with trajectory overlays
- **Animated Maps**: GIF files showing map building process
- **Metadata**: YAML files with map parameters

### Path Networks
- **Navigation Paths**: XML files with waypoint definitions
- **Station Definitions**: LISP files with station coordinates
- **Visual Paths**: PNG images showing planned routes

## 📚 Resources

- [SLAM Tutorials](https://www.mathworks.com/help/nav/ug/choose-a-slam-workflow-based-on-sensor-data.html)
- [Navigation Toolbox Documentation](https://www.mathworks.com/help/nav/mapping.html)
- [LiDAR SLAM Examples](https://www.mathworks.com/help/lidar/ug/build-map-from-2d-lidar-scans-using-slam.html)


## 📝 Notes

- Ensure LiDAR data is properly formatted in CSV files
- Adjust SLAM parameters (resolution, range) based on your environment
- Modify waypoint coordinates in path planning scripts for your specific setup
- Generated maps are saved in multiple formats for different applications