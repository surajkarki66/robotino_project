# Robotino SLAM and Path Planning Project

A comprehensive MATLAB-based project for Simultaneous Localization and Mapping (SLAM) using 2D LiDAR data and path planning algorithms for the Robotino mobile robot platform.

## 🚀 Overview

This project provides a complete solution for:
- **Map Generation**: Building occupancy grid maps from 2D LiDAR scans using SLAM
- **Path Planning**: Generating optimal navigation paths using A* and RRT* algorithms

## 📁 Project Structure

```
robotino_project/
├── data/
│   ├── lidar_data/          # LiDAR scan data and preprocessing
│   │   └── *.mat           # Processed LiDAR data files
│   └── maps/               # Generated occupancy maps
│       ├── *.mat           # MATLAB occupancy grid maps
│       ├── *.pgm           # Portable Gray Map format
│       ├── *.yaml          # Map metadata
│       ├── *.png           # Visualized maps
│       ├── IOTFactoryMapwithExp*/  # IOT Factory experiment maps
│       ├── navigation/      # Navigation paths and stations
│       └── old_data/        # Legacy map data
├── src/
│   ├── OccupancyMapGeneration/  # SLAM and map building
│   ├── PathNetworkGeneration/   # Path planning algorithms
│   └── Utils/                   # Data collection and visualization
└── docs/                   # Documentation and resources
```

## 🛠️ Main Tasks

### 1. Generate Occupancy Map
**Main Files**: 
- `src/OccupancyMapGeneration/BuildMapFrom2DLidarScansForIOTFactory.m` - IOT Factory environment
- `src/OccupancyMapGeneration/BuildMapFrom2DLidarDataForWarehouse.m` - Warehouse environment  
- `src/OccupancyMapGeneration/BuildMapFrom2DLidarDataForIndoorArea.m` - Indoor area environment

These scripts perform SLAM to build occupancy grid maps from 2D LiDAR scans:
- Processes LiDAR scan data from .mat files
- Implements SLAM algorithm with loop closure detection
- Generates occupancy grid maps in multiple formats (.mat, .pgm, .png)
- Creates animated GIF showing map building process
- Supports different environments (IOT Factory, Warehouse, Indoor areas)

**Additional Scripts**:
- `BuildMapFrom2DLidarScansForIOTFactory2.m` - Alternative IOT Factory implementation
- `BuildMapFrom2DLidarScansForIOTFactoryGIF.m` - Creates animated GIF of map building
- `HelperPlotLidarScanMap.m` - Helper function for plotting LiDAR scan maps

**Usage**:
```matlab
% Run from MATLAB
cd src/OccupancyMapGeneration/
BuildMapFrom2DLidarScansForIOTFactory    % For IOT Factory
BuildMapFrom2DLidarDataForWarehouse      % For Warehouse
BuildMapFrom2DLidarDataForIndoorArea     % For Indoor Area
```

### 2. Generate Path Network
Choose between two path planning algorithms for different environments:

#### Option A: A* Algorithm
**Files**: 
- `src/PathNetworkGeneration/IOTPathPlanningAStarGrid.m` - IOT Factory environment
- `src/PathNetworkGeneration/WarehousePathPlanningAStarGrid.m` - Warehouse environment

- Implements A* pathfinding on occupancy grid
- Defines docking points, parking points, and stations
- Generates optimal paths between predefined waypoints
- Visualizes path network on occupancy map

#### Option B: RRT* Algorithm  
**Files**:
- `src/PathNetworkGeneration/IOTPathPlanningRRTStar.m` - IOT Factory environment
- `src/PathNetworkGeneration/WarehousePathPlanningRRTStar.m` - Warehouse environment

- Implements Rapidly-exploring Random Tree Star (RRT*)
- Provides probabilistic path planning
- Better for complex environments with dynamic obstacles
- Generates smooth, optimal paths

**Helper Files**:
- `HelperCheckIfGoal.m` - Helper function for goal checking
- `IOTMapTransformation.m` - Map transformation utilities for IOT Factory

**Usage**:
```matlab
% Run from MATLAB
cd src/PathNetworkGeneration/
IOTPathPlanningAStarGrid        % A* for IOT Factory
WarehousePathPlanningAStarGrid  % A* for Warehouse
IOTPathPlanningRRTStar          % RRT* for IOT Factory
WarehousePathPlanningRRTStar    % RRT* for Warehouse
```

## 🔧 Prerequisites

- **MATLAB** with Lidar Toolbox and Navigation Toolbox
- **Python 3.x** (for data collection utilities)
- **Robotino API** (for real-time data collection)

## 📊 Data Collection and Utilities

The project includes Python utilities for real-time data collection from Robotino and MATLAB utilities for data processing:

**Python Utilities** (`src/Utils/Python/`):
- `get_lidar_data.py`: Collects LiDAR scan data from Robotino API
- `rest_connection.py`: Handles API communication
- `vis_mat_data.py`: Visualizes MATLAB data files

**MATLAB Utilities** (`src/Utils/`):
- `ConvertPGMToMat.m`: Converts PGM map files to MATLAB format
- `CSVtoMAT.m`: Converts CSV data files to MATLAB .mat format

## 🗺️ Generated Outputs

### Maps
- **Occupancy Grid Maps**: Binary occupancy representation (.mat files)
- **Visual Maps**: PNG images with trajectory overlays
- **Animated Maps**: GIF files showing map building process
- **Metadata**: YAML files with map parameters
- **PGM Maps**: Portable Gray Map format for ROS integration

### Path Networks
- **Navigation Paths**: XML files with waypoint definitions (`navigation-paths.xml`)
- **Station Definitions**: LISP files with station coordinates (`positions.lisp`, `stations.lisp`)
- **Visual Paths**: PNG images showing planned routes

### Data Files
- **LiDAR Data**: Preprocessed .mat files containing scan data and odometry
- **Experiment Maps**: Multiple versions for different experimental setups (IOTFactoryMapwithExp1-5)

## 📝 Notes

- Ensure LiDAR data is properly formatted in .mat files
- Adjust SLAM parameters (resolution, range) based on your environment
- Modify waypoint coordinates in path planning scripts for your specific setup
- Generated maps are saved in multiple formats for different applications
- Use the appropriate script for your target environment (IOT Factory, Warehouse, or Indoor Area)
