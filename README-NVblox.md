# Isaac ROS Nvblox Isaac Sim Tutorial

This repository provides a step-by-step guide on how to implement NVIDIA's **Isaac ROS Nvblox** package for Isaac Sim 3D reconstruction and navigation using assets provided by Nvidia.

## Table of Contents
- [Isaac ROS Nvblox Isaac Sim Tutorial](#isaac-ros-nvblox-isaac-sim-tutorial)
  - [Table of Contents](#table-of-contents)
  - [Introduction](#introduction)
  - [System Requirements](#system-requirements)
  - [Installation](#installation)
    - [Prerequisites](#prerequisites)
    - [Install Isaac ROS Nvblox](#install-isaac-ros-nvblox)
      - [Do the following commands in Terminal 1](#do-the-following-commands-in-terminal-1)
      - [Open new terminal 2:](#open-new-terminal-2)
      - [Open a new terminal 3](#open-a-new-terminal-3)
  - [Setting Up Nvblox](#setting-up-nvblox)
    - [Modes of Operation](#modes-of-operation)
  - [Running the Application](#running-the-application)
  - [Visualization in Rviz](#visualization-in-rviz)
  - [Using with Nav2](#using-with-nav2)
  - [Conclusion](#conclusion)

## Introduction

**Isaac ROS Nvblox** processes depth and pose information to reconstruct a 3D scene in real time and generates a 2D costmap for obstacle avoidance using Nav2. The reconstruction runs on the GPU for faster computation, and the costmap is continuously updated for navigation planning.

## System Requirements

To use Isaac ROS Nvblox, ensure your system meets the following requirements:

- Ubuntu 22.04
- ROS 2 (Humble)
- NVIDIA GPU (Driver 535.180.x+)
- [Isaac Sim](https://developer.nvidia.com/isaac-sim) for simulation
- Docker

## Installation

### Prerequisites
Ensure you have ROS 2 set up on your system. If not, follow the official [ROS 2 installation guide](https://docs.ros.org/en/foxy/Installation.html).

### Install Isaac ROS Nvblox
Follow the official [Isaac ROS Nvblox quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html#quickstart). Set up the dev environment (https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html)

#### Do the following commands in Terminal 1
1. Clone `isaac ros common`
    ```bash
    cd ${ISAAC_ROS_WS}/src && \
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
    ```
2. Download data from NGC:
    ```bash
    sudo apt-get install -y curl tar
    NGC_ORG="nvidia"
    NGC_TEAM="isaac"
    NGC_RESOURCE="isaac_ros_assets"
    NGC_VERSION="isaac_ros_nvblox"
    NGC_FILENAME="quickstart.tar.gz"

    REQ_URL="https://api.ngc.nvidia.com/v2/resources/$NGC_ORG/$NGC_TEAM/$NGC_RESOURCE/versions/$NGC_VERSION/files/$NGC_FILENAME"

    mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets/${NGC_VERSION} && \
        curl -LO --request GET "${REQ_URL}" && \
        tar -xf ${NGC_FILENAME} -C ${ISAAC_ROS_WS}/isaac_ros_assets/${NGC_VERSION} && \
        rm ${NGC_FILENAME}
    ```

    Note: The NGC Catalog is a curated set of GPU-optimized software. It consists of containers, pre-trained models, Helm charts for Kubernetes deployments and industry specific AI toolkits with software development kits (SDKs).
   
    

3. Launch the Docker container using the run_dev.sh script:
    ```bash
    cd $ISAAC_ROS_WS && ./src/isaac_ros_common/scripts/run_dev.sh
    ```
4. Install the dependencies:
    ```bash
    sudo apt update &&
    sudo apt-get install -y ros-humble-isaac-ros-nvblox && \
    rosdep update && \
    rosdep install isaac_ros_nvblox
    ```

#### Open new terminal 2:
1. Go to the path where "isaac-sim.sh" for your version is located.
    ```bash
    cd /home/user/.local/share/ov/pkg/isaac-sim-4.1.0
    ```
    Note: This path wll change according to your own installation. So make sure you find it properly
2. Start the simulator
    ```bash
    bash ./isaac-sim.sh -v
    ```
    Note: do `find / -name "*isaac-sim.sh"` to find where your bin is located if the above doesn't work
3. Find the `localhost/NVIDIA/Assets/Isaac/4.0/Isaac/Samples/NvBlox/nvblox_sample_scene.usd` in your assets.
    
   ![alt text](image.png)

4. Drag and drop it in stage
   
   ![alt text](image-1.png)

5.  Play the scene

#### Open a new terminal 3
1. Start the Isaac ROS Dev Docker container (if not started in the install step):
    ```bash
    cd $ISAAC_ROS_WS && ./src/isaac_ros_common/scripts/run_dev.sh
    ```
    This will start the docker conatiner with necessary files

2. Navigate (inside the Docker terminal) to the workspace folder and source the workspace:
    ```bash
    cd /workspaces/isaac_ros-dev
    source install/setup.bash
    ```
    Note:  Don't have to do if you have set it up in bashrc of your host machine

3. Launch the example with ROS:
    ```bash
    sudo apt update &&
    sudo apt-get install -y ros-humble-isaac-ros-nvblox && \
    rosdep update && \
    rosdep install isaac_ros_nvblox

    ros2 launch nvblox_examples_bringup isaac_sim_example.launch.py
    ```
    ![alt text](image-2.png)
   
## Setting Up Nvblox
Isaac ROS Nvblox can work with either stereo cameras or 3D LiDAR. Below are steps to configure and set up different modes of operation.

### Modes of Operation
Nvblox offers multiple modes:

1. **Static Reconstruction Mode**: Assumes the environment is static.
2. **People Reconstruction Mode**: Maps scenes with people using segmentation.
3. **Dynamic Reconstruction Mode**: Maps scenes with more general dynamic objects.

## Running the Application

To launch the application:

1. For stereo cameras:
    ```bash
    ros2 launch isaac_ros_nvblox stereo_depth.launch.py
    ```

2. For 3D LiDAR:
    ```bash
    ros2 launch isaac_ros_nvblox lidar_3d_reconstruction.launch.py
    ```

3. For simulation with Isaac Sim:
    ```bash
    ros2 launch isaac_ros_nvblox isaac_sim_integration.launch.py
    ```

## Visualization in Rviz

Nvblox provides real-time 3D reconstruction visualization using Rviz. To visualize the mesh:

1. Open Rviz:
    ```bash
    rviz2
    ```

2. Add the Mesh Visualization Plugin:
    - In the Rviz panel, click **Add**.
    - Select **Mesh Plugin** to view the 3D reconstruction.
    - You should now see the updated mesh of the scene as it's being reconstructed.

## Using with Nav2

Nvblox integrates seamlessly with Nav2 for navigation tasks. It provides a 2D costmap for obstacle avoidance. To use Nav2:

1. Launch the Nav2 integration:
    ```bash
    ros2 launch isaac_ros_nvblox nvblox_navigation.launch.py
    ```

2. The system will begin generating a costmap for navigation, allowing the robot to navigate while avoiding obstacles based on the 3D reconstruction.

## Conclusion

In this tutorial, you've learned how to set up and run the Isaac ROS Nvblox package, visualize the 3D reconstruction in Rviz, and use Nav2 for navigation based on the costmap generated by Nvblox.

For further customization and advanced features, refer to the [official documentation](https://developer.nvidia.com/isaac-ros).
