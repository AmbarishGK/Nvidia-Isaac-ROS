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
      - [Isaac Sim (new terminal 2):](#isaac-sim-new-terminal-2)
      - [Open Rviz to Visualize (new terminal 3)](#open-rviz-to-visualize-new-terminal-3)
  - [Running the Application](#running-the-application)
  - [Visualization in Rviz](#visualization-in-rviz)
  - [Troubleshooting](#troubleshooting)
  - [Using with Nav2](#using-with-nav2)
  - [Conclusion](#conclusion)
  - [Contributing](#contributing)
  - [License](#license)

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

#### Isaac Sim (new terminal 2):
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

#### Open Rviz to Visualize (new terminal 3)
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
   
## Running the Application
Once everything is set up, run the example and observe the real-time 3D reconstruction of the environment in RViz.

## Visualization in Rviz
To visualize the generated maps and reconstructed environment, open RViz and add relevant topics like `/nvblox/pointcloud`.

## Troubleshooting
If the robot doesn't move or reconstruction fails:
- Check ROS topics using `ros2 topic list` to ensure correct publishing/subscribing.
- Verify the GPU drivers are up to date.
- Review logs for any specific errors in the terminal or Isaac Sim console.

## Using with Nav2
Integrate **Nvblox** with the Nav2 stack for autonomous navigation. Use the provided launch files to connect the 2D costmap from **Nvblox** to the Nav2 planner.

## Conclusion
By following these steps, you will successfully run the **Isaac ROS Nvblox** sample for 3D reconstruction and navigation in Isaac Sim.

## Contributing
We welcome contributions! Please follow the guidelines outlined in the `CONTRIBUTING.md` file.

## License
This project is licensed under the NVIDIA Source Code License. See the `LICENSE.md` file for more details.