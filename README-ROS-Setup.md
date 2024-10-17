# Nvidia Isaac ROS Setup Guide

In this document I ll walk you through the Nvidia Isaac ROS setup. **Only ROS2 Setup but compatabile with Nvidia Isaac Sim version 4.0.0**. There isn't a big difference from the actual setup but this is all at one place and few commands will be changed. **We will enable the ROS2-Isaac Bridge in this tutorial**. Make sure to follow it before you install other drivers etc coz this might disrupt the other installations if you are working with kernel 6.5.0.41.

## Prerequisites

Before diving into the installation, make sure you have the following:

- **Ubuntu**: ROS primarily supports Ubuntu. This guide assumes you’re using **Ubuntu 22.04** (Focal). Other versions may work, but your mileage may vary. If you can use Pop_!OS go for it.
- **Sudo access**: You’ll need admin rights to install packages.
- **Basic command-line knowledge**: You should be comfortable using bash commands.


### MY environment for your knowledge
- **OS**            : Linux Ubuntu 22.04 
- **Kernel**        : 6.5.0.41-Low latency
- **Nvidia Driver** : 535.186.1
- **Cuda Version**  : 12.2
- **GPU**           : RTX 4070


## Step 1: Set Up Your Sources

First, let’s make sure your system is set up to pull from the ROS2 repositories.
`
### 1.1 Set locale

We need to add the official locale suppport for UTF-8:
```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
### 1.2 Update your package index

Open a terminal and update your system’s package list:

```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe
# Add GPG Keys
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
#Add repos to your source list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
### 1.2 Update & Upgrade your machine to get those repos

We need to add the official locale suppport for UTF-8:
```bash
sudo apt update 
sudo apt upgrade -y
```
### 1.3 Install ROS2
```bash
# desktop version since we need Rviz
sudo apt install ros-humble-desktop -y
```
or (if you dont want Rviz etc)
```bash
# Nano version you can skip rvis and demos etc for size constraints
sudo apt install ros-humble-ros-base
```
#### Install Optionals but good to have
Detection2DArray and Detection3DArray used for publishing bounding boxes nned the below
```bash
sudo apt install ros-humble-vision-msgs
```
AckermannDriveStamped used for publishing and subscribing to Ackermann steering commands in the ROS 2 bridge ned the below
```bash
sudo apt install ros-humble-ackermann-msgs
```
Additional packages which may come handy when coding
```bash
# For rosdep install command
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
# For colcon build command
sudo apt install python3-colcon-common-extensions
```

### 1.4 Add sourcing to your terminal

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

source ~/.bashrc
```

### 1.5 Run talker & listener
Important step to create `/.ros` folder so don't skip. This is required for bridge between Isaac and ROS2

#### Open two Terminals
##### Terminal 1
```bash
ros2 run demo_nodes_cpp talker
```
##### Terminal 2
```bash
ros2 run demo_nodes_py listener
```

![alt text](./img/rostalklisten.png)

This would have created the `/.ros` directory we can use this to enable the bridge

### 1.6 Enable ROS Bridge Extension Isaac SIm
If you intend to use a ROS 2 bridge, before launching Isaac Sim, you need to set the Fast DDS middleware on all terminals that will be passing ROS 2 messages

Create a file `fastdds.xml` under `~/.ros/` and paste the following in it and save
```bash
<?xml version="1.0" encoding="UTF-8" ?>

<license>Copyright (c) 2022-2024, NVIDIA CORPORATION.  All rights reserved.
NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto.  Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.</license>


<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles" >
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>UdpTransport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="udp_transport_profile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>UdpTransport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
```

### 1.7 Add the bridge support in terminal
```bash
echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml' >> ~/.bashrc
source ~/.bashrc
```
This will enable ros extentsion in all your isaac sim apps opened from terminals.