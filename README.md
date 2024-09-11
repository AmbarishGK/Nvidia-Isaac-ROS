# Isaac ROS ESS Tutorial

Welcome to the **Isaac ROS ESS (Efficient Semi-Supervised Stereo Disparity)** tutorial. This repository provides step-by-step instructions for setting up and running stereo depth estimation using NVIDIA's Isaac ROS ESS package.

## Table of Contents
1. [Introduction](#introduction)
2. [System Requirements](#system-requirements)
3. [Installation](#installation)
   - [ROS 2 Humble Setup](#ros-2-humble-setup)
   - [Isaac ROS ESS Package Installation](#isaac-ros-ess-package-installation)
4. [Running the Code](#running-the-code)
   - [Launching the ESS Node](#launching-the-ess-node)
   - [Testing with Stereo Images](#testing-with-stereo-images)
5. [Configuration](#configuration)
6. [Troubleshooting](#troubleshooting)
7. [References](#references)

## Introduction
This tutorial demonstrates how to use **Isaac ROS ESS** for stereo depth estimation in robotic applications. The tutorial covers installation, configuration, and usage of the **ESS DNN** model, which is optimized to run on NVIDIA GPUs with **TensorRT** acceleration.

## System Requirements
- **Hardware**:
  - NVIDIA Jetson (Orin or later) or x86_64 system with an NVIDIA GPU.
  - Stereo camera setup (RGB global shutter recommended).
  
- **Software**:
  - **ROS 2 Humble** (required, other versions are not supported).
  - **NVIDIA GPU drivers** and **CUDA** toolkit installed.
  - **TensorRT** (used for DNN inference optimization).
  
## Installation

### ROS 2 Humble Setup
1. Install **ROS 2 Humble** by following the official ROS 2 installation guide:  
   [ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)

2. Ensure the environment is sourced:
   ```bash
   source /opt/ros/humble/setup.bash
