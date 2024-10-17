# MMDetection3D Tutorial

## Introduction

This tutorial provides an in-depth guide on using **MMDetection3D**, an open-source toolbox for 3D object detection. It is built upon the MMDetection framework and designed to help users implement various 3D detection methods with minimal effort.

Note: This setup details if to make sure that you are connecting your MMDetection3D while having Nvidia Isaac Sim to use the MMDetection model in your ISaac Sim pipepline for more efficient 3D mapping. You'll run into many compatibility issues here. Make sure to understand that and be ready to setup the machine in such a way to make all them work


The tutorial will cover:
- Setting up the [MMDetection3D environment](https://mmdetection3d.readthedocs.io/en/latest/get_started.html)
- Preparing datasets for 3D object detection
- Training and evaluation of 3D detection models
- Customizing models and configurations

## Prerequisites

Before you begin, ensure you have met the following requirements:
- A machine with **NVIDIA GPU** and **CUDA support** (optional but recommended for faster training).
- **Python 3.8+** installed.
- **Pytorch 1.7+** installed.
- **MMCV** and **MMDetection** set up.

## Installation Guide for MMDet3D

In this section, I will describe how to install MMDetection3D version 1.4.0 on my system (Ubuntu 22.04 with a GTX 4070 GPU) using a conda environment named `openmmlab` with Python 3.8.

### Step 1: Create a Virtual Environment

First, create and activate a new conda environment:

```bash
conda create --name openmmlab python=3.8 -y
conda activate openmmlab
```

### Step 2: Install PyTorch and Related Libraries

Install the PyTorch and related libraries:

```bash
conda install pytorch==1.12.1 torchvision==0.13.1 torchaudio==0.12.1 cudatoolkit=11.3 -c pytorch
```

### Step 3: Install OpenMMLab Libraries

Use the package manager `mim` to install OpenMMLab-related libraries with specific versions:

```bash
pip install -U openmim
mim install mmengine==0.10.3
mim install mmcv==2.1.0
mim install mmdet==3.3.0
```

### Step 4: Install MMDetection3D

Clone the MMDetection3D repository and install it:

```bash
git clone https://github.com/open-mmlab/mmdetection3d.git -b dev-1.4.0
cd mmdetection3d
pip install -v -e .
```

> The `-v` flag provides more output, while the `-e` flag installs the package in editable mode, allowing local modifications without reinstallation.

### Step 5: Install Additional Libraries

Install additional libraries required for specific models with sparse convolution:

```bash
pip install cumm-cu113
pip install spconv-cu113
```

Make sure to specify the CUDA version when installing `cumm` and `spconv`, as these libraries require the correct CUDA version that matches your environment.

### Step 6: Verify Installation

To confirm that MMDetection3D is successfully installed, you can check the installed libraries:

```bash
conda activate openmmlab
pip list | grep mm
```

### Running a Demo to Ensure Successful Installation

To verify that MMDetection3D is installed correctly, you can conduct inference on sample data using a pre-trained 3D detector. First, download the pre-trained weights for the PointPillar model:

```bash
mim download mmdet3d --config pointpillars_hv_secfpn_8xb6-160e_kitti-3d-car --dest .
```

This command downloads the configuration file and pre-trained weight for the PointPillar model into the current working directory:

- **Config:** `pointpillars_hv_secfpn_8xb6-160e_kitti-3d-car.py`
- **Weight:** `hv_pointpillars_secfpn_6x8_160e_kitti-3d-car_20220331_134606-d42d15ed.pth`

Once downloaded, you can run the demo to visualize the results:

```bash
cd mmdetection3d
python demo/pcd_demo.py demo/data/kitti/000008.bin pointpillars_hv_secfpn_8xb6-160e_kitti-3d-car.py hv_pointpillars_secfpn_6x8_160e_kitti-3d-car_20220331_134606-d42d15ed.pth --show
```

If all requirements are met, this command will plot the point cloud of the provided sample data along with the 3D bounding boxes predicted by the PointPillar model.
![My Video](./videos/mmdetection3dworking.webm?raw=True)



