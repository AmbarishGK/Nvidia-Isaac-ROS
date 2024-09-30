# MMDetection3D Tutorial

## Introduction

This tutorial provides an in-depth guide on using **MMDetection3D**, an open-source toolbox for 3D object detection. It is built upon the MMDetection framework and designed to help users implement various 3D detection methods with minimal effort.

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

### Installation
#### Step 0 
- Pytorch Installation
  - Install on your local linux machine from [here](https://pytorch.org/get-started/locally/)
- Install [pip](https://stackoverflow.com/questions/6587507/how-to-install-pip-with-python-3)
  
```bash
sudo apt-get install python3-pip
``` 
- Install [openmim](https://github.com/open-mmlab/mmengine)
```bash
pip install -U openmim
```
- Make sure that `mim` is accesible from terminal or add in path
```bash
ls ~/.local/bin/mim
echo $PATH
export PATH="$HOME/.local/bin:$PATH"
source ~/.bashrc
``` 

- Install mmengine
```bash
mim install mmengine
```

- Install MMCV and MMDetection
```bash
mim install 'mmcv>=2.0.0rc4'
mim install 'mmdet>=3.0.0'
```

#### Step 1

- Clone the repository and install the MMDet3D:

```bash
git clone https://github.com/open-mmlab/mmdetection3d.git -b dev-1.x
# "-b dev-1.x" means checkout to the `dev-1.x` branch.
cd mmdetection3d
pip install -v -e .
# "-v" means verbose, or more output
# "-e" means installing a project in editable mode,
# thus any local modifications made to the code will take effect without reinstallation.
```

- Verify the installation
```bash
python3 -c 'from mmengine.utils.dl_utils import collect_env;print(collect_env())'
```

Note: This is for the installation of mmdetection libraries, We will need to install the mmdet3d library for mmdetection3d toolkit. Currently I seem to face some issues with pytorch - cuda - mmdetection3d compatibility issues. This has to be resolved for us to use the inference.