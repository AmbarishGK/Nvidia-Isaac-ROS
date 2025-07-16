# Driver Issues

## FAQs

Most of the suggestions below are tailored to my particular hardware. Here are my specs:

### System Information
- **Machine Name**: amb-Dell-G16-7630
- **Operating System**: Ubuntu 22.04
- **Kernel Version**: 6.5.0-41-lowlatency

### Hardware Configuration
- **GPU**: NVIDIA GeForce 4070
- **NVIDIA Driver Version**: 535.186.xx
- **CUDA Version**: 12.2

## Setup Guide

### 1. What is the best NVIDIA driver to use?
Currently, the best driver that works for me with Isaac Sim is **535.186.1**.

### 2. What OS and version to use?
I recommend using **Ubuntu 22.04**, as it integrates well with ROS2. If you're able to set up **Pop!_OS**, it's also a great choice.

### 3. Which kernel to select?
Opt for **kernel 6.5.0-41** instead of the latest 6.8.x.x versions. The NVIDIA drivers are not well-tested with newer kernels, which can lead to system crashes.

### 4. How to install the driver without any errors or breaking the machine?

#### Purging NVIDIA Drivers
Before installing the new driver, it’s crucial to remove existing NVIDIA and CUDA installations:
```bash
# Uninstall everything related to NVIDIA & CUDA
sudo apt-get remove --purge '^nvidia-.*'
sudo apt-get remove --purge '^libnvidia-.*'
sudo apt-get remove --purge '^cuda-.*'
reboot
```
#### Installing the Driver

After purging, install the preferred version of the driver:
```bash
# Update your package lists and install the NVIDIA driver
sudo apt update
sudo apt upgrade
# change to the version you like, now its 570 for mine
sudo apt install nvidia-driver-535 nvidia-dkms-535
```
### 5. My kernel is 6.8.xx, how to downgrade?

If you're running an unsupported kernel version, you can downgrade as follows:
#### Find All Installed Kernels
```bash
apt-cache showpkg linux-image
```
#### Choose and Install the Desired Version

Replace <version> with the specific version you wish to install:

```bash
sudo apt-get install linux-image-<version> linux-headers-<version>
sudo update-grub
sudo reboot
# Reconfigure driver for your kernel
dpkg-reconfigure nvidia-dkms-xxx
```

### Current Update of working System Information
- **Machine Name**: amb-Dell-G16-7630
- **Operating System**: Ubuntu 22.04.5
- **Kernel Version**: 6.5.0-41-lowlatency

### Hardware Configuration
- **GPU**: NVIDIA GeForce 4070
- **NVIDIA Driver Version**: 570.169.xx
- **CUDA Version**: 12.8
