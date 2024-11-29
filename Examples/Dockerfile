FROM nvidia/cuda:11.7.1-devel-ubuntu22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV TORCH_CUDA_ARCH_LIST="6.0 6.1 7.0 8.0 8.5+PTX"
ENV FORCE_CUDA="1"

# Install system dependencies
RUN apt-get update && apt-get install -y \
    ffmpeg \
    libsm6 \
    libxext6 \
    git \
    ninja-build \
    libglib2.0-0 \
    libxrender-dev \
    python3-pip \
    wget \
    curl \
    gnupg2 \
    software-properties-common \
    python3-setuptools \
    python3-dev \
    build-essential

# Install ROS2 Humble
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get upgrade -y && \
    apt-get install -y ros-humble-desktop

RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libxkbcommon-x11-0 \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Install PyTorch with CUDA support
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu117

# Install MMDetection3D dependencies
RUN pip3 install -U openmim && \
    mim install mmengine && \
    mim install 'mmcv>=2.0.0rc4' && \
    mim install 'mmdet>=3.0.0' && \
    mim install "mmdet3d>=1.1.0" && \
    pip3 install 'numpy==1.26'

    

# RUN git clone https://github.com/open-mmlab/mmdetection3d.git -b dev-1.x
# Set working directory
WORKDIR /workspace/
# RUN mim download mmdet3d --config pointpillars_hv_secfpn_8xb6-160e_kitti-3d-car --dest .
# Source ROS2 setup in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
COPY Entrypoint.sh /Entrypoint.sh

# Make the script executable
RUN chmod +x /Entrypoint.sh
# Default command
ENTRYPOINT ["/Entrypoint.sh"]
CMD ["tail", "-f", "/dev/null"]


