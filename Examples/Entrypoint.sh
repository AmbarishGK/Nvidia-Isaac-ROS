#!/bin/bash

# Clone the mmdetection3d repository and checkout the specified branch
git clone https://github.com/open-mmlab/mmdetection3d.git -b dev-1.x /workspace/mmdetection3d

# Set working directory to /workspace
cd /workspace/mmdetection3d

# Download the pre-trained model config
mim download mmdet3d --config pointpillars_hv_secfpn_8xb6-160e_kitti-3d-car --dest .

# Execute any additional commands passed to the container
exec "$@"
