# MM3D + ROS2 (Docker)
The Dockerfile contains insctruction to build docker images that canbe used to run mmdetection3d and ros2 in docker. 
### Build Image
```bash
$ xhost +local:
$ docker build -t ros2-mmdetection .
```
The above instruction should let you create the image in your machine

### Or pull image
```bash
$ docker pull ambarishgk007/robotics-research 
```
### Run container
```bash
# without rviz or GUI
$ docker run --name <container-name> --gpus all -it mmdetection3d bash
# with gui
$ docker run --name peaceful --gpus all -it     --env="DISPLAY=$DISPLAY"     --env="QT_X11_NO_MITSHM=1"     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"     mmdetect bash
```

This will let you build it but you wont be able to run rviz until it is ported out