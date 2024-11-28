# MM3D + ROS2 (Docker)
The Dockerfile contains insctruction to build docker images that canbe used to run mmdetection3d and ros2 in docker. 
### Build Image
```bash
$ docker build -t ros2-mmdetection .
```
The above instruction should let you create the image in your machine

### Run container
```bash
$ docker run --name <container-name> --gpus all -it mmdetection3d bash
```

This will let you build it but you wont be able to run rviz until it is ported out