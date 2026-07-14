# ROS2 Dockerfile

## Docker images

### Download docker images

Full SLAM docker images for different ROS distributions are available:
- ROS2 Humble: [download link](https://send.kitware.io/download.php?id=3409&token=494aad1e318ae65ab997b866a310c66b)
- ROS2 Jazzy: [download link](https://send.kitware.io/download.php?id=3411&token=f87dedee1cc685db5ad800805ad805c1)

Apply this command to add it locally to your system :

```
docker load -i slam_image.tar
```

### Build docker images

Kitware provides 2 dockerfiles :
* [Dockerfile.env](Dockerfile.env) : it sets the environment (dependencies, variables, etc.) for development tests.
* [Dockerfile.slam](Dockerfile.slam) : it calls the image built from the previous Dockerfile and adds the SLAM package.

You can build the environment image with the following command in the docker folder:
```
docker build -t ros2_env -f Dockerfile.env .
```

Then you can build the full package image with the following command (still from the docker folder):
```
docker build -t ros2_slam -f Dockerfile.slam .
```

## Use docker containers

This section explains how to run the docker SLAM container and to communicate with it.

In this example, we run the slam container, run another container to send the data and we establish a communication between both.

### 1. Create network
```
docker network create ros_net
```

### 2. Launch SLAM

The following steps allow to launch the SLAM ROS nodes expecting data in a first terminal.

#### 2.1 Run SLAM container

To be able to communicate with the slam container, you need to run the image with a network setting and eventually the domain ID.
```
docker run -it \
  --net ros_net \
  -e ROS_DOMAIN_ID=5 \
  --rm \
  ros2_slam bash
```

**NOTE**: replace ```ros2_slam``` by the current name of your image. You can check it with the `docker images` command.

To enable graphic display on your host (to use rviz notably), share the tmp driver folder, the DISPLAY environment variable and the device between the container and the host.
The command should become :

```
docker run -it \
  --net ros_net \
  -e ROS_DOMAIN_ID=5 \
  --rm \
  -e "DISPLAY=$DISPLAY" \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix \
  --device=/dev/dri:/dev/dri \
  ros2_slam bash
```

#### 2.2 Launch SLAM pipeline

```
ros2 launch lidar_slam slam_velodyne.launch.py domain_id:=5
```

**NOTES**:
* Change the launch file relatively to your data.
* You can add the command to the [slam_entrypoint.bash](slam_entrypoint.bash) file and rebuild the image using its [Dockerfile](Dockerfile.slam) if you want the command to be executed at your container start.

### 3. Launch data sender container

The following steps allow to launch a new container running ros2 bag node which replays data in a second terminal.

#### 3.1 Download ros humble image
```
docker pull osrf/ros:humble-desktop
```

#### 3.2 Run replay container

This container mounts a volume to share the data with host. It should be on the same network as the SLAM container.
```
docker run -it \
  --rm \
  --net ros_net \
  -e ROS_MASTER_URI=http://roscore:11311 \
  -e ROS_DOMAIN_ID=5 \
  -v=/path/on/host/to/bag_folder:/data \
  --name replay \
  osrf/ros:humble-desktop
```

**NOTE**: If you don't want to mount a volume, you can download the data directly in the container from an external source.

##### 3.3 Launch replay command
```
ros2 bag play /data/<bag_name> -d1 --clock
```

## Store image on gitlab registry

This section is for Kitware developers.

To create a container image on gitlab, you first need to build the image locally and then you can push it on the remote repository.

Example to create an image in the Slam repository under the name slam:my_image
```
docker build . -t my_image
docker login gitlab.kitware.com:4567
docker push gitlab.kitware.com:4567/keu-computervision/slam:my_image
```