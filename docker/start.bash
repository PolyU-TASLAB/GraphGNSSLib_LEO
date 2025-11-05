# graphlib_leo#!/bin/bash
docker container ls -a -f name=graphlib_leo | grep graphlib_leo$ > /dev/null

if [ $? == 0 ]
then
	docker container start graphlib_leo
	docker exec -it graphlib_leo /bin/bash

else
	xhost +
	SHARED_DOCKER_DIR=/root/GraphGNSSLib_LEO_v1.2
	SHARED_HOST_DIR=$(realpath ../../../GraphGNSSLib_LEO_v1.2)
	docker run -it 	-v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE -v $SHARED_HOST_DIR:$SHARED_DOCKER_DIR --privileged=true  --name graphlib_leo ros:GraphGNSSLib_LEO /bin/bash
	#docker run --gpus all -it -v /home/hrz/project/ros/melodic/LvisamTest:/root/LvisamTest  -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE -e ROS_MASTER_URI=http://172.17.0.2:8899 -e ROS_HOSTNAME=172.17.0.2 --cpus 0.5 --cpuset-cpus=2 --name melodic melodic-gpu /bin/bash 
	#docker run -it   -v /tmp/.X11-unix:/tmp/.X11-unix -v $SHARED_HOST_DIR:$SHARED_DOCKER_DIR  -e DISPLAY=unix$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE -e ROS_MASTER_URI=http://172.17.0.2:11311 -e ROS_HOSTNAME=172.17.0.2 --name graphlib_leo ros:GraphGNSSLib_LEO /bin/bash 
	sudo chmod 777 $SHARED_HOST_DIR
fi