#/bin/bash
nvidia-docker run --init --name=ros-gpu --rm -it -e DISPLAY=:1 -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0:rw -v $PWD:/home/ros/catkin_ws/src/rap_pose_lab --privileged --net host robopaas/rap-lab-kinetic-gpu:rap20210323
