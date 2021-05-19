This project is based on ROS Kinetic and covers the topics of the RAP course lab: **RAP_POSE_LAB**

You have three options to work with the project as detailed below.

## On the GPU VMs provided by the course

When working with the VMs with GPU provided by the course, you can launch the containers with the following command:

    ./run_gpu.sh

This will start one containers with the ROS Kinetic environment installed for you to work. Note that you will be required to set a password that you will need to access the desktops using the browser. You can access with your browser at: https://IP_ADDRESS/vnc.html

Click on the Connect button and you will be asked to insert the password you have defined when you started the container.

-------------
## On your own laptop (Any OS supporting docker compose)

When working with your own laptop, you can launch the containers using docker-compose with the following command:

    docker-compose up

This will launch two containers, one with the ROS Kinetic environment installed for you to work on, and one for novnc (the GUI) which you can access through your browser at: http://IP_ADDRESS/vnc.html.

In another terminal, you can access the ROS container with:

    docker exec -it rap_pose_lab_ros_1 bash
  
From that console you can launch the scripts of the lab (e.g., roslaunch ...).
In the end you'll have a split environment where you have anything that has to do with the GUI in the browser, but you can launch commands from the console.

*NOTE:* the ZHAW VPN client gets in the way of the networking between the containers. If you experience issues try disconnecting from the VPN and restarting the composition.

### Windows users

Thank to a student volunteer (thanks Michael Y!) we have an update for running on Win. You'll need a TTY application, so:
- find out the ID of the ros container in the composition with *docker ps*
- launch bash in that container with:

        winpty docker exec -it YOUR_CONTAINER_ID bash

You can use that command multiple times to get multiple consoles.

-------------
## On your own laptop (any Linux with Xserver and docker)

You may also want to start a container without novnc, but connecting to a *local running Xserver instance*. For this you can use the following command:

    ./run.sh

Note that the xhost needs to be correctly set in order to make it work correctly. (xhost+ would do the trick)
