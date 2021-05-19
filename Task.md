# 
Lab Transformations				


# Introduction and Prerequisites

In this lab you will explore the use of Augmented Reality tags and a ROS library for identifying and tracking these tags, or bundles of multiple tags, using camera images (see this page for more details on the ROS package we are going to use [http://wiki.ros.org/ar_track_alvar](http://wiki.ros.org/ar_track_alvar) ) 

To reach the objectives of the lab you will also get more confidence with the use of **coordinate frames** and **coordinate transformations** in ROS using TF / TF2. 

The following resources and tools are required for this laboratory session:



*   Any modern web browser, 
*   Any modern SSH client application.

A set of VMs have been set up: one for each group. See on Moodle for IPs and credentials.


## Useful documentation

Use of aruco tags: [http://wiki.ros.org/ar_track_alvar](http://wiki.ros.org/ar_track_alvar) 


# Time & Assessment

The entire session will take about 30 hours.


# Important: Environment

The lab repository directory is mounted from the host machine into your container under catkin_ws/src/&lt;lab_name>

This means that if you **don’t want to lose all your work after exiting the container**, you should create any project and file under that path. Notice that catkin allows nesting multiple directories by default.


# Task

1. Identify the pose of a marker visible from a camera on the robot
2. Navigate "in proximity" of the marker (e.g., 0.5 meters, configurable)
3. Start rotating until a different marker appears in the camera, go back to point 1 and keep looping

# Important: Commit, Push, Cleanup!

Change the scale of the model.sdf from marker13 to 0.45

```
<scale>0.45 0.45 0.45</scale></mesh>
```

You will find it under /home/ros/.gazebo/models/marker13 But first you will have to change the permissions of the file as a sudo.

Your lab directory (a git local repository) was mounted inside the container.

If you forked the repository, this is the moment to **commit your code** and **push it** on the remote repo.

Don’t forget to stop the running container in your VM.

Press CTRL-C in the console where you launched the run_gpu.sh command

In case that doesn’t work, start another ssh connection to the VM



*   Find running containers with 
```
docker ps
```


*   Kill the remaining container with

```
docker kill <container_id>
```
