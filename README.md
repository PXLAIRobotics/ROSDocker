# ROSDocker

This repository contains Dockerfiles and Bash scripts to build simple Computer Vision with OpenCV (Python and C++) and ROS Docker container images for exemplary and educational purposes. 


## Prerequisites

First of all you need to run Linux ubuntu natively on your host. An Ubuntu LTS distribution is recommended.
Although other distributions (Arch/Manjaro, Debian, RedHat/CentOS) might also work as long as Docker, `bash` and `glxinfo` from `mesa-utils` are available.

If your host is equipped with a (recent) NVIDIA graphics card, nvidia-docker is highly recommended instead of the standard docker installation.

### TL;DR

* Linux (Ubuntu LTS recommended)
* Docker or [nvidia docker](https://github.com/NVIDIA/nvidia-docker)
* `bash`
* mesa-utils (`sudo apt install mesa-utils`)


## Structure and usage

The end results of this repository are an OpenCV Docker container and a ROS Docker container based on Ubuntu 18.04 based.
To differentiate between NVIDIA hosts and normal computers, an intermedate "base" image will be created and used as the fundament for the OpenCV container.
The provided Bash script `0001_build_images.sh` will automatically build the correct base image (in `01a_nvidia_ubuntu18.04/` or `01b_dri_ubuntu18.04/`) for your system.
Thereafter the OpenCV Docker image (in `02_opencv/`) and the ROS Docker image (in `03_ros_full_desktop`) will be build. 

A few Python and C++ OpenCV and ROS examples reside in the `Projects` directory. 
This directory will be used as an volume when running the OpenCV container.
All the files and directories will be accessible from inside and when the container is destroyed, the files will still be on your host.
Besides source code the `Projects` directory also contains a few images.

### Building the images

Just run:

```bash
$ ./0001_build_images.sh

```


### Use the OpenCVDocker container

Execute the following command:

```bash
$ ./0002_start_opencv_container.sh
non-network local connections being added to access control list
user@476750474b19:~$
```


#### Running the OpenCV examples
The following commands are executed from within the OpenCV container.

*Remark: Use `<esc>` to close the created windows.*

##### Python

```bash
user@476750474b19:~$ cd ExampleCode/Python/Examples
user@476750474b19:~/ExampleCode/Python/Examples$ python3 example1.py 

   . . .
   
user@476750474b19:~/ExampleCode/Python/Examples$ python3 example2.py 

   . . .
   
user@476750474b19:~/ExampleCode/Python/Examples$ cd ../Webcam/
user@476750474b19:~/ExampleCode/Python/Webcam$ python3 webcam.py 

   . . . 

```

##### C++ 
C++ examples must be built before running.


```bash
user@476750474b19:~$ cd Projects/C++/DisplayImage/
user@476750474b19:~/Projects/C++/DisplayImage$ cmake .
-- The C compiler identification is GNU 7.4.0
-- The CXX compiler identification is GNU 7.4.0
-- Check for working C compiler: /usr/bin/cc
-- Check for working C compiler: /usr/bin/cc -- works
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Detecting C compile features
-- Detecting C compile features - done
-- Check for working CXX compiler: /usr/bin/c++
-- Check for working CXX compiler: /usr/bin/c++ -- works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Found OpenCV: /usr (found version "3.2.0") 
-- Configuring done
-- Generating done
-- Build files have been written to: /home/user/Projects/C++/DisplayImage
user@476750474b19:~/Projects/C++/DisplayImage$ make
Scanning dependencies of target DisplayImage
[ 50%] Building CXX object CMakeFiles/DisplayImage.dir/DisplayImage.cpp.o
[100%] Linking CXX executable DisplayImage
[100%] Built target DisplayImage
user@476750474b19:~/Projects/C++/DisplayImage$ ./DisplayImage 
usage: DisplayImage.out <Image_Path>
user@476750474b19:~/Projects/C++/DisplayImage$ ./DisplayImage ../../Images/Lenna.png 

   . . .

```
### Use the ROS container

Execute the following command:

```bash
$ ./0003_start_ros_container.sh
non-network local connections being added to access control list
user@basestation:~$
```



# Docker swarm usage
## 0. Before creating the swarm
Before creating the swarm, make sure all host machines are on the same network and can ping each other. Both ways!
Otherwise creating a Docker swarm will not be possible.

One host will work as the master of the Docker swarm all the others will be workers. Use the most network stable machine as the Docker master. Use the strongest machine to run the heaviest load, like Gazebo or certain heavy algorithms.

## 1. Setup the Docker swarm master
Execute the command below on the most network stable machine. This machine will act as the Docker swarm master.

```bash
$ ./0005a_create_docker_swarm_master.sh
SWARM SUCCESSFULLY CREATED.OVERLAY NETWORK SUCCESSFULLY CREATED.SHOWING JOIN COMMAND:Swarm initialized: current node (wo13x3ticl7w0ykx8hu4ofl6h) is now a manager.To add a worker to this swarm, run the following command:    docker swarm join --token SWMTKN-1-3bid0upa4o7n6ihwjkid6fq9jqc349oskqwl4syo98za51n996-3ht3ogbn8wsmcu8ng29racmxt 192.168.1.183:2377To add a manager to this swarm, run 'docker swarm join-token manager' and follow the instructions.
$
```

The command above creates a Swarm master and an overlay network which can be used by standalone containers.
**The output in your terminal will differ!** An unique `join` command will be generated. Execute this generated command on all the other hosts who want to join the swarm.


## 2. Add other host machines as workers
Use the command generated on the Docker master, the previous step, to join the Docker swarm. This command is **unique** for each Docker swarm. Don't just copy and paste the command below! It won't work.

```bash
$ docker swarm join --token SWMTKN-1-3bid0upa4o7n6ihwjkid6fq9jqc349oskqwl4syo98za51n996-3ht3ogbn8wsmcu8ng29racmxt 192.168.1.183:2377
This node joined a swarm as a worker.$
```

## 3. Start a container on the Docker swarm master
Execute the `./0005b_start_ros_container_attached_to_swarm_network.sh` script on the master. It will automatically start the container with the hostname set to `swarm_master`.


```bash
$ ./0005b_start_ros_container_attached_to_swarm_network.shnon-network local connections being added to access control listuser@swarm_master:~$
```
## 4. Start a container on each Docker swarm worker
If the same script (`./0005b_start_ros_container_attached_to_swarm_network.sh`) is executed on a worker, it will automatically start a container with a random generated hostname.


```bash
$ ./0005b_start_ros_container_attached_to_swarm_network.shnon-network local connections being added to access control listuser@iqEamTb3P0loQvZ9UQgxkHqRmo0Dku31:~$ 
```

## 5. Find the overlay network IP address of the Docker swarm master container
Execute the following script on the master host, not in the container! The IP address can differ!
We need this IP in Step 7 on each worker. 

```bash
$ ./0005c_container_overlay_network_ip.sh The overlay network IP address of the container is...10.0.1.2
``` 


## 6. Generate and source the ROS config command for the Docker swarm master
This step also needs to be executed on the master host, not in the running master container! This script will generate a command.
There will be no output on the master host!

```bash
$  ./0005d_generate_ros_config_command.sh 
$
```

Switch to the running master container and execute the following command. This command must be executed each time a new `bash` is started using Tmux or the `0004_start_bash_in_ros_container.sh` script.

```bash
user@swarm_master:~$ source ~/bin/set_swarm_settings.bash user@swarm_master:~$ ```
## 7. Generate and source the ROS config command for each Docker swarm worker
This step also needs to be executed on each worker host, not in the running worker container! This script will generate a command.
The script will ask for the IP address of the master container. Fill in the IP from Step 5.

```bash
$ ./0005d_generate_ros_config_command.shIP on overlay network of the master container: 10.0.1.2
$```

Switch to the running worker container and execute the following command. This command must be executed each time a new `bash` is started using Tmux or the `0004_start_bash_in_ros_container.sh` script.

```bash
user@swarm_master:~$ source ~/bin/set_swarm_settings.bash user@swarm_master:~$ ```

## 8. Start roscore and/or a Gazebo simulation world
On the master container start `roscore` and/or start a Gazebo simulation world. 
**It's important to check if the IP  from Step 5 is used in the output.** This will indicate if the connection could work or not.
 
 
### A. `roscore`
```bash
user@swarm_master:~$ roscore... logging to /home/user/.ros/log/acb46938-62f9-11ea-9635-02420a000102/roslaunch-swarm_master-225.logChecking log directory for disk usage. This may take awhile.Press Ctrl-C to interruptDone checking log file disk usage. Usage is <1GB.started roslaunch server http://10.0.1.2:33943/ros_comm version 1.14.3SUMMARY========PARAMETERS * /rosdistro: melodic * /rosversion: 1.14.3NODESauto-starting new masterprocess[master]: started with pid [237]ROS_MASTER_URI=http://10.0.1.2:11311/setting /run_id to acb46938-62f9-11ea-9635-02420a000102process[rosout-1]: started with pid [250]started core service [/rosout]
```

### B. A Gazebo simulation world
```bash
user@swarm_master:~$ roslaunch turtlebot3_racetrack turtlebot3_pxl_race_battle.launch                        ... logging to /home/user/.ros/log/26745b8e-62fa-11ea-92ca-02420a000102/roslaunch-swarm_master-856.logChecking log directory for disk usage. This may take awhile.                                                       Press Ctrl-C to interrupt                                                                                           Done checking log file disk usage. Usage is <1GB.                                                                                                                                             xacro: in-order processing became default in ROS Melodic. You can drop the option.                     xacro: in-order processing became default in ROS Melodic. You can drop the option.                  started roslaunch server http://10.0.1.2:33259/                                                                                                                                                            SUMMARY                                                                                                                     ========                                     PARAMETERS * /gazebo/enable_ros_network: True * /player_one/robot_description: <?xml version="1.... * /player_one/robot_state_publisher/publish_frequency: 50.0 * /player_one/robot_state_publisher/tf_prefix: player_one * /player_two/robot_description: <?xml version="1.... * /player_two/robot_state_publisher/publish_frequency: 50.0 * /player_two/robot_state_publisher/tf_prefix: player_two * /rosdistro: melodic * /rosversion: 1.14.3 * /use_sim_time: TrueNODES  /player_two/    robot_state_publisher (robot_state_publisher/robot_state_publisher)    spawn_urdf (gazebo_ros/spawn_model)  /player_one/    robot_state_publisher (robot_state_publisher/robot_state_publisher)

  . . .
 
```

## 9. Test, on the worker, the topics on the command line and/or a video stream in rviz
Go to a worker container. If you followed the previous steps, all containers will be to communicate. You can test this with `rostopic list` or start `rviz` and add the camera topic. Be amazed with the video stream working over the network.

```bash
user@iqEamTb3P0loQvZ9UQgxkHqRmo0Dku31:~$ rostopic list        /clock                                                   /gazebo/link_states/gazebo/model_states/gazebo/parameter_descriptions/gazebo/parameter_updates/gazebo/set_link_state/gazebo/set_model_state/player_one/camera/depth/camera_info/player_one/camera/depth/image_raw/player_one/camera/depth/points/player_one/camera/parameter_descriptions

  . . .

```


## 10. Leave the swarm
To remote a host from the swarm, be it a worker of the master, the following scripts needs to be executed.
If something in the previous steps goes wrong, execute this command and start from Step 1.

```bash
$ ./0005z_leave_the_swarm.sh Node left the swarm.$
```
