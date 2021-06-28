# Installation

1. Install Ubuntu 18.04. You can download the ISO Desktop image from [here](https://releases.ubuntu.com/18.04/) and:
   
   a. Create a bootable USB drive, boot from it and install Ubuntu on your computer ([link](https://itsfoss.com/install-ubuntu-dual-boot-mode-windows/))
   
   b. Or use a virtual machine like VirtualBox ([example for Windows](https://itsfoss.com/install-linux-in-virtualbox/)). Note that you may experience performance issues depending on your cpu/gpu power.  
2. Install ROS Melodic ([link](http://wiki.ros.org/melodic/Installation/Ubuntu))

   a. Install the recommended `ros-melodic-desktop-full` version

   b. Complete also the steps in "1.6 Dependencies for building packages"

3. Install IRI dependencies ([iriutils](https://gitlab.iri.upc.edu/labrobotica/algorithms/iriutils)):

    ```bash
    sudo sh -c 'echo "deb [arch=amd64] https://labrepo.iri.upc.edu/packages $(lsb_release -cs) main" > /etc/apt/sources.list.d/labrobotica_repo.list'
    
    wget -O - https://labrepo.iri.upc.edu/labrobotica_repo.gpg.key | sudo apt-key add -
    
    sudo apt update
    
    sudo apt install iri-iriutils-dev
    ```

4. Use an existing [ROS workspace](http://wiki.ros.org/catkin/workspaces), or [create a new one](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). For example:

    ```bash
    mkdir -p ~/summer_school/catkin_ws/src

    cd ~/summer_school/catkin_ws

    source /opt/ros/melodic/setup.bash

    catkin_make
    ```

5. Be sure to load/source correctly your workspace. To confirm it, if you run `roscd`, the path should change to your workspace/devel directory. 

   a. For each terminal:
   
      ```bash
      source ~/summer_school/catkin_ws/devel/setup.bash
      ```

   b. Or add it to your bashrc file so it’s auto-loaded for all new opened terminals: 
   
      ```bash
      echo "source ~/summer_school/catkin_ws/devel/setup.bash" >> ~/.bashrc
      
      source ~/.bashrc
      ```

6. Download the following rosinstall file: [iri_summer_school_nav_workshop.rosinstall](../rosinstall/iri_summer_school_nav_workshop.rosinstall), and use [wstool](http://wiki.ros.org/wstool#Using_wstool) to merge it with your workspace.

   ```bash
   roscd && cd ..
   mkdir -p rosinstall && cd rosinstall
   wget https://gitlab.iri.upc.edu/mobile_robotics/summer_school/navigation_workshop/iri_summer_school_nav_workshop_how_to/-/raw/master/rosinstall/iri_summer_school_nav_workshop.rosinstall

   roscd && cd ../src
   
   wstool init .
   
   wstool merge ../rosinstall/iri_summer_school_nav_workshop.rosinstall
   ```

8. Use wstool to update/download all the packages.

   ```bash
   wstool update
   ```

9. Install all package dependencies with:

   ```bash
   roscd && cd ../src

   rosdep install -i -y --from-paths . 
   ```

10. Compile your workspace with:

    ```bash
    roscd && cd .. 

    catkin_make
    ```
