## Configuring the Jet Robot

This document describes the first steps you will need to go through in order to prepare
the Jetson board for use with Jet.

After opening the NVIDIA Jetson box, you will need the following items:

* Jetson board
* micro USB cable
* AC power adapter

In addition, you will need:

* an ethernet cable
* a router that is connected to the Internet

## Preparing the Jetson Board

* Plug one end of the ethernet cable into the Jetson board and plug the other end into the router
* Do not plug in power adapter just yet (you will plug in power during the OS flash step)

## Running JetPack

Download and install the [NVIDIA Jetpack](https://developer.nvidia.com/embedded/jetpack).
You will need to be running Ubuntu 14.04 on your host computer.

Once Jetpack is up and running perform the following actions:

* Select TK1 as target board
* Click the `Clear Actions` button (upper right corner).  This will de-select all of the options.
* Select the following options for install:
  * `Driver for OS`
  * `File System`
  * `Flash OS`
  * `CUDA Toolkit for L4T`
  * `cuDNN Package`
  * `OpenCV for Tegra`
  * also select the box `Automatically resolve dependency conflicts`
* On the Network Layout page:
  * Select `Device accesses Internet via router/switch`
* On the Network Interface Selection page:
  * select `eth0` (assuming your host computer only has 1 ethernet)

## Flash the OS

After you have started the installation, the JetPack installer will download files and
install them.  As the installer continues, it will reach a point where it will flash the
OS to the Jetson board.  At this point, the installer will ask you to put the board into
Force USB Recovery Mode:

* Connect the micro USB cable to the Jetson TK1 and the host computer
* Plug the AC power adapter into AC power and connect the power jack to the board
* Enter USB Recovery Mode using the following sequence:
  * Press and release Power button
  * Hold down the Force Recovery button, then press and release Reset (while holding down Force Recovery)
  * You can check if the board is in Recovery mode by connecting to the Jetson board via ssh and running `lsusb`.  There should be a device with vendor 'NVidia Corp'
  * Begin the flashing process on the host by pressing Enter.  This will take several minutes.

## Installing rosjet

Once the install is complete, you can go ahead and disconnect the micro USB cable.  Now we will install the rosjet libraries and utilities.

* Connect to the Jetson via ssh
  The username is 'ubuntu' and the password is 'ubuntu'. You can find it's ip address by looking at your router's configuration.

* Install git

  `sudo apt-get install git`

* Make the ROS workspace directory

  `mkdir -p ~/catkin_ws/src; cd ~/catkin_ws/src`

* Clone this repository into the workspace

  `git clone https://github.com/NVIDIAGPUTeachingKit/rosjet.git`

* Run the ros configuration script

  `./rosjet/rosjet_install.sh`

* Configuration of the Jetson is now complete; reboot the Jetson board before continuing.
