if [[ "$(uname -a)" =~ ^.*aarch64.*$ ]]; then
  ISTX1=true
else
  ISTX1=false
fi

# Install Grinch Kernel if Tk1
if ! $ISTX1
then
  cd ~/; git clone https://github.com/jetsonhacks/installGrinch.git
  cd installGrinch; ./installGrinch.sh
fi

#Configure time-zone
sudo dpkg-reconfigure tzdata

#Ros Prerequisites
sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update

#Ros Jade Base
sudo apt-get -y install ros-jade-ros-base

#Python Dependencies
sudo apt-get -y install python-rosdep python-dev python-pip python-rosinstall python-wstool

sudo rosdep init
rosdep update
echo "source /opt/ros/jade/setup.bash" >> ~/.bashrc
source ~/.bashrc

#Ros packages
sudo apt-get -y install ros-jade-rosserial-arduino
sudo apt-get -y install ros-jade-rosserial
sudo apt-get -y install ros-jade-eigen-conversions
sudo apt-get -y install ros-jade-tf2-geometry-msgs
sudo apt-get -y install ros-jade-angles
sudo apt-get -y install ros-jade-web-video-server
sudo apt-get -y install ros-jade-rosbridge-suite
sudo apt-get -y install ros-jade-rospy-tutorials
sudo apt-get -y install ros-jade-joy
sudo apt-get -y install ros-jade-teleop-twist-joy
sudo apt-get -y install ros-jade-roslint
sudo apt-get -y install ros-jade-controller-manager
sudo apt-get -y install ros-jade-camera-calibration-parsers
sudo apt-get -y install ros-jade-xacro
sudo apt-get -y install ros-jade-robot-state-publisher
sudo apt-get -y install ros-jade-diff-drive-controller
sudo apt-get -y install ros-jade-usb-cam
sudo apt-get -y install ros-jade-ros-control
sudo apt-get -y install ros-jade-dynamic-reconfigure

# Configure Catkin Workspace
source /opt/ros/jade/setup.bash
cd ~/catkin_ws/src
catkin_init_workspace

#Install Ros Opencv bindings from source
cd ~/catkin_ws
wstool init src
wstool merge -t src src/rosjet/rosjet.rosinstall
wstool update -t src

#Install Caffe (https://gist.github.com/jetsonhacks/acf63b993b44e1fb9528)
sudo add-apt-repository universe
sudo apt-get update
sudo apt-get install libprotobuf-dev protobuf-compiler gfortran \
libboost-dev cmake libleveldb-dev libsnappy-dev \
libboost-thread-dev libboost-system-dev \
libatlas-base-dev libhdf5-serial-dev libgflags-dev \
libgoogle-glog-dev liblmdb-dev -y
sudo usermod -a -G video $USER
cd ~/
git clone https://github.com/BVLC/caffe.git
cd caffe && git checkout dev
cp Makefile.config.example Makefile.config
make -j 4 all

# System Optimizations
gsettings set org.gnome.settings-daemon.plugins.power button-power shutdown
gsettings set org.gnome.desktop.screensaver lock-enabled false
sudo apt-get -y install compizconfig-settings-manager
gsettings set org.gnome.desktop.interface enable-animations false
gsettings set com.canonical.Unity.Lenses remote-content-search none
echo '[SeatDefaults]\nautologin-user=ubuntu' > login_file; sudo mv login_file /etc/lightdm/lightdm.conf
gsettings set org.gnome.Vino enabled true
gsettings set org.gnome.Vino disable-background true
gsettings set org.gnome.Vino prompt-enabled false
gsettings set org.gnome.Vino require-encryption false

echo "alias sr='source ~/catkin_ws/devel/setup.bash'" >> ~/.bashrc

cd ~/catkin_ws
catkin_make && source devel/setup.sh
