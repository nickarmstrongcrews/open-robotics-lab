#!/usr/bin/env bash

# Break on error
set -e

PROGNAME=$(basename $0)
function error_exit
{
    echo "${PROGNAME}: ${1:-"Unknown Error"}" 1>&2
    exit 1
}

##############################################################################
# Install Essentials
##############################################################################
 
# Set MIT Mirror
sudo sed -i -e 's/us\.archive\.ubuntu\.com/mirrors\.mit\.edu/' -e 's/security\.ubuntu\.com/mirrors\.mit\.edu/' /etc/apt/sources.list
 
# Update the package list, and install all available package updates.
sudo apt-get update && sudo apt-get dist-upgrade -y
 
# Ubuntu Extras
# Installs fonts and media, standard compilers, chrony for time sync, ssh for remote access, and ack
sudo apt-get install -y ubuntu-restricted-extras build-essential keychain chrony openssh-server ack
 
# Install VCS
## All of the essential verison control tools
sudo apt-get install -y git tig git-gui git-svn gitk subversion mercurial python-pip
 
# Editors
## Install vim and emacs, for those who don't like pico.
sudo apt-get install -y vim vim-gtk emacs
 
# Bonus Terminals
## Extra terminals and terminal multiplexers
sudo apt-get install -y rxvt-unicode tmux byobu terminator
 
# Google Chrome
pushd /tmp
wget https://dl.google.com/linux/direct/google-chrome-stable_current_i386.deb
sudo dpkg -i /tmp/google-chrome-stable_current_i386.deb
popd

##############################################################################
# Essential Configuration
##############################################################################

# Get hostname
HOSTNAME=`cat /etc/hostname`

# Dotfiles
## Install Michael Carroll's dotfiles
sudo pip install dotfiles
mkdir -p ~/devel/lib/dotfiles
git clone https://github.com/mjcarroll/dotfiles ~/devel/lib/dotfiles
pushd ~/devel/lib/dotfiles/
git branch -t iap origin/iap
git checkout iap
git submodule init && git submodule update
dotfiles -s -R ~/devel/lib/dotfiles --force
popd

# Indicators
## 
mkdir -p ~/.config/autostart
sudo apt-get install -y indicator-multiload indicator-cpufreq
indicator-multiload&
#TODO: add cpu, memory, and network graphs

# Power
## Do not suspend, hybernate, or shut down when the lid is closed
# MANUAL: System Settings -> Power
gsettings set org.gnome.settings-daemon.plugins.power lid-close-ac-action 'nothing'
gsettings set org.gnome.settings-daemon.plugins.power lid-close-battery-action 'nothing'

# Host table
## Set /etc/hosts
sudo bash -c "cat >/etc/hosts" <<'EOF'
127.0.0.1	localhost
127.0.1.1	HOSTNAME

192.168.1.200   turtlevm-static
192.168.1.202   turtlebot-static

# The following lines are desirable for IPv6 capable hosts
::1     ip6-localhost ip6-loopback
fe00::0 ip6-localnet
ff00::0 ip6-mcastprefix
ff02::1 ip6-allnodes
ff02::2 ip6-allrouters
EOF
sudo sed -e "s/HOSTNAME/$HOSTNAME/" -i /etc/hosts

# Network
## Create TurtleBot-Static-Address, TurtleVM-Static-Address, and DHCP-Address system connections
sudo bash -c "cat >/etc/NetworkManager/system-connections/TurtleBot-Static-Address" <<'EOF'
[802-3-ethernet]
duplex=full

[connection]
id=TurtleBot-Static-Address
uuid=UUID
type=802-3-ethernet
autoconnect=false

[ipv6]
method=auto

[ipv4]
method=manual
addresses1=192.168.1.202;24;192.168.1.1;
EOF
UUID=`uuidgen`
sudo sed -i /etc/NetworkManager/system-connections/TurtleBot-Static-Address -e "s/UUID/$UUID/"
sudo chmod 300 /etc/NetworkManager/system-connections/TurtleBot-Static-Address 
#
sudo bash -c "cat >/etc/NetworkManager/system-connections/TurtleVM-Static-Address" <<'EOF'
[802-3-ethernet]
duplex=full

[connection]
id=TurtleVM-Static-Address
uuid=UUID
type=802-3-ethernet
autoconnect=false

[ipv6]
method=auto

[ipv4]
method=manual
addresses1=192.168.1.200;24;192.168.1.1;
EOF
UUID=`uuidgen`
sudo sed -i /etc/NetworkManager/system-connections/TurtleVM-Static-Address -e "s/UUID/$UUID/"
sudo chmod 300 /etc/NetworkManager/system-connections/TurtleVM-Static-Address 
#
sudo bash -c "cat >/etc/NetworkManager/system-connections/DHCP-Address" <<'EOF'
[802-3-ethernet]
duplex=full

[connection]
id=DHCP-Address
uuid=UUID
type=802-3-ethernet

[ipv6]
method=auto

[ipv4]
method=auto
EOF
UUID=`uuidgen`
sudo sed -i /etc/NetworkManager/system-connections/DHCP-Address -e "s/UUID/$UUID/"
sudo chmod 300 /etc/NetworkManager/system-connections/DHCP-Address 

# Launcher
## Setup the Launcher
gsettings set com.canonical.Unity.Launcher favorites "['nautilus-home.desktop', 'firefox.desktop', 'terminator.desktop']"

##############################################################################
# ROS
##############################################################################
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update

sudo apt-get install -y python-rosdep python-rosinstall python-rospkg
sudo apt-get install -y ros-groovy-desktop-full
source /opt/ros/groovy/setup.bash
sudo rosdep init; rosdep update
 
##############################################################################
# Turtlebot-specific
##############################################################################

# Note: Much, but not all, of the below is redundant w/ ros-groovy-desktop-full.

# Basic ROS
sudo apt-get install -y ros-groovy-ros ros-groovy-ros-comm
 
# General Catkinized stacks
sudo apt-get install -y ros-groovy-geometry-msgs ros-groovy-sensor-msgs ros-groovy-nav-msgs ros-groovy-tf ros-groovy-pcl ros-groovy-pcl-ros
sudo apt-get install -y ros-groovy-nodelet ros-groovy-robot-state-publisher ros-groovy-diagnostic-msgs ros-groovy-dynamic-reconfigure
 
# General non catkin stacks
sudo apt-get install -y ros-groovy-diagnostics ros-groovy-xacro ros-groovy-visualization-common
sudo apt-get install -y ros-groovy-navigation ros-groovy-slam-gmapping ros-groovy-bullet
 
# Zeroconf
sudo apt-get install -y ros-groovy-zeroconf-avahi
 
# Kobuki
sudo apt-get install -y ros-groovy-ecl-threads ros-groovy-ecl-devices ros-groovy-ecl-mobile-robot ros-groovy-ecl-streams ros-groovy-ecl-sigslots ros-groovy-yujin-ocs
 
# Perception
sudo apt-get install -y ros-groovy-openni-launch ros-groovy-depthimage-to-laserscan
 
# Vizualisation
sudo apt-get install -y ros-groovy-rviz ros-groovy-robot-model-visualization ros-groovy-rqt
 
# Turtlebot Apps
sudo apt-get install -y ros-groovy-joystick-drivers ros-groovy-image-transport ros-groovy-image-transport-plugins
sudo apt-get install -y ros-groovy-multimaster-experimental ros-groovy-warehouse-ros ros-groovy-python-orocos-kdl
 
# Arm
sudo apt-get install -y ros-groovy-dynamixel-motor
 
# Gazebo
sudo apt-get install -y libprotobuf-dev libprotoc-dev libtar-dev

# Turtlebot and Viz
sudo apt-get install -y ros-groovy-turtlebot* ros-groovy-rqt*

##############################################################################
# IAP
##############################################################################

# IAP Class Git Repo / open-robotics-lab workspace
##
mkdir -p ~/open-robotics-lab
git clone https://github.com/mjcarroll/open-robotics-lab ~/open-robotics-lab
pushd ~/open-robotics-lab
git branch -t 1.0 origin/1.0
git checkout 1.0
rosws regenerate
rosws merge /opt/ros/groovy
rosws update
popd

# Setup .rosrc-local
##
bash -c "cat >~/.rosrc-local" <<'EOF'
export TURTLEBOT_BASE=kobuki
export TURTLEBOT_STACKS=hexagons
export TURTLEBOT_3D_SENSOR=kinect
export TURTLEBOT_SIMULATION=false

source ~/open-robotics-lab/setup.bash

#export ROS_HOSTNAME=turtlebot-static
#export ROS_MASTER_URI=http://turtlebot-static:11311/
EOF

# Setup iap_support
##
mkdir -p ~/devel/ros/iap_support
bash -c "cat >~/devel/ros/iap_support/.rosinstall" <<'EOF'
# IT IS UNLIKELY YOU WANT TO EDIT THIS FILE BY HAND,
# UNLESS FOR REMOVING ENTRIES.
# IF YOU WANT TO CHANGE THE ROS ENVIRONMENT VARIABLES
# USE THE rosinstall TOOL INSTEAD.
# IF YOU CHANGE IT, USE rosinstall FOR THE CHANGES TO TAKE EFFECT
- git: { uri: 'https://github.com/Auburn-Automow/rososc.git', local-name: rososc, version: '0.2'}
- git: { uri: 'https://github.com/Auburn-Automow/rososc_tutorials.git', local-name: rososc_tutorials, version: '0.2'}
- git: { uri: 'https://github.com/Auburn-Automow/rososc_utilities.git', local-name: rososc_utilities, version: 'master'}
- git: { uri: 'https://github.com/mjcarroll/yujin_ocs.git', local-name: yujin_ocs }
EOF
pushd ~/devel/ros/iap_support
rosws regenerate
rosws merge /opt/ros/groovy
rosws update
popd

# Set .ssh folder
##
mkdir -p ~/.ssh
cp ~/open-robotics-lab/setup/id_rsa.turtlebot ~/.ssh/id_rsa
cp ~/open-robotics-lab/setup/id_rsa.pub.turtlebot ~/.ssh/id_rsa.pub
cp ~/open-robotics-lab/setup/id_rsa.pub.turtlevm ~/.ssh/authorized_keys

#
##
sudo cp /opt/ros/groovy/stacks/kobuki/kobuki_ftdi/57-kobuki.rules /etc/udev/rules.d
