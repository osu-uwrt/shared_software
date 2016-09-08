#!/bin/bash


# A script to install ROS (~desktop-full) with Gazebo 7.
#
#   For Ubuntu releases prior to Xenial, this script
#   installs the gazebo7-ros-pkgs and gazebo7-ros-control
#   packages from the OSRF repository and the perception,
#   robot and viz packages from the ROS repository (to
#   provide ~desktop-full functionality).
#
#   Also installed, are other packages from the Ubuntu
#   and Sixaxis repositories to enable system development.


# Identify the current OS release:

RELEASE="$(lsb_release -cs)"

# Select the appropriate version ROS for the current OS:

if [ "${ROS_DISTRO:-false}" == "false" ]; then

  if [ "$RELEASE" == "trusty" ]; then ROS_DISTRO=indigo
  elif [ "$RELEASE" == "vivid" ]; then ROS_DISTRO=jade
  elif [ "$RELEASE" == "xenial" ]; then ROS_DISTRO=kinetic
  else echo "We've had a MAIN B BUS UNDERVOLT."; fi

fi

# Send and notification of what is being installed to std out:

distro="$ROS_DISTRO"
distro=$(tr '[:lower:]' '[:upper:]' <<< "${distro:0:1}")"${distro:1}"
release="$RELEASE"
release=$(tr '[:lower:]' '[:upper:]' <<< "${release:0:1}")"${release:1}"
echo -e "\nInstalling ROS $distro on Ubuntu $release.\n"


# Add ROS, Gazebo and Sixaxis repositories and keys:

echo -n "Adding ROS repository...\n"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros-latest.list'
wget -nv http://packages.ros.org/ros.key -O - | sudo apt-key add -
echo -e "...done.\n"

if [ "$RELEASE" != "xenial" ]; then

  echo -n "Adding Gazebo 7 repository...\n"
  sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
  wget -nv http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
  echo -e "...done.\n"

  echo "Adding Sixaxis repository..."
  sudo apt-add-repository -y ppa:falk-t-j/qtsixa
  echo -e "...done.\n"

fi

# Update repositories to reflect the preceding additions:

echo "Updating package lists..."
sudo apt-get -qq update
echo -e "...done.\n"


# Install Ubuntu packages:

echo -e "Installing Ubuntu packages...\n"
sudo apt-get -yqq install openssh-server joystick
echo -e "\n...done.\n"

if [ "$RELEASE" == "xenial" ]; then

  # Install ROS Desktop Full:

  sudo apt-get -yqq install ros-"$ROS_DISTRO"-destop-full

else

  # Install Sixaxis package:

  echo -e "Installing Sixaxis package...\n"
  sudo apt-get -yqq install sixad
  echo -e "\n...done.\n"

  # Install (most of) ROS Desktop Full (sans tutorials):

  echo -e "Installing Gazebo 7 packages...\n"
  sudo apt-get -yqq install ros-"$ROS_DISTRO"-gazebo7-ros-pkgs
  sudo apt-get -yqq install ros-"$ROS_DISTRO"-gazebo7-ros-control
  echo -e "\n...done.\n"

  echo -e "Installing ROS packages...\n"
  sudo apt-get -yqq install ros-"$ROS_DISTRO"-perception
  sudo apt-get -yqq install ros-"$ROS_DISTRO"-robot
  sudo apt-get -yqq install ros-"$ROS_DISTRO"-viz
  echo -e "\n...done.\n"

fi

# Requirements not found in "(most of) ROS Desktop Full":

echo -e "Initializing and updating rosdep...\n"
sudo rosdep -yq init
rosdep -yq update # Do not sudo!
echo -e "\n...done.\n"

# Automatically source ROS from now on:

echo "Adding ROS to ~/.bashrc..."
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo -e "...done.\n"


# Donezo!

echo -e "Installation complete.\n"


# Let the user know how to make the current terminal work:

echo -e "\nRun \`source ~/.bashrc\` to update the current shell.\n"
