apt-get update
apt-get install lsb -y

sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
apt update
apt install ros-melodic-desktop-full

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

apt install python-rosdep
rosdep init
rosdep update

echo "alias cm='cd ~/catkin_ws && catkin_make'" >> ~/.bashrc
echo "eb='vim ~/.bashrc'" >> ~/.bashrc
echo "sb='source ~/.bashrc'" >> ~/.bashrc
