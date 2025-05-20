locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update -y && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update -y

sudo apt upgrade -y

sudo apt install ros-dev-tools ros-jazzy-desktop -y

git submodule init
git submodule update

cd WiringPi

./build debian
mv ./debian-template/wiringpi_*.deb .

# install it
sudo apt install ./wiringpi_*.deb -y

sudo apt-get install qt6-base-dev qt6-svg-dev libxml2-dev socat -y

cd ..

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/modelec-marcel-ROS/install/setup.bash" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=~/modelec-marcel-ROS/fastdds_setup.xml
export ROS_DOMAIN_ID=128" >> ~/.bashrc

source ~/.bashrc