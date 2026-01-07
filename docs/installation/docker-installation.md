# Docker Installation

In order to run the simulations explained in the [Tutorials](/tutorials) section, you will need to proceed with the Docker installation.

!!! Note
    Also refer to [Autoware's Docker Installation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/docker-installation/) for the Docker-based installation of Autoware.universe.

## Installing Autoware and AutowareV2X (Docker version)

Since AutowareV2X hasn't been updated in a while, it's adequate to use an older release of Autoware inside a Docker container, as opposed to the latest (as per the [Official Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/)). Run the following commands:

```bash
# Clone repository
mkdir -p ~/workspace && cd ~/workspace
git clone https://github.com/autowarefoundation/autoware.git autoware_docker
cd autoware_docker

# Switch to the last commit of 2023, this is the version we'll use
git checkout bf95c380db6debdf07fb9b6854036df567e98903

# Make directory to store maps
mkdir -p ~/data/maps

# Create the Docker container (let's name it aw-v2x_devel, for example)
docker run -it --name aw-v2x_devel --gpus all --privileged --user root -e DISPLAY=$DISPLAY -e XAUTHORITY=/root/.Xauthority -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $HOME/.Xauthority:/root/.Xauthority:rw -v $HOME/workspace:/root/workspace -v $HOME/data:/root/data -w /root/workspace ghcr.io/autowarefoundation/autoware:20240315-devel-cuda
```

!!! Note
    From here, run commands inside the container.

```bash
# Fix the outdated signing keys
rm /etc/apt/sources.list.d/ros2*
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade
```

Replace the `autoware.repos` file with the following:

```
repositories:
  core/autoware.core:
    type: git
    url: https://github.com/autowarefoundation/autoware.core.git
    version: 6bafedfb24fb34157ed65bfe3f6f4c1ed0fbc80b
  core/autoware_adapi_msgs:
    type: git
    url: https://github.com/autowarefoundation/autoware_adapi_msgs.git
    version: 9679b5a7a1f4cfff2fa50b80d2759d3937f2f953
  core/autoware_common:
    type: git
    url: https://github.com/autowarefoundation/autoware_common.git
    version: 6916df26fafe6749db4b1d5bd6636a92444fc48d
  core/autoware_msgs:
    type: git
    url: https://github.com/autowarefoundation/autoware_msgs.git
    version: 4f13d4b8b465ed7f424fce9af17882dbe1752875
  core/external/autoware_auto_msgs:
    type: git
    url: https://github.com/tier4/autoware_auto_msgs.git
    version: 6b5bc4365f9a2fc913bc11afa74ec21ffa2dbf32
  launcher/autoware_launch:
    type: git
    url: https://github.com/autowarefoundation/autoware_launch.git
    version: e4abe673667a8d4f2d783ed22edacbf5d4784b8f
  param/autoware_individual_params:
    type: git
    url: https://github.com/autowarefoundation/autoware_individual_params.git
    version: 79cff0ba014808050be6f5cb3b4764ba2c96c21c
  sensor_component/external/sensor_component_description:
    type: git
    url: https://github.com/tier4/sensor_component_description.git
    version: 475857daeb4c4883ab0295336713364b326e8278
  sensor_component/external/tamagawa_imu_driver:
    type: git
    url: https://github.com/tier4/tamagawa_imu_driver.git
    version: 28ad3cd4fb043e5f92353a540c3531cd4cb7bef3
  sensor_component/external/velodyne_vls:
    type: git
    url: https://github.com/tier4/velodyne_vls.git
    version: baeafaf9a376c5798f7b67a77211890c33900f84
  sensor_kit/external/awsim_sensor_kit_launch:
    type: git
    url: https://github.com/RobotecAI/awsim_sensor_kit_launch.git
    version: d9022ee9bbfd958c239b673cfbb230eea50607be
  sensor_kit/sample_sensor_kit_launch:
    type: git
    url: https://github.com/autowarefoundation/sample_sensor_kit_launch.git
    version: 03decbd31bb954eb9f52daaf3a3fa2b921dbb0c3
  universe/autoware.universe:
    type: git
    url: https://github.com/autowarefoundation/autoware.universe.git
    version: febbc135b8e09e993ed345ee6d3cd7e65b6c1d68
  universe/external/morai_msgs:
    type: git
    url: https://github.com/MORAI-Autonomous/MORAI-ROS2_morai_msgs.git
    version: 6fd6a711e4bbf8a9989b54028e8074acabbbce6f
  universe/external/muSSP:
    type: git
    url: https://github.com/tier4/muSSP.git
    version: c79e98fd5e658f4f90c06d93472faa977bc873b9
  universe/external/ndt_omp:
    type: git
    url: https://github.com/tier4/ndt_omp.git
    version: f59e1667390fe66d72c5c3aa0b25385b5b6dd8cf
  universe/external/pointcloud_to_laserscan:
    type: git
    url: https://github.com/tier4/pointcloud_to_laserscan.git
    version: 948a4fca35dcb03c6c8fbfa610a686f7c919fe0b
  universe/external/tier4_ad_api_adaptor:
    type: git
    url: https://github.com/tier4/tier4_ad_api_adaptor.git
    version: 5084f9c8eaf03458a216060798da2b1e4fa96f28
  universe/external/tier4_autoware_msgs:
    type: git
    url: https://github.com/tier4/tier4_autoware_msgs.git
    version: a360ee9f5235a0d426427813f26e43027e32139d
  vehicle/external/pacmod_interface:
    type: git
    url: https://github.com/tier4/pacmod_interface.git
    version: b5ae20345f2551da0c6e4140a3dc3479d64efd1f
  vehicle/sample_vehicle_launch:
    type: git
    url: https://github.com/autowarefoundation/sample_vehicle_launch.git
    version: 157238ca77de7b0a59f71a0b28f456741fab3ca2
  v2x/autowarev2x:
    type: git
    url: https://github.com/tlab-wide/AutowareV2X.git
    version: 48a1f2d3db6ae59e92febb93aad7cde760f4f3ec
  v2x/vanetza:
    type: git
    url: https://github.com/yuasabe/vanetza.git
    version: cfffe9afda177297c59bbb804d3e8f66120c8453
```

```shell
# Use vcstool to import more repositories
mkdir src
vcs import src < autoware.repos

# Apply patch to fix build errors (this will be updated properly in the future)
cd src/universe/autoware.universe
git apply <(curl https://github.com/diogotavc/autoware.universe/commit/a33574b0373c66250f30af4fd7d59ab1cc30b7d8.patch)
cd -

# Install dependent ROS packages
source /opt/ros/humble/setup.bash
rosdep update
# seems rosdep tries to pull Vanetza as an external dependency, dunno why -- it's compiled later
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys "Vanetza"

# Build the workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```