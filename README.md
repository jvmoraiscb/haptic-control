# RHCR - Remote Haptic Control Robot

RHCR is a scientific initiation project carried out by [NTA's Laboratory](https://nta.ufes.br/) at [UFES](https://www.ufes.br/).

Our goal is to promote a user-friendly robot controller using virtual reality elements, creating an environment where the user can see, feel and control a robot even if they are not in the same place.

## 1. Components

### 1.1. Hardware Components:

-   #### Wheeltec Ackermann Robot

    ![ackermann](/Documentation/images/ackermann.jpg)

    -   using specific ROS 2 packages provided by [Wheeltec](https://wheeltec.net).

-   #### Meta Quest 2

    ![quest2](/Documentation/images/quest2.jpg)

    -   using [Oculus](https://www.meta.com/quest/setup/) software provided by [Meta](https://www.meta.com).

-   #### Novint Falcon

    ![falcon](/Documentation/images/falcon.jpg)

    -   using [libnifalcon](https://github.com/libnifalcon/libnifalcon) library.

### 1.2. Software Components:

-   #### Unity

    ![unity-project](/Documentation/images/unity-project.png)

    -   using [XR](https://docs.unity3d.com/Manual/XR.html) and [ROS-TCP-Connector](https://github.com/RobotecAI/ros2-for-unity) packages.

-   #### ROS 2

    ![ros-terminal](/Documentation/images/ros-terminal.png)

    -   using [ros2-falcon](https://github.com/jvmoraiscb/ros2-falcon), [navigation2](https://github.com/ros-planning/navigation2), [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) and [ros-tcp-endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) packages.

## 2. Installing RHCR:

This project needs two operating systems, a Linux machine to run ROS2 and connect to Novint Falcon, and a Windows machine to run Unity and connect to Quest2.

Below is a step-by-step tutorial on how to prepare each environment:

### 2.1. Preparing the Linux environment:

_Recommended version:_ **_Ubuntu 22.04_**

#### 2.1.1. Configuring network adapters (virtual machines only):

-   Set network adapter to Bridged mode and **ONLY** turn on the **adapter that the robot is connected to**:

    ![vmware-network](/Documentation/images/vmware-network.jpg)

#### 2.1.2. Installing ROS and its dependencies:

-   Install ROS 2 Humble using the [official documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) or the script below:

```bash
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop -y
```

-   Install Colcon and Rosdep:

```bash
sudo apt install python3-colcon-common-extensions -y
sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update
```

#### 2.1.3 Creating and populating the rhcr workspace:

-   Create a workspace and clone the **ros-packages** there:

```bash
mkdir -p ~/rhcr_ws/src
cd ~/rhcr_ws/src
git clone -b ros-packages --recurse-submodules https://github.com/jvmoraiscb/rhcr.git .
```

-   Install ros2-falcon (libnifalcon) drivers:

```bash
cd ~/rhcr_ws/src/ros2-falcon/libnifalcon
./install.sh
```

-   Install packages depencies and build the colcon workspace:

```bash
cd ~/rhcr_ws
rosdep install --from-paths src -y --ignore-src
colcon build
```

### 2.2. Preparing the Windows environment:

_Recommended version:_ **_Windows 11_**

#### 2.2.1. Install Git:

-   Visit [git-scm.com/downloads](https://git-scm.com/downloads), download **64-bit Git for Windows Setup** and install it.
-   Restart the computer.

#### 2.2.2. Install Oculus Software:

-   Visit [meta.com/quest/setup](https://www.meta.com/quest/setup/), scroll down to the Quest 2 section, click on **Download Software** and install it.
-   Sign in or create a Meta Account and set up your Quest 2.
-   Go to Settings -> General and enable **Unknown Sources** and **OpenXR Runtime**.
    ![quest2-settings](/Documentation/images/quest2-config.jpg)

#### 2.2.3. Install Unity:

-   Visit [unity.com/download](https://unity.com/download), download and install **Unity Hub**.
-   Visit [unity.com/releases/editor/archive](https://unity.com/releases/editor/archive) and find **Unity 2020.3.29** version, then click the Unity Hub button and proceed to install the editor.
-   Choose a folder and clone the **unity-project** there:

```bash
git clone -b unity-project https://github.com/jvmoraiscb/rhcr.git
```

-   Add the project to Unity Hub.

## 3. Running RHCR:

_Before running the project, it's a good idea to check that both computers are "seeing" each other (just ping them)._

### 3.1. Running in the Linux environment:

-   Disconnect **Novint Falcon** from host and connect to virtual machine (virtual machines only):
    ![falcon-vmware](/Documentation/images/falcon-vmware.png)

-   Open a terminal, source rhcr workspace, set a domain id and run ros2-falcon (follow the terminal instructions to calibrate the controller):

```bash
source ~/rhcr_ws/install/setup.bash
ROS_DOMAIN_ID=42 ros2 run ros2-falcon main
```

-   Open another terminal, source rhcr workspace, set a domain id and launch rhcr:

```bash
source ~/rhcr_ws/install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch rhcr start.launch.py
```

### 3.2. Running in the Windows environment:

#### 3.2.1. Configuring the network:

-   Disable Windows Firewall:

    ![windows-firewall](/Documentation/images/windows-firewall.jpg)

-   Go to Windows Advanced network settings and disable **all adapters that are not connected to the same network as the robot** (same way as in virtual machine):

    ![windows-network](/Documentation/images/windows-network.jpg)

#### 3.2.2. Connecting to the robot and launching its default package:

-   Connect to the same network as the robot:

    ![windows-wifi](/Documentation/images/windows-wifi.jpg)

-   Open a PowerShell terminal and connect to robot through ssh protocol:

    ![wheeltec-ssh](/Documentation/images/wheeltec-ssh.jpg)

-   Set a domain id and launch the robot's default package:

```bash
ROS_DOMAIN_ID=42 ros2 launch <robot_package> <robot_package_file>
```

_Replace **<robot_package>** for the robot ros2 package name, and **<robot_package_file>** for the launch file name._

#### 3.2.3. Configuring and running the unity project:

-   Open RHCR project using Unity Hub and load Main or Mockup scene.
-   Go to **RemoteHapticControlRobot -> ROS2 -> Connector** and enter the IP address of the Linux environment in the **Ros IP Address** field.
    ![unity-config](/Documentation/images/unity-config.jpg)
