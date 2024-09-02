
# mycobot_client
    
The `mycobot_client` package enables users to connect and control the MyCobot 280 Pi via ROS (Robot Operating System). This guide will walk you through the installation and setup process.
    
## Prerequisites
You will need to use a computer with Ubuntu 20.04 installed, either via a VM (https://www.virtualbox.org/), Windows Subsystem for Linux (https://learn.microsoft.com/en-us/windows/wsl/install), or Docker (https://docs.docker.com/desktop/install/windows-install/).  

Before installing `mycobot_client`, you need to have Mamba installed on your system. Mamba is a fast, flexible, and user-friendly package manager. If you don't have Mamba installed, follow the installation instructions available [here](https://github.com/conda-forge/miniforge).  

Ensure you add the relevant directories to your path variables. This may be an installation option during install. If you don't, you will type a `mamba` command and it will not find the executable. You can alternatively run `mamba` commands passing the full path to the executable.
    
## Installation
    
Once Mamba is installed, note you may need to restart your terminal. From there, you can proceed with setting up `mycobot_client`. Here are the steps you need to follow:

### Step 1: Create the Mamba Environment (python)
Download this repository and create a python environment using the instructions in the repo. https://github.com/VModugno/RoboEnv

For example:
```
git clone https://github.com/VModugno/RoboEnv
cd RoboEnv
mamba env create -f environment.yaml
```

Activate the newly created environment with Mamba:
    
```
conda activate mycobot_env
```

If you see errors relating to the below, note we do not support windows for all the environments we use. You will need to use Linux, ideally Ubuntu 20.04.  
```
PS D:\ziegl\git\mycobot_client\RoboEnv> mamba env create -f environment.yaml      
Channels:
 - conda-forge
 - robostack-staging
Platform: win-64
Collecting package metadata (repodata.json): done
Solving environment: failed

PackagesNotFoundError: The following packages are not available from current channels:

  - gepetto-viewer-corba
```

### Step 2: Create a ROS Workspace
    
Create a ROS workspace (if you do not already have one) and navigate into it:
    
```
mkdir -p ~/mycobot_ws/src
cd ~/mycobot_ws/src
```

Clone the mycobot client repository and any other nescessary ROS packages to this workspace:
```
git clone https://github.com/VModugno/mycobot_client
```
    
Then return to the workspace root:
    
```
cd ~/mycobot_ws
```
    
### Step 3: Build the Workspace
    
With the Mamba environment active, build the workspace using `catkin`:
    
```
catkin build
```
    
### Step 4: Source the ROS Environment
    
Source the setup file to configure your shell:
    
```
source ./devel/setup.bash
```
    
### Step 5: Set Environmental Variables
    
Finally, set the necessary environment variables to establish a connection between your computer and the ROS master running on the MyCobot 280 Pi. Replace `YOUR_ROBOT_IP` with the IP address of the MyCobot 280 Pi and `YOUR_COMPUTER_IP` with the IP address of the computer where ROS master is hosted.
    
```
export ROS_MASTER_URI=http://YOUR_ROBOT_IP:11311
export ROS_IP=YOUR_COMPUTER_IP
```
    
## Usage
    
After completing the setup, you are ready to start sending commands to your MyCobot 280 Pi through ROS. Ensure that all connections and settings are correct and that the ROS master on your robot is running properly.
    
For more detailed examples and usage instructions, refer to the `examples` directory in this repository.
    
## Support
    
If you encounter any issues or have questions, please file an issue on the GitHub repository issue tracker.
    
## License
    
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.


