    # mycobot_client
    
    The `mycobot_client` package enables users to connect and control the MyCobot 280 Pi via ROS (Robot Operating System). This guide will walk you through the installation and setup process.
    
    ## Prerequisites
    
    Before installing `mycobot_client`, you need to have Mamba installed on your system. Mamba is a fast, flexible, and user-friendly package manager. If you don't have Mamba installed, follow the installation instructions available [here](https://mamba.readthedocs.io/en/latest/installation.html).
    
    ## Installation
    
    Once Mamba is installed, you can proceed with setting up `mycobot_client`. Here are the steps you need to follow:
    
    ### Step 1: Clone the Repository
    
    Clone this repository to your local machine using the following command:
    
    ```
    git clone https://github.com/yourusername/mycobot_client.git
    cd mycobot_client
    ```
    
    ### Step 2: Create Mamba Environment
    
    Use the `environment.yaml` file provided in the repository to create a Mamba environment. This file contains all the necessary dependencies for `mycobot_client`.
    
    ```
    mamba env create -f environment.yaml
    ```
    
    ### Step 3: Activate the Environment
    
    Activate the newly created environment with Mamba:
    
    ```
    conda activate mycobot_env
    ```
    
    ### Step 4: Create a ROS Workspace
    
    Create a ROS workspace (if you do not already have one) and navigate into it:
    
    ```
    mkdir -p ~/mycobot_ws/src
    cd ~/mycobot_ws/src
    ```
    
    Clone any necessary ROS packages here, and then return to the workspace root:
    
    ```
    cd ~/mycobot_ws
    ```
    
    ### Step 5: Build the Workspace
    
    With the Mamba environment active, build the workspace using `catkin`:
    
    ```
    catkin build
    ```
    
    ### Step 6: Source the ROS Environment
    
    Source the setup file to configure your shell:
    
    ```
    source ./devel/setup.bash
    ```
    
    ### Step 7: Set Environmental Variables
    
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


