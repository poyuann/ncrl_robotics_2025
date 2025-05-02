# ncrl_robotics_2025
[Gazebo and PX4-Autopilot Tutorial](https://hackmd.io/@poyuann/rJNX75skxl)
## Setup

Creat your workspace
```cmd=
mkdir -p {your_workspace}/src
cd {your_workspace}
catkin_make
```
Clone the package
```cmd=
cd ~/{your_workspace}/src
git clone https://github.com/poyuann/ncrl_robotics_2025.git
```

make
```=cmd
catkin_make
```

Add the cmd line in your .bashrc file

```
source ~/{your_workspace}/devel/setup.bash
```

### Vision-based Map Localization Set up
Create a new python3 virtual environment (optional)
```cmd=
cd
python3 -m venv env 

source ~/env/bin/activate
```

Install python dependencies

```cmd=
roscd map_featurematcher/src

git submodule update --init --recursive
cd superglue_lib
```
```cmd=
pip3 install -r requirements.txt
```

## run the code 

world environment
```cmd=
roslaunch final_2025 start_sim.launch
```
offboard_node
```cmd=
roscd final_2025/scripts
python3 offb_node.py
```
Vision-based map localization
```
source ~/env/bin/activate

roslaunch map_featurematcher vml.launch
```
