# Quick Start
1. Make sure that you have installed ROS2 and the compiling tool `colcon` successfully. Also note that Long-Term-Supported (LTS) versions are more recommended considering the stability, e.g. Foxy Fitzroy for Ubuntu 20
2. `mkdir -p ~/bea_ws2/src/ && cd ~/bea_ws2/src/`
3. `git clone https://github.com/BEASensors/bea_flatscan2.git`
4. `cd ~/bea_ws2/ && colcon build`
5. `source install/setup.bash`
6. `ros2 launch bea_sensors flatscan_launch.py`
