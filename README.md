# Simple Autonomous Vehicle Simulation

This ROS 2 package provides two Python nodes for simulating basic autonomous vehicle behaviors in Gazebo using TurtleBot3:

* **`obstacle_stop`** – Simple stop/go behavior based on LiDAR distance.
* **`obstacle_avoid`** – Basic obstacle avoidance: moves forward when clear and turns around obstacles automatically.

---

## Requirements

* Ubuntu 22.04
* ROS 2 Humble
* TurtleBot3 packages
* Gazebo simulation

---

## Installation

1. Create a ROS 2 workspace if you don’t have one:

```bash
mkdir -p ~/autonomous_ws/src
cd ~/autonomous_ws
colcon build
source install/setup.bash
```

2. Clone or create the `simple_autonomous` package inside `src/`.

3. Install TurtleBot3 packages:

```bash
sudo apt update
sudo apt install ros-humble-turtlebot3* -y
```

4. Set TurtleBot3 model:

Open the `.bashrc` file:

```bash
nano ~/.bashrc
````

Add the following line at the end of the file:

```bash
export TURTLEBOT3_MODEL=burger
```

Save and exit (`Ctrl+O`, `Enter`, `Ctrl+X`), then apply the changes:

```bash
source ~/.bashrc
```


5. Build the workspace:

```bash
cd ~/autonomous_ws
colcon build
source install/setup.bash
```

---

## Usage

### Launch TurtleBot3 in Gazebo

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

---

### Run `obstacle_stop` node

```bash
ros2 run simple_autonomous obstacle_stop
```

* Publishes stop/go based on LiDAR.
* Moves forward if no obstacle is within 0.2 m, stops otherwise.


---

### Run `obstacle_avoid` node

```bash
ros2 run simple_autonomous obstacle_avoid
```

* Moves forward when clear.
* Turns left or right automatically to avoid obstacles.
* Provides simple autonomous navigation in Gazebo.


---


### Topics

* **Subscriber:** `/scan` – LiDAR data (`sensor_msgs/LaserScan`)
* **Publisher:** `/cmd_vel` – Robot velocity commands (`geometry_msgs/Twist`)

---

### Notes

* Adjust thresholds in the nodes for different obstacle distances.
* Can be extended with additional sensors or path-planning algorithms.

---

### Author

Natnael Takele

---

