# 📦🤖 Package Delivery Robot (ROS2 Jazzy · TurtleBot3 · Gazebo)

A beginner-friendly ROS2 simulation project for package pickup and delivery using **TurtleBot3** in **Gazebo**.  
This project shows how to combine navigation, waypoints, and custom logic to simulate real-world delivery tasks — all in software!

---

## ✨ Features
- Uses **TurtleBot3 (Burger or Waffle)** and **Gazebo simulation** — no hardware needed  
- Navigates between **pickup** and **drop-off** waypoints  
- Simulates **package pickup/delivery** with ROS2 Python node logic  
- Uses **ROS2 Nav2 stack** for waypoint navigation and path planning  
- Supports **loop deliveries** or **manual teleoperation**  
- Visualize in **RViz** for easy monitoring  

---

## 📝 Prerequisites
- 🐧 **Ubuntu 24.04**
- ⚡ **ROS2 Jazzy**
  ```bash
  sudo apt install ros-jazzy-desktop-full
  ```
- 🟥 **Gazebo simulator**
- 🤖 **TurtleBot3 simulation packages:**
  ```bash
  sudo apt install ros-jazzy-turtlebot3* ros-jazzy-gazebo*
  ```

---

## 🚦 Quick Start

### 1️⃣ Clone this repo
```bash
git clone https://github.com/Karthikraja2345/Delivery_robot.git
cd Delivery_robot
```

### 2️⃣ Build the workspace
```bash
colcon build --symlink-install
source install/setup.bash
```

### 3️⃣ Set TurtleBot model
```bash
export TURTLEBOT3_MODEL=burger   # Or waffle/waffle_pi
```

### 4️⃣ Launch Gazebo simulation
```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### 5️⃣ Open a new terminal in your workspace and source again
```bash
source install/setup.bash
```

### 6️⃣ Start delivery node 🚚
```bash
ros2 run Delivery_robot robot
```

---

## 🗺️ How It Works
1. Robot starts at the **pickup location**
2. Drives to pickup (prints “Picked up!” or triggers simulated event)
3. Navigates to **drop-off waypoint** (prints “Delivered!”)
4. Loops as many times as you like
5. All movements visible in **Gazebo** and **RViz**

---

## 💡 Customization
- Edit Python node to set your own waypoints or logic  
- Add new waypoints, goals, or “events” via simple code tweaks  
- Connect to RViz anytime to visualize waypoints and robot state  

---

## 📚 Credits
- TurtleBot3 Manual  
- Gazebo simulation with TurtleBot3  
