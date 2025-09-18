# UR5e AI Agent

## Overview

This project develops an experimental natural language processing (NLP) and large language model (LLM)-based control system for the UR5e robot, leveraging NASA/JPL's ROSA framework. It aims to bridge the gap between industrial robotics and state-of-the-art AI models like LLaMA, Qwen, GPT, and others by enabling intuitive, natural language robot control. This enables robotics researchers and engineers to interact with industrial robot manipulators using conversational commands.

## Features

- Control joint movements and end-effector (TCP) motions through natural language commands.
- Retrieve feedback about the current robot state such as joint angles and Cartesian pose.
- Switch between different robot controllers dynamically.
- Integration and experimentation with multiple LLM backends for flexible AI-driven control.

## Prerequisites and Setup

- **ROS2 Humble** (tested on Ubuntu 22.04 recommended)
- Install NASA/JPL ROSA framework:
  ```bash
  git clone https://github.com/nasa-jpl/rosa.git
  cd rosa
  # Follow ROSA installation instructions
  ```
- Clone this repository inside the `src` folder of your ROS2 workspace:
  ```bash
  git clone https://github.com/Abhi-creator1/ur5e_ai_agent.git src/ur5e_ai_agent
  ```
- (Optional) Install a local AI model or configure API access by editing `llm_config.py` to select your preferred LLM backend.

- Clone necessary robot driver and controller packages into your ROS2 workspace `src`:
  ```bash
  git clone -b <branch> https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git 
  git clone https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers.git src/cartesian_controllers
  ```
- Install dependencies and build workspace:
  ```bash
  rosdep install --from-paths ./src --ignore-src -r -y
  # Remove or ignore tests and simulation packages for cartesian_controllers if conflicts occur
  colcon build
  source install/setup.bash
  ```

- Update UR driver package configuration to enable the cartesian controller plugin as per instructions in the additional_instruction.txt.

## Running the Agent

Open multiple terminals:

1. Start UR5e robot (simulation or real):

   - For simulation:
     ```bash
     ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e use_fake_hardware:=true launch_rviz:=false
     ```

   - For real robot (replace `<your_ur5e_ip>`):
     ```bash
     ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=<your_ur5e_ip> launch_rviz:=false
     ```

2. Launch MoveIt 2 for motion planning and visualization:
   ```bash
   ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
   ```

3. Launch ROSA agent services:
   ```bash
   ros2 launch ur5e_agent services.launch.py
   ```

4. Run the UR5e ROSA AI agent:
   ```bash
   ros2 run ur5e_agent ur5e_rosa_agent.py
   ```

### Example Prompts to Test

- Move to home position
- Move TCP right by 5 cm
- What are the current joint angles?
- Rotate joint 1 by 10 degrees

## Architecture

*To be added.*

## Known Limitations and Safety

- This project is in active development and experimental. Inappropriate or misinterpreted commands can cause unsafe robot motions.
- Joint limits and safety constraints are implemented in software, but exercise caution while testing.
- Always ensure a clear workspace and be ready to trigger emergency stop commands if needed.

## Contributing

Interested in advancing AI-driven industrial robotics? Contributions are welcome! Feel free to submit issues, feature requests, or pull requests.

## License

This project is licensed under the MIT License.
