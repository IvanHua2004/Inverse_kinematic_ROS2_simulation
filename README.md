# 🤖 Kaiju IK - Inverse Kinematics with ROS2 and Turtlesim

## 📋 Description

**Kaiju IK** is an educational project demonstrating **Inverse Kinematics (IK)** using **ROS2** and **Turtlesim**. The project implements a 3-degree-of-freedom robotic arm that follows a target point using the **Jacobian pseudo-inverse method**.

### What is Inverse Kinematics?

Inverse Kinematics is the mathematical process of calculating the joint angles needed to position the end-effector (the "claw") at a desired location. Instead of manually specifying each joint angle, you simply give the target position, and the IK algorithm computes the necessary angles.

## 🎯 Features

- **3-DOF Robotic Arm** : Shoulder, Elbow, Wrist, and Claw
- **Jacobian Pseudo-Inverse IK** : Iterative numerical solution
- **Real-time Visualization** : Using Turtlesim turtles as arm segments
- **Smooth Interpolation** : Ease-in-out movement between positions
- **Interactive Target** : Randomly moving target point

## 🏗️ Arm Structure


### Turtles Assignment

| Turtle | Role | Description |
|--------|------|-------------|
| `turtle1` | Claw | End-effector (follows target) |
| `turtle2` | Elbow | Middle joint |
| `turtle5` | Wrist | Intermediate joint |
| `turtle4` | Target | Moving target point |

## 🔧 How It Works

### Forward Kinematics

Given joint angles (θ₁, θ₂, θ₃), calculates the position of the claw:


Where:
- **J** = Jacobian matrix (relates joint velocities to end-effector velocity)
- **J⁺** = Pseudo-inverse of J
- **α** = Step gain (learning rate)

## 📦 Installation

### Prerequisites

# Install ROS2 (Humble or later)
# Install Turtlesim
sudo apt install ros-humble-turtlesim

Steps to launch:
- colcon build --packages-select kaiju_ik --symlink-install
- source /opt/ros/kilted/setup.bash
- ros2 launch kaiju_ik kaiju_ik.launch.py
