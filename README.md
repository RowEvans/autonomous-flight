# Autonomous Flight — PX4 + ROS 2 Offboard Control

## Background

This project is built on top of a simulated autonomous fixed-wing aircraft mission, originally proposed as an undergraduate research project by a PhD researcher at Georgia Tech. This project's focus is to build a drone to provide detailed inspection of points of interest. The framing and phased structure of the mission below come from that proposal; everything in the "My Project" section past that point is my own implementation and progress.

To understand what this project actually does, it helps to know the two main pieces of software involved:

**PX4** is open-source flight controller firmware. It is essentially the "brain" that flies the aircraft at a low level. It keeps the quadcopter stable, manages the throttle, reads sensor data, and enforces safety limits. In this project, PX4 runs in simulation (via a physics engine called Gazebo) rather than on real hardware, so there's no real drone, seaplane, or outdoor space required. PX4 still behaves exactly as it would on a real quadcopter.

**ROS 2 (Robot Operating System 2)** is a framework for writing robotics software, used widely in both academic research and industry. It lets separate programs talk to each other by publishing and subscribing to named data streams called "topics." In this project, my code is a ROS 2 node, a self-contained program that publishes flight commands (like "climb to this altitude" or "fly to this position") and subscribes to telemetry data coming back from PX4 (like current position and flight status).

Together, PX4 handles _how_ the aircraft flies, and my ROS 2 node decides _where_ it goes, sending it continuous position commands the way a human pilot might via remote control, except entirely in code.

**Q Ground Control** is a ground station app. While the main simulation runs, it provides a live map connected to the simulation and shows you current position, airspeed and other aircraft telemetry. It is never edited by these programs, but I have used it to help debugging.

**MicroXRCE-DDS Agent** is the translator between PX4 and ROS2. PX4's messaging system runs through uORB and ROS2 has its own separate messaging system called DDS. XRCE-DDS runs as a background process and bridges the two. uORB messages into DDS topics, and DDS topics into uORB messages, these message types are defined by _px4_msgs_.

## Scope

The full mission this project is built toward has five stages:

1. **Environment setup** — getting PX4, Gazebo, ROS 2, and the supporting tools installed and talking to each other.
2. **Arm and loiter** — get the aircraft armed, in offboard mode, and holding a stable circular pattern.
3. **Automated takeoff** — climb to a target cruise altitude automatically, confirmed by real telemetry rather than a timer.
4. **GPS circle mission** — orbit a user-specified GPS coordinate at a configurable radius.
5. **Return and land** — fly back to the takeoff point and land automatically within a close tolerance.

**Where I am right now:** I'm currently working on stage 3: getting the drone capable of automatic takeoff to a predetermined altitude, and reporting once it reaches that altitude.

## My Project

The core of what I've built so far is a single ROS 2 node (`multicopter_offboard.py`) that controls a simulated Cessna in PX4.

Right now, the node:

- After a short startup period, sends the commands needed to switch the aircraft into offboard mode and arm it.
- Sends continuous position setpoints — this is required by PX4's safety design, which expects a steady stream of commands at least a few times per second, or it assumes something has gone wrong and takes back control.
- Commands the aircraft to climb toward a target altitude (50m) and then hold a circular orbit of radius 15m at that altitude, to "track marine life" except it's just a simulation

The orbit's radius and turn rate are exposed as adjustable parameters, so I can tune the flight pattern without changing code.

### Setup

This runs inside a ROS 2 workspace, alongside the PX4 message definitions package (`px4_msgs`) it depends on. With PX4 running in simulation and the ROS 2 bridge active, the workspace is built with `colcon build` and the node is launched with `ros2 run`. From there, the aircraft's behavior can be watched live either in the Gazebo simulation window or in QGroundControl, a ground station application that shows the aircraft's position and status on a map in real time.

## Status

This project is in its early stages and is being actively developed. The current priority is getting automated takeoff logic before moving on GPS-based waypoint generation and the retuern-and-land sequence.

## Personal Reflection

This project has been a challenge. I came into it with basically no knowledge of PX4 or ROS2, which I quickly learned are leading pieces of software in controls systems. It's taught me a lot about how real software development works, like breaking a big goal into small stages I can actually check instead of trying to get everything working at once. Debugging has probably been the biggest jump for me. When the simulation doesn't behave how I expect, the bug usually isn't obvious, so I've had to get comfortable reading logs, testing one thing at a time, and not assuming I already know where the problem is. Right now I understand the core architecture pretty well, like how PX4 and ROS2 split up responsibilities and why offboard mode works the way it does, but I'm still building intuition for the harder parts ahead.
