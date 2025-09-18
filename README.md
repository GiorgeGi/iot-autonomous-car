# IoT Autonomous Car

A miniature autonomous car project combining IoT, robotics, and edge computing.  
The system integrates sensors, an ESP32, Raspberry Pi, ROS 2 (micro-ROS), and MQTT,  
with remote monitoring through a lightweight web dashboard.

---

## Table of Contents

- [Introduction](#introduction)  
- [Features](#features)  
- [Architecture](#architecture)  
- [Repository Structure](#repository-structure)  
- [Getting Started](#getting-started)  
- [Networking & Remote Access](#networking--remote-access)  
- [MQTT Broker Setup](#mqtt-broker-setup)  
- [Web Dashboard](#web-dashboard)  
- [Raspberry Pi as Access Point](#raspberry-pi-as-access-point)  
- [ROS & Docker](#ros--docker)  
- [License](#license)  
- [Contributing](#contributing)  
- [Contact](#contact)

---

## Introduction

The IoT Autonomous Car demonstrates a functional prototype of an autonomous vehicle.  
It combines edge computing and IoT to provide real-time sensing, decision making, and control.  
The project includes both autonomous navigation and manual control via a web interface.

---

## Features

- Autonomous navigation with obstacle avoidance  
- Continuous telemetry via MQTT  
- Local broker and lightweight web dashboard  
- Remote access for configuration and monitoring  
- ROS 2 integration for sensor fusion and higher-level tasks  

---

## Architecture

1. **Sensors & Hardware** – ESP32, IMU, LiDAR (optional), motor drivers.  
2. **Communication Layer** – MQTT broker for telemetry and control.  
3. **ROS 2 Nodes** – Sensor processing, decision making, and actuation.  
4. **Web Dashboard** – Browser-based visualization of live data.  
5. **Raspberry Pi** – Acts as an access point and local host for ROS / broker.  

---

## Repository Structure

```
iot-autonomous-car/
├── config/                   # Configuration files (network, sensors, etc.)
├── diagrams/                 # Architecture and system diagrams
├── esp_broker_local_website/ # Local web interface and broker
├── ros2_nodes/               # ROS 2 nodes for sensing and control
├── LICENSE
├── README.md
```

---

## Getting Started

### Prerequisites

- Raspberry Pi or equivalent SBC  
- ESP32 microcontroller  
- Sensors (IMU, ultrasonic/LiDAR, etc.)  
- ROS 2 (Humble or later)  
- Mosquitto MQTT broker  
- Python 3 (for local HTTP server)  
- Docker (optional, for containerized ROS)  

### Installation Overview

1. Clone the repository.  
2. Install dependencies for ROS 2 nodes.  
3. Install and configure Mosquitto broker.  
4. Configure Raspberry Pi networking (AP mode).  
5. Launch web dashboard locally for monitoring.  

---

## Networking & Remote Access

- Find host IP address:

  ```bash
  ifconfig
  ```

- Scan subnet for devices:

  ```bash
  sudo nmap -sP x.x.x.0/24
  ```

- Connect to Raspberry Pi via SSH:

  ```bash
  ssh username@x.x.x.x
  ```

---

## MQTT Broker Setup

- Install Mosquitto and clients:

  ```bash
  sudo apt-get install mosquitto mosquitto-clients -y
  ```

- Configure `/etc/mosquitto/mosquitto.conf` with:
  - Persistence enabled  
  - Log file enabled  
  - Listener on `1883` (MQTT) and `8083` (WebSockets)  
  - Anonymous access enabled (for local testing)  

- Start and enable service:

  ```bash
  sudo systemctl start mosquitto
  sudo systemctl enable mosquitto
  ```

- Verify broker is running:

  ```bash
  sudo netstat -plnt | grep 1883
  ```

- Test with publish/subscribe commands (`mosquitto_pub` and `mosquitto_sub`).  

For secure deployment:  
[How to set up a Mosquitto MQTT broker securely](https://medium.com/gravio-edge-iot-platform/how-to-set-up-a-mosquitto-mqtt-broker-securely-using-client-certificates-82b2aaaef9c8)

---

## Web Dashboard

- Serve dashboard files locally:

  ```bash
  python3 -m http.server 8000
  ```

- Access via browser:

  ```
  http://<raspberry-ip>:8000/
  ```

- Dashboard connects to MQTT broker over WebSockets.  

---

## Raspberry Pi as Access Point

Two approaches can be used: **Netplan** or **NetworkManager**.

- **Netplan YAML**: define SSID, password, and AP mode in `/etc/netplan/`.  
- **NetworkManager Keyfile**: create `.nmconnection` file under  
  `/etc/NetworkManager/system-connections/` with `mode=ap` and WPA2 security.  
- Alternatively, configure with `nmcli` commands to create and activate AP profile.  

**Default setup**  
- SSID: `MovingFortress`  
- Password: `1234567890`  

---

## ROS & Docker

- Run ROS 2 inside Docker with host networking and GPU access:

  ```bash
  sudo docker run --privileged --net=host \
    --env="DISPLAY" \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    --device=/dev/dri:/dev/dri \
    -it osrf/ros:humble-desktop-full /bin/bash
  ```

- Open additional shells:

  ```bash
  sudo docker exec -it <container_id> /bin/bash
  ```

- ROS nodes in Docker share the same network as the Raspberry Pi.  

---

## License

This project is licensed under the **GNU General Public License v3.0 or later**.  

---

## Contributing

Contributions are welcome!  
- Fork the repo and submit pull requests.  
- Document configuration changes.  
- Add diagrams or tests where possible.  

---

## Contact

**Author**: Giorgos Varvarigos  
**GitHub**: [GiorgeGi](https://github.com/GiorgeGi)  

---
