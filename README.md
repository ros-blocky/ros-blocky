<p align="center">
  <img src="build/icon.png" alt="ROS Blocky Logo" width="150" height="150">
</p>

<h1 align="center">ROS Blocky</h1>

<p align="center">
  <strong>Visual block-based programming for ROS2 robotics</strong>
</p>

<p align="center">
  <a href="../../releases/latest">
    <img src="https://img.shields.io/badge/version-0.1.1--alpha-blue.svg" alt="Version">
  </a>
  <a href="LICENSE">
    <img src="https://img.shields.io/badge/license-MIT-green.svg" alt="License">
  </a>
  <img src="https://img.shields.io/badge/platform-Windows-lightgrey.svg" alt="Platform">
  <img src="https://img.shields.io/badge/ROS2-Jazzy-orange.svg" alt="ROS2 Support">
</p>

---

**ROS Blocky** is a desktop application that makes ROS2 robotics development accessible through visual block-based programming. Built with Electron and Google Blockly, it allows you to create ROS2 nodes and URDF robot descriptions without writing code.

## ✨ Features

### 🧩 Visual Block Programming
- **Node Editor** — Create ROS2 nodes using drag-and-drop blocks
  - Publishers, Subscribers, Timers, and Services
  - Log messages with different severity levels
  - Math operations and control flow blocks
  - Auto-generates Python code from blocks

- **URDF Editor** — Build robot descriptions visually
  - Define links with geometry (box, cylinder, sphere)
  - Configure joints (fixed, revolute, prismatic, continuous)
  - Set inertial properties and visual materials
  - Live preview with RViz2 integration

### 📦 Project Management
- Create and manage ROS2 workspace projects
- Automatic package structure generation (`setup.py`, `package.xml`)
- Package explorer with organized file tree
- Recent projects for quick access

### 🚀 Build & Run
- One-click build with `colcon build`
- Run nodes directly from the editor
- Launch URDF in `robot_state_publisher`
- Integrated RViz2 visualization
- Real-time log panel with process output

### 🌍 Multilingual Support
- English, French, and Arabic languages
- RTL (Right-to-Left) support for Arabic

---

## 🖥️ Installation

### Prerequisites
- **Windows 10/11** (x64)

> **Note:** ROS2 Jazzy will be installed automatically on first launch.

### Download
👉 **[Download the latest release](https://github.com/ros-blocky/ros-blocky/releases)**

1. Download `ROS-Blocky-Setup.exe`
2. Run the installer

> **Note:** The application is not code-signed yet. Windows may show a SmartScreen warning.
> Click **More info** → **Run anyway** to proceed.

---

## 🚀 Quick Start

Check out our **[video tutorials](https://ros-blocky.github.io/)** to get started quickly!

---

## 📁 Project Structure

ROS Blocky creates a standard ROS2 workspace:

```
my_ros_project/
├── src/
│   └── my_package/
│       ├── my_package/
│       │   ├── __init__.py
│       │   ├── my_node.py          # Generated Python code
│       │   └── my_node.nodeblocks  # Block workspace (internal)
│       ├── urdf/
│       │   ├── robot.urdf          # Generated URDF
│       │   └── robot.blocks        # Block workspace (internal)
│       ├── launch/
│       ├── package.xml
│       └── setup.py
├── build/                          # Build artifacts
├── install/                        # Installed packages
└── log/                            # Build logs
```

---

## 🛠️ Development

### Tech Stack

| Technology | Version |
|------------|--------|
| Node.js | 20+ |
| Electron | 39.0.0 |
| electron-builder | 26.0.12 |
| Blockly | 10.4.0 |
| i18next | 25.7.2 |

### Building from Source

```bash
# Clone the repository
git clone https://github.com/ros-blocky/ros-blocky.git
cd ros-blocky

# Install dependencies
npm install

# Run in development mode
npm start

# Build for Windows
npm run build
```

---

## 📋 Roadmap

- [ ] macOS and Linux support
- [ ] Custom block creation
- [ ] ROS2 service clients
- [ ] Action servers/clients
- [ ] Simulation integration (Gazebo)
- [ ] Code signing for Windows

---

## 🤝 Contributing

Contributions are welcome! Please feel free to:
- Open an issue for bugs or feature requests
- Submit a pull request with improvements
- Share feedback on your experience

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 💬 Contact

- **Author:** Yacin Hamdi
- **Issues:** [GitHub Issues](../../issues)

---

<p align="center">
  Made with ❤️ for the ROS community
</p>
