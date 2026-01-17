# Robot Manipulation Playground

A lightweight robotics simulation lab for visualizing and testing robot pick-and-place tasks in physics-based simulators.

This repository currently focuses on **scripted robot control and visualization** using PyBullet. It serves as a clean, minimal foundation for future extensions to imitation learning and reinforcement learning.

---

## Features

* Robot pick-and-place simulation
* Physics-based environment using PyBullet
* Simple, readable scripted control
* Visual demonstration of robot kinematics and grasping
* Easy to extend to learning-based methods

---

## Getting Started

### 1. Create a virtual environment

```bash
python -m venv .venv
```

### 2. Activate the virtual environment

**Windows (PowerShell):**

```bash
.venv\Scripts\Activate.ps1
```

*(On macOS/Linux: `source .venv/bin/activate`)*

---

### 3. Install required packages

```bash
pip install -r requirements.txt
```

---

### 4. Run the simulation

```bash
python simulations/pybullet/pick_and_place.py
```

A PyBullet window will open, showing a robot arm performing a scripted pick-and-place motion.

---

## Roadmap

* Support for multiple objects and object randomization
* Additional grippers and robot arms
* Alternative simulators (e.g., CoppeliaSim)
* Imitation learning from scripted demonstrations
* Reinforcement learning for robust manipulation