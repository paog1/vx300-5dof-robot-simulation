# VX300 5-DOF Robot Manipulation Simulation

A Python-based robotics project simulating a **VX300 5-DOF robotic arm** performing a cube-transfer task. The implementation combines task-space trajectory generation, quadratic-programming (QP) inverse kinematics, and joint-space PID control in RobotDART.

## Project overview

The objective is to move a cube from an initial position to a target position using a simulated robotic manipulator. The control pipeline generates end-effector waypoints, computes joint configurations for those waypoints, and tracks the resulting joint targets.

### Main components

- **RobotDART simulation** — robotic arm, gripper, and scene setup.
- **Task-space trajectory generation** — waypoints for lifting, transferring, and lowering the cube.
- **QP-based inverse kinematics** — computes joint configurations for desired end-effector poses while respecting joint-position limits.
- **Joint-space PID control** — tracks calculated joint configurations.
- **Configurable task positions** — source and destination positions can be adjusted in the entry-point configuration.

## Workflow

1. Initialize the VX300 robot and simulation scene.
2. Set the initial and destination cube poses.
3. Generate intermediate end-effector waypoints.
4. Solve inverse kinematics for each waypoint using a quadratic programming formulation.
5. Track the joint targets with a PID controller.
6. Run the simulation and observe the cube-transfer motion.

## Repository structure

```text
vx300-5dof-robot-simulation/
├── main.py
├── controller.py
├── inverse_kinematics.py
├── simulation.py
├── task_level_trajectory.py
├── requirements.txt
├── LICENSE
├── .gitignore
└── README.md
```

## Requirements

- Python 3.10 or a compatible version (verify against your local environment).
- NumPy
- RobotDART Python bindings
- ProxSuite (`proxsuite`)
- Pinocchio (`pin`)

Some robotics dependencies have platform-specific installation requirements. Follow the official installation instructions for RobotDART, Pinocchio, and ProxSuite for your operating system. Installing the Python packages listed in `requirements.txt` may not be sufficient by itself to install RobotDART.

## Run the simulation

After installing and configuring the dependencies, run the entry point from the repository root:

```bash
python main.py
```

The program is expected to launch a RobotDART simulation window and run the cube-transfer task. If an import fails, make sure the dependencies are installed in the same Python environment used to execute the script.

## Configuration

The example task positions are defined in `main.py`. Review the selected start and target poses, joint limits, and controller settings before changing the task.

## Authors

This project was developed collaboratively by:

- **[Georgios Stasinos](https://github.com/paog1)**
- **[Vasileios Chouliaras](https://github.com/ChouliarasVasileios)**

Both contributors are credited as co-authors. Add a more detailed breakdown of individual responsibilities here if you and your collaborator would like to document who worked on each component.

## Acknowledgments

We acknowledge the developers and maintainers of RobotDART, Pinocchio, and ProxSuite for the tools used in this project.

## License

This project is licensed under the MIT License. See the [`LICENSE`](LICENSE) file for details.
