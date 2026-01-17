"""
Docstring for simulations.pick_and_place
"""

# Import libraries
import pybullet as p
import pybullet_data
import time
import math

# Start PyBullet
# Start the simulation in GUI mode
p.connect(p.GUI) # start PyBullet with a graphical interface (window)
# p.connect(p.DIRECT) # start PyBullet without a graphical interface (for faster computation
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # set the path to PyBullet's data
p.setGravity(0, 0, -9.81) # apply earth gravity donwards negative z-axis (without this, objects will float, shifted slightly upwards)

# Environment
p.loadURDF("plane.urdf")
p.loadURDF("table/table.urdf", [0.5, 0, 0])

# Load the robot 
robot = p.loadURDF(
    "franka_panda/panda.urdf", # path to URDF file (loads the Franka Emika Panda robot model)
    [0, 0, 0],                 # base position (x, y, z)
    useFixedBase=True           # fix the base of the robot to the world (prevents it from falling due to gravity)
)

# Load the object to be picked (placce a small cube on top of the table, roughly in front of the robot)
cube = p.loadURDF( 
    "cube_small.urdf",
    [0.6, 0, 0.65]
)

# Let simulation settle (allows the cuve to fall onto the table naturally, removes any initial jitter)
for _ in range(240):
    p.stepSimulation()
    time.sleep(1/240) # sleep to match real-time (240 steps per second) -> 240 steps ≈ 1 second of simulation.

# End-effector index
ee_index = 11 # Define the end-effector link index for the Franka Panda robot (define the robot hand, 11 = panda's gripper center)
target_orn = p.getQuaternionFromEuler([math.pi, 0, 0]) # rotates the gripper so it points downwards

# Move above cube (no contact yet)
target_pos = [0.6, 0, 0.85] # position above the cube
# Calculate inverse kinematics for the target position and orientation: 
# “Given where I want the hand to be, compute the joint angles that achieve it.”
joint_angles = p.calculateInverseKinematics(
    robot, ee_index, target_pos, target_orn
)

# Move the robot joints to the calculated angles
for i in range(7):
    p.setJointMotorControl2(
        robot, i, p.POSITION_CONTROL, joint_angles[i], force=200
    )

# Step simulation to move the robot
for _ in range(240):
    p.stepSimulation()
    time.sleep(1/240)

# Close gripper
p.setJointMotorControl2(robot, 9, p.POSITION_CONTROL, 0.0, force=50) # left finger
p.setJointMotorControl2(robot, 10, p.POSITION_CONTROL, 0.0, force=50) # right finger

# Step simulation to close gripper
for _ in range(120):
    p.stepSimulation() # step the simulation forward
    time.sleep(1/240) # sleep to match real-time

# Lift object
lift_pos = [0.6, 0, 1.0] # position above the cube
# Calculate inverse kinematics for the lift position, new joint angles for lifted hand
lift_angles = p.calculateInverseKinematics(
    robot, ee_index, lift_pos, target_orn
)

# Move the robot joints to the new lift angles
for i in range(7):
    p.setJointMotorControl2(
        robot, i, p.POSITION_CONTROL, lift_angles[i], force=200
    )

# Step simulation to lift the object
for _ in range(240):
    p.stepSimulation()
    time.sleep(1/240)
