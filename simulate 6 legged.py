import numpy as np
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import time
import math
import ctrnn

nn_size = 10
input_size = 4
output_size = 12
dt = 1/60
duration = 50
steps = int(duration/dt)
WeightRange = 5.0
BiasRange = 5.0
TimeConstMin = 0.5
TimeConstMax = 5.0

leg_joints = [0,1,2,3,4,5,6,7,8,9,10,11]

best_ind = np.load("best_robot2_2.npy")

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("robot2.urdf")
pyrosim.Prepare_To_Simulate(robotId)

nn = ctrnn.CTRNN(nn_size, input_size, output_size)
nn.setParameters(best_ind, WeightRange, BiasRange, TimeConstMin, TimeConstMax)
nn.initializeState(np.zeros(nn_size))

for step in range(steps):
    t = step*dt
    inputs = np.array([
        math.sin(2*math.pi*0.5*t),
        math.cos(2*math.pi*0.5*t),
        math.sin(2*math.pi*1.0*t),
        math.cos(2*math.pi*1.0*t),
    ])
    nn.step(dt, inputs)
    motor_output = nn.out()
    target_angles = motor_output-0.5

    for j, angle in zip(leg_joints, target_angles):
        p.setJointMotorControl2(robotId, j, p.POSITION_CONTROL, targetPosition=angle, force=100)

    p.stepSimulation()
    time.sleep(dt)
