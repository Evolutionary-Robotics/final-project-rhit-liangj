import numpy as np
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import time
import math
import ctrnn

nn_size = 10
input_size = 4
output_size = 8
dt = 1/60
duration = 50
steps = int(duration/dt)
WeightRange = 5.0
BiasRange = 5.0
TimeConstMin = 0.5
TimeConstMax = 5.0

leg_joints = [0,1,2,3,4,5,6,7]

best_ind = np.load("best_robot5.npy")

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("robot3.urdf")
pyrosim.Prepare_To_Simulate(robotId)

nn = ctrnn.CTRNN(nn_size, input_size, output_size)
nn.setParameters(best_ind, WeightRange, BiasRange, TimeConstMin, TimeConstMax)
nn.initializeState(np.zeros(nn_size))

for step in range(steps):
    p.stepSimulation()
    time.sleep(dt)