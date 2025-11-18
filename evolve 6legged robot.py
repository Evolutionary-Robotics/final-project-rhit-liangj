import numpy as np
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import time
import math
import eas
import ctrnn

# ---------------------------------
# CTRNN / EA parameters
# ---------------------------------
nn_size = 10
input_size = 4
output_size = 12  # 6 legs × 2 joints each
dt = 1/60
duration = 30
steps = int(duration / dt)

WeightRange = 5.0
BiasRange = 5.0
TimeConstMin = 0.5
TimeConstMax = 5.0

# compute genome size for CTRNN
genesize = nn_size*nn_size + input_size*nn_size + nn_size*output_size + nn_size + nn_size

# Joint indices (0–11 for 12 joints)
leg_joints = list(range(12))

# ---------------------------------
# FITNESS FUNCTION
# ---------------------------------
def fitnessFunction(genotype):
    # connect physics engine (headless mode)
    physicsClient = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    planeId = p.loadURDF("plane.urdf")

    # load the 6-legged robot
    robotId = p.loadURDF("robot2.urdf")
    pyrosim.Prepare_To_Simulate(robotId)
    p.changeDynamics(planeId, -1, lateralFriction=1)

    # create and initialize CTRNN
    nn = ctrnn.CTRNN(nn_size, input_size, output_size)
    nn.setParameters(genotype, WeightRange, BiasRange, TimeConstMin, TimeConstMax)
    nn.initializeState(np.zeros(nn_size))

    # get start position
    start_pos = p.getBasePositionAndOrientation(robotId)[0]

    # main simulation loop
    for step in range(steps):
        t = step * dt
        # simple rhythmic sensory input
        inputs = np.array([
            math.sin(2*math.pi*0.5*t),
            math.cos(2*math.pi*0.5*t),
            math.sin(2*math.pi*1.0*t),
            math.cos(2*math.pi*1.0*t),
        ])

        # update neural network
        nn.step(dt, inputs)
        motor_output = nn.out()
        target_angles = (motor_output - 0.5) * 1.0

        # set joint targets
        for j, angle in zip(leg_joints, target_angles):
            p.setJointMotorControl2(robotId, j, p.POSITION_CONTROL,
                                    targetPosition=angle, force=100)
        p.stepSimulation()

    # compute distance traveled (x-axis)
    end_pos = p.getBasePositionAndOrientation(robotId)[0]
    distance = end_pos[0] - start_pos[0]

    # disconnect from physics
    p.disconnect()

    return distance

# ---------------------------------
# MICROBRIAL EVOLUTIONARY ALGORITHM
# ---------------------------------
popsize = 10
recombProb = 0.5
mutatProb = 0.05
demeSize = 4
generations = 100

ga = eas.Microbial(fitnessFunction, popsize, genesize, recombProb, mutatProb, demeSize, generations)
ga.run()
ga.showFitness()

# Get best evolved network
af, bf, bi = ga.fitStats()

# Save best genotype
np.save("best_robot2_6.npy", bi)

