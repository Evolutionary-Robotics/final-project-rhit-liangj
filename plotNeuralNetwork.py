import numpy as np
import matplotlib.pyplot as plt
import ctrnn

nnsize = 10
motor_outputs = 2
dt = 0.01
duration = 30
dur = np.arange(0.0, duration, dt)

WeightRange = 10.0
BiasRange = 10.0
TimeConstMin = 1.0
TimeConstMax = 5.0

best_genotype = np.load("best_robot3_2.npy")
nn = ctrnn.CTRNN(nnsize, 0, motor_outputs)
nn.setParameters(best_genotype, WeightRange, BiasRange, TimeConstMin, TimeConstMax)
nn.initializeState(np.zeros(nnsize))

output = np.zeros((len(dur), nnsize))
motorout = np.zeros((len(dur), motor_outputs))

for k, t in enumerate(dur):
    nn.step(dt, [])
    output[k] = nn.Output
    motorout[k] = nn.out()

#plot
plt.figure(figsize=(10, 6))
plt.plot(dur, output, alpha=0.4)
plt.plot(dur, motorout.T[0], 'k')
plt.plot(dur, motorout.T[1], 'k--')
plt.xlabel("Time (s)")
plt.ylabel("Output")
plt.title("Neural Activity of Evolved CTRNN")
plt.legend()
plt.show()