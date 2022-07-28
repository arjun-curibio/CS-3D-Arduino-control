import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv('samplefile.txt', names=['t', 'motor','stretch'], header=0, index_col=0)

# plt.ion()
ax = df.plot()

plt.savefig('t vs both.png', format='png')