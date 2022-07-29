import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv('samplefile.txt', names=['t', 'motor','stretch'], header=0, index_col=0)
df.index = df.index - df.index[0]
# plt.ion()
ax = df.plot()

plt.savefig('t vs both.png', format='png')

plt.close('all')
plt.plot(df['motor'], df['stretch'])
plt.xlabel('Motor Position (steps)')
plt.ylabel('Stretch %')
plt.savefig('position vs stretch.png', format='png')