from scipy import interpolate
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
import pandas as pd

# x = np.arange(-5.01, 5.01, 0.25)
y = np.arange(-5.01, 5.01, 0.25)
x = np.arange(-5.01, 5.01, 1)
# y = np.arange(-5.01, 5.01, 1)

xx, yy = np.meshgrid(x, y)
z = np.sin(xx**2+yy**2)
f = interpolate.interp2d(x, y, z, kind='cubic')

# # 2d plot show 
# xnew = np.arange(-5.01, 5.01, 1e-2)
# ynew = np.arange(-5.01, 5.01, 1e-2)
# znew = f(xnew, ynew)
# plt.plot(x, z[0, :], 'ro-', xnew, znew[0, :], 'b-')
# plt.show()

# # 3dplot show
# fig1 = plt.figure()
# ax = fig1.add_subplot(111, projection='3d')
# # dots
# # ax.scatter3D(axes[0], axes[1],dots[1], c=dots[1].flatten(), cmap='Reds')
# # function
# ax.plot_surface(xx, yy, z, rstride=1, cstride=1,
#                 cmap='viridis', edgecolor='none')

# Plot using `.trisurf()`:
x = xx.reshape(1681)
y = yy.reshape(1681)
z = z.reshape(1681)
df = pd.DataFrame({'x': x, 'y': y, 'z': z}, index=range(len(x)))
fig1 = plt.figure()
ax = fig1.add_subplot(111, projection='3d')
ax.plot_trisurf(df.x, df.y, df.z, cmap=cm.jet, linewidth=0.2)


plt.show()
