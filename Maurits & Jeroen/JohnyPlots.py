from JohnyKinematics import Johny
import matplotlib.pyplot as plt
from math import pi


rb  = Johny()
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.set_xlim(-0.3, 0.3)
ax.set_xlabel("x [m]")

ax.set_ylim(-0.3, 0.3)
ax.set_ylabel("y [m]")

ax.set_zlim(0, 0.6)
ax.set_zlabel("z [m]")

# Plot single points
for name, location in rb.sweep_over_ang0().items():
    ax.scatter3D(location["x"], location["y"], location["z"], label=name, alpha=.5, s=3)

# Sweep over J0
# for name, location in rb.sweep_over_all().items():
#     ax.scatter3D(location["x"], location["y"], location["z"], label=name, alpha=0.3, s=5)

plt.legend()
plt.show()