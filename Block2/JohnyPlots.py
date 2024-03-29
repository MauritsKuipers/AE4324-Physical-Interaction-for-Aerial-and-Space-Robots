from JohnyKinematics import Johny
import matplotlib.pyplot as plt
from math import pi, sqrt
import numpy as np


rb  = Johny()
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.set_xlim(-0.3, 0.3)
ax.set_xlabel("x [m]")

ax.set_ylim(-0.3, 0.3)
ax.set_ylabel("y [m]")

ax.set_zlim(0, 0.3)
ax.set_zlabel("z [m]")

rb.ang0 = 0
rb.ang1 = 0
rb.ang2 = pi/2
rb.ang3 = 0
# print(rb.get_angles())
# rb.manual_ik(0.2, 0.2, 0.0, np.deg2rad(45))
print("---\nInitial angles")
# print(np.rad2deg(rb.get_angles()))
# Plot single points

for name, location in rb.symbolic_fk_test().items():
    print(name, location)
    ax.scatter3D(location["x"], location["y"], location["z"], 
                 label=name + "_sym", alpha=1, s=50)

# location_EE = rb.symbolic_fk_test()['EE']
# location_J3 = rb.symbolic_fk_test()['J3']
# l3ee = 0.075
# l3ee_fk = sqrt(sum((location_J3[dim] - location_EE[dim])**2 for dim in ["x", "y", "z"])) - l3ee
#
# print("---\nFK l3ee error pos:\n",l3ee_fk)
# rb.manual_ik(location_EE["x"], location_EE["y"], location_EE["z"], np.deg2rad(90))
# print("---\nInverse Angels")
# print(np.rad2deg(rb.get_angles()))


# for name, location in rb.symbolic_fk_test().items():
#     # print(name, location)
#     ax.scatter3D(location["x"], location["y"], location["z"],
#                  label=name + "_ik", alpha=1, s=50)

# rb.manual_ik(0.2427081528017131, 0, 0.07414213562373097, np.deg2rad(45))
# print(np.rad2deg(rb.get_angles()))

# Sweep over J0
# for name, location in rb.sweep_over_all().items():
    # ax.scatter3D(location["x"], location["y"], location["z"],
    #              label=name, alpha=0.3, s=5)

plt.legend()
plt.show()