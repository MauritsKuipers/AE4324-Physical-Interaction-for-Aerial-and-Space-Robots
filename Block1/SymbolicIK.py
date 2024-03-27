import sympy as sym
from math import pi

# Define robot symbols
# Joint angles
t0 = sym.Symbol('t0')
t1 = sym.Symbol('t1')
t2 = sym.Symbol('t2')
t3 = sym.Symbol('t3')
# Link lengts and displacements
lb0 = sym.Symbol('lab0')
l01 = sym.Symbol('l01')
l12  = sym.Symbol('l12')
l23  = sym.Symbol('l23')
l3ee = sym.Symbol('l3ee')
# Cartesian coordinates
x = sym.Symbol('x')
y = sym.Symbol('y')
z = sym.Symbol('z')

# Symbolic cos and sin functions
def s(a):
    return sym.sin(a)

def c(a):
    return sym.cos(a)

tab_0 = sym.Matrix([[0, -s(t0), -c(t0), 0],
                    [0, c(t0), -s(t0), 0],
                    [1, 0, 0, lb0],
                    [0, 0, 0, 1]
                   ])
print("# Transform joint 0 to arm base")
print("tab_0 = ", tab_0)
t0_1 = sym.Matrix([
    [0, s(t1), c(t1), l01],
    [0, c(t1), -s(t1), 0],
    [-1, 0, 0, 0],
    [0, 0, 0, 1]
    ])
print("# Transform joint 1 to joint 0")
print("t0_1 = ", t0_1)

t2_1 = sym.Matrix([
    [1, 0, 0, 0],
    [0, c(t2), -s(t2), 0],
    [0, s(t2), c(t2), l12],
    [0, 0, 0, 1]
    ])
print("# Transform joint 2 to joint 1")
print("t2_1 = ", t2_1)
t3_2 = sym.Matrix([
    [1, 0, 0, 0],
    [0, c(t3), -s(t3), 0],
    [0, s(t3), c(t3), l23],
    [0, 0, 0, 1]
])
print("# Transform joint 3 to joint 2")
print("t3_2 = ", t3_2)
tee_3 = sym.Matrix([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, l3ee],
    [0, 0, 0, 1]
])
print("# Transform EE to joint 3")
print("tee_3 = ", tee_3)


# Calculate the homogenous matrixes for all frames and print the x,y,z eq
# EE
trans_to_ground = sym.simplify(tab_0*t0_1*t2_1*t3_2*tee_3)
print("#---\n# EE position in ground frame:")
print("x =", trans_to_ground[0,3])
print("y =", trans_to_ground[1,3])
print("z =", trans_to_ground[2,3])
print('joints["EE"] = {"x": x, "y": y, "z": z}')
# J3
#trans_to_ground = sym.simplify(tab_0*t0_1*t2_1*t3_2)
print("#---\n# J3 position in ground frame:")
print("x =", trans_to_ground[0,3])
print("y =", trans_to_ground[1,3])
print("z =", trans_to_ground[2,3])
print('joints["J3"] = {"x": x, "y": y, "z": z}')
# J2
#trans_to_ground = sym.simplify(tab_0*t0_1*t2_1)
print("#---\n# J2 position in ground frame:")
print("x =", trans_to_ground[0,3])
print("y =", trans_to_ground[1,3])
print("z =", trans_to_ground[2,3])
print('joints["J2"] = {"x": x, "y": y, "z": z}')
# J1
#trans_to_ground = sym.simplify(tab_0*t0_1)
print("#---\n# J1 position in ground frame:")
print("x =", trans_to_ground[0,3])
print("y =", trans_to_ground[1,3])
print("z =", trans_to_ground[2,3])
print('joints["J1"] = {"x": x, "y": y, "z": z}')
# J0
#trans_to_ground = sym.simplify(tab_0)
print("#---\n# J0 position in ground frame:")
print("x =", trans_to_ground[0,3])
print("y =", trans_to_ground[1,3])
print("z =", trans_to_ground[2,3])
print('joints["J0"] = {"x": x, "y": y, "z": z}')
# End
print("#---")



# print('Transform matrix EE to G:')
sym.print_latex(trans_to_ground)

# x,y,z equations
# x_sym = trans_to_ground[0:3,3]


# print(x_sym)
