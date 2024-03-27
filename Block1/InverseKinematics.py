import sympy as sym
from sympy.abc import x, y, z, a, b, d, e, f

def s(x):
    return sym.sin(x)

def c(x):
    return sym.cos(x)



tab_0 = sym.Matrix(
    [[0, -s(a), -c(a), 0],
    [0, c(a), -s(a), 0],
    [1, 0, 0, 45],
    [0, 0, 0, 1]]
)

t0_1 = sym.Matrix(
    [[0, s(b), c(b), 20],
    [0,  c(b), -s(b), 0],
    [-1, 0, 0, 0],
    [0, 0, 0, 1 ]]
)

t2_1 = sym.Matrix(
    [[1, 0, 0, 0],
    [0, c(d), -s(d), 0],
    [0, s(d), c(d), 95],
    [0, 0, 0, 1 ]]
)

t3_2 = sym.Matrix(
    [[1, 0, 0, 0],
    [0, c(e), -s(e), 0],
    [0, s(e), c(e), 105],
    [0, 0, 0, 1 ]]
)

tee_3 = sym.Matrix([
    [1, 0, 0, 75],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]]
)
sym.print_latex(sym.simplify(tab_0*t0_1*t2_1*t3_2*tee_3))


# l0 = 0.19 # Distance from ground to the driven acces of J1
# l1 = 1.132 # Length of the driven side of T1 from axle center to axle center
# l2 = 0.14 # Distance between the driven side of T1 axle on J2 and the driven axle of T2 side of J2
# l3 = 1.132 # Length of the driven side of T2 from axle center to axle center
# dx = 0.09163 # The distance between the axle center of the driven T2 side of J3 and the front of the faraday cage in the local x-direction (horizontal forward positive)
# dy = -(0.10607 + 0.08673) # The distance between the axle center of the driven T2 side of J3 and the top of the faraday cage in the local y-direction (vertical downward positive)

t0 = sym.Symbol('t0')
t1 = sym.Symbol('t1')
t2 = sym.Symbol('t2')
t3 = sym.Symbol('t3')
t4 = sym.Symbol('t4')
tee = sym.Symbol('tee')

res = sym.solve(
    [95*s(t0)*s(t1) + 105*s(t0)*s(t1+t2)+75*c(t0) - x,
     75*s(t0) - 95*s(t1)*c(t0) - 105*s(t1+t2)*c(t0) - y,
     95*c(t1) + 105*c(t1+t2) + 65 - z
     ],
    [t0,
     t1,
     t2]
)
print(res)
# res = sym.solve(
#     [a + b - x, b - d - y],
#     [x, y]
# )

# print(res)
#sym.print_latex(sym.simplify(res))