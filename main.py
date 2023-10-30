import roboticstoolbox as rtb
from spatialmath.base import plot_sphere
import numpy as np
import matplotlib.pyplot as plt
import swift

fig, ax = plt.subplots()

# center of sphere
p_o = np.array([-0.25, -0.25, 0.0])


# radius of sphere
r = .25

# num points
points = 3

# down vector
down = np.array([0.0, 0.0, -1.0])

robot = rtb.models.DH.Panda()
ets = robot.ets()
sol = rtb.IK_LM()

qs = []

y = 0

xs = np.linspace(-r, r, points)

for x in np.nditer(xs):
    
    # get a random point on the sphere
    z = np.abs(np.sqrt(r**2 - x**2))
    p_e = np.array([x - p_o[0], y - p_o[1], z])

    # Z column
    R_z = (p_o - p_e)
    R_z /= np.linalg.norm(R_z)

    # X column
    R_x = np.cross(down, R_z)
    R_x /= np.linalg.norm(R_x)

    # Y column
    R_y = np.cross(R_z, R_x)

    # Full transformation matrix
    T = np.array([[ R_x[0], R_y[0], R_z[0], p_e[0] ],
                    [ R_x[1], R_y[1], R_z[1], p_e[1] ],
                    [ R_x[2], R_y[2], R_z[2], p_e[2] ],
                    [      0,      0,      0,      1]])

    solution = sol.solve(ets=ets, Tep=T)

    qs.append(solution.q)

qs = np.array(qs)

tj = rtb.mstraj(qs, dt=.1, tacc=1, qdmax=1)

plot_sphere(1.0, [0,0,0], ax=ax)

robot.plot(tj.q, loop=True, dt=.01, fig=fig)


