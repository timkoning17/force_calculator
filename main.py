import math
from lib import *
import pandas as pd
import matplotlib.pyplot as plt

# Variables
l_swingarm = 1034  # mm
phi_deg = 40  # degree
F_magnet_front = 6600  # N
F_magnet_rear = 1800  # N
F_thrust = 350  # N
wheel_base_x = 451  # fwd = 451, bwd = 355 mm
wheel_base_y = 337
w_side_plate = 250
d_tube_wheel = 220
d_tube_swing = 125
d_wheel_act = 120
d_rear_magnet_wheel = 403

R1 = Robot(
    l_swingarm,
    phi_deg,
    F_magnet_front,
    F_magnet_rear,
    F_thrust,
    wheel_base_x,
    wheel_base_y,
    w_side_plate,
    d_tube_wheel,
    d_tube_swing,
    d_wheel_act,
    d_rear_magnet_wheel,
)

R2 = Robot(
    l_swingarm,
    phi_deg,
    F_magnet_front,
    F_magnet_rear,
    F_thrust,
    wheel_base_x,
    wheel_base_y,
    250,
    d_tube_wheel,
    d_tube_swing,
    d_wheel_act,
    d_rear_magnet_wheel,
)

R3 = Robot(
    l_swingarm,
    phi_deg,
    F_magnet_front,
    F_magnet_rear,
    F_thrust,
    wheel_base_x,
    wheel_base_y,
    450,
    d_tube_wheel,
    d_tube_swing,
    d_wheel_act,
    d_rear_magnet_wheel,
)


R1.assess_parameter(
    parameter_name="wheel_base_x", nr_steps=20, value_range=200, plotting=True
)


# R2.forces_swing_angle(True)
# R3.forces_swing_angle(True)
# R.assess_width(False)
# R.assess_tube_wheel(True, parameter=R.wbx)

plt.show()
