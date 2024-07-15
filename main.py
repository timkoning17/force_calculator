import math
from lib import *

import pandas as pd
import matplotlib.pyplot as plt

# Variables
l_swingarm = 1034  # mm
phi_deg = 40  # degree
F_magnet_front = 6000  # N
F_magnet_rear = 1800  # N
F_thrust = 350  # N
wheel_base_x = 451  # fwd = 451, bwd = 355 mm
wheel_base_y = 337
w_side_plate = 250
d_tube_wheel = 220
d_tube_swing = 125
d_wheel_act = 120
d_rear_magnet_wheel = 403
mu = 0.48
dload_com = 0.35

R = Robot(
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
    mu,
    dload_com,
)


R.assess_parameter(
    parameter_name="d_tube_wheel", nr_steps=10, value_range=100, plotting=True
)

plt.show()

# from kivy.app import App
# from kivy.uix.gridlayout import GridLayout
# from kivy.uix.label import Label
# from kivy.uix.image import Image
# from kivy.uix.button import Button
# from kivy.uix.textinput import TextInput


# class SayHello(App):
#     def build(self):
#         self.window = GridLayout()
#         self.window.cols = 2

#         # add widgets to window
#         self.text1 = Label(text="force calculator")
#         self.window.add_widget(self.text1)

#         self.leftfront = Label(text="")
#         self.window.add_widget(self.leftfront)
#         self.leftrear = Label(text="")
#         self.window.add_widget(self.leftrear)
#         self.rightfront = Label(text="")
#         self.window.add_widget(self.rightfront)
#         self.rightrear = Label(text="")
#         self.window.add_widget(self.rightrear)

#         self.button = Button(text="Run simulator")
#         self.button.bind(on_press=self.callback)
#         self.window.add_widget(self.button)

#         return self.window

#     def callback(self, instance):
#         # Fl_rear, Fr_rear, Fl_front, Fr_front
#         self.leftrear_int, self.rightrear_int, self.leftfront_int, self.leftrear_int = (
#             R1.compute_wheel_normal_forces()
#         )
#         self.leftfront.text = str(self.leftfront_int)
#         self.leftrear.text = str(self.leftrear_int)
#         self.rightfront.text = str(self.rightfront_int)
#         self.rightrear.text = str(self.rightrear_int)


# if __name__ == "__main__":
#     SayHello().run()

# R2.forces_swing_angle(True)
# R3.forces_swing_angle(True)
# R.assess_width(False)
# R.assess_tube_wheel(True, parameter=R.wbx)
