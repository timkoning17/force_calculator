import math
import pandas as pd
import matplotlib.pyplot as plt


class Robot:
    def __init__(
        self,
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
    ):
        self.l_swingarm = l_swingarm
        self.phi_deg = phi_deg
        self.F_magnet_front = F_magnet_front
        self.F_magnet_rear = F_magnet_rear
        self.F_thrust = F_thrust
        self.wheel_base_x = wheel_base_x
        self.wheel_base_y = wheel_base_y
        self.w_side_plate = w_side_plate
        self.d_tube_wheel = d_tube_wheel
        self.d_tube_swing = d_tube_swing
        self.d_wheel_act = d_wheel_act
        self.d_rear_magnet_wheel = d_rear_magnet_wheel

    def compute_wheel_normal_forces(self):
        phi = self.phi_deg * 2 * math.pi / 360
        l1 = self.l_swingarm * math.cos(phi) - self.d_tube_swing
        l2 = self.l_swingarm * math.sin(phi)
        l3 = self.d_wheel_act + self.d_tube_wheel

        Fact = (l1 / l3) * self.F_thrust
        Fl = (
            self.F_thrust * (l2 + 0.5 * self.w_side_plate)
            - Fact * 0.5 * self.w_side_plate
        ) / self.w_side_plate
        Fr = (
            self.F_thrust * (l2 - 0.5 * self.w_side_plate)
            + Fact * 0.5 * self.w_side_plate
        ) / self.w_side_plate
        Fl_rear = round(
            (
                0.5 * self.F_magnet_rear * self.d_rear_magnet_wheel
                - Fl * self.d_tube_wheel
                + 0.5 * Fact * self.d_wheel_act
            )
            / self.wheel_base_x
        )
        Fr_rear = round(
            (
                0.5 * self.F_magnet_rear * self.d_rear_magnet_wheel
                + Fr * self.d_tube_wheel
                + 0.5 * Fact * self.d_wheel_act
            )
            / self.wheel_base_x
        )
        Fl_front = round(
            0.5 * (self.F_magnet_front + self.F_magnet_rear - Fact) - Fl_rear - Fl
        )
        Fr_front = round(
            0.5 * (self.F_magnet_front + self.F_magnet_rear - Fact) - Fr_rear + Fr
        )

        return Fl_rear, Fr_rear, Fl_front, Fr_front

    def forces_swing_angle(self, plotting):
        Fl_front_df = pd.DataFrame()
        Fr_front_df = pd.DataFrame()
        Fl_rear_df = pd.DataFrame()
        Fr_rear_df = pd.DataFrame()

        Fl_front_vec = []
        Fr_front_vec = []
        Fl_rear_vec = []
        Fr_rear_vec = []
        deg_vec = []

        for j in range(180):
            self.phi_deg = j - 90

            Fl_rear, Fr_rear, Fl_front, Fr_front = self.compute_wheel_normal_forces()
            Fl_front_vec.append(Fl_front)
            Fr_front_vec.append(Fr_front)
            Fl_rear_vec.append(Fl_rear)
            Fr_rear_vec.append(Fr_rear)
            deg_vec.append(self.phi_deg)

        Fl_front_df["angle"] = deg_vec
        Fl_front_df["force"] = Fl_front_vec

        Fr_front_df["angle"] = deg_vec
        Fr_front_df["force"] = Fr_front_vec

        Fl_rear_df["angle"] = deg_vec
        Fl_rear_df["force"] = Fl_rear_vec

        Fr_rear_df["angle"] = deg_vec
        Fr_rear_df["force"] = Fr_rear_vec

        if plotting:
            fig, axs = plt.subplots(2, 2)
            x = Fl_front_df["angle"]
            y_lf = Fl_front_df["force"]
            y_rf = Fr_front_df["force"]
            y_lr = Fl_rear_df["force"]
            y_rr = Fr_rear_df["force"]
            axs[0, 0].plot(x, y_lf, color="red")
            axs[0, 0].set_title("Front left")
            axs[0, 0].set_xlabel("swing arm angle [deg]")
            axs[0, 0].set_ylabel("Normal force [N]")

            axs[0, 1].plot(x, y_rf, color="red")
            axs[0, 1].set_title("Front right")
            axs[0, 1].set_xlabel("swing arm angle [deg]")
            axs[0, 1].set_ylabel("Normal force [N]")

            axs[1, 0].plot(x, y_lr, color="red")
            axs[1, 0].set_title("Rear left")
            axs[1, 0].set_xlabel("swing arm angle [deg]")
            axs[1, 0].set_ylabel("Normal force [N]")

            axs[1, 1].plot(x, y_rr, color="red")
            axs[1, 1].set_title("Rear right")
            axs[1, 1].set_xlabel("swing arm angle [deg]")
            axs[1, 1].set_ylabel("Normal force [N]")

    def assess_parameter(self, parameter_name, nr_steps, value_range, plotting):
        Fl_front_df = pd.DataFrame()
        Fr_front_df = pd.DataFrame()
        Fl_rear_df = pd.DataFrame()
        Fr_rear_df = pd.DataFrame()
        parameter_list = []
        Fl_front_vec = []
        Fr_front_vec = []
        Fl_rear_vec = []
        Fr_rear_vec = []
        deg_vec = []

        parameter = getattr(self, parameter_name)
        parameter_start_value = parameter
        step = value_range * 2 / nr_steps

        for i in range(nr_steps):
            parameter = parameter_start_value - value_range + step * i

            setattr(self, parameter_name, parameter)
            # self.d_rear_magnet_wheel = (
            #     self.wheel_base_x
            # )  # comment this line if you dont want the rear magnet to move with wheelbase
            for j in range(180):
                self.phi_deg = j - 90

                Fl_rear, Fr_rear, Fl_front, Fr_front = (
                    self.compute_wheel_normal_forces()
                )

                parameter_list.append(parameter)

                Fl_front_vec.append(Fl_front)
                Fr_front_vec.append(Fr_front)
                Fl_rear_vec.append(Fl_rear)
                Fr_rear_vec.append(Fr_rear)
                deg_vec.append(self.phi_deg)

        Fl_front_df["{}".format(parameter)] = parameter_list
        Fl_front_df["angle"] = deg_vec
        Fl_front_df["force"] = Fl_front_vec

        Fr_front_df["{}".format(parameter)] = parameter_list
        Fr_front_df["angle"] = deg_vec
        Fr_front_df["force"] = Fr_front_vec

        Fl_rear_df["{}".format(parameter)] = parameter_list
        Fl_rear_df["angle"] = deg_vec
        Fl_rear_df["force"] = Fl_rear_vec

        Fr_rear_df["{}".format(parameter)] = parameter_list
        Fr_rear_df["angle"] = deg_vec
        Fr_rear_df["force"] = Fr_rear_vec

        if plotting:
            ### Plotting
            fig, axs = plt.subplots(2, 2)
            fig.suptitle("Parameter: {}".format(parameter_name))
            for i in range(nr_steps):
                x = Fl_front_df[
                    Fl_front_df["{}".format(parameter)]
                    == parameter_start_value - value_range + step * i
                ]["angle"]
                y_lf = Fl_front_df[
                    Fl_front_df["{}".format(parameter)]
                    == parameter_start_value - value_range + step * i
                ]["force"]
                y_rf = Fr_front_df[
                    Fl_front_df["{}".format(parameter)]
                    == parameter_start_value - value_range + step * i
                ]["force"]
                y_lr = Fl_rear_df[
                    Fl_front_df["{}".format(parameter)]
                    == parameter_start_value - value_range + step * i
                ]["force"]
                y_rr = Fr_rear_df[
                    Fl_front_df["{}".format(parameter)]
                    == parameter_start_value - value_range + step * i
                ]["force"]

                # Subplot 1
                axs[0, 0].plot(
                    x,
                    y_lf,
                    color="red",
                    alpha=i / 20,
                    label="{}".format(parameter_start_value - value_range + step * i),
                )
                axs[0, 0].set_title("Front left wheel")
                axs[0, 0].legend()
                axs[0, 0].set_xlabel("swing arm angle [deg]")
                axs[0, 0].set_ylabel("Normal force [N]")
                # Subplot 2
                axs[0, 1].plot(
                    x,
                    y_rf,
                    color="red",
                    alpha=i / 20,
                )
                axs[0, 1].set_title("Front right wheel")
                axs[0, 1].set_xlabel("swing arm angle [deg]")
                axs[0, 1].set_ylabel("Normal force [N]")

                # Subplot 3
                axs[1, 0].plot(
                    x,
                    y_lr,
                    color="red",
                    alpha=i / 20,
                )
                axs[1, 0].set_title("Rear left wheel")
                axs[1, 0].set_xlabel("swing arm angle [deg]")
                axs[1, 0].set_ylabel("Normal force [N]")

                # Subplot4
                axs[1, 1].plot(
                    x,
                    y_rr,
                    color="red",
                    alpha=i / 20,
                )
                axs[1, 1].set_title("Rear right wheel")
                axs[1, 1].set_xlabel("swing arm angle [deg]")
                axs[1, 1].set_ylabel("Normal force [N]")
