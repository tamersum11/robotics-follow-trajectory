import numpy as np
import matplotlib.pyplot as plt

lenght_base = 0.3 #[m]
wheel_robot = 0.1 #[m]

init_pose = []
x_rota, y_rota = [], []
x_traj, y_traj = [], []

dt = 0.1
d_asterisk = 1 #[m]
error_sum = 0 #initial value for error sum 

kv = 0.5 #velocity gain
ki = 0.1 #integration gain
kh = 2 #heading gain

class FollowTrajectory:
    def __init__(self, init_pose, select):
        self.init_pose = init_pose
        self.select = select

    def plot_robot(self, x, y, theta):
        global x_rota, y_rota, x_traj, y_traj

        p1_i = np.array([0.3, 0, 1]).T
        p2_i = np.array([-0.3, 0.2, 1]).T
        p3_i = np.array([-0.3, -0.2, 1]).T

        last_pose_point = self.transformation_matrix(x, y, theta)
        
        p1_move = np.matmul(last_pose_point, p1_i)
        p2_move = np.matmul(last_pose_point, p2_i)
        p3_move = np.matmul(last_pose_point, p3_i)

        plt.plot([p1_move[0], p2_move[0]], [p1_move[1], p2_move[1]] , 'b-')
        plt.plot([p2_move[0], p3_move[0]], [p2_move[1], p3_move[1]], 'b-')
        plt.plot([p3_move[0], p1_move[0]], [p3_move[1], p1_move[1]], 'b-')

        plt.plot(x_traj, y_traj, 'g-')

        plt.plot(x_rota, y_rota, 'r--')

        plt.pause(0.1)

    def transformation_matrix(self, x, y, theta):
        return np.array([[np.cos(theta), -np.sin(theta), x], [np.sin(theta), np.cos(theta), y], [0, 0, 1]])

    def pi_controller(self, e):
        global error_sum, kv, ki, dt

        error_sum += e 
        v = kv * e + ki * error_sum * dt  #proportional-integral controller for linear velocity

        return v

    def p_controller(self, theta_asterisk):
        global kh

        w = kh * theta_asterisk #proportional controller for angular velocity

        return w

    def calculate_error_heading(self, theta_asterisk):
        e_theta = np.arctan2(np.sin(theta_asterisk - self.init_pose[2]), np.cos(theta_asterisk - self.init_pose[2])) #error heading [-pi, pi]

        return e_theta

    def diff_drive(self, v, w):
        global dt

        x_n = self.init_pose[0] + v * np.cos(self.init_pose[2]) * dt
        y_n = self.init_pose[1] + v * np.sin(self.init_pose[2]) * dt
        theta_n = self.init_pose[2] + w * dt

        return x_n, y_n, theta_n

    def follow_trajectory(self):
        global d_asterisk, dt, error_sum

        t = 0
        while t <= 20:
            x_trajectory = t
            if self.select == 1:
                y_trajectory = np.sin(x_trajectory)
            elif self.select == 2:
                y_trajectory = np.cos(x_trajectory)
            elif self.select == 3:
                y_trajectory = np.log(x_trajectory)
            elif self.select == 4:
                y_trajectory = np.exp2(x_trajectory)
            elif self.select == 5:
                y_trajectory = x_trajectory + 3
            else:
                break

            x_traj.append(x_trajectory)
            y_traj.append(y_trajectory)

            x_diff = x_trajectory - self.init_pose[0]
            y_diff = y_trajectory - self.init_pose[1]

            e = np.hypot(x_diff, y_diff) - d_asterisk
            theta_asterisk = np.arctan2(y_trajectory - self.init_pose[1], x_trajectory - self.init_pose[0])

            e_theta = self.calculate_error_heading(theta_asterisk)
            v = self.pi_controller(e)
            w = self.p_controller(e_theta)
            x_n, y_n, theta_n = self.diff_drive(v, w)

            x_rota.append(x_n)
            y_rota.append(y_n)

            self.init_pose[0] = x_n
            self.init_pose[1] = y_n
            self.init_pose[2] = theta_n

            plt.cla()
            self.plot_robot(x_n, y_n, theta_n)

            t += dt
        

if __name__ == '__main__':
    x_i = float(input("Initial x position:"))
    init_pose.append(x_i)
    y_i = float(input("Initial y position:"))
    init_pose.append(y_i)
    theta_i = float(input("Initial angular (theta):"))
    init_pose.append(theta_i)

    print("*" * 20)
    print("TRAJECTORY FORM MENU")
    print("1- SIN")
    print("2- COS")
    print("3- LOG")
    print("4- EXPONANTIAL")
    print("5- LINE")
    print("*" * 20)

    select = int(input("Select trajectory: "))

    myRobot = FollowTrajectory(init_pose, select)
    myRobot.follow_trajectory()