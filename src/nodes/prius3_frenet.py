#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState, ModelStates
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../PythonRobotics/PathPlanning/QuinticPolynomialsPlanner/")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../PythonRobotics/PathPlanning/CubicSpline/")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +"/../classes/ExteriorCircuit/")

try:
    from quintic_polynomials_planner import QuinticPolynomial
    import cubic_spline_planner
    from circuit import Circuit
except ImportError:
    raise


# obstacle lists

ob = np.array([
               [0.0, 0.0],
               [1.0, 0.0],
               [0.0, 1.0],
               [1.0, 1.0],
               [2.0, 0.0],
               [2.0, 1.0],
               [2.0, 2.0]
               ]) 

SIM_LOOP = 800

# Parameter
MAX_SPEED = 20.0 / 3.6  # maximum speed [m/s]
MAX_ACCEL = 0.5  # maximum acceleration [m/ss]
MAX_CURVATURE = 1.0  # maximum curvature [1/m]
MAX_ROAD_WIDTH = 10.0  # maximum road width [m]
D_ROAD_W = 2.0  # road width sampling length [m]
DT = 0.5  # time tick [s]
DT2 = 0.05
MAX_T = 8.0  # max prediction time [m]
MIN_T = 7.0  # min prediction time [m]
TARGET_SPEED = 10.0 / 3.6  # target speed [m/s]
D_T_S = 5.0 / 3.6  # target speed sampling length [m/s]
N_S_SAMPLE = 1  # sampling number of target speed
ROBOT_RADIUS = 4.0  # robot radius [m]


# cost weights
K_J = 0.1
K_T = 0.1
K_D = 1.0
K_LAT = 1.0
K_LON = 1.0

show_animation = True


class QuarticPolynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, time):
        # calc coefficient of quartic polynomial

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * time ** 2, 4 * time ** 3],
                      [6 * time, 12 * time ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt


class FrenetPath:

    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []


def calc_frenet_paths(c_speed, c_acc, c_d, c_d_d, c_d_dd, s0):
    frenet_paths = []

    # generate path to each offset goal
    for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):

        # Lateral motion planning
        for Ti in np.arange(MIN_T, MAX_T, DT):
            fp = FrenetPath()

            # lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
            lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # Longitudinal motion planning (Velocity keeping)
            for tv in np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE,
                                TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S):
                tfp = copy.deepcopy(fp)
                lon_qp = QuarticPolynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

                tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                ds = (TARGET_SPEED - tfp.s_d[-1]) ** 2

                tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
                tfp.cv = K_J * Js + K_T * Ti + K_D * ds
                tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

                frenet_paths.append(tfp)

    return frenet_paths


def calc_global_paths(fplist, csp):
    for fp in fplist:

        # calc global positions
        for i in range(len(fp.s)):
            ix, iy = csp.calc_position(fp.s[i])
            if ix is None:
                break
            i_yaw = csp.calc_yaw(fp.s[i])
            di = fp.d[i]
            fx = ix + di * math.cos(i_yaw + math.pi / 2.0)
            fy = iy + di * math.sin(i_yaw + math.pi / 2.0)
            fp.x.append(fx)
            fp.y.append(fy)

        # calc yaw and ds
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(math.atan2(dy, dx))
            fp.ds.append(math.hypot(dx, dy))

        try:

            fp.yaw.append(fp.yaw[-1])
            fp.ds.append(fp.ds[-1])

        except IndexError:
            pass

        # calc curvature
        for i in range(len(fp.yaw) - 1):
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

    return fplist


def check_collision(fp, ob):
    for i in range(len(ob[:, 0])):
        d = [((ix - ob[i, 0]) ** 2 + (iy - ob[i, 1]) ** 2)
             for (ix, iy) in zip(fp.x, fp.y)]

        collision = any([di <= ROBOT_RADIUS ** 2 for di in d])

        if collision:
            return False

    return True


def check_paths(fplist, ob):
    ok_ind = []
    for i, _ in enumerate(fplist):
        if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            continue
        elif any([abs(a) > MAX_ACCEL for a in
                  fplist[i].s_dd]):  # Max accel check
            continue
        elif any([abs(c) > MAX_CURVATURE for c in
                  fplist[i].c]):  # Max curvature check
            continue
        elif not check_collision(fplist[i], ob):
            continue

        ok_ind.append(i)

    return [fplist[i] for i in ok_ind]

    ok_ind.append(i)

    return [fplist[i] for i in ok_ind]


def frenet_optimal_planning(csp, s0, c_speed, c_acc, c_d, c_d_d, c_d_dd, ob):

    fplist = calc_frenet_paths(c_speed, c_acc, c_d, c_d_d, c_d_dd, s0)
    fplist = calc_global_paths(fplist, csp)
    fplist = check_paths(fplist, ob)

    # find minimum cost path
    min_cost = float("inf")
    best_path = None
    for fp in fplist:
        if min_cost >= fp.cf:
            min_cost = fp.cf
            best_path = fp

    return best_path


def generate_target_course(x, y):
    csp = cubic_spline_planner.Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 1.0)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp



def callback(data):


    for i in range(len(data.name)):
        if(data.name[i] == "prius"):
            prius = i
            break

    for j in range(len(data.name)):
        if(data.name[j] == "prius_hybrid"):
            prius_hybrid_0 = j
            break       


    orientation = [data.pose[prius].orientation.x, data.pose[prius].orientation.y,
                                    data.pose[prius].orientation.z, data.pose[prius].orientation.w]

    roll, pitch, yaw = euler_from_quaternion(orientation)

    x_prius = data.pose[prius].position.x - (2.0*math.sin(yaw))
    y_prius = data.pose[prius].position.y + (2.0*math.cos(yaw))

    x_prius_0 = data.pose[prius_hybrid_0].position.x - (2.0*math.sin(yaw))
    y_prius_0 = data.pose[prius_hybrid_0].position.y + (2.0*math.cos(yaw))


    x1 = x_prius + (2.5*math.cos(yaw))
    y1 = y_prius + (2.5*math.sin(yaw))

    x4 = x_prius + (5.0*math.cos(yaw))
    y4 = y_prius + (5.0*math.sin(yaw))

    x5 = x_prius + (7.5*math.cos(yaw))
    y5 = y_prius + (7.5*math.sin(yaw))

    x6 = x_prius + (10.0*math.cos(yaw))
    y6 = y_prius + (10.0*math.sin(yaw))

 

    distance = math.sqrt(math.pow(x_prius_0 - x6, 2) + math.pow(y_prius_0 - y6, 2))

    if distance <= 5.0:
        ob[4][0] = 4
        ob[4][1] = 4

        ob[5][0] = 5
        ob[5][1] = 5

        ob[6][0] = 6
        ob[6][1] = 6

        ob[0][0] = 0
        ob[0][1] = 0

        ob[1][0] = 1
        ob[1][1] = 1

        ob[2][0] = 2
        ob[2][1] = 2

        ob[3][0] = 3
        ob[3][1] = 3

    else:

        ob[4][0] = x5
        ob[4][1] = y5

        ob[5][0] = x_prius
        ob[5][1] = y_prius

        ob[6][0] = x6
        ob[6][1] = y6

        ob[0][0] = x1
        ob[0][1] = y1

        ob[3][0] = x4
        ob[3][1] = y4


def main():
    print(__file__ + " start!!")

    rospy.init_node("planificacion_2", anonymous=False)
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    sub = rospy.Subscriber('gazebo/model_states', ModelStates, callback)
    rate = rospy.Rate(1.0/DT2)

    nodo = ModelState()
    nodo.model_name = "prius_hybrid_2"

    circuit = Circuit()
    circuit.run()


    # way points
    wx = []
    wy = []
    tx = []
    ty = []

     
    for i in range(len(circuit.points)):
        wx.append(circuit.points[i][0])
        wy.append(circuit.points[i][1])


    print(len(wx))
    print(len(wy))


    tx, ty, tyaw, tc, csp = generate_target_course(wx, wy)

    print(len(tx))
    print(len(ty))


    # initial state
    c_speed = 1.0  # current speed [m/s]
    c_acc = 0.0
    c_d = 0.0  # current lateral position [m]
    c_d_d = 0.0  # current lateral speed [m/s]
    c_d_dd = 0.0  # current lateral acceleration [m/s]
    s0 = 273.0*3  # current course position

    area = 20.0  # animation area length [m]


    
    for i in range(SIM_LOOP):
    

        path = None

        while path==None: 
            path = frenet_optimal_planning(
                csp, s0, c_speed, c_acc, c_d, c_d_d, c_d_dd, ob)


        s0 = path.s[1]
        c_d = path.d[1]
        c_d_d = path.d_d[1]
        c_d_dd = path.d_dd[1]
        c_speed = path.s_d[1]
        c_acc = path.s_dd[1]


        try:
            if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0:
                break
        except:
            break

        
        
        try:

            X =  path.x[4] - path.x[3]
            Y = path.y[4] - path.y[3]
            angle = (math.atan2(Y,X) + 1.57)

            nodo.pose.position.x = path.x[4] + (2.5*math.sin(angle - 1.57))
            nodo.pose.position.y = path.y[4] - (2.5*math.sin(angle - 1.57))
            nodo.pose.position.z = 0.111641

            quaternion = quaternion_from_euler(0.0, 0.0, angle)

            nodo.pose.orientation.x = quaternion[0]
            nodo.pose.orientation.y = quaternion[1]
            nodo.pose.orientation.z = quaternion[2]
            nodo.pose.orientation.w = quaternion[3]

        except:
            break

    
        pub.publish(nodo)
        rate.sleep()


    while not rospy.is_shutdown():

        # initial state
        c_speed = 1.0  # current speed [m/s]
        c_acc = 0.0
        c_d = 0.0  # current lateral position [m]
        c_d_d = 0.0  # current lateral speed [m/s]
        c_d_dd = 0.0  # current lateral acceleration [m/s]
        s0 = 0.0  # current course position

        area = 20.0  # animation area length [m]


        
        for i in range(SIM_LOOP): 

            path = None

            while path==None: 
                path = frenet_optimal_planning(
                    csp, s0, c_speed, c_acc, c_d, c_d_d, c_d_dd, ob)


            s0 = path.s[1]
            c_d = path.d[1]
            c_d_d = path.d_d[1]
            c_d_dd = path.d_dd[1]
            c_speed = path.s_d[1]
            c_acc = path.s_dd[1]


            try:
                if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0:
                    break
            except:
                break

            
            
            try:

                X =  path.x[4] - path.x[3]
                Y = path.y[4] - path.y[3]
                angle = (math.atan2(Y,X) + 1.57)

                nodo.pose.position.x = path.x[4] + (2.5*math.sin(angle - 1.57))
                nodo.pose.position.y = path.y[4] - (2.5*math.sin(angle - 1.57))
                nodo.pose.position.z = 0.111641

                quaternion = quaternion_from_euler(0.0, 0.0, angle)

                nodo.pose.orientation.x = quaternion[0]
                nodo.pose.orientation.y = quaternion[1]
                nodo.pose.orientation.z = quaternion[2]
                nodo.pose.orientation.w = quaternion[3]

            except:
                break

        
            pub.publish(nodo)
            rate.sleep()

            """            
            if show_animation:  # pragma: no cover
                plt.cla()
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                plt.plot(tx, ty)

                
                plt.plot(ob[:, 0], ob[:, 1], "xk")
                plt.plot(path.x[1:], path.y[1:], "-or")
                plt.plot(path.x[1], path.y[1], "vc")
                #plt.xlim(path.x[1] - area, path.x[1] + area)
                #plt.ylim(path.y[1] - area, path.y[1] + area)
                plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4])
                plt.grid(True)
                plt.pause(0.0001)
            """
                       

if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass