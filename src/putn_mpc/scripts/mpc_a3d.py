#!/usr/bin/env python

import casadi as ca
import casadi.tools as ca_tools
import rospy
import numpy as np
import math
import time
from nav_msgs.msg import  OccupancyGrid
import math
def distance_global(c1, c2):
    return np.sqrt((c1[0] - c2[0]) * (c1[0] - c2[0]) + (c1[1] - c2[1]) * (c1[1] - c2[1]))

def MPC(self_state1,goal_state,global_path,obstacle_num,obstacle_pred,last_state,last_input,car_r,safe_dist):

        
        # self.curr_pose_lock.acquire()
        # global_path_lock.acquire()
        # self.obstacle_lock.acquire()
        self_state=np.zeros(3)
        for i in range(3):
            self_state[i]=self_state1[i]
        N=20 
        #self_state

        #goal_state
        #global_path
        #self.ob
        obstacle=[]
        map_range=6
        
        turn_tag=False

        Obtag=0
        if not  (obstacle_pred[obstacle_num-1] == []): 
            Obtag=1
        
        if Obtag:
            for i in range (obstacle_num):
                if (obstacle_pred[i][0][0]-self_state[0])*(obstacle_pred[i][0][0]-self_state[0])+ (obstacle_pred[i][0][1]-self_state[1])*(obstacle_pred[i][0][1]-self_state[1])<map_range*map_range:
                    for j in range(len(obstacle_pred[i])): 
                        ob_i=np.zeros(5)
                        ob_i[0]=  obstacle_pred[i][j][0] #x
                        ob_i[1]=  obstacle_pred[i][j][1] #y
                        ob_i[2]=  obstacle_pred[i][j][3]+car_r #a
                        ob_i[3]=  obstacle_pred[i][j][3] +car_r#b=a
                        ob_i[4]=  obstacle_pred[i][j][2] #yaw
                        ##obstacle_pred[i][j][4]  =time_stamp 

                        obstacle.append(ob_i)

        # pre_x=self_state[0]
        # pre_y=self_state[1]
        # for i in range(len(goal_state)):
        #     cur_x=goal_state[i][0]
        #     cur_y=goal_state[i][1]
        #     goal_state[i][2]=math.atan2(cur_y-pre_y,cur_x-pre_x)
        #     pre_x=cur_x
        #     pre_y=cur_y
        # if abs(goal_state[1][2]-self_state[2])>2*math.pi/3:
        #     self_state[2]=self_state[2]-math.pi
        #     #self_state[2]=self_state[2]%(2*math.pi)
        #     two_pi = 2 * self_state[2]
        #     self_state[2] = (self_state[2] + math.pi) % two_pi - math.pi
   
        #     turn_tag=True

        # self.last_input = u_res
        # self.last_state = state_res
        # self.mpc_success = True

        opti = ca.Opti()
        # parameters for optimization
        T = 0.1
        #gamma_k = 0.15
        gamma_k = 0.3

        v_max = 1.2
        v_min = 0.1
        omega_max = 1.2

        opt_x0 = opti.parameter(3)

        # state variables
        opt_states = opti.variable(N + 1, 3)
        opt_controls = opti.variable(N, 2)
        v = opt_controls[:, 0]
        omega = opt_controls[:, 1]

        def f(x_, u_): return ca.vertcat(*[u_[0] * ca.cos(x_[2]), u_[0] * ca.sin(x_[2]), u_[1]])

        def exceed_ob(ob_):
            l_long_axis = ob_[2]
            l_short_axis = ob_[3]
            long_axis = np.array([np.cos(ob_[4]) * l_long_axis, np.sin(ob_[4]) * l_long_axis])

            ob_vec = np.array([ob_[0], ob_[1]])
            center_vec = goal_state[N-1, :2] - ob_vec
            dist_center = np.linalg.norm(center_vec)
            cos_ = np.dot(center_vec, long_axis.T) / (dist_center * l_long_axis)

            if np.abs(cos_) > 0.1:
                tan_square = 1 / (cos_ ** 2) - 1
                d = np.sqrt((l_long_axis ** 2 * l_short_axis ** 2 * (1 + tan_square) / (
                    l_short_axis ** 2 + l_long_axis ** 2 * tan_square)))
            else:
                d = l_short_axis

            cross_pt = ob_vec + d * center_vec / dist_center

            vec1 = goal_state[N-1, :2] - cross_pt
            vec2 = self_state[:2] - cross_pt
            theta = np.dot(vec1.T, vec2)

            return theta > 0

        def h(curpos_, ob_):
            #safe_dist = 1.3 # for scout
            safe_dist = 0.5 # for jackal

            c = ca.cos(ob_[4])
            s = ca.sin(ob_[4])
            a = ca.MX([ob_[2]])
            b = ca.MX([ob_[3]])

            ob_vec = ca.MX([ob_[0], ob_[1]])
            center_vec = curpos_[:2] - ob_vec.T

            dist = b * (ca.sqrt((c ** 2 / a ** 2 + s ** 2 / b ** 2) * center_vec[0] ** 2 + (s ** 2 / a ** 2 + c ** 2 / b ** 2) *
                                center_vec[1] ** 2 + 2 * c * s * (1 / a ** 2 - 1 / b ** 2) * center_vec[0] * center_vec[1]) - 1) - safe_dist
            
            return dist

        def quadratic(x, A):
            return ca.mtimes([x, A, x.T])

        # init_condition
        opti.subject_to(opt_states[0, :] == opt_x0.T)

        # Position Boundaries
        # if(distance_global(self_state, global_path[-1, :2]) > 1):
        #     opti.subject_to(opti.bounded(v_min, v, v_max))
        # else:
        #     opti.subject_to(opti.bounded(-v_min, v, v_max))
        opti.subject_to(opti.bounded(-v_max, v, v_max))
        opti.subject_to(opti.bounded(-omega_max, omega, omega_max))

        # System Model constraints
        for i in range(N):
            x_next = opt_states[i, :] + T * f(opt_states[i, :], opt_controls[i, :]).T
            opti.subject_to(opt_states[i + 1, :] == x_next)

        num_obs = int(len(obstacle)/N)
        print("num_obs:",num_obs)

        if Obtag:
            for j in range(num_obs):
                if not exceed_ob(obstacle[N*j]):
                    for i in range(N-1):
                        opti.subject_to(h(opt_states[i + 1, :], obstacle[j * N + i + 1]) >=
                                        (1 - gamma_k) * h(opt_states[i, :], obstacle[j * N + i]))

        obj = 0
        R = np.diag([0.1, 0.02])
        A = np.diag([0.1, 0.02])
        for i in range(N):
            # Q = np.diag([1.0+0.05*i,1.0+0.05*i, 0.02+0.005*i])
            Q = np.diag([1.0+0.05*i,1.0+0.05*i, 0])
            if i < N-1:
                obj += 0.1 * quadratic(opt_states[i, :] - goal_state[[i]], Q) + quadratic(opt_controls[i, :], R)
            else:
                obj += 0.1 * quadratic(opt_states[i, :] - goal_state[[i]], Q)
        #Q = np.diag([1.0,1.0, 0.02])*5
        Q = np.diag([1.0,1.0, 0])*5
        obj += quadratic(opt_states[N-1, :] - goal_state[[N-1]], Q)

        opti.minimize(obj)
        opts_setting = {'ipopt.max_iter': 2000, 'ipopt.print_level': 0, 'print_time': 0, 'ipopt.acceptable_tol': 1e-3,
                        'ipopt.acceptable_obj_change_tol': 1e-3}
        opti.solver('ipopt', opts_setting)
        opti.set_value(opt_x0, self_state[:3])

        mpc_success=False
        try:
            sol = opti.solve()
            u_res = sol.value(opt_controls)
            state_res = sol.value(opt_states)
            
            # if(turn_tag):
            #     for i in range(N):
            #         u_res[i][0]=-u_res[i][0]
            
            # self.last_input = u_res
            # self.last_state = state_res
            mpc_success = True

        except:
            rospy.logerr("Infeasible Solution")

            if mpc_success:
                mpc_success = False
            else:
                for i in range(N-1):
                    last_input[i] = last_input[i+1]
                    last_state[i] = last_state[i+1]
                last_input[N-1] = np.zeros([1, 2])

            u_res = last_input
            state_res = last_state
        # self.curr_pose_lock.release()
        # global_path_lock.release()
        # obstaclestacle_lock.release()
        return state_res, u_res
