#!/usr/bin/env python

import casadi as ca
import casadi.tools as ca_tools
import rospy
import numpy as np
import math
import time
from nav_msgs.msg import  OccupancyGrid

SCOUT_WHEELBASE = 0.498  #/putn_ws1/src/putn/src/scout_simulator/scout_gazebo_sim/include/scout_gazebo/scout_skid_steer.hpp
esdf_map_global=OccupancyGrid()
def MPC(self_state, goal_state, obsjectory_pre,obstacles_num,last_u,mpc_param,esdf_map=[],obstacles=[]):   ##Nx3  
    print("mpc")
    T= 0.2 # sampling time [s]
    N_test=20
    CBF_Threshold=mpc_param.CBF_Threshold
    N = N_test # prediction horizon
    N_cbf=N_test
    N_MPC=N_test

    v_max = 1.0
    omega_max = 0.6
    car_r=0.7

    path_smoothing=0
    gamma=mpc_param. gamma## cbf parameter


    # static obstacle
    static=0
    if static:
        static_mode=True
        if esdf_map==[]:
            static_mode=False
        if  static_mode:
            static_obstacle_penalty_weight=mpc_param.static_obstacle_penalty_weight
            global esdf_map_global 
            esdf_map_global= esdf_map
            for i in range(1,N_MPC+1-1):
                obj+=static_obstacle_penalty_weight*0.5*(query_d_and_cal_c(X[i+1],epsilon)+query_d_and_cal_c(X[i],epsilon))*ca.norm_1(X[i+1][:2]-X[i][:2])

        # print("static_modestatic_modestatic_modestatic_mode",static_mode)
        #print("static_obstacle_penalty_weight",static_obstacle_penalty_weight)
            d_safe=mpc_param.d_safe
            ##d_min=0.5
            epsilon=car_r+d_safe
    #print("fwehfufhu",query_d_and_cal_c(self_state[0,:2],epsilon))

    #query_d_and_cal_c(self_state[0,:2],epsilon)


    goal = goal_state[:,:3] ##Nx3

    states = ca_tools.struct_symSX([
        (
            ca_tools.entry('x'),
            ca_tools.entry('y'),
            ca_tools.entry('theta')
        )
    ])
    x, y, theta = states[...]
    n_states = states.size
    controls  = ca_tools.struct_symSX([
        (
            ca_tools.entry('v'),
            ca_tools.entry('omega')
        )
    ])
    v, omega = controls[...]
    n_controls = controls.size


    ## function
    ## rhs----------------------------------------------------
    theta_x = self_state[0][4]*np.cos(self_state[0][2]) - self_state[0][3]*np.sin(self_state[0][2])
    theta_y = self_state[0][4]*np.sin(self_state[0][2]) + self_state[0][3]*np.cos(self_state[0][2])
    rhs = ca_tools.struct_SX(states)
    rhs['x'] = v*ca.cos(theta)*np.cos(theta_x)
    rhs['y'] = v*ca.sin(theta)*np.cos(theta_y)
    rhs['theta'] = omega

    f = ca.Function('f', [states, controls], [rhs], ['input_state', 'control_input'], ['rhs'])

    ## for MPC
    optimizing_target = ca_tools.struct_symSX([
        (
            ca_tools.entry('U', repeat=N_MPC, struct=controls),
            ca_tools.entry('X', repeat=N_MPC+1, struct=states)
        )
    ])
    U, X, = optimizing_target[...] # data are stored in list [], notice that ',' cannot be mis
    P=goal
    ## Path smoothing
    if(path_smoothing==1):
        for i in range(N_MPC):
            if(i ==0 or i==N_MPC-1):
                P[i]=goal[i]
            else:
                P[i]=0.25*goal[i-1]+0.5*goal[i]+0.25*goal[i+1]

     #### cost function      #### constrains

    obj = 0 #### cost
    g = [] 
    lbg = []
    ubg = []

    g.append(X[0]-self_state[0,:3]) # initial condition constraints   3xN   Nx3

    Q = np.array([[1, 0.0, 0.0],[0.0, 1, 0.0],[0.0, 0.0, 0.0]])
    R = np.array([[1, 0.0], [0.0, 2]]) #penalty omega
    error_penalty_weight=mpc_param.error_penalty_weight
    u_penalty_weight=mpc_param. u_penalty_weight
    for i in range(N_MPC):
        # obj = obj + 0.1*N_MPC*ca.mtimes([(X[i+1]-P[i]).T, Q, X[i+1]-P[i]])+ca.mtimes([U[i].T, R, U[i]])
        obj = obj + error_penalty_weight*i*ca.mtimes([(X[i]-P[i]).T, Q, X[i]-P[i]]) 
        #obj=obj+u_penalty_weight*ca.mtimes([(U[i]).T, R, U[i]])
        x_next_ = f(X[i], U[i])*T + X[i]
        g.append(X[i+1] - x_next_)
    # obj = obj + error_penalty_weight*N_MPC*ca.mtimes([(X[N_MPC-1]-P[N_MPC-1]).T, Q, X[N_MPC-1]-P[N_MPC-1]])
    #obj=obj+u_penalty_weight*(v_max-ca.sqrt((U[0][0])**2))
    for _ in range(3*(N_MPC+1)):
        lbg.append(0)
        ubg.append(0)
    # U_max=np.array([v_max,omega_max])
    # max=U_max*R*U_max.T
    # obj=obj+u_penalty_weight*(max-ca.mtimes([(U[0]).T, R,U[0]]))


    acceleration_penalty_weight=mpc_param.acceleration_penalty_weight
    ## acceleration constraint   
    # for i in range(N_MPC-1):
    #     obj = obj +  acceleration_penalty_weight*ca.mtimes([(U[i+1]-U[i]).T, R, U[i+1]-U[i]])
    # obj=obj+ 0.1*N_MPC*ca.mtimes([(U[0]-last_u).T, R, U[0]-last_u])


  #dynamic obstacle
    dynamic=1
    if dynamic:
        if obstacles_num != 0:

            Cbf_tag=np.zeros([obstacles_num])
            Cbf_num=0

            for i in range(obstacles_num):
                current_distance=np.linalg.norm(self_state[0][:2]-obsjectory_pre[i][0][:2])
                if (current_distance<=CBF_Threshold):
                    Cbf_num+=1
                    Cbf_tag[i]=1
                    d_safe=mpc_param.d_safe
                    # new_ellipse_ab=cal_Minkowski(car_r,obsjectory_pre[i])
                    # # new_ellipse_ab=cal_newEllipse(car_r,obsjectory_pre)
                    # new_obsjectory_pre1=np.hstack((obsjectory_pre[i][:,:2],new_ellipse_ab))
                    # new_obsjectory_pre=np.hstack((new_obsjectory_pre1,obsjectory_pre[i][:,4:]))
                    # # print("new_obsjectory_pre_min",new_obsjectory_pre)
                    # h=[]
                    # for j in range(N_test+1):
                    #     h.append(cal_h(new_obsjectory_pre[j], X[j])) 
                    #     # h.append(cal_h(new_obsjectory_pre[0], X[i])) 
                    # for j in range(N_test-1+1):
                    #     g.append(h[j+1]+(gamma-1)*h[j])
                    #     # g.append(h[i+1])
                    #     lbg.append(0)
                    #     ubg.append(ca.inf)


                    h=[]
                    for j in range(N_test+1):
                        h.append(cal_h_putn_original(obsjectory_pre[i][j], X[j],d_safe+car_r)) 
                        # h.append(cal_h(new_obsjectory_pre[0], X[i])) 
                    for j in range(N_test-1+1):
                        g.append(h[j+1]+(gamma-1)*h[j])
                        # g.append(h[i+1])
                        lbg.append(0)
                        ubg.append(ca.inf)






    ## constraint for acceleration
    acceletation_con=1
    if acceletation_con:
        Z1=np.array([[1, SCOUT_WHEELBASE], [SCOUT_WHEELBASE, SCOUT_WHEELBASE**2 ]])
        Z2=np.array([[1, -SCOUT_WHEELBASE], [-SCOUT_WHEELBASE, SCOUT_WHEELBASE**2 ]])
        g.append(ca.mtimes([(last_u-U[0]).T, Z1,last_u-U[0]]))
        g.append(ca.mtimes([(last_u-U[0]).T, Z2,last_u-U[0]]))

        wheel_arpha_max_constraint=mpc_param.wheel_arpha_max_constraint
        constraint_acceleration=wheel_arpha_max_constraint*T
        lbg.append(0)
        lbg.append(0)
        ubg.append(constraint_acceleration)
        ubg.append(constraint_acceleration)


    param = []
    nlp_prob = {'f': obj, 'x': optimizing_target, 'p':param, 'g':ca.vertcat(*g)}
    opts_setting = {'ipopt.max_iter':80, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-3, 'ipopt.acceptable_obj_change_tol':1e-3}
    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)


    lbx = optimizing_target(-ca.inf)
    ubx = optimizing_target(ca.inf)
    lbx['U', :, 'v'] = -v_max
    lbx['U', :, 'omega'] = -omega_max
    ubx['U', :, 'v'] = v_max
    ubx['U', :, 'omega'] = omega_max


    init_control = optimizing_target(0)
    try:
        res = solver(x0=init_control, p=param, lbg=lbg, lbx=lbx, ubg=ubg, ubx=ubx)
        estimated_opt = res['x'].full()
        ff_last_ = estimated_opt[-3:] 
        temp_estimated = estimated_opt[:-3].reshape(-1, 5)
        u_res = temp_estimated[:, :2]
        ff_value = temp_estimated[:, 2:].T
        state_res = (np.concatenate((ff_value, estimated_opt[-3:].reshape(3, 1)), axis=1) ).T
    except:
        state_res = np.repeat(self_state[:3],N_MPC+1,axis=0)
        u_res = np.zeros([N_MPC,2])
        #print("MPC unsolve")
    #print("u_res",u_res)
    #print(" state_res", state_res)
    return state_res, u_res

# wrong   
def cal_Minkowski(r,obj):
    new_ellipse=np.zeros([obj.shape[0],2])

    for i in range (obj.shape[0]):
        a=obj[i][2]
        b=obj[i][3]
        num1=r**2/a**2
        num2=r**2/b**2
        lb=(np.sqrt((num1+num2)*(num1+num2+6*num1*num2))-(num1+num2))/(6*num1*num2)
        ub=min((num1+np.sqrt(num1*(num1+8)))/(2*num1),(num2+np.sqrt(num2*(num2+8)))/(2*num2))
        # print("lb", lb)
        # print("ub",ub)
        e=0.1
        belta=root_solver(lb,ub,e,num1,num2)
        # print(belta,belta)
        new_ellipse[i][0]=np.sqrt(float((1 + 1  /belta) * a**  2 + (1 + belta)  *r ** 2))
        new_ellipse[i][1]=np.sqrt(float((1 + 1    /belta)  * b**  2 + (1 + belta) * r**2))
    return new_ellipse

def root_solver(lb,ub,e,num1,num2):
    m=(lb+ub)/2
    p =p3(num1,num2,m)
    if(np.abs(p)<=e):
        return m
    elif(p<0):
        return root_solver(m,ub,e,num1,num2)
    else:
        return root_solver(lb, m, e,num1,num2)

def p3(num1,num2,b):
    p=2*num1*num2*b**3+(num1+num2)*b**2-(num1+num2)*b-2
    return p

def cal_newEllipse(r,obj):
    delta=np.zeros(obj.shape[0])
    new_ellipse=np.zeros([obj.shape[0],2])
    for i in range (obj.shape[0]):
        a=obj[i][2]
        b=obj[i][3]
        result_=(-4*(4*a**2*b**2/(a + b)**2 + 3*r**2)**3 + (16*a**3*b**3/(a + b)**3 + 18*a*b*r**2/(a + b) - 27*a*r**2/2 - 27*b*r**2/2)**2)

        if result_>=0:
            result__ = (8 * a ** 3 * b ** 3 / (a + b) ** 3 + 9 * a * b * r ** 2 / (a + b) - 27 * a * r ** 2 / 4 - 27 * b * r ** 2 / 4 + math.sqrt(result_)/ 2)
            result=-2*a*b/(3*(a + b)) - (4*a**2*b**2/(a + b)**2 + 3*r**2)/(3*np.cbrt(float(result__ ))) - np.cbrt(float(result__ ))/3
            delta[i]=max(result,r)
        else:
            delta[i]=r
            # rospy.logwarn("ellipse------r!!")
        new_ellipse[i][0]=a+delta[i]
        new_ellipse[i][1]=b+delta[i]

    # print("obj_x",obj[:,0])
    # print("obj_y",obj[:,1])
    # print("a_new",new_ellipse[:,0])
    # print("b_new",new_ellipse[:,1])
    # print("delta",delta)
    return new_ellipse


def cal_h(obj,x_p):
    a = obj[2]
    b = obj[3]
    theta = obj[4]
    delta_X=ca.vertcat(x_p[:2]-obj[:2]).T
    Mat_r=np.mat([[np.cos(theta),-np.sin(theta)],
                    [np.sin(theta),np.cos(theta)]])
    Mat_ellipse=np.mat([[1/a**2,0],[0,1/b**2]])
    M=Mat_r.T*Mat_ellipse*Mat_r
    m=ca.mtimes(ca.mtimes(delta_X,M),delta_X.T)
    h_i=m-1  #my change
    # print(h_i)
    return h_i
def cal_h_putn_original(obj,x_p,safe_distance):
    a = obj[2]
    b = obj[3]
    theta = obj[4]
    l = np.sqrt((np.square(a) *np. square(b) * (1 +np. square(np.tan(theta))))
                / (np.square(b) + np.square(a) * np.square(np.tan(theta))))
    h_i=ca.norm_2(x_p[:2]-obj[:2])-l-safe_distance
    return h_i

def query_d_and_cal_c(x,epsilon):

    x_min =esdf_map_global.info.origin.position.x 
    y_min =esdf_map_global.info.origin.position.y
    map_resolution=esdf_map_global.info.resolution
    #print(map_resolution)
    num=esdf_map_global.info.width * esdf_map_global.info.height
    #print(num)


    # i=ca.floor((x[0] - x_min) / map_resolution)
    # j=ca.floor ((x[1] - y_min) / map_resolution)
    # k=int(i + j * esdf_map_global.info.width)
    # d=(esdf_map_global.data[k] +128.0)/51.0

    d=2.5
    i=ca.floor((x[0] - x_min) / map_resolution)
    j=ca.floor ((x[1] - y_min) / map_resolution)
    abc2=ca.MX.ones(1,1)*1e-1
    for k in range(num):
        abc1=ca.norm_2(i + j * esdf_map_global.info.width-k)
        if abc1<abc2:
            d=(esdf_map_global.data[k] +128.0)/51.0
            break
    #print ("d",d)
    # print("fadfaaf",type(data))
    # i =data.toarray()
    # #i =int( ca.SX_from_array([(x[0] - x_min) / map_resolution,0]))
    # j = int(ca.floor (((x[1] - y_min) / map_resolution)))
    # d=(esdf_map_global.data[i + j * esdf_map_global.info.width] +128.0)/51.0

    #print("query_d",d)
    ####calculate c
    if d<0:
        c=-d+epsilon/2
    elif d>=0 and d<=epsilon:
        c=(d-epsilon)**2/(2*epsilon)
    else:
        c=0
    print("c",c)
    return c

# if __name__ == '__main__':
#     rospy.init_node("test")
#     obstacle_trajectory_predict= np.zeros([ 10,5])
#     for i in range(10):
#         for j in range(5):
#             obstacle_trajectory_predict[i][j] = 1
#     r = 0.6
#     new_ellipse_ab,delta=cal_newEllipse(r,obstacle_trajectory_predict)
#     print(new_ellipse_ab)