#!/usr/bin/env python
from collections import defaultdict
import itertools
import json
import random
import numpy as np
from collections import namedtuple
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import Odometry  
from visualization_msgs.msg import Marker, MarkerArray  
from std_msgs.msg import Float32MultiArray  
import math 

from lstm import LSTMPredictor
import torch





odom_ob_all=[]  # x y t  3*ped_num
ped_lists = []
ped_num=2
delta_time=0
time_index=0
cp=[]
predicted_paths=[]
predicted_paths_all=[]
predicted_paths_all_=[]

odom_num=0
odom_ob_xy=[]
odom_last_time=0

"""Loading the APPROPRIATE model"""
device='cpu'
model='/home/linanji/src/map/src/simulator/scripts/lstm_goals_social_None_modes1/lstm_vanilla_None.pkl'
predictor = LSTMPredictor.load(model)
predictor.model.to(torch.device(device))

marker_pub = rospy.Publisher('visualization_marker', MarkerArray, queue_size=10) 


def odom_all_callback(msg): 
    #del odom_ob_all[:]
    i=0
    
    odom_ob_all_past= []  # 每次处理前重置odom_ob_all  

    for value in msg.data: 
        odom_ob_all[i]=value # x y t
        odom_ob_all_past.append(value)
        i=i+1
    global odom_last_time
    #print("odom_last_time",odom_last_time)
    #print("odom_ob_all[2]",odom_ob_all[2])
    if(odom_ob_all[2]-odom_last_time>delta_time):
        odom_last_time=odom_ob_all[2]
        global  odom_num
        if odom_num<9:
            odom_num+=1
            odom_ob_xy.append(odom_ob_all_past)
        else:
            odom_ob_xy.pop(0)
            odom_ob_xy.append(odom_ob_all_past)
    #print(odom_ob_all)

def predict_odom():
    global ped_num
    xy= np.full((9, ped_num, 2), np.nan)
    for frame_index in range(9):
        for ped_index in range(ped_num):
            entry = xy[frame_index][ped_index]
            entry[0] = odom_ob_xy[frame_index][ped_index*3+0]
            entry[1] =  odom_ob_xy[frame_index][ped_index*3+1]
    print(xy)
    scene_goal= np.full(( ped_num, 2), 0)
    predictions = predictor(xy, scene_goal, n_predict=12, obs_length=9, modes=1)#, args=args)
    cylinder_marker_publisher(predictions[0])
    all_ob_state_pub(predictions[0])
    # return predictions
    

marker_gt_pub = rospy.Publisher('gt_marker', MarkerArray, queue_size=10) 

def create_pose_from_point(point):  


    quaternion = Quaternion(0.0, 0.0, 0.0, 1.0)  

    pose = Pose(point, quaternion)  

    return pose 
def cylinder_marker_publisher(pred_path):  
    surtrajs = MarkerArray()  
    #rospy.loginfo("index: ( {})".format(start_index))  


    for i in range(ped_num):  
        traj = Marker()  
        traj.action = Marker.ADD  
        traj.id = i  
        traj.type = Marker.LINE_STRIP  
        traj.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  
        traj.color.r = 0.0  
        traj.color.g = 1.0  
        traj.color.b = 0.0  
        traj.color.a = 1.0  
        traj.scale.x = 0.1  
        traj.scale.y = 0.1  
        traj.scale.z = 0.1  
        traj.header.frame_id = "map"  

        traj.header.stamp =rospy.Time.now()  
        
        for j in range(len(cp)):  
            t = (j+1)* 0.5  
            point1 = Point()  
            if i==0:
                point1.x = pred_path[i][j][0]
                point1.y =  pred_path[i][j][1]  
            else:
                point1.x = pred_path[i][j][0][0]
                point1.y =  pred_path[i][j][0][1]
            
            point1.z = t  
            traj.points.append(point1)  

            circle_marker = Marker() 
            circle_marker.header.frame_id= "map"  
            circle_marker.action = Marker.ADD  
            circle_marker.id = i *len(cp) + j +ped_num  
            circle_marker.type = Marker.SPHERE
            circle_marker.pose = create_pose_from_point(point1)  
            circle_marker.scale.x = cp[j]  
            circle_marker.scale.y =  cp[j]
            # circle_marker.scale.x = 0#cp[j]  
            # circle_marker.scale.y =  0#cp[j]
            circle_marker.scale.z = 1  
            circle_marker.color.r = 0.0  
            circle_marker.color.g = 1.0  
            circle_marker.color.b = 0.0  
            circle_marker.color.a = 0.5 
            surtrajs.markers.append(circle_marker)
        surtrajs.markers.append(traj)  
    
    marker_pub.publish(surtrajs)

state_pub = rospy.Publisher('ob_state_all', Float32MultiArray, queue_size=10)  
def all_ob_state_pub(pred_path):
    odom_ob_all_in=odom_ob_all
    if(len(odom_ob_all_in)>=3*ped_num):
        print("len(odom_ob_all_in) :",len(odom_ob_all_in))
        
        msg = Float32MultiArray() 
        for i in range(ped_num):
            print("i*3+2 :",i*3+2)
            # try:
            #     time_stamp=odom_ob_all_in[i*3+2]
            #     x_pre=odom_ob_all_in[i*3]
            #     y_pre=odom_ob_all_in[i*3+1]
            # except IndexError:
            #     print(f"iindex {i*3+2} out of range, lengthshould be {len(odom_ob_all_in)}")
            #     # x_pre=odom_ob_all_in[i*3]
            #     # y_pre=odom_ob_all_in[i*3+1]
            time_stamp=odom_ob_all_in[i*3+2]
            x_pre=odom_ob_all_in[i*3]
            y_pre=odom_ob_all_in[i*3+1]
            msg.data.append(x_pre)
            msg.data.append(y_pre)
            msg.data.append(0) #angle
            msg.data.append(0) #vx
            msg.data.append(0) #vy
            msg.data.append(ob_r[i]) # r
            msg.data.append(time_stamp)
            for j in range(len(cp)):
                #print("j",j)
                if i==0:
                    x = pred_path[i][j][0]
                    y =  pred_path[i][j][1]  
                else:
                    x = pred_path[i][j][0][0]
                    y =  pred_path[i][j][0][1]

                vx=(x-x_pre)/delta_time
                vy=(y-y_pre)/delta_time
                t=time_stamp+(j+1)*delta_time
                # r=ob_r[i]+ 0#cp[j]
                r=ob_r[i]+cp[j]
                angle=math.atan2(vy,vx)
                msg.data.append(x)
                msg.data.append(y)
                msg.data.append(angle) #angle
                msg.data.append(vx) #vx
                msg.data.append(vy) #vy
                msg.data.append(r) # r
                msg.data.append(t)
                x_pre=x
                y_pre=y

        state_pub.publish(msg)

if __name__ == '__main__':
    print("beginning0")
    rospy.init_node("ssc_pred")
    print("beginning1")
    # read predestrians truth path
    ped_num=rospy.get_param('/ssc_pred/obstacle_num',5)
    ped_start_index=rospy.get_param('/ssc_pred/ped_start_index',0)#1024
    scene_start_index=rospy.get_param('/ssc_pred/scene_start_index',0)#38188
    obstacle_radius=rospy.get_param('/ssc_pred/obstacle_radius',0)#0.25
    delta_time=rospy.get_param('/ssc_pred/delta_time',0)  #0.5
    ob_r=[obstacle_radius for _ in range(ped_num)] 
    odom_ob_all = [0 for _ in range(3 * ped_num)]  

    print("beginning")
    
    ## read cp
    with open('/home/linanji/src/map/src/simulator/scripts/cp.txt', 'r') as f:  
        for line in f:  
            cp.append(float(line.strip()))  
    print(cp)  
    print("beginning2")

    
    

    


   
    rospy.Subscriber("/odom_all", Float32MultiArray, odom_all_callback, queue_size=10)
    #rospy.spin()
    rate = rospy.Rate(10)  
    #__timer_pub_vis=rospy.Timer(rospy.Duration(0.05), cylinder_marker_publisher())
    while not rospy.is_shutdown():
        
        print("123")
        print("odom_num",odom_num)
        if(odom_num>=9):
            print("1234")
            predict_odom()
        # gt_path_vis(time_index)
        # cylinder_marker_publisher(time_index)
        # all_ob_state_pub(time_index)
        #rospy.spin()  
        rate.sleep()  

    #rospy.spin()