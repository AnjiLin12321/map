#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, Float64, Float32MultiArray
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point, Twist
from nav_msgs.msg import Path, Odometry, OccupancyGrid
import numpy as np
import tf
from mpc_a3d import MPC
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker,MarkerArray
from std_srvs.srv import SetBool

cur_time=0.0

class Local_Planner():
    def __init__(self):
        self.replan_period = rospy.get_param('/local_planner/replan_period', 0.01)
        self.curr_state = np.zeros(7)  # x y yaw roll pitch -----> x y yaw roll  pitch z t
        self.z = 0
        self.N = 20
        self.goal_state = np.zeros([self.N,3]) # xy t 0
        self.ref_path_close_set = False
        self.target_state = np.array([-1,4,np.pi/2])
        self.target_state_close = np.zeros(3)
        self.desired_global_path = [ np.zeros([400,3]) , 0]   # self.desired_global_path[1]=size   x y t   20/0.5*10=400
        self.have_plan = False
        self.is_close = False
        self.is_get = False
        self.is_grasp = False
        self.is_all_task_down = False
        self.robot_state_set = False
        self.ref_path_set = False
        self.ob=[]
        self.is_end=0
        self.ob_total = []

        self.max_err_dis=1
        self.time_dif_max_inter=10

        self.obstacle_num=rospy.get_param('/local_planner/obstacle_num',5)
        self.robot_r=rospy.get_param('/local_planner/robot_r',0)
        self.safe_dis_cbf=rospy.get_param('/local_planner/safe_dis_cbf',0)
        self.nt_ori=12+1
        self.sample_ind=10
        self.ped_all=[[] for _ in range(self.obstacle_num)] 
        self.ped_scale=[[] for _ in range(self.obstacle_num)]  
        # self.pre_time=0.5
        # self.deltatime
        self.last_states_sol= np.zeros([self.N,2])
        self.last_input_sol= np.zeros([self.N,2])

        self.__timer_replan = rospy.Timer(rospy.Duration(self.replan_period), self.__replan_cb)
        self.__sub_curr_state = rospy.Subscriber('/curr_state', Float32MultiArray, self.__curr_pose_cb, queue_size=10)
        self.__sub_obs = rospy.Subscriber('/obs', Float32MultiArray, self.__obs_cb, queue_size=10)
        self.__sub_goal_state = rospy.Subscriber('/path_inter', Float32MultiArray, self._global_path_callback2, queue_size=10)
        self.__pub_local_path = rospy.Publisher('/local_path', Path, queue_size=10)
        self.__pub_local_plan = rospy.Publisher('/local_plan', Float32MultiArray, queue_size=10)
        self.control_cmd = Twist()
        self.listener = tf.TransformListener()
        self.times = 0
        self.obstacle_markerarray = MarkerArray()
        self.ob_pub = rospy.Publisher('/ob_draw', MarkerArray, queue_size=10)
        self.__sub_ob_state = rospy.Subscriber('/ob_state_all', Float32MultiArray, self.__ob_state_cb, queue_size=10)

    def distance_sqaure(self,c1,c2):
        distance = (c1[0]-c2[0])*(c1[0]-c2[0])+(c1[1]-c2[1])*(c1[1]-c2[1])
        return distance

    def draw_ob(self):
        self.obstacle_markerarray.markers=[]
        num = 0
        for i in range(len(self.ob)):
            t_ob = Marker()
            t_ob.header.frame_id = "map"
            t_ob.id = num
            t_ob.type = t_ob.CYLINDER
            t_ob.action = t_ob.ADD
            t_ob.pose.position.x = self.ob[i][0]
            t_ob.pose.position.y = self.ob[i][1]
            t_ob.pose.position.z=0.2
            t_ob.scale.x = 0.1
            t_ob.scale.y = 0.1
            t_ob.scale.z = 0.4
            t_ob.color.a= 1
            t_ob.color.r = 0
            t_ob.color.g = 1
            t_ob.color.b = 0
            self.obstacle_markerarray.markers.append(t_ob)
            num = num +1
        self.ob_pub.publish(self.obstacle_markerarray)

    # def _scan_callback(self, data):
    #     self.ob = []
    #     phi = data.angle_min
    #     point_last = np.array([100, 100])
    #     for r in data.ranges:
    #         point = np.array([self.curr_state[0]+r*np.cos(phi+self.curr_state[2]),self.curr_state[1]+r*np.sin(phi+self.curr_state[2])])
    #         if (r >= data.range_min and r <= data.range_max and r<=1.0 and self.distance_sqaure(point,point_last) > 0.04 ):
    #             self.ob.append( point )
    #             point_last = point
    #         phi += data.angle_increment
    #     self.draw_ob()

    def __obs_cb(self, data):
        self.ob = []
        if(len(data.data)!=0):

            size = len(data.data)/3
            for i in range(size):
                self.ob.append(( (data.data[3*i]//0.3)*0.3, (data.data[3*i+1]//0.3)*0.3) )
            dic = list(set([tuple(t) for t in self.ob]))
            self.ob = [list(v) for v in dic]
            self.draw_ob()

    def __replan_cb(self, event):
        if self.robot_state_set and self.ref_path_set:
            # print("cbf")
            target = []
            self.choose_goal_ob_state()        ##  gobal planning
            dist = 1
            goal = np.array([self.target_state[0], self.target_state[1], self.target_state[2]])
            start_time = rospy.Time.now()
            #states_sol, input_sol = MPC(np.expand_dims(self.curr_state, axis=0),self.goal_state,self.obstacle_num,self.ped_scale) ##  gobal planning
            # if not  (self.ped_scale[self.obstacle_num-1] == []): 
            #     print("a ", self.ped_scale[0][0][3])
            #     print("a end", self.ped_scale[0][19][3])
            states_sol, input_sol = MPC(self.curr_state,self.goal_state,self.desired_global_path[0],0,[[] for _ in range(self.obstacle_num)] ,self.last_states_sol,self.last_input_sol,self.robot_r,self.safe_dis_cbf) ##  gobal planning
            self.last_states_sol=states_sol
            self.last_input_sol=input_sol
            end_time = rospy.Time.now()
            #rospy.loginfo('[pHRI Planner] phri so[lved in {} sec'.format((end_time-start_time).to_sec()))
            #print(input_sol)
            if(self.is_end == 0):
                self.__publish_local_plan(input_sol,states_sol)
            self.have_plan = True
        elif self.robot_state_set==False and self.ref_path_set==True:
            print("no pose")
        elif self.robot_state_set==True and self.ref_path_set==False:
            print("no path")
        else:
            print("no path and no pose")
        

    def __publish_local_plan(self,input_sol,state_sol):
        local_path = Path()
        local_plan = Float32MultiArray()
        sequ = 0
        local_path.header.stamp = rospy.Time.now()
        local_path.header.frame_id = "/map"

        for i in range(self.N):
            this_pose_stamped = PoseStamped()
            this_pose_stamped.pose.position.x = state_sol[i,0]
            this_pose_stamped.pose.position.y = state_sol[i,1]
            this_pose_stamped.pose.position.z = self.z+0.5 #self.desired_global_path[0][0,2]
            this_pose_stamped.header.seq = sequ
            sequ += 1
            this_pose_stamped.header.stamp = rospy.Time.now()
            this_pose_stamped.header.frame_id="/map"
            local_path.poses.append(this_pose_stamped)
            
            for j in range(2):
                local_plan.data.append(input_sol[i][j])

        self.__pub_local_path.publish(local_path)
        self.__pub_local_plan.publish(local_plan)

    def distance_global(self,c1,c2):
        distance = np.sqrt((c1[0]-c2[0])*(c1[0]-c2[0])+(c1[1]-c2[1])*(c1[1]-c2[1]))
        return distance
    

    def find_min_distance(self,c1):
        #number =  np.argmin( np.array([self.distance_global(c1,self.desired_global_path[0][i]) for i in range(self.desired_global_path[1])]) )
        min_time_dif=1000
        iter=0
        dis_err=0
        number=0
        for i in range(self.desired_global_path[1]):  
            time_dif=np.abs(c1[6]-self.desired_global_path[0][i][2])
            #print("c1[6]," ,c1[6],self.desired_global_path[0][i][2])
            if(iter>self.time_dif_max_inter):
                break
            if(time_dif<min_time_dif):
                iter =0
                min_time_dif=time_dif
                dis_err= np.sqrt((c1[0]-self.desired_global_path[0][i][0])*(c1[0]-self.desired_global_path[0][i][0])+(c1[1]-self.desired_global_path[0][i][1])*(c1[1]-self.desired_global_path[0][i][1]))

                number=i
            else:
                iter+=1
        if(dis_err>self.max_err_dis):
            print("global  path unuseful: too far!!!")
        #print("num",number)
        #print("err",min_time_dif,dis_err)
        return number

    def find_min_ob_distance(self,c1):
        #number =  np.argmin( np.array([self.distance_global(c1,self.desired_global_path[0][i]) for i in range(self.desired_global_path[1])]) )
        min_time_dif=1000
        iter=0
        number=0
        for i in range(len(self.ped_all[0])):  
            time_dif=np.abs(c1[6]-self.ped_all[0][i][4])
            #print("c1[6]," ,c1[6],self.desired_global_path[0][i][2])
            if(iter>self.time_dif_max_inter):
                break
            if(time_dif<min_time_dif):
                iter =0
                min_time_dif=time_dif

                number=i
            else:
                iter+=1
        return number

    def choose_goal_ob_state(self):
        num = self.find_min_distance(self.curr_state)
        num_ob=num
       
        # print("num",num)
        # print("num_ob",num_ob)
        # rospy.loginfo('num  {} '.format(num))
        # rospy.loginfo('num_ob  {} '.format(num_ob))
        scale = 2
        num_list = []
        num_list_ob = []
        for i in range(self.N):  
            num_path = min(self.desired_global_path[1]-1,int(num+i*scale))
            num_list.append(num_path)
        if(num  >= self.desired_global_path[1]):
            self.is_end = 1
        for k in range(self.N):
            self.goal_state[k] = self.desired_global_path[0][num_list[k]]
       
        if not  (self.ped_all[self.obstacle_num-1] == []): 
            num_ob=self.find_min_ob_distance(self.curr_state)
            for i in range(self.N):  
                num_ob_path = min(len(self.ped_all[0])-1,int(num_ob+i*scale))
                num_list_ob.append(num_ob_path)
            for i in range(self.obstacle_num):
                ped_i_scale=[]
                for k in range(self.N):
                    ped_i_scale.append(self.ped_all[i][num_list_ob[k]])
                self.ped_scale[i]=ped_i_scale
                
        #print(self.goal_state)

    def __curr_pose_cb(self, data):
        self.robot_state_set = True
        self.curr_state[0] = data.data[0]    #x
        self.curr_state[1] = data.data[1]   #y
        self.curr_state[2] = data.data[3]  #yaw
        self.curr_state[3] = data.data[4]  #raw
        self.curr_state[4] = data.data[5]  #pitch
        self.curr_state[5]= data.data[2]  #z
        self.curr_state[6]=data.data[6]   #t
        global cur_time
        cur_time=data.data[6]
        # rospy.loginfo('cur_time  {} '.format(self.curr_state[6]))
        # print("cur_time:",self.curr_state[6])
        self.z = data.data[2]
    
    def __ob_state_cb(self, data):
        #self.ped_all=[]
        for i in range(self.obstacle_num):
            ped_i=[]
            ped_pre=[]
            

            ped_pre.append(data.data[i*7*self.nt_ori])  #x
            ped_pre.append(data.data[i*7*self.nt_ori+1])  #y
            ped_pre.append(data.data[i*7*self.nt_ori+2])  #angle
            ped_pre.append(data.data[i*7*self.nt_ori+5])# r
            ped_pre.append(data.data[i*7*self.nt_ori+6])# time_stamp
            ped_i.append(ped_pre)
            for j in range(1,self.nt_ori):
                ped_cur=[]
                ped_cur.append(data.data[i*7*self.nt_ori+j*7])  #x
                ped_cur.append(data.data[i*7*self.nt_ori+j*7+1])  #y
                ped_cur.append(data.data[i*7*self.nt_ori+j*7+2])  #angle
                ped_cur.append(data.data[i*7*self.nt_ori+j*7+5])# r
                ped_cur.append(data.data[i*7*self.nt_ori+j*7+6])# time_stamp
                
                ped_delta=[]
                for l in range (len(ped_cur)):
                    ped_delta.append(ped_cur[l]-ped_pre[l])
                for k in range (1,self.sample_ind+1):
                    ped_mid=[]
                    for l in range (len(ped_cur)):
                        ped_mid.append(ped_delta[l]*k/self.sample_ind+ped_pre[l])
                    ped_i.append(ped_mid)
                ped_pre=ped_cur
            
            self.ped_all[i]=ped_i
            # rospy.loginfo('ped time  {} '.format(self.ped_all[i][0][3]))





    def _global_path_callback(self, data):
        if(len(data.data)!=0):
            self.ref_path_set = True
            size = len(data.data)/3
            self.desired_global_path[1]=size
            for i in range(size):
                self.desired_global_path[0][i,0]=data.data[3*(size-i)-3]
                self.desired_global_path[0][i,1]=data.data[3*(size-i)-2]
                self.desired_global_path[0][i,2]=data.data[3*(size-i)-1]
    
    def _global_path_callback2(self, data):
        if(len(data.data)!=0):
            self.ref_path_set = True
            # size = len(data.data)/5
            # self.desired_global_path[1]=size
            # for i in range(size):
            #     self.desired_global_path[0][i,0]=data.data[5*(size-i)-5]
            #     self.desired_global_path[0][i,1]=data.data[5*(size-i)-4]
            #     self.desired_global_path[0][i,2]=data.data[5*(size-i)-2]
            #     self.desired_global_path[0][i,3]=data.data[5*(size-i)-1]
            size = (len(data.data)-1)/3
            self.desired_global_path[1]=size
            #print("size",size)
            # pre_x=0
            # pre_y=0
            # for i in range(size):
            #     cur_x=data.data[3*(size-i)-3]
            #     cur_y=data.data[3*(size-i)-2]
            #     if(i==0):
            #         self.desired_global_path[0][i,0]=data.data[3*(size-i)-3]  #x
            #         self.desired_global_path[0][i,1]=data.data[3*(size-i)-2] #y
            #         self.desired_global_path[0][i,2]=data.data[3*(size-i)-1]  #t
            #         self.desired_global_path[0][i,3]=atan2()
            #     else:


            #     pre_x=cur_x
            #     pre_y=cur_y


            for i in range(size):
                self.desired_global_path[0][i,0]=data.data[3*(size-i)-3]  #x
                self.desired_global_path[0][i,1]=data.data[3*(size-i)-2] #y
                self.desired_global_path[0][i,2]=data.data[3*(size-i)-1]  #t
            # rospy.loginfo('desired_global_path time0 {} '.format(self.desired_global_path[0][0,2]))
            # print("desired_global_path time0",self.desired_global_path[0][0,2])
            #print("desired_global_path time1",self.desired_global_path[0][1,2])

            
    def cmd(self, data):
        
        self.control_cmd.linear.x = data[0]
        self.control_cmd.angular.z = data[1]
        self.__pub_rtc_cmd.publish(self.control_cmd)



if __name__ == '__main__':
    rospy.init_node("local_planner")
    phri_planner = Local_Planner()

    rospy.spin()