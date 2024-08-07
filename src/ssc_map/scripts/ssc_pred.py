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

TrackRow = namedtuple('Row', ['frame', 'pedestrian', 'x', 'y', 'prediction_number', 'scene_id'])
TrackRow.__new__.__defaults__ = (None, None, None, None, None, None)
SceneRow = namedtuple('Row', ['scene', 'pedestrian', 'start', 'end', 'fps', 'tag'])
SceneRow.__new__.__defaults__ = (None, None, None, None, None, None)



ob_r=[]
odom_ob_all=[]  # x y t  3*ped_num
ped_lists = []
ped_num=5
delta_time=0
time_index=0
cp=[]
predicted_paths=[]
predicted_paths_all=[]
predicted_paths_all_=[]



class Reader(object):
    """Read trajnet files.

    :param scene_type: None -> numpy.array, 'rows' -> TrackRow and SceneRow, 'paths': grouped rows (primary pedestrian first), 'tags': numpy.array and scene tag
    :param image_file: Associated image file of the scene
    """
    def __init__(self, input_file, scene_type=None, image_file=None):
        if scene_type is not None and scene_type not in {'rows', 'paths', 'tags'}:
            raise Exception('scene_type not supported')
        self.scene_type = scene_type

        self.tracks_by_frame = defaultdict(list)
        self.scenes_by_id = dict()

        self.read_file(input_file)

    def read_file(self, input_file):
        with open(input_file, 'r') as f:
            for line in f:
                line = json.loads(line)

                track = line.get('track')
                if track is not None:
                    row = TrackRow(track['f'], track['p'], track['x'], track['y'], \
                                   track.get('prediction_number'), track.get('scene_id'))
                    self.tracks_by_frame[row.frame].append(row)
                    continue

                scene = line.get('scene')
                if scene is not None:
                    row = SceneRow(scene['id'], scene['p'], scene['s'], scene['e'], \
                                   scene.get('fps'), scene.get('tag'))
                    self.scenes_by_id[row.scene] = row

    def scenes(self, randomize=False, limit=0, ids=None, sample=None):
        scene_ids = self.scenes_by_id.keys()
        if ids is not None:
            scene_ids = ids
        if randomize:
            scene_ids = list(scene_ids)
            random.shuffle(scene_ids)
        if limit:
            scene_ids = itertools.islice(scene_ids, limit)
        if sample is not None:
            scene_ids = random.sample(scene_ids, int(len(scene_ids) * sample))
        for scene_id in scene_ids:
            yield self.scene(scene_id)

    @staticmethod
    def track_rows_to_paths(primary_pedestrian, track_rows):
        primary_path = []
        other_paths = defaultdict(list)
        for row in track_rows:
            if row.pedestrian == primary_pedestrian:
                primary_path.append(row)
                continue
            other_paths[row.pedestrian].append(row)

        return [primary_path] + list(other_paths.values())

    @staticmethod
    def paths_to_xy(paths):
        """Convert paths to numpy array with nan as blanks."""
        frames = set(r.frame for r in paths[0])
        pedestrians = set(row.pedestrian
                          for path in paths
                          for row in path if row.frame in frames)
        paths = [path for path in paths if path[0].pedestrian in pedestrians]
        frames = sorted(frames)
        pedestrians = list(pedestrians)

        frame_to_index = {frame: i for i, frame in enumerate(frames)}
        xy = np.full((len(frames), len(pedestrians), 2), np.nan)

        for ped_index, path in enumerate(paths):
            for row in path:
                if row.frame not in frame_to_index:
                    continue
                entry = xy[frame_to_index[row.frame]][ped_index]
                entry[0] = row.x
                entry[1] = row.y

        return xy

    def scene(self, scene_id):
        scene = self.scenes_by_id.get(scene_id)
        if scene is None:
            raise Exception('scene with that id not found')

        frames = range(scene.start, scene.end + 1)
        track_rows = [r
                      for frame in frames
                      for r in self.tracks_by_frame.get(frame, [])]

        # return as rows
        if self.scene_type == 'rows':
            return scene_id, scene.pedestrian, track_rows

        # return as paths
        paths = self.track_rows_to_paths(scene.pedestrian, track_rows)
        if self.scene_type == 'paths':
            return scene_id, paths

        ## return with scene tag
        if self.scene_type == 'tags':
            return scene_id, scene.tag, self.paths_to_xy(paths)

        # return a numpy array
        return scene_id, self.paths_to_xy(paths)

  
def actor1_odom_callback(msg):  

   # rospy.loginfo("Position: ({}, {})".format(msg.pose.pose.position.x, msg.pose.pose.position.y))  
    err=1000.0

    for i in range (len(ped_lists[0])):
        global  time_index
        err1=(ped_lists[0][i][0]-msg.pose.pose.position.x)*(ped_lists[0][i][0]-msg.pose.pose.position.x)+(ped_lists[0][i][1]-msg.pose.pose.position.y)*(ped_lists[0][i][1]-msg.pose.pose.position.y)
        if err1<err:
            err=err1
            time_index=i
    #rospy.loginfo("index: ( {})".format(time_index))  

def odom_all_callback(msg): 
    #del odom_ob_all[:]
    i=0
    for value in msg.data: 
        odom_ob_all[i]=value # x y t

        i=i+1
    

marker_pub = rospy.Publisher('visualization_marker', MarkerArray, queue_size=10) 
marker_gt_pub = rospy.Publisher('gt_marker', MarkerArray, queue_size=10) 

def create_pose_from_point(point):  


    quaternion = Quaternion(0.0, 0.0, 0.0, 1.0)  

    pose = Pose(point, quaternion)  

    return pose 
def cylinder_marker_publisher(start_index):  
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
        pred_path=predicted_paths_all_[start_index]
        
        for j in range(len(cp)):  
            t = (j+1)* 0.5  
            point1 = Point()  
            point1.x = pred_path[i][j][0]
            point1.y =  pred_path[i][j][1]  
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
def gt_path_vis(start_index):
    surtrajs = MarkerArray()  
    for i in range(ped_num):  
        traj = Marker()  
        traj.action = Marker.ADD  
        traj.id = i  
        traj.type = Marker.LINE_STRIP  
        traj.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  
        traj.color.r = 1.0  
        traj.color.g = 0.0  
        traj.color.b = 0.0  
        traj.color.a = 1.0  
        traj.scale.x = 0.1  
        traj.scale.y = 0.1  
        traj.scale.z = 0.1  
        traj.header.frame_id = "map"  
  
        traj.header.stamp =rospy.Time.now()  
    
        for j in range(start_index,len(ped_lists[0])):  
            t = (j-start_index) * 0.5  
            point1 = Point()  
            point1.x = ped_lists[i][j][0]   
            point1.y =  ped_lists[i][j][1]  
            point1.z = t  
            traj.points.append(point1)  

        surtrajs.markers.append(traj)  
    
    marker_gt_pub.publish(surtrajs)

state_pub = rospy.Publisher('ob_state_all', Float32MultiArray, queue_size=10)  
def all_ob_state_pub(start_index):
    odom_ob_all_in=odom_ob_all
    if(len(odom_ob_all_in)>=3*ped_num):
        print("len(odom_ob_all_in) :",len(odom_ob_all_in))
        pred_path=predicted_paths_all_[start_index]
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
                
                x=pred_path[i][j][0] 
                y=pred_path[i][j][1]
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
    rospy.init_node("ssc_pred")
    # read predestrians truth path
    ped_num=rospy.get_param('/ssc_pred/obstacle_num',5)
    ped_start_index=rospy.get_param('/ssc_pred/ped_start_index',0)#1024
    scene_start_index=rospy.get_param('/ssc_pred/scene_start_index',0)#38188
    obstacle_radius=rospy.get_param('/ssc_pred/obstacle_radius',0)#0.25
    delta_time=rospy.get_param('/ssc_pred/delta_time',0)  #0.5
    ob_r=[obstacle_radius for _ in range(ped_num)] 
    odom_ob_all = [0 for _ in range(3 * ped_num)]  
    #xy = np.full((len(frames), len(pedestrians), 2), np.nan)

    
    # episodes=1
    # tag=False
    ped_lists = [[] for _ in range(ped_num)] 
    with open('/home/linanji/src/map/src/simulator/scripts/orca_circle_crossing_5ped_1scenes_.txt', 'r') as file:  
        for line in file:  

            elements = line.strip().split(',')  
            time_ind=int(elements[0])
            if(time_ind<8+scene_start_index) or (time_ind>49+scene_start_index):
                continue
            ped_id=int(elements[1])
            if 0+ped_start_index <= ped_id < ped_num+ped_start_index:
                ped_lists[ped_id-ped_start_index].append((float(elements[2]), float(elements[3])))  
        

    
    # with open('/home/linanji/src/map/src/simulator/scripts/orca_circle_crossing_2ped_1000scenes_.txt', 'r') as file:  
    #     for line in file:  
    #         elements = line.strip().split(',') 
    #         time_ind=int(elements[0])
    #         if(time_ind<8+38188) or (time_ind>49+38188):
    #             continue
    #         ped_id=int(elements[1])
            
    #        if(ped_id==1024):
    #         ped_0.append((float(elements[2]),float(elements[3])))
    #     if(ped_id==1025):
    #         ped_1.append((float(elements[2]),float(elements[3])))
    #     if(ped_id==2):
    #         ped_2.append((float(elements[2]),float(elements[3])))
    #     if(ped_id==3):
    #         ped_3.append((float(elements[2]),float(elements[3])))
    #     if(ped_id==4):
    #         ped_4.append((float(elements[2]),float(elements[3])))

    #     if((ped_id)==1026):   #ped_num*episodes):
    #         break
    #     ped_all.append(ped_0)
    #     ped_all.append(ped_1)

    ## read cp
    with open('/home/linanji/src/map/src/simulator/scripts/cp.txt', 'r') as f:  
        for line in f:  
            cp.append(float(line.strip()))  
    #print(cp)  

    # read predestrians prediction 
    reader_list = {}
    label_dict = {}
    dataset_files=["/home/linanji/src/map/src/simulator/scripts/lstm_goals_social_None_modes1/orca_five_synth.ndjson"]
    for i, dataset_file in enumerate(dataset_files):
        name = dataset_file.split('/')[-2]
        label_dict[name] =  name
        reader_list[name] = Reader(dataset_file, scene_type='paths')

    scenes = reader_list[name].scenes()
    
    #pred_neigh_paths = {}
    for scene_id, paths in scenes:
        #print("Scene ID: ", scene_id)
        for dataset_file in dataset_files:
            name = dataset_file.split('/')[-2]
            scenes_pred = reader_list[name].scenes(ids=[scene_id])
            for scene_id, preds in scenes_pred:
                predicted_paths = [[t for t in pred if t.scene_id == scene_id] for pred in preds]
                predicted_paths_ = np.zeros((ped_num, 12, 2)) 
                Disturbance_x = random.random() 
                Disturbance_y = Disturbance_x#random.random() 
                for i  in range(ped_num):
                    for j in range(12):
                        #Disturbance_x = random.random() 
                        # print("Disturbance_x  ",Disturbance_x)
                        Disturbance_x=(Disturbance_x-0.5)* cp[j]/1.414
                        #Disturbance_y = random.random() 
                        #print("Disturbance_y  ",Disturbance_y)
                        Disturbance_y=(Disturbance_y-0.5)* cp[j]/1.414
                        predicted_paths_[i][j][0]=predicted_paths[i][j].x#+Disturbance_x
                        predicted_paths_[i][j][1]=predicted_paths[i][j].y#+Disturbance_y
                # for i  in range(ped_num):
                #     for j in range(12):
                #         Disturbance_x = random.random() 
                #         #print("Disturbance_x  ",Disturbance_x)
                #         Disturbance_x=Disturbance_x* cp[j]/1.414
                #         Disturbance_y = random.random() 
                #         #print("Disturbance_y  ",Disturbance_y)
                #         Disturbance_y=Disturbance_y* cp[j]/1.414
                #         predicted_paths[i][j].x=predicted_paths[i][j].x+Disturbance_x
                #         predicted_paths[i][j].y=predicted_paths[i][j].y+Disturbance_y
            #pred_paths[label_dict[name]] = predicted_paths
            predicted_paths_all.append(predicted_paths)
            predicted_paths_all_.append(predicted_paths_)
    

    


    rospy.Subscriber("actor1_odom", Odometry, actor1_odom_callback, queue_size=10) 
    rospy.Subscriber("/odom_all", Float32MultiArray, odom_all_callback, queue_size=10)
    # 
    rate = rospy.Rate(10)  
    #__timer_pub_vis=rospy.Timer(rospy.Duration(0.05), cylinder_marker_publisher())
    while not rospy.is_shutdown():
        #print("123")
        gt_path_vis(time_index)
        cylinder_marker_publisher(time_index)
        all_ob_state_pub(time_index)
        #rospy.spin()  
        rate.sleep()  

    #rospy.spin()