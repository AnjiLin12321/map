#!/usr/bin/env pythoncd
from collections import defaultdict
import itertools
import json
import random

import numpy as np

from collections import namedtuple

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import Odometry  # 注意这里可能是nav_msgs或者其他的包，取决于你的ROS发行版  
from visualization_msgs.msg import Marker, MarkerArray  
  
TrackRow = namedtuple('Row', ['frame', 'pedestrian', 'x', 'y', 'prediction_number', 'scene_id'])
TrackRow.__new__.__defaults__ = (None, None, None, None, None, None)
SceneRow = namedtuple('Row', ['scene', 'pedestrian', 'start', 'end', 'fps', 'tag'])
SceneRow.__new__.__defaults__ = (None, None, None, None, None, None)

ped_0=[]
ped_1=[]
ped_2=[]
ped_3=[]
ped_4=[]
ped_all=[]

time_index=0
cp=[]

predicted_paths=[]
predicted_paths_all=[]
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
    # 在这里处理odom消息  
   # rospy.loginfo("Position: ({}, {})".format(msg.pose.pose.position.x, msg.pose.pose.position.y))  
    err=1000.0

    for i in range (len(ped_0)):
        global  time_index
        err1=(ped_0[i][0]-msg.pose.pose.position.x)*(ped_0[i][0]-msg.pose.pose.position.x)+(ped_0[i][1]-msg.pose.pose.position.y)*(ped_0[i][1]-msg.pose.pose.position.y)
        if err1<err:
            err=err1
            time_index=i
    #rospy.loginfo("index: ( {})".format(time_index))  

marker_pub = rospy.Publisher('visualization_marker', MarkerArray, queue_size=10) 
marker_gt_pub = rospy.Publisher('gt_marker', MarkerArray, queue_size=10) 

def create_pose_from_point(point):  

    # 这里只是一个简单的示例，你可能需要根据你的实际情况来调整  

    quaternion = Quaternion(0.0, 0.0, 0.0, 1.0)  # 默认方向  

    pose = Pose(point, quaternion)  

    return pose 
def cylinder_marker_publisher(start_index):  
    surtrajs = MarkerArray()  
    # if start_index<8:
    #     marker_pub.publish(surtrajs)
    #     rospy.loginfo("beginning:wait for prediction : {}".format(8-start_index))  
    #     return
    # if start_index>39:
    #     rospy.loginfo("beginning:wait for prediction ")  
    #     return
    # surtrajs = MarkerArray()  

    # for i in range (ped_num):
    #     for j in range (len(cp)):
    #         marker = Marker()  
    #         # 设置Marker的Header  
    #         marker.header.frame_id = "map"  # 使用你的参考坐标系  
    #         marker.header.stamp = rospy.Time.now()  
    
    #         # 设置Marker的ID和类型  
    #         #marker.ns = "my_namespace"  # 命名空间  
    #         marker.id = i*len(cp)+j  # ID必须是唯一的  
    #         marker.type = Marker.CYLINDER  # 设置标记类型为CYLINDER  
    #         marker.action = Marker.ADD  # 添加标记  
    
    #         # 设置Marker的位置和姿态  
    #         marker.pose.position.x = predicted_paths[i][j].x 
    #         marker.pose.position.y = predicted_paths[i][j].y 
    #         marker.pose.position.z = j*0.5 
    #         marker.pose.orientation.x = 0.0  
    #         marker.pose.orientation.y = 0.0  
    #         marker.pose.orientation.z = 0.0  
    #         marker.pose.orientation.w = 1.0  # 四元数表示，这里是单位四元数，表示没有旋转  
    
    #         # 设置Marker的尺度  
    #         marker.scale.x = cp[j]  # 圆柱体的半径  
    #         marker.scale.y =  cp[j] # 圆柱体的高度（但在CYLINDER类型中，这通常被忽略，高度由z轴方向上的scale.z确定）  
    #         marker.scale.z = 0.5 # 圆柱体的高度  
    
    #         # 设置Marker的颜色  
    #         marker.color.a = 1.0  # 不透明度  
    #         marker.color.r = 0.0  # 红色  
    #         marker.color.g = 1.0  # 绿色  
    #         marker.color.b = 0.0  # 蓝色  
    
    #         # 发布Marker消息  
    #         surtrajs.markers.append(marker)  
    # marker_pub.publish(surtrajs)  

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
        # 注意：这里假设ob_data_all[i][4]包含时间戳（秒为单位）  
        traj.header.stamp =rospy.Time.now()  
        pred_path=predicted_paths_all[start_index]
        for j in range(len(cp)):  
            t = (j+1)* 0.5  
            point1 = Point()  
            point1.x = pred_path[i][j].x   
            point1.y =  pred_path[i][j].y  
            point1.z = t  
            traj.points.append(point1)  

            circle_marker = Marker() 
            circle_marker.header.frame_id= "map"  
            circle_marker.action = Marker.ADD  
            circle_marker.id = i *len(cp) + j +ped_num # 确保ID是唯一的  
            circle_marker.type = Marker.SPHERE  # 使用圆形  
            # circle_marker.pose.position.x = predicted_paths[i][j].x 
            # circle_marker.pose.position.y = predicted_paths[i][j].y 
            # circle_marker.pose.position.z = j*0.5 
            # circle_marker.pose.orientation.x = 0.0  
            # circle_marker.pose.orientation.y = 0.0  
            # circle_marker.pose.orientation.z = 0.0 
            # circle_marker.pose.orientation.w = 1.0 
            circle_marker.pose = create_pose_from_point(point1)  # 你需要定义一个函数来从Point创建Pose  
            circle_marker.scale.x = cp[j]  # 设置圆的大小  
            circle_marker.scale.y = cp[j]
            circle_marker.scale.z = 1  
            circle_marker.color.r = 0.0  # 设置颜色  
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
    
        for j in range(start_index,len(ped_0)):  
            t = (j-start_index) * 0.5  
            point1 = Point()  
            point1.x = ped_all[i][j][0]   
            point1.y =  ped_all[i][j][1]  
            point1.z = t  
            traj.points.append(point1)  

            # circle_marker = Marker() 
            # circle_marker.header.frame_id= "map"  
            # circle_marker.action = Marker.ADD  
            # circle_marker.id = i * 1000 + j  # 确保ID是唯一的  
            # circle_marker.type = Marker.SPHERE  # 使用圆形  
            # circle_marker.pose = create_pose_from_point(point1)  # 你需要定义一个函数来从Point创建Pose  
            # circle_marker.scale.x = 0.5  # 设置圆的大小  
            # circle_marker.scale.y = 0.5  
            # circle_marker.scale.z = 0.5  
            # circle_marker.color.r = 0.0  # 设置颜色  
            # circle_marker.color.g = 1.0  
            # circle_marker.color.b = 0.0  
            # circle_marker.color.a = 1.0  
            # surtrajs.markers.append(circle_marker)
        surtrajs.markers.append(traj)  
    
    marker_gt_pub.publish(surtrajs)


if __name__ == '__main__':
    rospy.init_node("ssc_pred")
    # read predestrians truth path
    ped_num=5
    episodes=1
    tag=False
    
    with open('/home/linanji/src/map/src/simulator/scripts/orca_circle_crossing_5ped_1scenes_.txt', 'r') as file:  
        for line in file:  
            elements = line.strip().split(',') 
            time_ind=int(elements[0])
            if(time_ind<8) or (time_ind>49):
                continue
            ped_id=int(elements[1])
            
            if(ped_id==0):
                ped_0.append((float(elements[2]),float(elements[3])))
            if(ped_id==1):
                ped_1.append((float(elements[2]),float(elements[3])))
            if(ped_id==2):
                ped_2.append((float(elements[2]),float(elements[3])))
            if(ped_id==3):
                ped_3.append((float(elements[2]),float(elements[3])))
            if(ped_id==4):
                ped_4.append((float(elements[2]),float(elements[3])))

            if((ped_id)==ped_num*episodes):
                break 
        ped_all.append(ped_0)
        ped_all.append(ped_1)
        ped_all.append(ped_2)
        ped_all.append(ped_3)
        ped_all.append(ped_4)

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
            #pred_paths[label_dict[name]] = predicted_paths
            predicted_paths_all.append(predicted_paths)
    

    ## read cp
    with open('/home/linanji/src/map/src/simulator/scripts/cp.txt', 'r') as f:  
        for line in f:  
            cp.append(float(line.strip()))  
    #print(cp)  


    rospy.Subscriber("actor1_odom", Odometry, actor1_odom_callback, queue_size=10)  
    # 
    rate = rospy.Rate(10)  
    # 保持节点运行，直到手动停止  
    #__timer_pub_vis=rospy.Timer(rospy.Duration(0.05), cylinder_marker_publisher())
    while not rospy.is_shutdown():
        print("123")
        gt_path_vis(time_index)
        cylinder_marker_publisher(time_index)
        #rospy.spin()  
        rate.sleep()  

    #rospy.spin()