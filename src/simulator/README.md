world/
em.world 空白   robot_world.launch  动态障碍物以小车形式加入，接收不同话题，因此需额外发布cmd命令
w1.world 四个行人  robot_world_pedestrain.launch

2 动态行人 接口
get_pedestrians.py 代码作用：从trajnet++获得的 txt轨迹  转换为   waypoint 的txt文件
需手动复制到world 行人轨迹 waypoint
TODO    1 多个行人代码存在重复问题
                2 解决手动复制问题
                3 多个episode同时生成
                4 行人和episode 参数化
                5 四个还是五个行人 

File "get_pedestrians.py", line 22
    '''
      ^
SyntaxError: invalid syntax

conda activate trajnetv

3 参数耦合
global_planner:
deltatime 0.5   和动态障碍物时间间隔相关-----world
sampletime 0.25   global planner t坐标轴分辨率       插值=deltatime/sampletime=2
interpolation_num   5    global_path 插值     间隔时间 sampletime/interpolation_num=0.05

在local_planner：
ob_state_all sample_ind=10  需要保证 deltatime/sample_ind=间隔时间=sampletime/interpolation_num
global path scale  =2    T=间隔时间*scale=0.1