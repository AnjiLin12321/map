1 cmake 升级到3.24.2
cmake 3.24.2 容易安装
2 随后 pip uninstall casadi 和pip install casadi
casadi可用
3 注意 cmake 卸载重装新版本后 可能需要重新安装ros
以及【putn】
sudo apt-get install ros-melodic-ompl
sudo apt-get install ros-melodic-robot-state-publisher*
sudo apt-get install ros-melodic-joint-state-controller*
sudo apt-get install ros-melodic-controller*
sudo apt-get install ros-melodic-velocity-controllers*
sudo apt-get install ros-melodic-eigen*
sudo apt-get install ros-melodic-velodyne*
pip install casadi

4 mpcc=mpc加入动态避障  点在圆外
1）注意以下代码不能正确运行。表现为mpc规划的state 会偏离起始位置或者参考路径，小车乱跳
delta_X=ca.vertcat(x_p[:2]-obj[:2]).T
Mat_ellipse=np.mat([[1/a**2,0],[0,1/a**2]])
m=ca.mtimes(ca.mtimes(delta_X,Mat_ellipse),delta_X.T)
2）正确代码：
a = obj[2]+safe_distance
h_i=ca.norm_2(x_p[:2]-obj[:2])-(a)**2
return h_i
3）待提升：MPC会生成不同于参考路径的方案，比如从左绕变为从右绕
MPC加入避障后效果可能更差，尤其是safe_dis或者car_r加大时
当前策略：safe_dis和car_r 很小 近似于减弱mpc中动态避障能力≈没有
效果好，且能够说明，加入动态避障时可凸的/可行的


5 cbf比较   具体视频可见2024年6月25日组会视频
3dA*+一般mpc效果已很好
    global_planner.cpp 运行 AStar_planner  
    local:local_planner.py  mpc_ob.py 内部可关闭考虑动态障碍物   #### for mpcc

a+cbf（自己写）
     global_planner.cpp 运行 AStar_planner   
     local:local_planner.py  mpc_ob.py    #### for cbf   一坨狗屎

jianzhuZHU 原版代码已开源

cbf 借鉴版本 :  a+cbf
     global_planner.cpp 运行 AStar_planner   
     local:local_planner_cbf.py  mpc_cbf.py

