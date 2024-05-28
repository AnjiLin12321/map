1 注意 地图范围 x y T（预测时间 pretime）
如果预测时间内直线行走也走不到终点 则找不到路径


TODO
1 goal point    （rviz）
2 goal 在 范围内不可达
3 resolution pretime deltatime 差值点个数   v_mean
和 local 中 T 匹配 
resolution_x =v_mean*deltatime
deltatime/差值点个数 和T   --->scale  (global_path点)

3 效果：
障碍物设置