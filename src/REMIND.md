1 调试 
<arg name="obstacle_num" default="4"/>
<arg name="pre_time" default="30"/>
<arg name="deltatime" default="1"/>
<node name="ssc_map" pkg="ssc_map" type="ssc_map" launch-prefix="gnome-terminal --title=ssc_map -x">
            <param name="/obstacle_num"           value="$(arg obstacle_num)"/>
            <param name="/pre_time"           value="$(arg pre_time)"/>
            <param name="/deltatime"           value="$(arg deltatime)"/>
  </node>

 launch-prefix="gnome-terminal --title=ssc_map -x" 不行 
 必须换为  output="screen" required="true"