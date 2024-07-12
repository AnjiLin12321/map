import math

#   parameter modification
ped_num=2
ped_start_index=1024
scene_start_index=38188
#   parameter modification end


def write_file(filename="",ped=[],len_max=0):
    x_value = 0
    y_value = 0
    omega_value=0 
    delta_time=0.5
    time=0
    i=0
    with open(filename, 'w') as file: 
        xml_fragment = f'''
            <sdf version='1.6'>
            <world name='default'>
                <light name='sun' type='directional'>
                <cast_shadows>1</cast_shadows>
                <pose frame=''>0 0 10 0 -0 0</pose>
                <diffuse>0.8 0.8 0.8 1</diffuse>
                <specular>0.2 0.2 0.2 1</specular>
                <attenuation>
                    <range>1000</range>
                    <constant>0.9</constant>
                    <linear>0.01</linear>
                    <quadratic>0.001</quadratic>
                </attenuation>
                <direction>-0.5 0.1 -0.9</direction>
                </light>
                <model name='ground_plane'>
                <static>1</static>
                <link name='link'>
                    <collision name='collision'>
                    <geometry>
                        <plane>
                        <normal>0 0 1</normal>
                        <size>100 100</size>
                        </plane>
                    </geometry>
                    <surface>
                        <friction>
                        <ode>
                            <mu>100</mu>
                            <mu2>50</mu2>
                        </ode>
                        <torsional>
                            <ode/>
                        </torsional>
                        </friction>
                        <contact>
                        <ode/>
                        </contact>
                        <bounce/>
                    </surface>
                    <max_contacts>10</max_contacts>
                    </collision>
                    <visual name='visual'>
                    <cast_shadows>0</cast_shadows>
                    <geometry>
                        <plane>
                        <normal>0 0 1</normal>
                        <size>100 100</size>
                        </plane>
                    </geometry>
                    <material>
                        <script>
                        <uri>file://media/materials/scripts/gazebo.material</uri>
                        <name>Gazebo/Grey</name>
                        </script>
                    </material>
                    </visual>
                    <self_collide>0</self_collide>
                    <enable_wind>0</enable_wind>
                    <kinematic>0</kinematic>
                </link>
                </model>
                <gravity>0 0 -9.8</gravity>
                <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
                <atmosphere type='adiabatic'/>
                <physics name='default_physics' default='0' type='ode'>
                <max_step_size>0.001</max_step_size>
                <real_time_factor>1</real_time_factor>
                <real_time_update_rate>1000</real_time_update_rate>
                </physics>
                <scene>
                <ambient>0.4 0.4 0.4 1</ambient>
                <background>0.7 0.7 0.7 1</background>
                <shadows>1</shadows>
                </scene>
                <audio>
                <device>default</device>
                </audio>
                <wind/>
                <spherical_coordinates>
                <surface_model>EARTH_WGS84</surface_model>
                <latitude_deg>0</latitude_deg>
                <longitude_deg>0</longitude_deg>
                <elevation>0</elevation>
                <heading_deg>0</heading_deg>
                </spherical_coordinates>
                <state world_name='default'>
                <sim_time>40 450000000</sim_time>
                <real_time>40 559796255</real_time>
                <wall_time>1712414292 688823464</wall_time>
                <iterations>40450</iterations>
                <model name='ground_plane'>
                    <pose frame=''>0 0 0 0 -0 0</pose>
                    <scale>1 1 1</scale>
                    <link name='link'>
                    <pose frame=''>0 0 0 0 -0 0</pose>
                    <velocity>0 0 0 0 -0 0</velocity>
                    <acceleration>0 0 0 0 -0 0</acceleration>
                    <wrench>0 0 0 0 -0 0</wrench>
                    </link>
                </model>
                <light name='sun'>
                    <pose frame=''>0 0 10 0 -0 0</pose>
                </light>
                </state>
                <gui fullscreen='0'>
                <camera name='user_camera'>
                    <pose frame=''>5 -5 2 0 0.275643 2.35619</pose>
                    <view_controller>orbit</view_controller>
                    <projection_type>perspective</projection_type>
                </camera>
                </gui>
            '''
        file.write(xml_fragment)  
        for j in range (ped_num):

            xml_fragment = f'''  
                <actor name="actor{j+1}">
                    <skin>
                    <filename>walk.dae</filename>
                    <scale>1.0</scale>
                    </skin>
                    <animation name="walking">
                    <filename>walk.dae</filename>
                    <scale>1.000000</scale>
                    <interpolate_x>true</interpolate_x>
                    </animation>
                    <script>
                    <loop>true</loop>
                    <delay_start>0.000000</delay_start>
                    <auto_start>true</auto_start>
                    <trajectory id="0" type="walking">
                '''
            file.write(xml_fragment)  
            for i in range (len(ped[j])-1):
                time=i*delta_time
                x_value = ped[j][i][0]
                y_value = ped[j][i][1] 
                omega_value=math.atan2(ped[j][i+1][1]-ped[j][i][1],ped[j][i+1][0]-ped[j][i][0])
        
        
                xml_fragment = f'''  
                    <waypoint>  
                        <time>{time}</time>  
                        <pose>{x_value} {y_value} 0.000000 0.000000 0.000000 {omega_value}</pose>  
                    </waypoint>
                '''
                file.write(xml_fragment)  
            print(i)
            for i in range(i+1,len_max):
                #print(i)
                time=i*delta_time
                x_value = ped[j][i][0]
                y_value = ped[j][i][1] 
                xml_fragment = f'''  
                    <waypoint>  
                        <time>{time}</time>  
                        <pose>{x_value} {y_value} 0.000000 0.000000 0.000000 {omega_value}</pose>  
                    </waypoint>
                '''
                file.write(xml_fragment)
            
            ### back  back ####
            for i in range (len(ped[j])-1):
                time=(i+len(ped[j]))*delta_time
                x_value = ped[j][len(ped[j])-1-i][0]
                y_value = ped[j][len(ped[j])-1-i][1] 
                omega_value=math.atan2(ped[j][len(ped[j])-2-i][1]-ped[j][len(ped[j])-1-i][1],ped[j][len(ped[j])-2-i][0]-ped[j][len(ped[j])-1-i][0])
        
        
                xml_fragment = f'''  
                    <waypoint>  
                        <time>{time}</time>  
                        <pose>{x_value} {y_value} 0.000000 0.000000 0.000000 {omega_value}</pose>  
                    </waypoint>
                '''
                file.write(xml_fragment)  
            print(i)
            for i in range(i+1,len_max):
                #print(i)
                time=(i+len(ped[j]))*delta_time
                x_value = ped[j][len(ped[j])-1-i][0]
                y_value = ped[j][len(ped[j])-1-i][1] 
                xml_fragment = f'''  
                    <waypoint>  
                        <time>{time}</time>  
                        <pose>{x_value} {y_value} 0.000000 0.000000 0.000000 {omega_value}</pose>  
                    </waypoint>
                '''
                file.write(xml_fragment)
            
            xml_fragment = f'''  
                </trajectory>
                </script>

            </actor>
                '''
            file.write(xml_fragment)  

        xml_fragment = f'''
             </world>
            </sdf>
            '''
        file.write(xml_fragment) 

#episodes=1
#tag=False

ped_lists = [[] for _ in range(ped_num)]  

with open('/home/linanji/src/map/src/simulator/scripts/orca_circle_crossing_2ped_1000scenes_.txt', 'r') as file:  
    for line in file:  

        elements = line.strip().split(',')  
        time_ind=int(elements[0])
        if(time_ind<8+scene_start_index) or (time_ind>49+scene_start_index):
            continue
        ped_id=int(elements[1])
        if 0+ped_start_index <= ped_id < ped_num+ped_start_index:
            ped_lists[ped_id-ped_start_index].append((float(elements[2]), float(elements[3])))  
    
# it is useful when the ped_lists[i] has different length
len_max=0
len_max = max(len(ped_list) for ped_list in ped_lists)

print(len_max)

for i in range(ped_num):  
    write_file(f'/home/linanji/src/map/src/simulator/scripts/waypoint_all.txt', ped_lists, len_max)
