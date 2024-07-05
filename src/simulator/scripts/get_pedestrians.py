import math

#   parameter modification
ped_num=5
ped_start_index=0
scene_start_index=0
#   parameter modification end


def write_file(filename="",ped=[],len_max=0):
    x_value = 0
    y_value = 0
    omega_value=0 
    delta_time=0.5
    time=0
    i=0
    with open(filename, 'w') as file: 
        for i in range (len(ped)-1):
            time=i*delta_time
            x_value = ped[i][0]
            y_value = ped[i][1] 
            omega_value=math.atan2(ped[i+1][1]-ped[i][1],ped[i+1][0]-ped[i][0])
    
    
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
            xml_fragment = f'''  
                <waypoint>  
                    <time>{time}</time>  
                    <pose>{x_value} {y_value} 0.000000 0.000000 0.000000 {omega_value}</pose>  
                </waypoint>
            '''
            file.write(xml_fragment)


#episodes=1
#tag=False

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
    
# it is useful when the ped_lists[i] has different length
len_max=0
len_max = max(len(ped_list) for ped_list in ped_lists)

print(len_max)

for i in range(ped_num):  
    write_file(f'/home/linanji/src/map/src/simulator/scripts/waypoint_{i}.txt', ped_lists[i], len_max)
