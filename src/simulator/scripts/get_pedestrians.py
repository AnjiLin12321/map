import math

def write_file(filename="",ped=[],len_max=0):
    x_value = 0
    y_value = 0
    omega_value=0 
    delta_time=0.5
    time=0
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

ped_num=5
episodes=1
tag=False
ped_0=[]
ped_1=[]
ped_2=[]
ped_3=[]
ped_4=[]
with open('orca_circle_crossing_5ped_1scenes_.txt', 'r') as file:  
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
len_max=0
len_max=max(len_max,len(ped_0))
len_max=max(len_max,len(ped_1))
len_max=max(len_max,len(ped_2))
len_max=max(len_max,len(ped_3))
len_max=max(len_max,len(ped_4))
print(len_max)



write_file('waypoint_0.txt',ped_0,len_max)
write_file('waypoint_1.txt',ped_1,len_max)
write_file('waypoint_2.txt',ped_2,len_max)
write_file('waypoint_3.txt',ped_3,len_max)
write_file('waypoint_4.txt',ped_4,len_max)