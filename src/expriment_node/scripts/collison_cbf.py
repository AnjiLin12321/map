import pandas as pd  
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np



# 假设你的文件名为'data.txt'，并且数据是用空格分隔的  
filename = '/home/linanji/src/map/result/5/cbf_5_collison.txt'
  
parts = []  
current_part = []  
resolution = 36 
 
n_cur_val=[]
# 读取文件  
with open(filename, 'r') as file:  
    for line in file:  
        line = line.strip()  # 去除行尾的换行符和可能的空白字符  
        if line.startswith('n_cur:'):  
            # 如果当前部分不为空，则将其添加到parts列表中 
            metadata_values = line.split('n_cur:', 1)[1].strip().split()  
            n_cur= float(metadata_values[0]) if metadata_values[0].replace('.', '', 1).isdigit() else metadata_values[0]
            #global n_cur_val
            n_cur_val.append(n_cur)
            if current_part:  
                parts.append(current_part)  
            # 重置当前部分为空列表，准备存储新部分的数据  
            current_part = []  
            # 可以选择性地跳过'n_cur:'这一行，或者将其解析为元数据  
            # 这里我们只是简单地跳过了它  
            continue  
        # 如果行不是'n_cur:'开头，则将其分割为浮点数并添加到当前部分  
        if line:  # 确保行不为空（避免处理空行）  
            floats = [float(x) for x in line.split()]  
            current_part.append(floats)  
  
# 不要忘记将最后一个部分也添加到parts列表中（如果有的话）  
if current_part:  
    parts.append(current_part)  
  
# 现在parts包含了所有由'n_cur:'分隔的数据部分  
# 你可以根据需要进一步处理这些数据  
  
# 打印结果以验证  
for i, part in enumerate(parts):  
    # if(i<=49):
    #     continue
    #if i!=5   and i!=10  and i!=20  and i!=22  and i!=24  and i!=30  
    if i!=34  and i!=36  and i!=42:   
    # 5 10  20 22 24 30
    #   34  36  42
    # 42!!
        continue
    print(f"Part {i}:")  
    print(f"n_cur {n_cur_val[i]}:")
    x=[]
    y=[]
    t=[]
    j=0
    ped_num=5
    ped_x = [[] for _ in range(ped_num)] 
    ped_y = [[] for _ in range(ped_num)] 
    
    fig = plt.figure()
    
    r_robot=0.4
    r_ped=0.25

    # 创建一个3D绘图区域
    ax = fig.add_subplot(111, projection='3d')
    times=0
    for row in part:  
        #print(row)  
        x_car=row[0]
        y_car=row[1]
        t_now=j*0.2
        x.append(x_car)
        y.append(y_car)
        t.append(t_now)
        j+=1
        
        for k in range(ped_num):
            ped_x_i=row[2+k*2]
            ped_y_i=row[3+k*2]
            ped_x[k].append(ped_x_i)
            ped_y[k].append(ped_y_i)
            
            if((x_car-ped_x_i)*(x_car-ped_x_i)+(y_car-ped_y_i)*(y_car-ped_y_i)<(r_robot+r_ped)*(r_robot+r_ped)):
                if times==0:
                    plt.plot((x_car+ped_x_i)/2, (y_car+ped_y_i)/2,t_now, 'ro',label='collision')  
                else:
                    plt.plot((x_car+ped_x_i)/2, (y_car+ped_y_i)/2,t_now, 'ro')  
                times+=1
    #ax.plot(x, y, t, label='3D curve')
    for i in range(len(t)):  
            # 生成圆上的点（在x-y平面上）  
            theta = np.linspace(0, 2 * np.pi, resolution)  
            x_r=x[i]+ r_robot * np.cos(theta)
            y_r=y[i]+ r_robot * np.sin(theta)  
      
            # 绘制圆的轮廓线  
            if i==0:
                ax.plot(x_r, y_r, np.full_like(x_r, t[i]), 'g-',alpha=0.2,label='robot trajectory')  # 使用蓝色线条  
            else:
                ax.plot(x_r, y_r, np.full_like(x_r, t[i]), 'g-',alpha=0.2)  # 使用蓝色线条  
            
    #ax.scatter(x, y, t, c='g', marker='o', s=s_robot,alpha=0.1)  
    for k in range(ped_num):
        #ax.plot(ped_x[k], ped_y[k], t,c='b', marker='o', s=10)
        
        #x1=np.full(resolution, x[0])  
        for i in range(len(t)):  
            # 生成圆上的点（在x-y平面上）  
            theta = np.linspace(0, 2 * np.pi, resolution)  
            x =ped_x[k][i]+ r_ped * np.cos(theta)
            y = ped_y[k][i]+ r_ped * np.sin(theta)  
      
            # 绘制圆的轮廓线 
            if (i==0)&(k==0):
                ax.plot(x, y, np.full_like(x, t[i]), 'b-',alpha=0.2,label='pedestrian trajectory')  # 使用蓝色线条  
            else:
                ax.plot(x, y, np.full_like(x, t[i]), 'b-',alpha=0.2)  # 使用蓝色线条     



        #ax.scatter(ped_x[k], ped_y[k], t, c='b', marker='o', s=s_ped,alpha=0.1)  

    
    # 设置标签
    ax.set_xlabel('X axis/m')
    ax.set_ylabel('Y axis/m')
    ax.set_zlabel('Time axis/s')
    
    # 显示图例
    ax.legend()
    
    # 展示图像
    plt.show()
    print()  # 打印空行以分隔不同的部分