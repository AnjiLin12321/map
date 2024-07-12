import pandas as pd  
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# 假设你的文件名为'data.txt'，并且数据是用空格分隔的  
filename = '/home/linanji/src/map/result/2/towards/a3d_2_towards_collison1.txt'
  
parts = []  
current_part = []  
  
# 读取文件  
with open(filename, 'r') as file:  
    for line in file:  
        line = line.strip()  # 去除行尾的换行符和可能的空白字符  
        if line.startswith('n_cur:'):  
            # 如果当前部分不为空，则将其添加到parts列表中  
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
    print(f"Part {i+1}:")  
    x=[]
    y=[]
    t=[]
    j=0
    ped_num=2
    ped_x = [[] for _ in range(ped_num)] 
    ped_y = [[] for _ in range(ped_num)] 
    
    fig = plt.figure()
    
    # 创建一个3D绘图区域
    ax = fig.add_subplot(111, projection='3d')
    
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
            if((x_car-ped_x_i)*(x_car-ped_x_i)+(y_car-ped_y_i)*(y_car-ped_y_i)<(0.4+0.25)*(0.4+0.25)):
                plt.plot(x_car, y_car,t_now, 'ro')  

    
    ax.plot(x, y, t, label='3D curve')
    for k in range(ped_num):
        ax.plot(ped_x[k], ped_y[k], t)
    # 设置标签
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    
    # 显示图例
    ax.legend()
    
    # 展示图像
    plt.show()
    print()  # 打印空行以分隔不同的部分