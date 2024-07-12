import pandas as pd  
  
# 假设你的文件名为'data.txt'，并且数据是用空格分隔的  
filename = '/home/linanji/src/map/cp0_10.txt'  
  
# 读取文件，跳过无法转换为float的行  
# 注意：这里我们假设所有有效的数据都可以转换为float，无效的用NaN表示  
data = []
collision=0
timeout=0

with open(filename, 'r') as file:  
   for line in file:  
        parts = line.strip().split()  
        if parts == ['-1', '0', '0', '0']: 
            collision+=1
        elif parts == ['0', '-1', '0', '0']: 
            timeout+=1
        else:  # 直接检查行是否不是全为'-1 0 0 0'  
            try:  
                # 尝试将每部分转换为float，忽略无法转换的（这里我们不再特别处理-1）  
                float_parts = [float(part) for part in parts]  
                data.append(float_parts)  
            except ValueError:  
                # 如果转换失败，则跳过这行  
                continue  

n=100-collision-timeout
df = pd.DataFrame(data, columns=['Col1', 'Col2', 'Col3', 'Col4'])  
  
# 计算每列的平均值，忽略NaN值  
averages = df.mean(skipna=True)  
  
# 打印结果  
print(averages)
# 打印结果  
#print(averages)