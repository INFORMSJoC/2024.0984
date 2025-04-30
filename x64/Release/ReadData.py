import os
import xlwt
# 设置Excel编码
file = xlwt.Workbook('encoding = utf-8')
#设置样式
Style = xlwt.XFStyle() 

# 设置单元格对齐方式
alignment = xlwt.Alignment()
# 0x01(左端对齐)、0x02(水平方向上居中对齐)、0x03(右端对齐)
alignment.horz = 0x02
# 0x00(上端对齐)、 0x01(垂直方向上居中对齐)、0x02(底端对齐)
alignment.vert = 0x01

Style.alignment = alignment 

# 为样式创建字体
font = xlwt.Font()
# 字体类型
font.name = 'Times New Roman'

Style.font = font

# 读取文件写入excel
filePath = './Results/'
path = os.listdir(filePath)

for p in path:
    file_name = filePath + p
    with open(file_name) as f:
        content = f.readlines()
        f.close() 
    n = int(len(content)/3)
    rows = n
    instances =[]
    
    if p[0] == 'g' or p[0] == 'M':#ga/gs系列算例或#M系列算例
        #统计和
        sum_totalruntime = 0
        sum_rootruntime = 0
        sum_nodes = 0
        sum_rootgap = 0
        sum_gap = 0
        for i in range(n):
            tmp = content[3*i].strip().split(':')
            name = tmp[1].strip()[:-1]
            
            tmp = content[3*i+1].strip().split(';')
            rootbound = float(tmp[1].strip().split(':')[1])
            rootgap = float(tmp[2].strip().split(':')[1])
            rootruntime = float(tmp[3].strip().split(':')[1])
            
            tmp = content[3*i+2].strip().split(';')  
            objvalue = float(tmp[0].strip().split(':')[1])
            gap = float(tmp[1].strip().split(':')[1])
            totalruntime = float(tmp[2].strip().split(':')[1])       
            nodes = int(tmp[3].strip().split(':')[1])
            status = int(tmp[4].strip().split(':')[1])
            
            instances.append([name, objvalue, totalruntime, rootbound, rootruntime, rootgap, nodes, gap, status]) 
        
            #统计均值
            sum_totalruntime += totalruntime
            sum_rootruntime += rootruntime
            sum_nodes += nodes
            sum_rootgap += rootgap
            sum_gap += gap
            if (i+1)%5 == 0:
                instances.append(['Mean', ' ', round(sum_totalruntime/5,4), ' ', round(sum_rootruntime/5,4), round(sum_rootgap/5,4), round(sum_nodes/5,1), round(sum_gap/5,4), ' '])
                sum_totalruntime = 0
                sum_rootruntime = 0
                sum_nodes = 0
                sum_rootgap = 0
                sum_gap = 0
                rows += 1
    else:
        #统计和
        sum_totalruntime = 0
        sum_rootruntime = 0
        sum_nodes = 0
        sum_rootgap = 0
        sum_gap = 0
        for i in range(n):
            tmp = content[3*i].strip().split(':')
            name = tmp[1].strip()[:-1]
            
            tmp = content[3*i+1].strip().split(';')
            rootbound = float(tmp[1].strip().split(':')[1])
            rootgap = float(tmp[2].strip().split(':')[1])
            rootruntime = float(tmp[3].strip().split(':')[1])
            
            tmp = content[3*i+2].strip().split(';')  
            objvalue = float(tmp[0].strip().split(':')[1])
            gap = float(tmp[1].strip().split(':')[1])
            totalruntime = float(tmp[2].strip().split(':')[1])       
            nodes = int(tmp[3].strip().split(':')[1])
            status = int(tmp[4].strip().split(':')[1])
            
            instances.append([name, objvalue, totalruntime, rootbound, rootruntime, rootgap, nodes, gap, status]) 
        
            #统计均值
            sum_totalruntime += totalruntime
            sum_rootruntime += rootruntime
            sum_nodes += nodes
            sum_rootgap += rootgap
            sum_gap += gap
        #将平均数据添加至表格最后一行    
        instances.append(['Mean', ' ', round(sum_totalruntime/n,4), ' ', round(sum_rootruntime/n,4), round(sum_rootgap/n,4), round(sum_nodes/n,1), round(sum_gap/n,4), ' ']) 
        rows += 1
    #Excel头部
    head = ['name', 'objvalue', 'totalruntime', 'rootbound', 'rootruntime', 'rootgap', 'nodes', 'gap', 'status']    
    # 创建sheet工作表
    sheet1 = file.add_sheet(p[0:-4],cell_overwrite_ok=True)
    # 先填标题 
    # sheet1.write(a,b,c，d) 函数中参数a、b、c,d分别对应行数、列数、单元格内容,单元格样式
    for i in range(len(head)):
        sheet1.write(0, i, head[i], Style) # 第0行第i列

    # 循环填入数据
    for i in range(rows):
        for j in  range(len(instances[i])):
            sheet1.write(i + 1, j, instances[i][j], Style)# 第i + 1行第j列
        
    # 保存Excel到.py源文件同级目录
    file.save('Result.xls')

#新建文件夹
import shutil
file_path = './Results_/'
if os.path.exists(file_path):
    print('true')
    #os.rmdir(file_dir)
    shutil.rmtree(file_path)#删除再建立
    os.makedirs(file_path)
else:
    os.makedirs(file_path)

#原始文件拷贝至新建文件夹
from shutil import copy #shutil 是用来复制黏贴文件的
 
# 获取 file_path 下的文件和文件夹列表
# 因为 file_path 里面没有文件夹，所以不处理有文件夹的情况
pathDir = os.listdir(filePath) #os.listdir(file_path) 是获取指定路径下包含的文件或文件夹列表
for filename in pathDir: #遍历pathDir下的所有文件filename
	print(filename)
	from_path = os.path.join(filePath, filename) #旧文件的绝对路径(包含文件的后缀名)
	copy(from_path, file_path)#完成复制黏贴
    
#获取所有文件路径    
path = os.listdir(file_path)
#重命名文件名
for p in path:
    #设置旧文件名（就是路径+文件名）
    oldname=file_path + p   # os.sep添加系统分隔符
    temp = p.split('_')   
    p = temp[2][0:-4]+'_'+temp[1]+'_'+temp[0]+'.txt'
    #设置新文件名
    newname = file_path + p    
    os.rename(oldname,newname)   #用os模块中的rename方法对文件改名
    print(oldname,'======>',newname)