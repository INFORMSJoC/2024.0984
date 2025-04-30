import os
import xlwt
#所有的gap都是百分数
#所有的时间保留两位小数


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
filePath = './Results_/'
path = os.listdir(filePath)

path1 = []
path2 = []
for p in path:
    if p[-10:-8] =='_g' or p[-5:-3] =='M.':
        path2.append(p)
    else:
        path1.append(p)
        
for i in range(len(path1)):
    if path1[i][3] =='K':
        temp = path1[i-10]
        path1[i-10] = path1[i]
        path1[i] = temp
        
for i in range(len(path2)):
    if path2[i][3] =='K':
        temp = path2[i-7]
        path2[i-7] = path2[i]
        path2[i] = temp
        
path = path1 + path2
Path = [path[0:50], path[50:120], path[120:170], path[170:219]]
rows = 0
instances =[]
statusCode = set()#统计求解状态
margin = 2 #excel留白
# 创建sheet工作表
sheet1 = file.add_sheet('Mean',cell_overwrite_ok=True)
for number in range(len(Path)):
    path = Path[number]
    for p in path:
        file_name = filePath + p
        with open(file_name) as f:
            content = f.readlines()
            f.close() 
        n = int(len(content)/3)    
        
        
        if p[-9] == 'g' and p[-10] == '_':#ga/gs系列算例      
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
                if i%5 == 0: 
                    num = 5
                    num_ErrorSolved = 0
                    num_TimeLimit = 0 
                    
                    #统计和
                    sum_totalruntime = 0
                    sum_rootruntime = 0
                    sum_nodes = 0
                    sum_rootgap = 0
                    sum_gap = 0
                    #记录求解状态
                    num_ErrorSolved = 0
                    num_TimeLimit = 0
                    sum_timelimit_gap = 0
                    
                #统计求解状态
                if status not in statusCode:
                    statusCode.add(status)
                if status == 107:
                    num_TimeLimit +=1
                    sum_timelimit_gap += gap
                if status == 110 or status == 109:
                    num_ErrorSolved +=1                     
            
                #统计求解出来的算法平均值
                if status != 110 and status != 109:
                    sum_totalruntime += totalruntime
                    sum_rootruntime += rootruntime
                    sum_nodes += nodes
                    sum_rootgap += rootgap
                    sum_gap += gap
                
                STATUS = str(num - num_ErrorSolved-num_TimeLimit) +'/'+str(num - num_ErrorSolved) +'/'+str(num)   
                #instances.append([name, objvalue, totalruntime, rootbound, rootruntime, rootgap, nodes, gap, status])
                
                if (i+1)%5 == 0:
                    # if num_TimeLimit > 0:
                    #     instances.append([p[0:-4]+name[5], round(sum_timelimit_gap/num_TimeLimit*100,2), round(sum_totalruntime/(num - num_ErrorSolved),2), round(sum_rootruntime/(num - num_ErrorSolved),2), round(sum_rootgap/(num - num_ErrorSolved)*100,2), round(sum_nodes/(num - num_ErrorSolved),1), round(sum_gap/(num - num_ErrorSolved)*100,2), STATUS])
                    # else:
                    #     instances.append([p[0:-4]+name[5], ' ', round(sum_totalruntime/(num - num_ErrorSolved),2), round(sum_rootruntime/(num - num_ErrorSolved),2), round(sum_rootgap/(num - num_ErrorSolved)*100,2), round(sum_nodes/(num - num_ErrorSolved),1), round(sum_gap/(num - num_ErrorSolved)*100,2), STATUS])
                    if num_TimeLimit > 0:
                        instances.append([p[0:-4]+name[5], round(sum_rootgap/(num - num_ErrorSolved)*100,2),round(sum_timelimit_gap/num_TimeLimit*100,2), round(sum_rootruntime/(num - num_ErrorSolved),2), round(sum_totalruntime/(num - num_ErrorSolved),2), round(sum_nodes/(num - num_ErrorSolved),1), STATUS])
                    else:
                        instances.append([p[0:-4]+name[5], round(sum_rootgap/(num - num_ErrorSolved)*100,2), '--', round(sum_rootruntime/(num - num_ErrorSolved),2), round(sum_totalruntime/(num - num_ErrorSolved),2),   round(sum_nodes/(num - num_ErrorSolved),1),  STATUS])
                 
                    rows += 1
                    
        elif p[-5] == 'M' :#M系列算例
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
                if i%5 == 0: 
                    num = 5
                    #统计和
                    sum_totalruntime = 0
                    sum_rootruntime = 0
                    sum_nodes = 0
                    sum_rootgap = 0
                    sum_gap = 0
                    #记录求解状态
                    num_ErrorSolved = 0
                    num_TimeLimit = 0
                    sum_timelimit_gap = 0
    
                if name[0:3] == 'MS1' or name[0:3] == 'MT1':
                    num = 1
                    #统计和
                    sum_totalruntime = 0
                    sum_rootruntime = 0
                    sum_nodes = 0
                    sum_rootgap = 0
                    sum_gap = 0
                    #记录求解状态
                    num_ErrorSolved = 0
                    num_TimeLimit = 0
                    sum_timelimit_gap = 0
    
                #统计求解状态
                if status not in statusCode:
                    statusCode.add(status)
                if status == 107:
                    num_TimeLimit +=1
                    sum_timelimit_gap += gap
                if status == 110 or status == 109:
                    num_ErrorSolved +=1                   
    
                #统计求解出来的算法平均值
                if status != 110 and status != 109:
                    sum_totalruntime += totalruntime
                    sum_rootruntime += rootruntime
                    sum_nodes += nodes
                    sum_rootgap += rootgap
                    sum_gap += gap
                    
                STATUS = str(num - num_ErrorSolved-num_TimeLimit) +'/'+str(num - num_ErrorSolved) +'/'+str(num)   
                #instances.append([name, objvalue, totalruntime, rootbound, rootruntime, rootgap, nodes, gap, status])
                if (i+1)%5 == 0 or name[0:3] == 'MS1' or name[0:3] == 'MT1':
                    # if num_TimeLimit > 0:
                    #     instances.append([p[0:-4]+name[1:2], round(sum_timelimit_gap/num_TimeLimit*100,2), round(sum_totalruntime/(num - num_ErrorSolved),2), round(sum_rootruntime/(num - num_ErrorSolved),2), round(sum_rootgap/(num - num_ErrorSolved)*100,2), round(sum_nodes/(num - num_ErrorSolved),1), round(sum_gap/(num - num_ErrorSolved)*100,2), STATUS])
                    # else:
                    #     instances.append([p[0:-4]+name[1:2], ' ', round(sum_totalruntime/(num - num_ErrorSolved),2), round(sum_rootruntime/(num - num_ErrorSolved),2), round(sum_rootgap/(num - num_ErrorSolved)*100,2), round(sum_nodes/(num - num_ErrorSolved),1), round(sum_gap/(num - num_ErrorSolved)*100,2), STATUS])
                    if num_TimeLimit > 0:
                        instances.append([p[0:-4]+name[1:2], round(sum_rootgap/(num - num_ErrorSolved)*100,2),round(sum_timelimit_gap/num_TimeLimit*100,2), round(sum_rootruntime/(num - num_ErrorSolved),2), round(sum_totalruntime/(num - num_ErrorSolved),2), round(sum_nodes/(num - num_ErrorSolved),1), STATUS])
                    else:
                        instances.append([p[0:-4]+name[1:2], round(sum_rootgap/(num - num_ErrorSolved)*100,2), '--', round(sum_rootruntime/(num - num_ErrorSolved),2), round(sum_totalruntime/(num - num_ErrorSolved),2),   round(sum_nodes/(num - num_ErrorSolved),1),  STATUS])
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
            #记录求解状态
            num_ErrorSolved = 0
            num_TimeLimit = 0
            sum_timelimit_gap = 0
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
                #统计求解状态
                if status not in statusCode:
                    statusCode.add(status)
                if status == 107:
                    num_TimeLimit +=1
                    sum_timelimit_gap += gap
                if status == 110 or status == 109:
                    num_ErrorSolved +=1    
                            
                #统计求解出来的算法平均值
                if status != 110 and status != 109:
                    sum_totalruntime += totalruntime
                    sum_rootruntime += rootruntime
                    sum_nodes += nodes
                    sum_rootgap += rootgap
                    sum_gap += gap
                    
                STATUS = str(n - num_ErrorSolved-num_TimeLimit) +'/'+str(n - num_ErrorSolved) +'/'+str(n)   
                #instances.append([name, objvalue, totalruntime, rootbound, rootruntime, rootgap, nodes, gap, status])
            #将平均数据添加至表格最后一行
            # if num_TimeLimit > 0:
            #     instances.append([p[0:-4],  round(sum_timelimit_gap/num_TimeLimit*100,2), round(sum_totalruntime/(n - num_ErrorSolved),2), round(sum_rootruntime/(n - num_ErrorSolved),2), round(sum_rootgap/(n - num_ErrorSolved)*100,2), round(sum_nodes/(n - num_ErrorSolved),1), round(sum_gap/(n - num_ErrorSolved)*100,2), STATUS]) 
            # else:
            #     instances.append([p[0:-4],  ' ', round(sum_totalruntime/(n - num_ErrorSolved),2), round(sum_rootruntime/(n - num_ErrorSolved),2), round(sum_rootgap/(n - num_ErrorSolved)*100,2), round(sum_nodes/(n - num_ErrorSolved),1), round(sum_gap/(n - num_ErrorSolved)*100,2), STATUS])
            if num_TimeLimit > 0:
                instances.append([p[0:-4],  round(sum_rootgap/(n - num_ErrorSolved)*100,2),round(sum_timelimit_gap/num_TimeLimit*100,2), round(sum_rootruntime/(n - num_ErrorSolved),2), round(sum_totalruntime/(n - num_ErrorSolved),2), round(sum_nodes/(n - num_ErrorSolved),1), STATUS]) 
            else:
                instances.append([p[0:-4],  round(sum_rootgap/(n - num_ErrorSolved)*100,2), '--', round(sum_rootruntime/(n - num_ErrorSolved),2), round(sum_totalruntime/(n - num_ErrorSolved),2),   round(sum_nodes/(n - num_ErrorSolved),1),  STATUS])
          
            rows += 1
    #Excel头部
    
    # head = ['name', 'timelimit_gap', 'totalruntime', 'rootruntime', 'rootgap', 'nodes', 'gap', 'status']
    head = ['Group',  'g_r[%]','g[%]', 't_r[s]', 't[s]', '#nodes', 'status']    
    
    # 先填标题 
    # sheet1.write(a,b,c，d) 函数中参数a、b、c,d分别对应行数、列数、单元格内容,单元格样式
    for i in range(len(head)):
        sheet1.write(margin+0, margin+number*9+i, head[i], Style) # 第0行第i列
    
    # 循环填入数据
    for i in range(rows):
        for j in  range(len(instances[i])):
            sheet1.write(margin+i + 1, margin+number*9+j, instances[i][j], Style)# 第i + 1行第j列
    #归零
    rows = 0
    instances =[]
    statusCode = set()#统计求解状态
    

# 打开已有的Excel文件
import xlrd

existing_excel_file = './fischetti.xls'
workbook = xlrd.open_workbook(existing_excel_file)
sheet2 = workbook.sheet_by_index(0)

# 读取已有数据
existing_data = []
for row in range(sheet2.nrows):
    existing_data.append(sheet2.row_values(row))    
for i in range(len(existing_data)):
    for j in range(len(existing_data[0])):
        entry = existing_data[i][j]
        if i>0 and j in [1,3,4]:
            entry = format(entry, '.2f')
        sheet1.write(margin+i + 24, margin+4*9+j, entry, Style)# 第i + 1行第j列
# 保存Excel到.py源文件同级目录
file.save('ResultAnalysis.xls')