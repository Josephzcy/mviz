# -*- coding: utf-8 -*- 
import matplotlib.pyplot as plt
from numpy import *
import os

sum_cpu_avg=0
sum_cpu_max=0

def getsysinfolist(flag):
    if flag == 1:
        filelist = os.listdir(folderpath)
        for i in filelist:
            sysinfopath = folderpath + "/" + i + "/test_data/sysinfo.txt"
            filelist[filelist.index(i)] = sysinfopath
    elif flag == 2:
        filelist = []
        sysinfopath = folderpath + "/test_data/sysinfo.txt"
        filelist.append(sysinfopath)
    return filelist


def combineSysinfoList(keyword):
    filelist = getsysinfolist(flag)
    combinelist = []
    max_dict={}
    for filepath in filelist:
        if (not os.path.exists(filepath)):
            # print("未找到该文件："+filepath)
            filelist.remove(filepath)
        else:
            temlist = getcpulist(keyword,filepath)
            print(temlist)
            if temlist:
                maxstr = max(temlist)
                max_dict[filepath]=maxstr
                combinelist.extend(temlist)
            else:
                maxstr = None
    sum = 0
    for i in max_dict.values():
        if max(combinelist) < 15:
            break
        elif max(combinelist)==i:
            print("当前%s"%(keyword)+"最大值:%d"%(i)+"所在的文件夹:"+get_key(i,max_dict))
    
    sum = 0
    for i in combinelist:
        if i >10:
            sum=sum+1
        else:
            continue
    print("当前%s"%(keyword)+"占用大于10的总数:%d"%(sum))
    return combinelist

def get_key(val,my_dict={}):
    for key, value in my_dict.items():
        if val == value:
            return key

def getcpulist(keyword,filepath):
    list = [];
    f = open(filepath,encoding='utf-8')  # 返回一个文件对象
    line = f.readline()  # 调用文件的 readline()方法，一次读取一行
    # print(line)
    while line:
        if keyword in line:
            # print(keyword)
            line = line.split(keyword, 1)[1]
            if line[0] == ":":
                line = line.split(":", 1)[1]
            elif sys_info_flag==1:
                line = line.split("cpu:", 1)[1]
                line = line.split("mem", 1)[0]
                line = line.split(" ", 1)[0]
            elif sys_info_flag==2:
                line = line.split("cpu:", 1)[1]
                line = line.split("mem", 1)[1]
                line = line.split(":", 1)[1]
                line = line.split("\n", 1)[0]
            #tmp = line.split('=', 1)[1]
            #print(line)
            tmp = line
            #tmp = tmp.lstrip()
            #print(tmp)
            #print(tmp.split(' ', 1)[0] , int(float(tmp.split(' ', 1)[0]))) # 后面跟 ',' 将忽略换行符
            if keyword in "idle":
                tmp = tmp.split(" ", 1)[0]
                print(tmp)
                list.append(100-int(tmp))
            else:
                list.append(float(tmp))
            #print(int(float(tmp.split(' ', 1)[0])))  # 后面跟 ',' 将忽略换行符


        line = f.readline()
    f.close()
    return list


def drawcpulist_designs(keyword_list):
    # for keyword in keyword_list:
    #     # print("开始绘制:"+keyword)
    #     finallist = combineSysinfoList(keyword)
    #     if finallist == []:
    #         return 0
    #     else:
    #         plt.figure(2)
    #         avg = int(float(mean(finallist)))
    #         strmax = max(finallist)
    #         if keyword in "idle":
    #             title = "A72 cpu(100-idle)" + "  avg = " + str(avg) + "  max = " + str(strmax)
    #         else:
    #             title =  keyword + "  avg = " + str(avg) + "  max = " + str(strmax)
    #
    #         plt.subplots_adjust(left=0.04,bottom=0.04,right=0.96,top=0.96,wspace=0.30,hspace=0.4)
    #         plt.subplot(4,4,keyword_list.index(keyword)+1)
    #         # plt.subplot(3, 4, new_keyword_list.index(keyword) + 1)
    #
    #         if keyword in "GPU":
    #             plt.ylim((0,100))
    #
    #         plt.plot(finallist)
    #         plt.title(title)

    fig= plt.figure()

    for keyword in keyword_list:
      print("开始绘制:" + keyword)
      finallist = combineSysinfoList(keyword)
      if finallist == []:
          return 0
      else:
          
        avg = "{:.2f}".format(mean(finallist))
        strmax = max(finallist)
        global sum_cpu_avg
        global sum_cpu_max
        sum_cpu_avg = mean(finallist) + sum_cpu_avg
        sum_cpu_max = strmax + sum_cpu_max

        if keyword in "idle":
            title = "A72 cpu(100-idle)" + "  avg = " + str(avg) + "  max = " + str(strmax)
        else:
            title = keyword + "  avg = " + str(avg) + "  max = " + str(strmax)

        plt.subplots_adjust(left=0.04, bottom=0.04, right=0.96, top=0.9, wspace=0.30, hspace=0.4)
        plt.subplot(6, 4, keyword_list.index(keyword) + 1)

        if keyword in "GPU":
            plt.ylim((0, 100))

        plt.plot(finallist)
        plt.title(title)

    str_sum_avg_max = "{:.2f}".format(sum_cpu_avg)   
    str_sum_cpu_max = "{:.2f}".format(sum_cpu_max)   

    fig.suptitle('cpu info:' + "" + "sum_cpu_avg:" + str_sum_avg_max  + " " + " sum_cpu_max:" + str_sum_cpu_max, fontsize=20)
       
    # plt.show()
def draw_end():
    plt.show()


def getsys_perflist(flag):
    filelist = os.listdir(folderpath)
    del_list = []
    if flag ==3:
        for i in filelist:
            date_path = folderpath+"/"+i
            if (not os.path.isdir(date_path)):
                del_list.append(filelist.index(i))
                # print("第一次删除"+filelist)
            else:
                test_data_path_list = os.listdir(date_path)
                # print(test_data_path_list)
                for j in test_data_path_list:
                    test_data_path = date_path+"/"+j
                    if (not os.path.isdir(test_data_path)) and "test_data" not in test_data_path_list:
                        del_list.append(filelist.index(i))
                        break
                    elif j == "test_data":
                        filelist[filelist.index(i)]= test_data_path
        for i in filelist:
            if (not os.path.isdir(i)):
                del_list.append(filelist.index(i))
            else:
                test_data_list = os.listdir(i)
                sum = 0    
                for j in test_data_list:
                    if "sys_perf" in j:
                        filelist[filelist.index(i)]= i+"/"+j
                    elif "sys_perf" not in j:
                        sum = sum+1
                if sum==len(test_data_list):
                    print(i+"文件系统中不存在sys_perf，请上报给mviz相关人员") 
                    del_list.append(filelist.index(i))
        del_list= list(set(del_list))
        del_list.sort(reverse=True)
        for i in del_list:
            filelist.remove(filelist[i])
    elif flag ==4:
        tmp_list =  os.listdir(folderpath+"/test_data")
        for j in tmp_list:
            if "sys_perf" in j:
                filelist.append(folderpath+"/test_data/"+j)
    return filelist

def combineSys_perfList(keyword):
    filelist = getsys_perflist(flag)
    combinelist = []
    for filepath in filelist:
        # print(filepath)
        if (not os.path.exists(filepath)):
            # print("未找到该文件："+filepath)
            filelist.remove(filepath)
        else:
            temlist = getmpulist(keyword,filepath)
            combinelist.extend(temlist)
    return combinelist


def getmpulist(keyword,filepath):
    # list = [];
    # f = open(filepath,encoding='utf-8')  # 返回一个文件对象
    # line = f.readline()  # 调用文件的 readline()方法，一次读取一行
    # # print(line)
    # last_len_line = 0
    # now_len_line = 0
    # while line:
    #     if keyword in line:
    #         now_len_line = len(line)
    #         if now_len_line<last_len_line:
    #             break
    #         else:
    #             line = line.split(keyword, 1)[1]
    #             tmp = line.split('=', 1)[1]
    #             tmp = tmp.lstrip()
    #             tmp = tmp.split('%', 1)[0]
    #             tmp = tmp.split(' ', 1)[0]
    #             # 修复log被拷贝时，中断文件写入，造成的数据读取异常
    #             if tmp != "" :
    #             # print(tmp.split(' ', 1)[0] , int(float(tmp.split(' ', 1)[0]))) # 后面跟 ',' 将忽略换行符
    #             #print(tmp)
    #             #print(tmp.split(' ', 1)[0] , int(float(tmp.split(' ', 1)[0]))) # 后面跟 ',' 将忽略换行符
    #                 list.append(int(float(tmp)))
    #             #print(int(float(tmp.split(' ', 1)[0])))  # 后面跟 ',' 将忽略换行符
    #             last_len_line = now_len_line
    #     line = f.readline()
    # f.close()
    # print(list)
    list = []
    f = open(filepath, encoding='utf-8')
    lines = f.readlines()
    f.close()

    last_line_index = len(lines) - 1
    for i, line in enumerate(lines):
        if i == last_line_index:
            break  # 跳过最后一行

        if keyword in line:
            line = line.split(keyword, 1)[1]
            tmp = line.split('=', 1)[1]
            tmp = tmp.lstrip()
            tmp = tmp.split('%', 1)[0]
            tmp = tmp.split(' ', 1)[0]
            if tmp != "" or '(null)':
                list.append(int(float(tmp)))
    return list

def draw_mpu_designs(sys_list):
    for keyword in sys_list:
        print("开始绘制:"+keyword)
        finallist = combineSys_perfList(keyword)
        if finallist == []:
            return 0
        else:
            avg = mean(finallist)
            strmax = max(finallist)
            # title = keyword + "    avg = " + str(avg) + "    max = " + str(strmax)
            title = keyword + "    avg = " + str(avg) + "    max = " + str(strmax)

            plt.subplots_adjust(left=0.04,bottom=0.04,right=0.96,top=0.96,wspace=0.30,hspace=0.4)
            plt.subplot(3,4,sys_list.index(keyword)+1)

            if keyword in "GPU":
                plt.ylim((0,100))

            plt.plot(finallist)
            plt.title(title)
# folderpath =r"Z:\\mviz采集数据\\奇瑞X50\\1.0.7.5"
# apalist = ["apa_localization","apa_manager","apa_planning","avm_calib_service","havp_inference","havp_parkingspace_postprocess","bird_eye_view","vehicle_control_dds","wheel_radius_calibration","dds_to_flow","app_camera_encode_libflow.out","avm-parking","bev_libflow.out","object_perception"]
# drawcpulist_designs(apalist)

# draw_end()

if __name__ == "__main__":
   
    # apalist = ["ap_slam","apa_localizatio", "apa_manager", "sensor_fusion", "havp_inference",
    #            "havp_parkingspa", "bird_eye_view", "vehicle_control", "avm-parking","avm_calib_servi","dds_to_flow","object_percepti"]
   
   
    apalist = ["havp_parkingspa", "havp_inference", "bird_eye_view", "vehicle_control",
               "apa_localizatio", "apa_manager", "avm-parking", "sensor_fusion","dds_to_flow",
               "mod","apa_ultrasonic_","avm_manager","app_camera_enco","bev_libflow.out","avm_libflow.out","object_percepti",
               "ap_slam","imu_asensing"
              ]
   
    sys_list = ["mpu1_0", "mcu2_0", "mcu2_1", "c6x_1", "c6x_2", "c7x_1", "GPU", "DDR: READ", "DDR: WRITE", "DDR: TOTAL"]

    #sys_info_flag设置为1时，绘制的为cpu性能数据，设置为2时，绘制的为mem性能数据
    sys_info_flag = 1
    # 提取多个数据中的sysinfo，将flag设置为1，并修改文件路径
    # 提取单个数据中的sysinfo，将flag设置为2，并修改文件路径
    # 提取多个数据中的sysperf，将flag设置为3，并修改文件路径
    # 提取单个数据中的sysperf，将flag设置为4，并修改文件路径
    flag = 2
    if flag == 1:
        # 提取多个路径下的sysinfo
        folderpath =r"/home/minieye/Documents/mviz/mviz_havp_vm_v2.0.9/mviz_data"
        drawcpulist_designs(apalist)
    elif flag == 2:
        # 提取单个路径下的sysinfo
        folderpath =r"/home/wingboy/mviz_pilot/mviz2_ros/mviz_data/20240223162325"
        # folderpath =r"/home/minieye/Documents/T1Q3/mviz_data/20240119103853"
        drawcpulist_designs(apalist)
        # 提取多个路径下的sys_perf文件
    elif flag == 3:
        folderpath =r"/home/minieye/Documents/mviz/mviz_havp_vm_v2.0.9/mviz_data"
        draw_mpu_designs(sys_list)
        # 提取单个路径下的sys_perf文件
    elif flag == 4:
        folderpath =r"/home/minieye/Documents/mviz/mviz_havp_vm_v2.0.7/mviz_data/20240124114059"
        draw_mpu_designs(sys_list)
    else:
        print("请输入正确的flag")
    draw_end()