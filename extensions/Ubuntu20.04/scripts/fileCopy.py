import os
import platform
import sys
import time
import os.path as osp
import json
import paramiko
import subprocess

PRINT_ALL = 0

Cur_Counter = 0
End_Counter = 1
START = time.perf_counter()

t1q_data_map = {
 'avm_calibs':['bev_car_info.json',
                'bev_front.json',
                'bev_rear.json',
                'bev_left.json',
                'bev_right.json',
                'car_info.json',
                'front.json',
                'rear.json',
                'left.json',
                'right.json',
                'front_avmmap.bin',
                'front_avmmap.json',
                'rear_avmmap.bin',
                'rear_avmmap.json',
                'left_avmmap.bin',
                'left_avmmap.json',
                'right_avmmap.bin',
                'right_avmmap.json',
                'avm_dsp_lut.bin'],
 'fault_diagnosis':['diaglogA.json',
                    'diaglogB.json']
}
class mySSH:
    def __init__(self, host='', username='root', port=22, keypath=''):
        self.keypath = keypath
        self.ip = host
        self.port = port
        self.username = username
        self.private_key = ''
        self.connection = None

    def connect(self):
        self.private_key = paramiko.RSAKey.from_private_key_file(self.keypath + "/debug.key")
        self.connection = paramiko.SSHClient()
        self.connection.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            if self.private_key.size > 0:
                self.connection.connect(self.ip, self.port, self.username, pkey=self.private_key, timeout=5.0)
            else:
                try:
                    self.connection.connect(self.ip, self.port, self.username, look_for_keys=False,
                                            allow_agent=False, timeout=5.0)
                except paramiko.ssh_exception.SSHException:
                    self.connection.get_transport().auth_none(self.username)
                    self.connection.exec_command('uname -a')
            if self.connection is not None:
                self.connection.sftp = paramiko.SFTPClient.from_transport(self.connection.get_transport())
        except Exception as e:
            try:
                print(str(e.args))
                self.connection = None
            finally:
                e = None
                del e
    def exec_command(self, command):
        stdin, stdout, stderr = self.connection.exec_command(command)
        return stdout.read().decode()  #block
    def close(self):
        self.connection.close()
        time.sleep(0.1)
                
class EnvPublish(object):
    def __init__(self):
        self.curr_module_path = os.path.dirname(os.path.realpath(__file__))
        self.os_platform = ''

    def init(self):
        self.os_platform = platform.system()  # 获取当前的系统，"Windows/Linux"

    def get_current_dir(self):
        return self.curr_module_path

def _is_exists(path, function):
    path = path.replace('\\', '/')
    try:
        function(path)
    except Exception as error:
        return False
    else:
        return True


def copyCalib(ssh,sftp,remote_path,local_path):
    calibList = ['bev_car_info.json',
                'bev_front.json',
                'bev_rear.json',
                'bev_left.json',
                'bev_right.json',
                'car_info.json',
                'front.json',
                'rear.json',
                'left.json',
                'right.json',
                'front_avmmap.bin'
                'front_avmmap.json'
                'rear_avmmap.bin'
                'rear_avmmap.json'
                'left_avmmap.bin'
                'left_avmmap.json'
                'right_avmmap.bin'
                'right_avmmap.json'
                'avm_dsp_lut.bin']
    
    try:
        # sftp = ssh.open_sftp;
        # 遍历文件夹列表
        for item in calibList:
            #  print('item name :',item)
            remote_file_path = os.path.join(remote_path,item)
            local_file_path = os.path.join(local_path,item)
            sftp.get(remote_file_path,local_file_path)
    
    except Exception as error:
            print(error)
            exit(1)
            return False

def copyRemoteData(ssh,sftp,target_data_name_list,remote_path,local_path):

    if len(target_data_name_list) != len(remote_path) != len(local_path):
        print("para error!!!")
        return
    try:
        # sftp = ssh.open_sftp;
        # 遍历文件夹列表
        for dir_name_index in range(len(target_data_name_list)):
            #  print('item name :',item)

            print("Info: target_name_key = %s" % target_data_name_list[dir_name_index])
            
            dir_files = t1q_data_map[target_data_name_list[dir_name_index]]
            remote_dir_path = remote_path[dir_name_index]
            local_dir_path  = local_path[dir_name_index]

            for file_name in dir_files:
                remote_file_path = os.path.join(remote_dir_path,file_name)
                local_file_path = os.path.join(local_dir_path,file_name)
                sftp.get(remote_file_path,local_file_path)
    
    except Exception as error:
            print(error)
            exit(1)
            return False


if __name__ == '__main__':

    # enviroment_path为当前工作路径
    env = EnvPublish()
    # 当前所在目录：mviz/extensions/Ubuntu20.04/script, 所以要返回mviz目录需要获取2次父目录
    enviroment_path = env.get_current_dir();
    parent_path = os.path.dirname(enviroment_path)
    grandparent_path = os.path.dirname(parent_path)

    # 把工作路径指向mviz目录
    enviroment_path = os.path.dirname(grandparent_path)
    print("\033[0;32;40mInfo: enviroment_path = %s \033[0m" % enviroment_path)

    # data_path为mviz_data目录，即 mviz/mviz_data
    mviz_data_path = os.path.join(enviroment_path,'mviz_data') 
    print("\033[0;32;40mInfo: mviz_data_path = %s \033[0m" % mviz_data_path)


    # 先清除相关进程，使系统性能数据停止录制
    # time.sleep(2)
    print("\033[0;32;40mInfo: next is stopping sys_info and sys_perf node\033[0m")
    killall_receive_sysinfo_cmd = 'killall -9 recv_sysinfo'
    run_sysinfo_cmd_ps = subprocess.Popen(killall_receive_sysinfo_cmd,shell=True)
    run_sysinfo_cmd_ps.wait()
    time.sleep(0.5)

    ssh_handle = mySSH('192.168.98.233', 'root', 22, enviroment_path)
    print("Info: Connecting %s " % '192.168.98.233')
    ssh_handle.connect()
    if ssh_handle.connection is not None:
        ssh_handle.exec_command("killall -9 sys_perf")


   # 用于存储文件名的空列表
    file_list = []
    for filename in os.listdir(mviz_data_path):
        file_list.append(filename)
    file_list.sort()
    target_director_name = file_list.pop()
    target_directory = os.path.join(mviz_data_path,target_director_name)
    target_directory = os.path.join(target_directory,'test_data')
    # 使用 os.makedirs() 方法创建文件夹，如果路径中包含不存在的文件夹则会自动创建
    if os.path.exists(target_directory):
        print("\033[0;32;40mInfo: %s existed \033[0m" % target_directory)
    else:
        os.makedirs(target_directory)
    print("\033[0;32;40mInfo: target_directory = %s \033[0m" % target_directory)

    # todo::copy screen_recording_video
    source_screen_recording_dir = os.path.join(enviroment_path,'screen_recordings')
    print("Info: source_screen_recording_dir:%s",source_screen_recording_dir)
    copy_screen_recording_cmd = "mv " + source_screen_recording_dir + " "+ target_directory
    
    print("\033[0;32;40mInfo: copy_screen_recording_cmd = %s  \033[0m" % copy_screen_recording_cmd)
    copy_screen_recording_cmd_ps = subprocess.Popen(copy_screen_recording_cmd,shell=True)
    copy_screen_recording_cmd_ps.wait()

    print("Info:next copy data ...")
    # 拷贝sysinfo.txt
    sysinfo_src_path = enviroment_path + "/extensions/Ubuntu20.04/scripts/sysinfo.txt"
    cp_sysinfo_cmd = 'mv "%s" "%s"' % (sysinfo_src_path, target_directory)
    status = subprocess.call(cp_sysinfo_cmd, shell=True)
    if status != 0:
        if status < 0:
            print("Killed by signal %s", status)
        else:
            print("Command failed with return code: %s", status)
    else:
        print("Info: Copy sysinfo.txt file success!")


    # 创建标定文件夹
    target_data_name_list = ['avm_calibs',
                             'fault_diagnosis']    
    target_data_local_path_list = []
    for target_data_name in target_data_name_list:
        target_data_dir = os.path.join(target_directory,target_data_name)
        if os.path.exists(target_data_dir):
            print("info: %s has existed" % target_data_dir )
        else:
            os.makedirs(target_data_dir)
        target_data_local_path_list.append(target_data_dir)

    target_data_remote_path_list = [ '/calib/avm_calib/',
                                     '/data/log/'    ]
    # todo::copyRemoteData
    copyRemoteData(ssh_handle.connection,ssh_handle.connection.sftp, target_data_name_list,target_data_remote_path_list, target_data_local_path_list)
  

    if ssh_handle.connection is not None:
        ssh_handle.close()
    # 拷贝sys_perf.txt、拷贝sys_perf_xxx.log
    #t1q_target_path = "/data/sys_perf_log/"、 从json中读出logName
    json_file = 'logName.json'
    if not os.path.exists(json_file):
        print(f'错误：文件 "{json_file}" 不存在！')
        exit()
    # 读取数据
    with open(json_file,'r') as f:
        logName_json_data = json.load(f)
    # 从数据中获取logname字段
    logName = logName_json_data.get('logname')           
    sysperf_filename = 'sys_perf_' + logName.strip()
    # 删除json文件
    if os.path.exists(json_file):
        os.remove(json_file)

    # 字符串组合为所需命令
    copy_sysperf_cmd = "sudo scp -i debug.key root@192.168.98.233:/data/sys_perf_log/"
    copy_sysperf_cmd += sysperf_filename
    copy_sysperf_cmd = copy_sysperf_cmd.strip() + " " + target_directory

    print("\033[0;32;40mInfo: copy_sysperf_cmd = %s  \033[0m" % copy_sysperf_cmd)
    copy_sysperf_cmd_ps = subprocess.Popen(copy_sysperf_cmd,shell=True)
    copy_sysperf_cmd_ps.wait()
    time.sleep(0.2)

    # 拷贝崩溃日志
    t1q_target_path = "/data/core"
    cpDataCore_cmd = "scp -i debug.key -r root@192.168.98.233:/data/core/ "
    cpDataCore_cmd += target_directory
    print("\033[0;32;40mInfo: cpDataCore_cmd = %s  \033[0m" % cpDataCore_cmd)
    run_sysinfo_cmd_ps = subprocess.Popen(cpDataCore_cmd,shell=True)
    run_sysinfo_cmd_ps.wait()
    time.sleep(0.2)

    # 拷贝地图数据
    t1q_target_path = "/ota/map_data"
    cpMapData_cmd = "scp -i debug.key -r root@192.168.98.233:/ota/map_data/ "
    cpMapData_cmd += target_directory
    print("\033[0;32;40mInfo: cpMapData_cmd = %s  \033[0m" % cpMapData_cmd)
    run_map_data_cmd_ps = subprocess.Popen(cpMapData_cmd,shell=True)
    run_map_data_cmd_ps.wait()
    time.sleep(0.2)

