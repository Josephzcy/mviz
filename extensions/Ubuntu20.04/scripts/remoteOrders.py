import os
import platform
import sys
import paramiko
import json
from datetime import datetime
import time
import subprocess
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
        print(" key %s " % self.keypath + "/debug.key")
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
        return stdout.read().decode()
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


if __name__ == '__main__':
    
    now = datetime.now()
    # 输出当前时间
    print("当前时间是:", now)
    time_str = now.strftime("%Y%m%d%H%M%S")
    print("time_str = ",time_str)
    
    env = EnvPublish()
    envPath = env.get_current_dir()
    print("\033[0;32;40mInfo: The current dir =%s \033[0m" % envPath)

    # 运行本地APA性能脚本
    run_sysinfo_cmd = envPath + "/run_sysinfo.sh "
    print("\033[0;32;40mInfo: run_sysinfo_cmd = %s \033[0m" % run_sysinfo_cmd)
    run_sysinfo_cmd_ps = subprocess.Popen(run_sysinfo_cmd,shell=True)
    try:
        run_sysinfo_cmd_ps.wait()
        if run_sysinfo_cmd_ps.poll() == 0:
            print(run_sysinfo_cmd_ps.communicate()[1])
        else:
            print("run_sysinfo_cmd_ps failed")
    except subprocess.TimeoutExpired:
        run_sysinfo_cmd_ps.kill()
        print("run_sysinfo_cmd timeout")

    time.sleep(1)

    sshHandle = mySSH('192.168.98.233','root',22,envPath)
    sshHandle.connect()
    if sshHandle.connection is not None:
        sshHandle.exec_command('mkdir -p /data/sys_perf_log')
    
    logName = (time_str +'.log').strip() 
    sysperf_exec_string = "sys_perf >> /data/sys_perf_log/sys_perf_" + logName + ' &'
    # 生成临时json存储logName
    sysperf_data = {} # 新建一个空的字典对象
    # 添加`logname`字段
    sysperf_data['logname'] = logName
    json_file = 'logName.json'
    # 如果文件已存在，删除它
    if os.path.exists(json_file):
        os.remove(json_file)
    
    # 将数据写入Json文件
    with open(json_file,'w') as f:
        json.dump(sysperf_data,f)
    print('JSON 文件已创建！')

    
    mviz_upstream_compononts_cmd = '/data/mviz_upstream_tools/start_mviz_upstream_components_board.sh oncar'
    print('Next step is to execute strings.')
    if sshHandle.connection is not None:
        sshHandle.exec_command(mviz_upstream_compononts_cmd) #run in no background
        sshHandle.exec_command(sysperf_exec_string)
    else:
        print("error: The connect is null")
   
    if sshHandle.connection is not None:
        sshHandle.close()
        time.sleep(0.1)
        print("debug: The connect is close")
