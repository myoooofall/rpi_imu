#!/bin/bash

# 检查/var/log/syslog和/var/log/daemon.log文件大小是否超过300M
syslog_file="/var/log/syslog"
daemon_file="/var/log/daemon.log"
max_size=314572800  # 300M的大小，以字节为单位
while true; do
# 检查syslog文件
if [ -f "$syslog_file" ] && [ $(stat -c %s "$syslog_file") -gt $max_size ]; then
    # 删除syslog文件
    sudo rm "$syslog_file"
    
    # 重新启动rsyslog服务
    sudo systemctl restart rsyslog.service
    
    echo "syslog文件已删除并rsyslog服务已重新启动。"
else
    echo "syslog文件未超过300M，无需处理。"
fi

# 检查daemon文件
if [ -f "$daemon_file" ] && [ $(stat -c %s "$daemon_file") -gt $max_size ]; then
    # 删除daemon文件
    sudo rm "$daemon_file"
    
    # 重新启动rsyslog服务
    sudo systemctl restart rsyslog.service
    
    echo "daemon文件已删除并rsyslog服务已重新启动。"
else
    echo "daemon文件未超过300M，无需处理。"
fi
sleep 600
done