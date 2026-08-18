#!/bin/bash

source environment.sh


# 判断是否安装chrony了
if ! command -v chronyc &> /dev/null; then
    echo "chrony is not installed, install chrony."
    echo $THE_PASSWORD | sudo -S apt-get install chrony
    echo "yes"
else
    echo "chrony is already installed."
fi

# 配置chrony
echo $THE_PASSWORD | sudo -S cat base.conf > chrony.conf

command=$1

# 用for循环判断当前conf文件夹下是否有包含command命令的.conf文件
for file in conf/*; do
    # echo "file: $file"
    if [ -f "$file" ]; then
        if [[ "$file" == "conf/$command.conf" ]]; then
            sudo cat "$file" >> chrony.conf
            echo "install chrony $file."
        fi
    fi
done

sudo cp chrony.conf /etc/chrony/chrony.conf

echo "-------------"

sleep 1
# 启动chrony
echo "restart chrony."
sudo systemctl daemon-reload

sudo systemctl restart chrony

sleep 1
echo "enable chrony."
sudo systemctl enable chrony
