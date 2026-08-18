#!/bin/bash

#脚本中输入sudo密码并执行chronyc makestep

source ../environment.sh

echo $THE_PASSWORD | sudo -S chronyc makestep

sleep 3

echo $THE_PASSWORD | sudo -S chronyc -a makestep

sleep 3

echo $THE_PASSWORD | sudo -S chronyc -a makestep

chronyc sources -v



