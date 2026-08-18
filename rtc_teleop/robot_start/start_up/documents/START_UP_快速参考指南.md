# START_UP 快速参考指南

## 快速安装

### 自动安装（推荐）
```bash
chmod +x install_by_ip.sh
./install_by_ip.sh
```

### 手动安装
```bash
chmod +x install.sh
./install.sh ccu    # CCU模式
./install.sh ecu    # ECU模式
./install.sh agx    # AGX模式
./install.sh test   # 测试模式
```

## 常用命令

### 服务管理
```bash
astrabot start      # 启动所有服务
astrabot stop       # 停止所有服务
astrabot enable     # 启用开机自启动
astrabot disable    # 禁用开机自启动
astrabot list       # 列出所有服务
```

### 日志查看
```bash
astrabot log <service_name>    # 查看指定服务日志
# 示例：astrabot log Astrabot_Bringup
```

### 配置管理
```bash
astrabot reload                 # 重新加载启动脚本
astrabot domain_id <id>        # 设置ROS domain ID 示例：astrabot domain_id 2
astrabot dds <implementation>   # 设置DDS实现 示例：astrabot dds rmw_fastrtps_cpp
```

## 添加新服务

### 1. 创建启动脚本
运行install命令安装完成后，在`/opt/ros/start_up/auto_start_script/`目录下创建新的`.start_script`文件，并合理配置如下参数

```bash
# 模块类型
MODEL_TYPE="Others"

# 是否在Docker中运行
RUN_IN_DOCKER=false

# 优先级（0-19，0最高）
PRIORITY=5

# 启动时执行一次的命令
COMMAND_RUN_ONCE_IN_BEGIN=(
    'source /opt/ros/humble/setup.bash'
    'ros2 run my_package my_node'
)

# 循环执行的命令
COMMAND_RUN_IN_LOOP=(
)
```

### 2. 重新加载
```bash
astrabot reload
```

## 故障排除

### 检查服务状态
```bash
systemctl status Astrabot_*
```

### 查看实时日志
```bash
astrabot log <service_name> #查看最新日志
journalctl -u Astrabot_* -f
```

### 检查时间同步
```bash
chronyc sources -v
systemctl status chrony
```

### 3.2 环境配置文件中关键配置参数
```bash
# 基本配置
THE_USER=astrabot              # 运行用户
START_UP_DIR='/opt/ros/start_up'  # 安装目录

# 目录配置
SERVICE_DIR="service"          # 服务文件目录
LOG_DIR="log"                  # 日志目录
RUN_DIR="run"                  # 运行脚本目录
CONFIG_DIR="config"           # 配置文件目录

# 日志配置
LOG_DELETE_OFFSET=+3          # 日志保留天数
```

### 3.3 ROS配置
```bash
# /opt/ros/start_up/config/ros_config.sh
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=2
```

### 3.4 时间同步配置
```bash
# /opt/ros/start_up/config/time_syn.sh
WAIT_SYN_TIME=10            #等待时间同步最长时间 （秒）
MAX_TIME_SYN_OFFSET=0.001   #最大同步误差 （秒）
```


---

**快速参考版本**：v1.0  
**最后更新**：2025年  
**维护团队**：astrabot开发团队 