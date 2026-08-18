# START_UP 工程详细说明文档

## 1. 项目概述

START_UP是一个用于ROS2机器人系统的开机自启动管理工程，支持多平台部署（CCU、ECU、AGX、TEST等）。该工程提供了完整的系统服务管理、时间同步、日志记录等功能，确保机器人系统在开机后能够自动启动所有必要的ROS2节点和服务。

### 主要特性
- **多平台支持**：支持CCU、ECU、AGX、TEST等多种部署模式
- **自动IP识别**：根据网络IP自动识别部署设备并自动部署
- **系统服务管理**：自动生成和管理systemd服务
- **时间同步**：集成chrony时间同步服务
- **日志管理**：完整的日志记录和查看功能
- **Docker支持**：支持在Docker容器中运行服务
- **优先级管理**：支持服务优先级设置

## 2. 工程结构

```
start_up/
├── install.sh                    # 主安装脚本
├── install_by_ip.sh             # 基于IP的自动安装脚本
├── execute.sh                    # 系统服务管理脚本
├── base_script.sh               # 基础脚本模板
├── generate_service.sh           # 服务生成脚本
├── relaod_auto_start_script.sh  # 重载脚本
├── logger.sh                     # 日志管理脚本
├── environment/                  # 环境配置文件目录
│   ├── ccu.sh                   # CCU环境配置
│   ├── ecu.sh                   # ECU环境配置
│   ├── agx.sh                   # AGX环境配置
│   ├── ccu_agx.sh              # CCU+AGX环境配置
│   └── test.sh                  # 测试环境配置
├── run_script/                  # 启动脚本目录
│   ├── ccu/                     # CCU启动脚本
│   ├── ecu/                     # ECU启动脚本
│   ├── agx/                     # AGX启动脚本
│   ├── ccu_agx/                # CCU+AGX启动脚本
│   └── test/                    # 测试启动脚本
├── config/                      # 配置文件目录
│   ├── ros_config.sh            # ROS2配置
│   └── time_syn.sh             # 时间同步配置
├── chrony_time_syn/            # 时间同步服务
│   ├── install_time_syn.sh     # 时间同步安装脚本
│   ├── base.conf               # 基础时间同步配置
│   └── conf/                   # 各平台时间同步配置
└── extra/                      # 额外文件目录
```

## 3. 核心文件详细说明

### 3.1 安装和管理脚本

#### `install.sh`
**功能**：主安装脚本，负责整个START_UP工程的安装部署
**参数**：
- `ecu`：安装ECU相关服务
- `ccu`：安装CCU相关服务
- `agx`：安装AGX相关服务
- `test`：安装测试相关服务
- `ccu_agx`：安装CCU+AGX相关服务

**主要功能**：
- 验证安装模式参数
- 创建必要的目录结构
- 复制环境配置文件
- 生成系统服务文件
- 安装时间同步服务
- 创建系统命令链接

#### `install_by_ip.sh`
**功能**：基于网络IP自动判断当前工程要部署于，ECU、CCU或者AGX并自动安装相应服务
**IP配置**：
- ECU: 192.168.123.100
- CCU: 192.168.123.106
- AGX: 192.168.123.105

**工作流程**：
1. 获取当前系统所有网卡IP地址
2. 匹配预定义的IP地址
3. 自动调用`install.sh`安装对应模式的服务

#### `execute.sh`
**功能**：系统服务管理脚本，提供统一的命令行接口
**支持命令**：
- `astrabot start`：启动所有服务
- `astrabot stop`：停止所有服务
- `astrabot enable`：启用所有服务开机自启动
- `astrabot disable`：禁用所有服务开机自启动
- `astrabot log <service_name>`：查看指定服务的日志
- `astrabot list`：列出所有服务
- `astrabot reload`：重新加载启动脚本
- `astrabot domain_id <id>`：设置ROS domain ID
- `astrabot dds <implementation>`：设置ROS DDS实现

### 3.2 环境配置文件

#### `environment/ccu.sh`（以CCU为例）
**功能**：定义CCU平台的环境变量和配置参数
**主要变量**：
```bash
THE_USER=astrabot              # 运行用户
THE_PASSWORD=1                 # 用户密码
START_UP_DIR='/opt/ros/start_up'  # 安装目录
CONFIG_DIR="config"            # 配置目录
SERVICE_DIR="service"          # 服务目录
LOG_DIR="log"                  # 日志目录
RUN_DIR="run"                  # 运行脚本目录
AOUT_START_SCRIPT_DIR="auto_start_script"  # 自启动脚本目录
FUNCTION_DIR="function"        # 功能脚本目录
```

### 3.3 启动脚本格式

#### `.start_script`文件格式
每个启动脚本包含以下配置项：

```bash
# 模块类型：Init, Sensors, Controller, MPC, Location, Navigation, Mapping, Others
MODEL_TYPE="Init"

# 是否在Docker中运行
RUN_IN_DOCKER=false

# Docker容器名称
DOCKER_CONTAINER_NAME="ecu_docker"

# 调度优先级（0-19，0最高）
PRIORITY=2

# 启动时执行一次的命令
COMMAND_RUN_ONCE_IN_BEGIN=(
    'source /opt/ros/humble/setup.bash'
    'source /opt/ros/astrabot/setup.bash'
    'ros2 launch astrabot_bringup astrabot_launch.py'
)

# 循环执行的命令
COMMAND_RUN_IN_LOOP=(
)
```

### 3.4 服务生成脚本

#### `generate_service.sh`
**功能**：自动生成systemd服务文件和可执行脚本
**主要函数**：

**Generate_Run()**
- 生成可执行脚本文件
- 集成基础脚本功能
- 设置执行权限

**Generate_Service()**
- 生成systemd服务文件
- 配置服务依赖关系
- 设置服务参数

### 3.5 基础脚本模板

#### `base_script.sh`
**功能**：提供所有启动脚本的基础功能
**主要功能**：
- 环境变量加载
- 日志初始化
- 时间同步等待
- 模块类型检查
- Docker容器管理
- 命令执行循环

## 4. 使用说明

### 4.1 安装部署

#### 方法一：手动指定模式
```bash
chmod +x install.sh
./install.sh ccu    # 安装CCU模式
./install.sh ecu    # 安装ECU模式
./install.sh agx    # 安装AGX模式
./install.sh test   # 安装测试模式
```

#### 方法二：自动IP识别
```bash
chmod +x install_by_ip.sh
./install_by_ip.sh  # 自动识别并安装
```

### 4.2 服务管理

安装完成后，可以使用以下命令管理服务：

```bash
# 启动所有服务
astrabot start

# 停止所有服务
astrabot stop

# 启用开机自启动
astrabot enable

# 禁用开机自启动
astrabot disable

# 查看服务列表
astrabot list

# 查看服务日志
astrabot log Astrabot_Bringup

# 重新加载脚本
astrabot reload

# 设置ROS domain ID
astrabot domain_id 6

# 设置DDS实现
astrabot dds rmw_fastdds_cpp
```

### 4.3 添加新的启动脚本

1. **创建启动脚本文件**
   运行install命令安装完成后，在`/opt/ros/start_up/auto_start_script/`目录下创建新的`.start_script`文件

2. **配置脚本参数**
   按照标准格式配置MODEL_TYPE、PRIORITY、COMMAND等参数

3. **重新加载**
   ```bash
   astrabot reload #reload后，新添加的服务会被安装到系统中
   ```

### 4.4 日志查看

日志文件位置：`/opt/ros/start_up/log/<service_name>/<date>/<time>.log`

查看最新日志：
```bash
astrabot log <service_name>
```

## 5. 配置说明

### 5.1 ROS2配置 (`config/ros_config.sh`)
```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=2
```

### 5.2 时间同步配置（`config/time_syn.sh`）
WAIT_SYN_TIME=10            #时间同步最长等待时间
MAX_TIME_SYN_OFFSET=0.001   #最大时间同步偏移

### 5.3 环境变量配置
根据部署平台修改`environment/<platform>.sh`中的变量：
- `THE_USER`：运行用户
- `THE_PASSWORD`：用户密码
- `START_UP_DIR`：安装目录
- `LOG_DELETE_OFFSET`：日志保留天数

## 6. 故障排除

### 6.1 常见问题

1. **服务启动失败**
   - 检查日志：`astrabot log <service_name>`
   - 验证环境变量配置
   - 确认ROS2环境是否正确加载

2. **时间同步问题**
   - 检查chrony服务状态：`systemctl status chrony`
   - 查看时间同步状态：`chronyc sources -v`

3. **Docker容器问题**
   - 检查容器状态：`docker ps`
   - 查看容器日志：`docker logs <container_name>`

### 6.2 调试方法

1. **查看服务状态**
   ```bash
   systemctl status Astrabot_*
   ```

2. **查看实时日志**
   ```bash
   journalctl -u Astrabot_* -f
   ```

3. **手动测试脚本**
   ```bash
   cd /opt/ros/start_up/run
   ./<service_name>.sh
   ```

## 7. 扩展开发

### 7.1 添加新平台支持

1. 创建环境配置文件：`environment/<new_platform>.sh`
2. 创建启动脚本目录：`run_script/<new_platform>/`
3. 创建时间同步配置：`chrony_time_syn/conf/<new_platform>.conf`
4. 修改`install.sh`添加新平台支持

### 7.2 自定义基础功能

修改`base_script.sh`添加新的基础功能：
- 环境检查
- 依赖验证
- 错误处理
- 性能监控

---

**文档版本**：v1.0  
**最后更新**：2025年  
**维护团队**：astrabot开发团队 
