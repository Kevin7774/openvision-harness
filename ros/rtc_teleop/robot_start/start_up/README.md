<!-- README ** -->

# 开机自启动脚本工程

该工程用于提供开机自启动功能

## 开机自启动脚本工程中，各个文件及目录介绍


start_up：开机启脚本工程根目录

install.sh：开机启动工程安装脚本
execute.sh：开机启动工程执行脚本，用于开启关闭所有开机启动服务
environment.sh：开机启动工程环境变量配置脚本
logger.sh：脚本打印相关内容
generate_service.sh：用于自动生成开机自启动服务
reload_auto_start_script.sh：用于在安装完成后动态重新加载修改后的启动脚本
base_script.sh：保存了各个启动脚本所需要继承的基本功能

run_script：此目录下保存了需要开机启动的脚本文件

*.start_script：是各个分系统提供的需要开机启动的脚本、命令、ROS launch等，拓展名设置为‘.start_script’用于与其他不需要开机启动的文件做区分，防止异常的出现。


service：保存开机启动工程自动生成的各个系统服务文件
run：保存开机启动工程自动生成的各个系统服务对应的可执行脚本文件

log：开机启动脚本启动后产生的各个log，log文件的结构为： “开机启动服务名/日期/时间/.log文件”


## 安装方法：

将start_up文件夹拷贝到目标机文件系统某位置，并进入start_up文件夹， 然后运行如下命令即可完成安装：

```
chmod +x install.sh
./install.sh ccu 安装ccu相关开机自启动服务
./install.sh ecu 安装ecu相关开机自启动服务
./install.sh agx 安装agx相关开机自启动服务
./install_by_ip.sh 根据ip判断当前是ccu、agx或者ecu等，然后自动安装相应的包
```

默认情况下environment.sh中的START_UP_DIR变量为：
START_UP_DIR='/opt/ros/start_up'
这意味着，默认情况下开机自启动相关脚本服务等，均被安装在目录：/opt/ros/start_up，改变START_UP_DIR变量，将更换开机工程安装目录

environment.sh中的如下变量应根据具体情况而修改，SOURCE_FILE为ROS的setup文件路径
THE_USER=user
THE_PASSWORD=******
START_UP_DIR='/opt/ros/start_up'


安装完成后可调用以下系统命令来操作所有开机启动服务：
```
astrabot start #启动所有服务
astrabot stop #关闭所有服务
astrabot start Astrabot_Mpc #仅启动 Astrabot_Mpc 服务；也可使用完整的 .service 名称
astrabot stop Astrabot_Mpc #仅停止 Astrabot_Mpc 服务
astrabot enable #使能所有服务的开机自启动
astrabot disable #关闭所有服务的开机自启动
astrabot reload #加载auto_start_script目录下的开机自启动脚本，并重新生成开机自启动服务等
astrabot list #列出所有开机启动脚本
astrabot log Astrabot_Ros_Monitor #查看Astrabot_Ros_Monitor服务所打印的最新log，Astrabot_Ros_Monitor可替换为其他开机启动服务
astrabot domain_id 6 # 将ros domain id设置为6, 6可替换为其他id
astrabot dds rmw_fastdds_cpp # 将ros dds 设置为 rmw_fastdds_cpp，rmw_fastdds_cpp可替换为其他dds，但是需要事先安装好所需要安装的dds库

```

单独启动服务时会检查并显示其 systemd `Requires` 和 `Wants` 依赖，未启动的声明依赖
由 systemd 自动拉起。服务自身的 `ExecStartPre` 前置检查也会正常执行，例如 MPC 和数采服务
会等待 controller active。单独停止服务前会列出当前 active 的反向依赖服务，但不会自动停止它们。

使用./install ***安装完成后,安装目录中(默认为：/opt/ros/start_up)有：
config目录：保存了工程所需配置文件，主要为ros和chrony时间同步所需要的配置
auto_start_script： 保存了需要自启动的开机脚本，如果需要新增开机脚本，可按照目录内其他文件格式编写新的文件并保存到该目录下，然后运行astrabot reload即可生成新添加的脚本对应的自启动工程

## 平台系统服务

THOR 会启动 Diagnostics、File Transfer、Gateway、Log Agent、Log Hub 和 Remote Shell。
其中 systemd 依赖保证启动顺序为：

```text
Astrabot_File_Transfer.service
  -> Astrabot_Log_Agent.service
  -> Astrabot_Log_Hub.service
```

Gateway 进程内动态加载 Data Collection 和 File Transfer 插件，不为插件创建独立服务。

ECU 只启动 Diagnostics、Log Agent 和 Remote Shell，不安装 Gateway、File Transfer 或
Log Hub 的启动文件。Diagnostics 根据 `/etc/astrabot/board.yaml` 的 `board.resource_id`
决定是否启动 aggregator：THOR 启动 system monitor 和 aggregator，ECU 只启动 system monitor。

## THOR ZED USB 生命周期

ZED 2i 同时使用 `2b03:f880` 视频接口和 `2b03:f881` HID/传感器接口。ZED SDK 运行时会通过
`usbfs` 独占 HID 接口；进程异常退出后，接口可能停留在未绑定状态，导致原生 V4L2 仍可抓帧，
但 SDK 报 `CAMERA FAILED TO SETUP` 或进入 USB 重置循环。

`Astrabot_ZED.service` 启动前会以受控 root 前置步骤执行 `prepare-zed-usb.sh`：只检查上述
Stereolabs VID/PID，在 HID 接口未绑定时恢复 `usbhid`；若接口已由另一个进程通过 `usbfs`
占用，则拒绝启动，避免两套 ZED 进程争用同一相机。服务停止使用 `SIGINT` 和 control-group
语义，确保 ROS launch 进程树统一退出；下一次启动会自动恢复 HID，无需人工重插 USB。

回归验证：

```bash
bash test/zed_usb_service_contract_test.sh
sudo systemctl stop Astrabot_ZED.service
sudo systemctl start Astrabot_ZED.service
ros2 topic hz /zed/zed_node/point_cloud/cloud_processed
```

## RTC 与 Teleop C++ 迁移服务

THOR 安装会增加两个独立的迁移服务：

```text
Astrabot_Gateway.service
  -> Astrabot_Rtc.service
  -> Astrabot_Teleop.service
```

依赖语义是：RTC 通过 `Wants` 软依赖 Gateway，Gateway 重启不会把媒体/会话生命周期塞进
Gateway；Teleop 通过 `Requires` 依赖 RTC，RTC 不可用时 Teleop 不应继续作为独立控制入口。
新服务不与 LiveKit 冲突，现有 `Astrabot_LiveKit.service`、`Astrabot_LiveKit_Srv.service`
仍作为旧遥操回滚链路保留。Data Collection 已解除对这两个服务的 systemd 依赖，可在 LiveKit
未启动时独立运行。

首次执行 `install.sh thor` 时，以下安全默认配置会以 `0640` 权限安装；后续升级会保留已有内容：

```text
/etc/astrabot/rtc.yaml
/etc/astrabot/teleop.yaml
/etc/astrabot/rtc-teleop.env
```

模板位于：

```text
run_script/thor/supplement/config/rtc/rtc.yaml.example
run_script/thor/supplement/config/rtc/rtc-teleop.env.example
run_script/thor/supplement/config/teleop/teleop.yaml.example
```

也可以单独安装或补齐配置：

```bash
sudo bash install_rtc_teleop_config.sh
# 已安装 robot_start 的目标机也可执行：
sudo bash /opt/ros/start_up/function/install_rtc_teleop_config.sh
```

RTC/Teleop 的模板和 `/etc/astrabot/rtc-teleop.env` 都保持安全默认值。RTC 切换到 `libdatachannel` 时，runner 会同时校验
`ASTRABOT_RTC_PRODUCTION_ENABLED=1`、精确版本的受控 runtime 和启用 backend 的构建能力标记。Teleop 切换到 `shadow`
前必须填写真实 `device_id` 和成对 grant 公钥；shadow 命令不具备生产 writer 权限。切换到 `cpp` 还必须设置
`ASTRABOT_TELEOP_CPP_ENABLED=1`、RTC 使用 `libdatachannel`，并确保 arbitration owner 与 Data Collection 状态上报服务可用。
这些 Gate 只允许进程进入对应模式，不替代 Quest/WebOps/TURN、THOR NVENC 和 HIL 验收。

RTC 和 Teleop 不参与批量 `astrabot start/init/enable`。完整安装也会把两个 unit 停止并禁用，避免迁移
代码随整机升级意外上线。需要验证接口时必须显式启动：

```bash
astrabot start Astrabot_Rtc
astrabot start Astrabot_Teleop
```

启动 Teleop 时 systemd 会自动拉起 RTC；启动 RTC 时会尝试拉起 Gateway。上述单服务命令只启动当前
会话，不会自动设置开机自启。两个启动包装器与其他 THOR 模块保持相同加载顺序：先加载
`/opt/ros/start_up/run/environment.sh` 获取 `.bashrc` 中的 `ROS_DOMAIN_ID`，再加载
`/opt/ros/start_up/config/ros_config.sh` 获取 `RMW_IMPLEMENTATION`，最后叠加 Astrabot ROS overlay。
受控 canary 若确需跨重启运行，可在完成配置与评审后显式执行：

```bash
sudo systemctl enable --now Astrabot_Rtc.service
sudo systemctl enable --now Astrabot_Teleop.service
```

`astrabot reload` 会分别保留操作员此前显式设置的 RTC、Teleop 以及两个 legacy LiveKit rollback unit 的
enable/disable 状态；不会因为 Data Collection 已解耦就擅自启用或禁用旧遥操链路。未启用的 RTC/Teleop 仍保持停止和禁用。
回滚时先停止 Teleop，再停止 RTC，确认不存在新 writer 后继续使用原 LiveKit/Data Collection 链路：

```bash
astrabot stop Astrabot_Teleop
astrabot stop Astrabot_Rtc
astrabot start Astrabot_Data_Collection
```

若回滚包含机器人软件刷新或降级，必须先停止全部 ROS/Fast DDS/Astrabot 进程，确认
`/dev/shm/fastrtps*` 无占用，再执行 `fastdds shm clean`，随后统一启动并验证关键 topic；禁止在活跃
ROS/Fast DDS 进程存在时清理 SHM。

`install.sh` 与 `reload_auto_start_script.sh` 会通过 `fastdds_shm_refresh_gate.sh` 强制执行该顺序：清理前同时
检查进程环境、打开的 SHM fd 和内存映射；只继承 ROS 环境但没有 SHM 映射的交互 shell 和远程编辑器宿主
不算 DDS 参与者。清理后以 `ROS_LOCALHOST_ONLY=1` 等待本机 `/diagnostics`。任一 Gate
失败都会中止刷新或触发 reload 回滚，不允许带着分裂的 Fast DDS SHM 继续启动。

部署契约测试：

```bash
bash test/rtc_teleop_startup_contract_test.sh
```

## Thor 数采模块集成

THOR 腕部相机服务名称统一为 `Astrabot_Wrist_Camera.service`，对应 ROS 包
`astrabot_wrist_camera` 和 launch 文件 `wrist_camera.launch.xml`。服务只使用稳定 udev 别名
`/dev/l_arm_cam`、`/dev/r_arm_cam`，并发布：

```text
/astrabot/data_sources/image/left_wrist
/astrabot/data_sources/image/right_wrist
```

`Astrabot_Recorder.service` 由 C++ `astrabot_recorder` 包提供，并通过 systemd 依赖该服务。Recorder
订阅上述两路 Topic；episode 的 warm-up/freshness Gate 会阻止相机未 ready 时开始采集。
相机、Recorder 和 Dataset Builder 均从 `/opt/ros/start_up/run/environment.sh` 读取
`ROS_DOMAIN_ID`，与其他 `robot_start` 模块保持相同的 `.bashrc` 覆盖和默认值逻辑，不在各 unit 内写死。

部署契约验证：

```bash
bash test/wrist_camera_startup_contract_test.sh
```

数采依赖安装与开机启动分离。仓库只提供不含密钥的配置模板：

```text
start_up/run_script/thor/supplement/config/data_collection/astrabot.yaml.example
```

执行 Thor 安装：

```bash
cd start_up
sudo ./install.sh thor
```

安装脚本会自动使用 Python 3.10 创建数采虚拟环境、从 Nexus 安装
`robot-data-collection[astrabot]`，并在首次安装时将配置模板以 `0660` 权限部署到：

```text
/home/astrabot/deploy/data_collection.yaml
```

安装完成后编辑该文件，填写设备、技能和服务配置：

```bash
sudo -u astrabot vim /home/astrabot/deploy/data_collection.yaml
```

只有需要保留 legacy Teleop 回滚能力时，才需要填写 LiveKit 配置：

```bash
sudo -u astrabot vim /home/astrabot/deploy/livekit.env
```

该回滚链路至少需要配置 `LIVEKIT_NODE_IP`、`LIVEKIT_API_KEY` 和
`LIVEKIT_API_SECRET`。密钥必须与 `livekit-server --dev` 使用的凭据一致；这些配置不再是
Data Collection 的启动前置。

真实的 `astrabot.yaml` 已通过 `.gitignore` 排除，不应提交设备密钥。也可以从仓库外
传入配置：

```bash
./install_data_collection.sh /secure/path/astrabot.yaml
```

也可以单独运行 `install_data_collection.sh`。该脚本使用 Python 3.10 创建
`/home/astrabot/deploy/.venv`，安装 `robot-data-collection[astrabot]`，并将 YAML
以 `0660` 权限复制为 `/home/astrabot/deploy/data_collection.yaml`。`install.sh thor` 会安装
`Astrabot_Data_Collection.service`。

数采服务在首次执行 `install.sh thor` 时不会自动启动或设为开机自启。完成数采环境安装、
硬件信息写入和配置检查后，执行：

```bash
astrabot start Astrabot_Data_Collection
```

该命令首次启动数采的同时会执行 `systemctl enable --now`，因此之后正常关机或重启时，
Data Collection 会自动启动。`astrabot stop Astrabot_Data_Collection` 只停止当前运行，
不会取消开机自启。无论首次安装还是软件更新，只要执行 `install.sh thor`，数采服务都会
恢复为停止和禁用，必须再次手动执行上述单服务启动命令。

执行 `astrabot reload` 时则会保留 Data Collection 原来的 enable 状态：原来已启用的服务
会在 reload 后恢复自启并启动，原来未启用的服务继续保持停止和禁用。

服务每次启动前会通过 `wait-controller-node.sh` 等待
`astrabot_arm_forward_position_controller` 处于 `active` 状态，与 MPC 服务使用相同的前置检查。
启动 Data Collection 时只等待 Controller 进入 active，不再拉起或等待 LiveKit。旧
`Astrabot_LiveKit.service` 和 `Astrabot_LiveKit_Srv.service` 继续保留给 legacy Teleop 回滚，
但不属于 Data Collection 的运行时依赖。Controller 检查通过后执行：

```bash
ASTRABOT_CONTROL_LAUNCHER=/opt/astrabot/run_control_node.sh \
  /home/astrabot/deploy/.venv/bin/astra run \
  /home/astrabot/deploy/data_collection.yaml
```
