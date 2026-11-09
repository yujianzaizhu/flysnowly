# 工作区联调说明

当前版本将 **内部仿真** 与 **接入导航进程仿真** 实现为两套独立应用。工作区构建只属于导航应用；内部仿真的地图、障碍、任务与日志回放状态不会传入导航应用。

## 启动与切换

```bash
cd robot_simulation
./run.sh                         # 内部仿真
./run.sh --tea-bridge            # 直接启动导航进程仿真
```

两套应用共用原 HTTP 地址。顶栏“仿真”下拉通过 `POST /api/sim_mode` 请求切换：启动器先停止当前任务、日志回放或遥控，再关闭并销毁当前 HTTP 应用，在原端口创建目标应用，浏览器随后完整刷新。

- `GET /api/mode` 只返回当前模式、目标模式和切换状态。
- 工作区构建中不允许切换，接口返回 HTTP 409；请先保存或取消。
- 导航应用不注册 `/api/task`、`/api/map/edit`、`/api/bag*`，也不提供独立日志分析页。
- 导航桥依赖或 tea 进程不可用时，页面保留并显示错误，不会回退到内部仿真状态。

## 独立模块

- `backend/server.py`、`frontend/index.html`、`frontend/app.js`：原内部仿真；页面只增加公共模式入口。
- `backend/app_launcher.py`、`backend/mode_control.py`：应用创建、停止和模式接口，不处理业务地图或任务。
- `backend/nav_process/server.py`：导航 HTTP 服务、仿真节拍和工作区 API。
- `backend/nav_process/robot.py`：导航机器人独立运动学与状态，不读取内部 `world`、`RobotCar` 或任务管理器。
- `backend/nav_process/tea_bridge.py`、`tea_proto/`：ZeroMQ / protobuf 通信，仅属于导航模块。
- `frontend/nav_process/`：独立导航页面、样式、状态轮询和 Canvas；只绘制 tea 地图、站点、构建轨迹与导航机器人。
- `frontend/shared/`：两套页面唯一共用的模式选择脚本和样式。

## 工作区操作

导航功能区提供“地图构建 / 任务”页签。进入工作区后，可创建区域、通道或禁区，并保存、取消或退出构建；也可删除地图元素、从磁盘重载和检查 tea 连接。

主要接口：

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| GET | `/api/nav/state` | 导航机器人、活动、配置、事件和 tea 摘要 |
| GET | `/api/nav/workarea/state` | tea 连接与构建会话状态 |
| GET | `/api/nav/workarea/map?v=版本` | tea 地图快照；版本未变时不重复传输地图 |
| POST | `/api/nav/workarea/connect` | 检查/连接导航进程桥 |
| POST | `/api/nav/workarea/start` | 开始区域、通道或禁区轨迹记录，可先做起点校验 |
| POST | `/api/nav/workarea/save` | 保存轨迹并刷新 tea 地图，可先做保存校验 |
| POST | `/api/nav/workarea/cancel` | 取消本次构建并刷新地图 |
| POST | `/api/nav/workarea/clear` | 删除指定地图元素，或按 tea 协议清空 |
| POST | `/api/nav/workarea/load` | 从 tea 配置的地图目录重新加载 |
| POST | `/api/nav/workarea/probe` | 探测 tea 在线状态 |
| POST | `/api/nav/workarea/raw` | 调试 `record_path` / `get_map_info` / `nav_action` 原始请求 |

`TeaBridge` 订阅 `combined`、`tf_dynamic` 和 `map_info`，直连 `record_path`（默认 20015）与 `get_map_info`（默认 20018）。保存和取消只在 tea 返回成功后结束本地会话镜像；地图保存位置由 tea 的 `cfg/top/map_man.json` 中 `map_dir` 决定，不是内部仿真的 `maps/` 目录。部署环境的地图目录必须预先存在且可写。

可通过 `--tea-host`、`--tea-record-port`、`--tea-map-port` 或对应环境变量调整连接参数。修改工作区协议时只需维护导航模块，不应把 tea 业务分支加入内部仿真。

## 验证与边界

`tests/test_server_workarea.py` 使用 `FakeTeaBridge` 覆盖导航页面、状态隔离、工作区命令和错误处理；`tests/test_workarea_session.py` 覆盖会话与轨迹；`tests/test_tea_bridge.py` 覆盖 protobuf 编解码；`tests/test_app_launcher.py` 覆盖同端口应用切换和构建中冲突。

这些测试不连接真实 tea。真实地图创建、保存、取消、删除和磁盘重载仍需在部署环境中联调；不要由开发测试擅自创建 `/userdata` 等系统目录。
