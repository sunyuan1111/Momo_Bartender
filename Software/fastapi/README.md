# Momo Bartender FastAPI

这个小服务把 `recordings/*.json` 轨迹封装成 HTTP POST，并提供一个局域网可访问的控制界面。

## 安装依赖

```bash
pip install -r Software/fastapi/requirements.txt
```

如果当前环境已经用 `pip install -e .` 安装项目，也可以使用：

```bash
pip install -e ".[api]"
```

## 启动

```bash
python Software/fastapi/app.py
```

默认监听：

```text
http://0.0.0.0:1999
```

本机打开 `http://127.0.0.1:1999/`，局域网其他电脑打开 `http://这台电脑的局域网IP:1999/`。

## HTTP 接口

每个 `recordings/<name>.json` 都可以通过 POST 触发：

```bash
curl -X POST http://127.0.0.1:1999/home
curl -X POST http://127.0.0.1:1999/begin
curl -X POST http://127.0.0.1:1999/daojiu1
curl -X POST http://127.0.0.1:1999/daojiu22
curl -X POST http://127.0.0.1:1999/daojiu33
curl -X POST http://127.0.0.1:1999/shake1
curl -X POST http://127.0.0.1:1999/end
```

也可以走统一 API 前缀：

```bash
curl -X POST http://127.0.0.1:1999/api/actions/begin
```

状态接口：

```bash
curl http://127.0.0.1:1999/state
curl http://127.0.0.1:1999/api/state
```

读取机械臂硬件状态：

```bash
curl http://127.0.0.1:1999/robot-state
curl "http://127.0.0.1:1999/state?robot=true"
```

## 可选环境变量

默认会沿用脚本自己的默认参数，相当于：

```bash
python scripts/2_playback_trajectory.py recordings/begin.json
```

如果要覆盖配置文件或串口，可以在启动前设置：

```bash
export MOMOTENDER_FASTAPI_CONFIG=configs/arm7_sts3215.example.json
export MOMOTENDER_FASTAPI_DEVICE_PORT=/dev/ttyACM0
python Software/fastapi/app.py
```
