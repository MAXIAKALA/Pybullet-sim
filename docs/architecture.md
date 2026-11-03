# 架构说明

## 1. 分层

```
┌────────────────────────────────────────────────────────────┐
│ task.py        WireConnectTask（状态机 / 失败恢复 / 指标）   │
├────────────────────────────────────────────────────────────┤
│ perception.py  ColorFiducialDetector（检测 + PnP）           │
│ robot.py       PandaRobot（IK / 关节控制 / 夹爪）            │
├────────────────────────────────────────────────────────────┤
│ scene.py       PyBullet 场景（Franka / 面板 / 插头 / 相机）  │
└────────────────────────────────────────────────────────────┘
```

任务层不直接调用 PyBullet 的关节接口；它只依赖：

- `perceive() -> {plug_center, socket, ...}`
- `robot.move_to(pos, R)` / `robot.gripper(width)`
- `scene.contact_force()` / `scene.plug_pose()`

所以把 `scene.py` + `robot.py` 换成 ROS2/MoveIt2 后端时，`task.py` 不需要改。

## 2. 坐标系

| 坐标系 | 定义 |
|---|---|
| world | 机器人基座原点，z 向上 |
| tcp | `panda_grasptarget`（link 11），夹爪指尖中心 |
| plug | 插头圆柱中心，+x 为插入方向 |
| plug_marker | 插头顶部标记，x 右 / y 上 / z 向上（世界水平） |
| panel_marker | 面板上方标记，x 右 / y 上 / z 向上 |
| camera | OpenCV 约定：x 右 / y 下 / z 向前 |

固定偏移：

- `PLUG_MARKER_OFFSET = [0, 0, 0.011]`：标记 → 插头中心
- `SOCKET_OFFSET = [0.08, 0, -0.20]`：面板标记 → 插座中心
- `PLUG_TIP_OFFSET = [0.05, 0, 0]`：插头中心 → 尖端

## 3. 感知

1. 把 RGB 转 HSV，按 8 个色相区间分割四色球。
2. 取每个颜色的像素质心（>25 px）。
3. 四个点按 TL/TR/BR/BL 对应到标记平面上的 (±h, ±h, 0)。
4. `cv2.solvePnP(..., SOLVEPNP_IPPE_SQUARE)` 得到 `R_cm, t_cm`。
5. 用相机外参 `R_wc, t_wc` 变到世界：`R_wm = R_wc R_cm`，`t_wm = t_wc + R_wc t_cm`。
6. 用固定偏移得到插头中心/插座中心。

相机内参：`f = h / (2 tan(vfov/2))`，`cx=w/2, cy=h/2`（PyBullet 的 FOV 是垂直 FOV）。

## 4. 抓取

1. 移动到插头正上方 8 cm。
2. 下降到示教高度 `z=0.49`（插座/支架固定，避免单目深度偏差）。
3. 夹爪闭合到 6 mm，读指爪接触。
4. 接触存在 → 建立 TCP↔plug 的固定约束（grasp assist），解除插头与世界约束。
5. 抬升 8 cm，检查插头是否跟随（`grasp_ok`）。

> 真实机器人上把第 4 步换成摩擦夹持 + 夹爪电流/指距判定。

## 5. 插入

1. 预插入：插头中心在孔前 2.5 cm（`socket - [0.075, 0, 0]`）。
2. 横向闭环：用插头实际位姿把 y/z 对齐到插座（最多 3 次）。
3. 以 1.5 mm/步沿 +x 进给。
4. 力 > 10 N：回退 4 mm，y-z 平面 3.5 mm 螺旋搜索，重试（≤4 次）。
5. 成功：尖端 x > 插座 x + 4 mm 且横向 < 10 mm。
6. 成功后 `seat_plug()` + `latch_plug()`，松开夹爪，回 home。

## 6. 状态机

| 状态 | 动作 | 失败 |
|---|---|---|
| PERCEIVE | 渲染 + 检测 + PnP | 用示教位姿 fallback |
| GRASP | 预抓取 → 下降 → 闭合 → attach → 抬升 | `grasp_failed` |
| ALIGN | 预插入 + 横向闭环 | 超时 |
| INSERT | 步进 + 力监控 + 螺旋 | `insert_stuck` |
| VERIFY | 尖端深度 + 横向误差 | `insert_timeout` |
| DONE | latch + 松开 + 回 home | — |

## 7. 接触力

`scene.contact_force()` 对 `getContactPoints(bodyA=plug)` 中 `bodyB ∈ panel_bodies` 的法向力求和。成功插入时插头与孔壁不接触，力为 0；卡滞/撞壁时才出现峰值。

## 8. 已知时序

- 仿真步长 1/240 s，`numSolverIterations=80`。
- 单次 episode 约 0.6 s 计算时间（本机）。
- 视频 30 fps，每 8 个仿真步取一帧。
