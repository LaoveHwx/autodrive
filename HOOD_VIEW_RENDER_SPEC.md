# SYSTEM SPEC v1.0

## 1. GOAL（系统目标）
将当前 AutoDrive Pygame 第一视角仿真升级为更稳定、更有速度感、更易维护的引擎盖上方视角动态驾驶演示系统。

## 2. INPUT / OUTPUT
- Input:
  - 键盘控制：`A / D` 横向干预，窗口关闭事件。
  - 仿真状态：道路中心线、车道宽度、车速、车辆横向位置、方向盘/转向量、自动驾驶 PID 状态。
  - 配置参数：分辨率、FPS、相机 FOV、相机高度、相机俯仰角、道路长度、雾化距离、动态物体密度。
- Output:
  - 主窗口：引擎盖上方视角的道路、车道线、车头、路肩、路灯、护栏、天空、地面、HUD。
  - 感知窗口：基于相机投影参数自动生成 ROI 的 OpenCV 车道边缘/车道线检测视图。
  - 调试输出：FPS、渲染耗时、车辆偏移、目标车道、自动/人工控制模式。

## 3. HARD CONSTRAINTS（硬约束）
- RAM: 常规运行低于 300 MB，不缓存无上限历史帧，不为每帧重复创建大尺寸临时数组。
- Latency: 输入到画面响应低于 50 ms，主循环目标 60 FPS，单帧预算 16.7 ms。
- CPU: 普通笔记本单核可运行，Pygame 绘制和 OpenCV 感知总占用应尽量低于 1 个 CPU 核心的 80%。
- Power: 默认不开启昂贵后处理，视觉增强以 2D 分层、预计算、多边形批量绘制为主，适合无独显环境。

## 4. ARCHITECTURE（架构分层）
- Layer 1: Simulation Core
  - 负责道路生成、车辆控制、车道目标、动态对象生命周期。
- Layer 2: Camera & Projection
  - 负责引擎盖相机参数、世界坐标到屏幕坐标投影、近远裁剪、ROI 反推。
- Layer 3: Rendering & Perception
  - 负责 Pygame 分层渲染、HUD、OpenCV 感知预览、调试叠层。

## 5. MODULES（模块定义）

### RoadModel
- responsibility: 生成连续道路中心线、曲率、车道边界、车道虚线采样点。
- input: `Config.ROAD_SPEED`, `Config.CURVE_CHANCE`, `Config.MAX_ANGLE`, 随机种子。
- output: 按 z 排序的道路采样段、每段曲率、灯杆/护栏/路肩挂点。
- dependency: `Config`, `math`, `random`, `collections.deque`。

### VehicleController
- responsibility: 处理自动驾驶 PID、人工接管、横向位置、转向平滑和车道吸附。
- input: 键盘状态、道路中心线、目标车道。
- output: 车辆 x 坐标、实际转向、控制模式、车身摇摆/俯仰提示量。
- dependency: `RoadModel`, `Config`, `pygame`。

### HoodCamera
- responsibility: 定义引擎盖上方相机，并提供统一投影接口。
- input: 车辆 x、道路点、相机高度、相机前移量、俯仰角、FOV。
- output: `project(world_x, world_y, world_z) -> screen_x, screen_y, scale, visible`。
- dependency: `Config`, `math`。

### RoadRenderer
- responsibility: 分层绘制天空、远景、地面、道路梯形、车道线、路肩、护栏、路灯、雾化。
- input: 道路采样段、相机投影结果、时间、天气/光照参数。
- output: Pygame 主画面道路层。
- dependency: `HoodCamera`, `RoadModel`, `pygame`, `numpy`。

### HoodOverlayRenderer
- responsibility: 绘制引擎盖，而不是驾驶舱方向盘视角。
- input: 车辆转向、速度、相机震动量。
- output: 底部 18% 到 24% 屏幕高度的车头/引擎盖遮罩、高光、轻微横摆。
- dependency: `pygame`, `Config`。

### DynamicSceneManager
- responsibility: 管理会增强速度感的非核心对象。
- input: 道路挂点、车辆速度、随机种子。
- output: 路灯、护栏柱、反光桩、路牌、远处车辆影子或简单交通车。
- dependency: `RoadModel`, `HoodCamera`。

### PerceptionView
- responsibility: 基于引擎盖相机自动更新 OpenCV ROI，避免 ROI 与画面机位脱节。
- input: 主窗口帧、相机投影参数、车道边界投影点。
- output: 边缘图、ROI mask、可选车道线 overlay。
- dependency: `cv2`, `numpy`, `HoodCamera`。

### DebugHUD
- responsibility: 显示模式、FPS、转向、目标车道、相机参数、性能耗时。
- input: 车辆状态、性能计时器、配置。
- output: 左上角简洁 HUD 与可切换调试叠层。
- dependency: `pygame`。

## 6. DATA FLOW（数据流）
Keyboard / Timer → VehicleController → RoadModel.update → HoodCamera.update → RoadRenderer → HoodOverlayRenderer → Pygame Display → PerceptionView → OpenCV Display

## 7. ALGORITHM CHOICES
- option1: 继续使用当前单参数伪透视投影。
  - 优点：改动小。
  - 缺点：缺少相机高度和俯仰角，视角更像平面压缩，车头与道路空间关系不稳定。
- option2: 在 Pygame 中实现轻量 2.5D 引擎盖相机投影。
  - 优点：仍然简单高效，同时拥有相机高度、俯仰、近裁剪、远雾化和稳定车头遮挡。
  - 缺点：需要重写 `project()` 和道路段绘制顺序。
- option3: 迁移到完整 3D 引擎。
  - 优点：真实 3D、光照和模型表现更好。
  - 缺点：项目复杂度明显增加，不利于当前自动驾驶教学演示和 OpenCV 窗口同步。
- selected: option2
  - 先保持 Pygame 架构，重构为 `HoodCamera + 分层 Renderer`，获得足够好的引擎盖视角和动态展示效果。

## 8. EMBEDDED OPTIMIZATION STRATEGY
- fixed-point conversion
  - Pygame 绘制前统一将投影结果转换为 `int`，模拟核心保留 `float`，避免反复隐式转换。
- memory reuse
  - 复用道路点 `deque`、动态对象池、OpenCV mask、HUD surface，减少每帧临时对象。
- cache alignment
  - 将道路段投影结果整理成连续列表，一帧内 RoadRenderer、PerceptionView 共用，避免重复投影。
- DMA usage (if applicable)
  - 当前桌面 Pygame 不需要 DMA；如果未来接入嵌入式摄像头或硬件显示，可将主帧 buffer 和感知输入 buffer 分离。
- draw budget
  - 远处道路段合并绘制，近处保留高密度采样；灯杆、护栏、路牌按距离裁剪。
- perception budget
  - OpenCV 可降采样处理，例如 900x600 渲染，450x300 感知，再将结果放大显示。

## 9. FAILURE MODES
- what can break
  - 近处 z 值过小导致投影爆炸或多边形翻转。
  - 引擎盖遮挡过多，导致 OpenCV ROI 检测到车头边缘而不是车道线。
  - 车道线虚线滚动与道路速度不一致，产生漂移感。
  - 路灯/护栏没有按距离排序，出现远处物体盖住近处物体。
  - 曲率过大时道路边界穿插，车道线断裂。
  - 配置注释编码损坏，后续调参困难。
  - 主窗口和 OpenCV 窗口双显示导致低配机器掉帧。
- how to detect
  - 对 `project()` 增加 near clip 和 `visible` 标志，记录异常投影计数。
  - 开启调试叠层，显示道路边界投影点、ROI 多边形、车道中心线。
  - 记录 FPS、render_ms、cv_ms，超过阈值时 HUD 变色。
  - 固定随机种子回放同一段路，截图对比车道线和路灯排序。
  - 在 OpenCV 结果中统计 ROI 内边缘密度，异常过高时提示 ROI 被车头污染。

## 10. VALIDATION STRATEGY
- unit test
  - `HoodCamera.project()`：近裁剪、远裁剪、左右对称、z 增大时 scale 变小。
  - `RoadModel.get_center_x_at()`：边界查询、插值、空道路 fallback。
  - `VehicleController.update()`：无人为输入时回正，人工输入时软接管，松手后回到目标车道。
- profiling
  - 在主循环内记录 `road_update_ms`, `render_ms`, `cv_ms`, `frame_ms`。
  - 目标：900x600 下稳定 60 FPS，开启 OpenCV 后不低于 45 FPS。
- simulation (gem5)
  - 当前项目是桌面 Python/Pygame 演示，不建议把 gem5 作为首要验证手段。
  - 如果未来迁移到嵌入式平台，可将 `RoadModel + VehicleController + PerceptionView` 抽成无窗口逻辑，用固定输入帧做 gem5 运行成本评估。

## 11. VISUAL UPGRADE REQUIREMENTS（视觉升级要求）
- 引擎盖视角
  - 默认移除方向盘，保留底部车头/引擎盖。
  - 引擎盖占屏幕底部约 18% 到 24%，带轻微高光、中心隆起和转向横摆。
  - 相机位置绑定车辆横向位置，但加入 0.08 到 0.15 的平滑延迟，避免画面机械抖动。
- 道路观感
  - 道路从车头下方延伸，而不是从屏幕底边突然出现。
  - 车道线宽度随距离变化，近处更粗，远处融合进雾化。
  - 增加路肩、边线、护栏柱或反光桩，强化速度感和空间尺度。
- 动态展示
  - 路灯和护栏按道路曲率挂载，而不是简单跟随中心点。
  - 增加轻微相机震动：速度越高震动越明显，转向时产生很小的横向 roll。
  - 增加远处雾化、天际线渐变、路面明暗分段，减少平面感。
- 感知窗口
  - ROI 顶点由相机和车道边界投影生成，不再写死 `w//2 ± 95, 295`。
  - 可选叠加原图、边缘图、ROI mask、检测线，便于调参。

## 12. IMPLEMENTATION PLAN（建议落地顺序）
1. 清理编码与配置
   - 修复 `README.md`, `config.py`, `road_simu.py` 中损坏注释。
   - 新增相机配置：`CAMERA_HEIGHT`, `CAMERA_FORWARD_OFFSET`, `CAMERA_PITCH_DEG`, `NEAR_CLIP`, `FAR_CLIP`, `HOOD_HEIGHT_RATIO`。
2. 拆出 HoodCamera
   - 将 `Road.project()` 移出 `Road`，变成独立相机模块或类。
   - 所有道路、物体、OpenCV ROI 共用同一个投影结果。
3. 重写渲染分层
   - 顺序：background → far scenery → road strips → lane markings → roadside objects → hood → HUD。
   - 所有世界物体按 z 从远到近绘制。
4. 改造车头显示
   - 用引擎盖覆盖替代方向盘。
   - 车头位置跟随 `actual_steer` 做很小横摆，不直接大幅平移。
5. 增强动态对象
   - 添加护栏柱、反光桩、路牌挂点。
   - 对象生命周期与道路采样点绑定，避免突然出现/消失。
6. 同步感知 ROI
   - PerceptionView 读取相机投影后的左右车道边界，自动生成 mask。
   - OpenCV 显示降采样帧，减轻性能压力。
7. 加验证与调试
   - 增加投影测试、中心线插值测试、性能 HUD。
   - 固定随机种子录制 10 秒场景，用于视觉回归检查。

## 13. ACCEPTANCE CRITERIA（验收标准）
- 画面第一眼应明显是“引擎盖上方视角”，而不是驾驶舱方向盘视角。
- 车辆左右移动时，车头、道路、车道线、ROI 的相对关系稳定。
- 三车道在直道和弯道中都不应出现明显穿插、闪烁或透视反转。
- 开启 OpenCV 感知窗口后，主窗口仍能流畅运行。
- 代码结构中相机投影、道路模型、渲染和感知不再互相硬编码。
- 后续调参主要改 `Config`，不需要在渲染函数里查找 magic number。
