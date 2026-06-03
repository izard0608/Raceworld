#! /usr/bin/env python3
import cv2
import cv_bridge
import numpy
import rospy
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped

# 全局速度参数
MAX_SPEED = 1.22  # 全局最高速度上限
PROCESS_RESIZE_ENABLED = True
PROCESS_WIDTH = 480
PROCESS_HEIGHT = 360
VISION_BASE_WIDTH = 640.0
VISION_BASE_HEIGHT = 480.0

# 图像ROI参数：ratio越大，ROI越靠近图像下方，也就是看得越近
ROI_TOP_RATIO_BASE = 0.58  # 初始ROI顶部比例
ROI_TOP_RATIO_MIN = 0.3  # 动态ROI最远观察位置
ROI_TOP_RATIO_MAX = 0.7  # 动态ROI最近观察位置
ROI_RATIO_ALPHA = 0.20  # ROI位置低通滤波系数，越大变化越快
ROI_HEIGHT = 70  # ROI高度，单位像素
ROI_TARGET_DEADBAND = 0.035  # ROI目标比例变化小于该值时认为是同一个目标，避免层切换抖动
ROI_TARGET_CONFIRM_FRAMES = 3  # 新ROI目标连续出现该帧数后才切换，抑制曲率临界抖动
STARTUP_ROI_TOP_RATIO = 0.55  # 缓启动阶段固定ROI顶部比例，取画面下往上约55%附近
MIN_MASK_AREA = 160  # 黄色线mask最小有效面积，低于此值认为黄线不可靠
ROI_Y_GAIN_MIN = 0.55  # 质心靠近ROI顶部时的误差倍率
ROI_Y_GAIN_MAX = 1.15  # 质心靠近ROI底部时的误差倍率

# 路面兜底检测参数：用于黄色线不可靠时估计道路中心和曲率
ROAD_MIN_AREA = 1800  # 路面mask最小有效面积
ROAD_ROW_MIN_PIXELS = 60  # 单行路面像素数量下限，低于此行不参与拟合
ROAD_MIN_ROWS = 12  # 有效路面行数下限
ROAD_MAX_S = 75  # 路面HSV饱和度上限，偏灰色区域
ROAD_MIN_V = 35  # 路面HSV亮度下限
ROAD_MAX_V = 145  # 路面HSV亮度上限
ROAD_FEATURE_INTERVAL = 3  # 黄线可靠时每隔几帧更新一次路面兜底特征，降低每帧扫描开销
YELLOW_LOWER = numpy.array([26, 43, 46], dtype=numpy.uint8)
YELLOW_UPPER = numpy.array([34, 255, 255], dtype=numpy.uint8)
ROAD_LOWER = numpy.array([0, 0, ROAD_MIN_V], dtype=numpy.uint8)
ROAD_UPPER = numpy.array([179, ROAD_MAX_S, ROAD_MAX_V], dtype=numpy.uint8)

# 曲率与多ROI参数
CURVE_MIN_POINTS = 80  # 曲率拟合所需最少mask点数
CURVE_SPLIT_MIN_HEIGHT = 8  # 分层曲率评估的最小层高，避免过薄ROI误判
CURVE_CANVAS_TOP_RATIO = 0.25  # Only build the curve mask below this image ratio.
FAST_CURVE_LAYER_STEP = 0.03  # 每多一层曲率接近0时额外增加的速度
CURVE_ENTER = 0.12  # 曲率大于该值时从直道模式进入弯道模式
CURVE_EXIT = 0.04  # 曲率小于该值时从弯道模式退出到直道模式
TURN_EXIT_MAX_ERR = 0.35  # Hold turn mode until lateral error has settled.
CURVE_ROI_SCALE = 0.35  # 曲率映射到动态ROI位置的缩放量
ROAD_CURVE_BLEND = 0.35  # 黄色线和路面曲率融合时，路面曲率所占权重
GEOMETRY_WEIGHT_MAX = 0.65  # 曲率较大时几何朝向规划任务的最大权重
GEOMETRY_CURVE_SCALE = 0.18  # 曲率映射到几何任务权重的尺度
GEOMETRY_ERR_GAIN = 2.0  # 几何朝向误差转换成等效控制误差的倍率
GEOMETRY_ERR_LIMIT = 0.45  # 几何朝向等效误差限幅

# 直道高速与速度-转向耦合参数
FAST_SPEED_BOOST = 0.38  # 下1/2曲率接近0时的基础高速加成
FAST_MAX_ERR = 0.24  # 允许进入高速模式的最大横向误差
FAST_MAX_CURVE = 0.065  # 允许进入高速模式的最大曲率
FAST_CONTROL_CURVE_LIMIT = 0.105  # 控制曲率超过该值时禁止直道高速
STARTUP_FAST_BLOCK_FRAMES = 12  # 缓启动退出后暂时禁止高速，避免刚入正轨就冲弯
STRAIGHT_STEER_SPEED_GAIN_MIN = 0.88  # 直道最高速时PID转角倍率下限
STEER_SPEED_COUPLING = 0.22  # Scale speed down by normalized steering demand.

# 控制模型参数
DT = 1.0 / 30.0  # Fallback control period when image timestamps are invalid.
DT_MIN = 0.01  # Clamp very small frame intervals to reduce derivative spikes.
DT_MAX = 0.10  # Clamp stalls or simulator pauses to avoid integral jumps.
L = 0.164  # Wheelbase from car.xacro: 0.082311 - (-0.081663).

# 误差处理参数
LINE_TARGET_ERR = 0.0  # 目标横向误差，0表示让线位于图像中心
ERR_ALPHA = 0.55  # 横向误差低通滤波系数，越大越相信当前帧
STARTUP_TARGET_STEP = 0.015  # 缓启动阶段每帧把目标位置推向图像中心的步长
STARTUP_TARGET_DONE_ERR = 0.02  # 缓启动目标位置接近中心到该阈值内后退出
STARTUP_EXIT_RAW_ERR = 0.18  # 缓启动退出时，黄线实际位置也必须接近目标，避免目标到中心但车还没跟上
ROAD_FALLBACK_MAX_SPEED = 0.26  # 只靠路面兜底时的最高速度，防止黄线丢失后继续高速外冲
ROAD_FALLBACK_TURN_MAX_SPEED = 0.22  # 弯道且只靠路面兜底时的最高速度
LARGE_ERR_SPEED_LIMIT = 0.82  # 大横向误差时的最高速度
SEVERE_ERR_SPEED_LIMIT = 0.50  # 严重横向误差时的最高速度
LARGE_ERR_THRESHOLD = 0.65  # 进入大误差限速的误差阈值
SEVERE_ERR_THRESHOLD = 0.88  # 进入严重误差限速的误差阈值

STRAIGHT_PARAMS = {
    "target_speed": 0.88,  # 直道目标速度
    "min_speed": 0.20,  # 直道最低速度
    "max_steer": 0.38,  # 直道最大转角
    "deadband": 0.04,  # 直道误差死区，小于该值按0处理
    "turn_speed_drop": 0.085,  # 误差越大速度越低的降速系数
    "pid_kp": 0.88,  # 直道PID比例系数
    "pid_ki": 0.01,  # 直道PID积分系数
    "pid_kd": 0.15,  # 直道PID微分系数
    "pid_integral_limit": 0.60,  # PID积分限幅，防止积分饱和
    "steer_rate_limit": 0.17,  # 单帧转角变化限制
    "sharp_turn_err": 0.75,  # 大误差强制补转向的触发阈值
    "sharp_turn_steer": 0.18,  # 大误差时的最小转角
}

STARTUP_PARAMS = {
    "target_speed": 0.18,  # 缓启动目标速度，低速靠近黄线
    "min_speed": 0.10,  # 缓启动最低速度
    "max_steer": 0.18,  # 缓启动最大转角，允许初始偏在黄线右侧时更快收敛
    "deadband": 0.04,  # 缓启动误差死区
    "turn_speed_drop": 0.35,  # 缓启动误差降速系数
    "pid_kp": 0.70,  # 缓启动PID比例系数
    "pid_ki": 0.00,  # 缓启动不使用积分，避免初始偏差造成积分累积
    "pid_kd": 0.12,  # 缓启动PID微分系数
    "pid_integral_limit": 0.30,  # 缓启动积分限幅
    "steer_rate_limit": 0.06,  # 缓启动转向变化限制，起步更平滑
    "sharp_turn_err": 0.60,  # 缓启动大误差强制补转向阈值
    "sharp_turn_steer": 0.16,  # 缓启动大误差时的最小转角
}

TURN_PARAMS = {
    "target_speed": 1.00,  # 弯道目标速度
    "min_speed": 0.56,  # 弯道最低速度
    "max_steer": 0.72,  # 弯道最大转角
    "deadband": 0.00,  # 弯道误差死区
    "turn_speed_drop": 0.024,  # 弯道误差降速系数
    "pid_kp": 1.04,
    "pid_ki": 0.02,
    "pid_kd": 0.13,
    "pid_integral_limit": STRAIGHT_PARAMS["pid_integral_limit"],
    "steer_rate_limit": 0.25,  # 弯道单帧转角变化限制
    "sharp_turn_err": 0.55,  # 弯道大误差强制补转向阈值
    "sharp_turn_steer": 0.47,  # 弯道大误差时的最小转角
}

# 丢线恢复参数
RECOVERY_FRAMES = 42  # 最多尝试恢复的帧数
RECOVERY_EDGE_FRAMES = 16  # 边缘丢线后继续原方向恢复的帧数
RECOVERY_SPEED = 0.06  # 丢线恢复速度
RECOVERY_CONTINUE_GAIN = 0.90  # 边缘丢线时沿上次转角继续的比例
RECOVERY_REVERSE_GAIN = 0.85  # 非边缘丢线时反向搜索的比例
RECOVERY_MIN_STEER = 0.18  # 恢复时最小转角
RECOVERY_MAX_STEER = 0.34  # 恢复时最大转角
RECOVERY_SOURCE_STEER_MIN = 0.03  # 上次转角超过该值才作为恢复方向依据
RECOVERY_ERR_FALLBACK = 0.08  # 无有效上次转角时，用历史误差恢复的阈值
RECOVERY_EDGE_RAW_ERR = 0.82  # 原始误差超过该值认为是边缘丢线
RECOVERY_EDGE_LINE_ERR = 0.50  # 线误差超过该值认为是边缘丢线

# 调试输出参数
DEBUG_OUTPUT = True  # 是否输出ROS节流日志
DEBUG_PERIOD = 0.5  # 日志节流周期，单位秒
DEBUG_DRAW = False  # 是否在图像上绘制5行关键调试文字，默认关闭以提高帧率
DEBUG_SHOW_MASKS = False  # 是否显示ROI和路面mask调试窗口，跑速度时应关闭
DEBUG_DRAW_MARKERS = True  # 是否在camera画面上绘制目标线和检测质心
DEBUG_VERSION = "direct_line_near_curve_multiroi_v17_stable_speed_push"  # 当前调试版本标识

# 滑轨可调参数说明：
# 数值后带 x100 的滑轨采用百分制缩放，例如滑轨值 60 表示实际参数 0.60。
# Rec frames 是恢复帧数，Show/Draw 类滑轨是 0/1 开关。
# - Speed cap x100：全局速度上限，限制所有正常循迹模式的最高指令速度。
# - Error filter alpha x100：横向误差低通滤波系数，越大越相信当前帧，越小越平滑。
# - Steer gain floor x100：直道高速时的最小转向增益，防止高速下转向过猛。
# - Straight target speed x100：直道目标速度。
# - Straight min speed x100：直道最低速度，误差较大时也不会低于该值。
# - Straight max steer x100：直道最大转角限幅。
# - Straight error deadband x100：直道误差死区，小误差在控制中按 0 处理。
# - Straight error speed drop x100：直道误差降速系数，横向误差越大速度降得越多。
# - Straight PID Kp/Ki/Kd x100：直道 PID 的比例、积分、微分系数。
# - Straight steer rate x100：直道单帧最大转角变化量，限制方向变化速度。
# - Turn target speed x100：弯道目标速度。
# - Turn min speed x100：弯道最低速度，弯中降速后也不会低于该值。
# - Turn max steer x100：弯道最大转角限幅。
# - Turn error deadband x100：弯道误差死区。
# - Turn error speed drop x100：弯道误差降速系数。
# - Turn PID Kp/Ki/Kd x100：弯道 PID 的比例、积分、微分系数。
# - Turn steer rate x100：弯道单帧最大转角变化量。
# - Startup target speed x100：启动阶段目标速度。
# - Startup max steer x100：启动阶段最大转角限幅。
# - Startup PID Kp/Kd x100：启动阶段 PID 的比例和微分系数。
# - Recovery speed x100：丢线恢复时的速度。
# - Recovery min steer x100：丢线恢复时的最小搜索转角。
# - Recovery max steer x100：丢线恢复时的最大搜索转角。
# - Recovery continue gain x100：边缘丢线时沿上次转向继续搜索的增益。
# - Recovery reverse gain x100：非边缘丢线时反向搜索的增益。
# - Recovery frame count：丢线后最多执行恢复动作的帧数。
# - Show mask windows：是否显示 ROI 和路面 mask 调试窗口。
# - Draw debug text：是否在相机画面叠加调试文字。
# - Draw target markers：是否在相机画面绘制目标线和检测质心标记。

# OpenCV control tuning. Trackbars live in a separate window so the camera
# image remains visible while control parameters are adjusted.
CAMERA_WINDOW = "camera"
TUNING_WINDOW = "control tuning"
TUNING_ENABLED = True
TUNING_POLL_INTERVAL = 3
TUNING_TRACKBARS = (
    ("Max speed x100", "Speed cap x100", int(round(MAX_SPEED * 100)), 140),
    ("Err alpha x100", "Error filter alpha x100", int(round(ERR_ALPHA * 100)), 100),
    ("Steer gain min x100", "Steer gain floor x100", int(round(STRAIGHT_STEER_SPEED_GAIN_MIN * 100)), 100),
    ("S speed x100", "Straight target speed x100", int(round(STRAIGHT_PARAMS["target_speed"] * 100)), 120),
    ("S min x100", "Straight min speed x100", int(round(STRAIGHT_PARAMS["min_speed"] * 100)), 100),
    ("S steer x100", "Straight max steer x100", int(round(STRAIGHT_PARAMS["max_steer"] * 100)), 100),
    ("S dead x100", "Straight error deadband x100", int(round(STRAIGHT_PARAMS["deadband"] * 100)), 100),
    ("S drop x100", "Straight error speed drop x100", int(round(STRAIGHT_PARAMS["turn_speed_drop"] * 100)), 100),
    ("S kp x100", "Straight PID Kp x100", int(round(STRAIGHT_PARAMS["pid_kp"] * 100)), 300),
    ("S ki x100", "Straight PID Ki x100", int(round(STRAIGHT_PARAMS["pid_ki"] * 100)), 100),
    ("S kd x100", "Straight PID Kd x100", int(round(STRAIGHT_PARAMS["pid_kd"] * 100)), 200),
    ("S rate x100", "Straight steer rate x100", int(round(STRAIGHT_PARAMS["steer_rate_limit"] * 100)), 100),
    ("T speed x100", "Turn target speed x100", int(round(TURN_PARAMS["target_speed"] * 100)), 120),
    ("T min x100", "Turn min speed x100", int(round(TURN_PARAMS["min_speed"] * 100)), 100),
    ("T steer x100", "Turn max steer x100", int(round(TURN_PARAMS["max_steer"] * 100)), 100),
    ("T dead x100", "Turn error deadband x100", int(round(TURN_PARAMS["deadband"] * 100)), 100),
    ("T drop x100", "Turn error speed drop x100", int(round(TURN_PARAMS["turn_speed_drop"] * 100)), 100),
    ("T kp x100", "Turn PID Kp x100", int(round(TURN_PARAMS["pid_kp"] * 100)), 300),
    ("T ki x100", "Turn PID Ki x100", int(round(TURN_PARAMS["pid_ki"] * 100)), 100),
    ("T kd x100", "Turn PID Kd x100", int(round(TURN_PARAMS["pid_kd"] * 100)), 200),
    ("T rate x100", "Turn steer rate x100", int(round(TURN_PARAMS["steer_rate_limit"] * 100)), 100),
    ("U speed x100", "Startup target speed x100", int(round(STARTUP_PARAMS["target_speed"] * 100)), 80),
    ("U steer x100", "Startup max steer x100", int(round(STARTUP_PARAMS["max_steer"] * 100)), 100),
    ("U kp x100", "Startup PID Kp x100", int(round(STARTUP_PARAMS["pid_kp"] * 100)), 300),
    ("U kd x100", "Startup PID Kd x100", int(round(STARTUP_PARAMS["pid_kd"] * 100)), 200),
    ("Rec speed x100", "Recovery speed x100", int(round(RECOVERY_SPEED * 100)), 50),
    ("Rec min steer x100", "Recovery min steer x100", int(round(RECOVERY_MIN_STEER * 100)), 100),
    ("Rec max steer x100", "Recovery max steer x100", int(round(RECOVERY_MAX_STEER * 100)), 100),
    ("Rec cont x100", "Recovery continue gain x100", int(round(RECOVERY_CONTINUE_GAIN * 100)), 200),
    ("Rec rev x100", "Recovery reverse gain x100", int(round(RECOVERY_REVERSE_GAIN * 100)), 200),
    ("Rec frames", "Recovery frame count", RECOVERY_FRAMES, 120),
    ("Show masks", "Show mask windows", int(DEBUG_SHOW_MASKS), 1),
    ("Draw debug", "Draw debug text", int(DEBUG_DRAW), 1),
    ("Draw marks", "Draw target markers", int(DEBUG_DRAW_MARKERS), 1),
)
TUNING_LABELS = {name: label for name, label, _, _ in TUNING_TRACKBARS}
TUNING_DEFAULTS = {name: default for name, _, default, _ in TUNING_TRACKBARS}
tuning_values = dict(TUNING_DEFAULTS)
tuning_poll_count = 0
tuning_cache_valid = False
tuned_max_speed = MAX_SPEED
tuned_err_alpha = ERR_ALPHA
tuned_steer_gain_min = STRAIGHT_STEER_SPEED_GAIN_MIN
tuned_straight_params = dict(STRAIGHT_PARAMS)
tuned_turn_params = dict(TURN_PARAMS)
tuned_startup_params = dict(STARTUP_PARAMS)
tuned_recovery_params = {
    "speed": RECOVERY_SPEED,
    "min_steer": RECOVERY_MIN_STEER,
    "max_steer": RECOVERY_MAX_STEER,
    "continue_gain": RECOVERY_CONTINUE_GAIN,
    "reverse_gain": RECOVERY_REVERSE_GAIN,
    "frames": RECOVERY_FRAMES,
}
tuned_show_masks = DEBUG_SHOW_MASKS
tuned_draw_debug = DEBUG_DRAW
tuned_draw_markers = DEBUG_DRAW_MARKERS

prev_err = 0.0
prev_steer = 0.0
last_raw_err = 0.0
last_line_err = 0.0
roi_top_ratio = ROI_TOP_RATIO_BASE
roi_stable_target_ratio = ROI_TOP_RATIO_BASE
roi_candidate_target_ratio = ROI_TOP_RATIO_BASE
roi_candidate_count = 0
pid_integral = 0.0
pid_prev_err = 0.0
control_dt = DT
last_frame_time = None
control_mode = "straight"
lost_count = 0
startup_active = True
startup_stable_count = 0
startup_target_err = None
normal_frame_count = 0
debug_info = {}
tuning_initialized = False
camera_window_initialized = False
vision_frame_count = 0
last_debug_log_time = 0.0
bridge = None


def on_tuning_change(_value):
    global tuning_cache_valid
    tuning_cache_valid = False


def resize_for_processing(frame):
    if not PROCESS_RESIZE_ENABLED:
        debug_info["input_w"] = frame.shape[1]
        debug_info["input_h"] = frame.shape[0]
        debug_info["process_w"] = frame.shape[1]
        debug_info["process_h"] = frame.shape[0]
        debug_info["process_resized"] = False
        return frame

    input_h, input_w = frame.shape[:2]
    debug_info["input_w"] = input_w
    debug_info["input_h"] = input_h
    debug_info["process_w"] = PROCESS_WIDTH
    debug_info["process_h"] = PROCESS_HEIGHT
    if input_w == PROCESS_WIDTH and input_h == PROCESS_HEIGHT:
        debug_info["process_resized"] = False
        return frame

    debug_info["process_resized"] = True
    return cv2.resize(frame, (PROCESS_WIDTH, PROCESS_HEIGHT), interpolation=cv2.INTER_AREA)


def image_width_scale():
    return max(0.1, float(debug_info.get("image_w", VISION_BASE_WIDTH)) / VISION_BASE_WIDTH)


def image_height_scale():
    return max(0.1, float(debug_info.get("image_h", VISION_BASE_HEIGHT)) / VISION_BASE_HEIGHT)


def image_area_scale():
    return image_width_scale() * image_height_scale()


def update_control_dt(stamp=None):
    global control_dt, last_frame_time

    if stamp is None:
        stamp = rospy.Time.now()

    stamp_sec = stamp.to_sec()
    if stamp_sec <= 0.0:
        stamp_sec = rospy.Time.now().to_sec()

    if last_frame_time is None or stamp_sec <= last_frame_time:
        control_dt = DT
    else:
        raw_dt = stamp_sec - last_frame_time
        control_dt = max(DT_MIN, min(DT_MAX, raw_dt))

    last_frame_time = stamp_sec
    debug_info["control_dt"] = control_dt
    return control_dt


def ensure_tuning_controls():
    global tuning_initialized

    if tuning_initialized or not TUNING_ENABLED:
        return

    cv2.namedWindow(TUNING_WINDOW, 0)
    for name, label, default, maximum in TUNING_TRACKBARS:
        cv2.createTrackbar(label, TUNING_WINDOW, default, maximum, on_tuning_change)
    tuning_initialized = True
    refresh_tuning_cache(force=True)


def refresh_tuning_cache(force=False):
    global tuning_poll_count, tuning_cache_valid

    if not TUNING_ENABLED or not tuning_initialized:
        if not tuning_cache_valid or force:
            rebuild_tuned_controls()
        tuning_cache_valid = True
        return

    if tuning_cache_valid and not force:
        tuning_poll_count += 1
        if tuning_poll_count < TUNING_POLL_INTERVAL:
            return

    tuning_poll_count = 0
    for name, label, _default, _maximum in TUNING_TRACKBARS:
        try:
            tuning_values[name] = cv2.getTrackbarPos(label, TUNING_WINDOW)
        except cv2.error:
            tuning_values[name] = TUNING_DEFAULTS[name]
    rebuild_tuned_controls()
    tuning_cache_valid = True


def get_tuning_value(name):
    return tuning_values.get(name, TUNING_DEFAULTS[name])


def get_tuning_float(name, scale=100.0, minimum=0.0, maximum=None):
    value = get_tuning_value(name) / scale
    value = max(minimum, value)
    if maximum is not None:
        value = min(maximum, value)
    return value


def fixed_yellow_bounds():
    debug_info["yellow_h_min"] = int(YELLOW_LOWER[0])
    debug_info["yellow_h_max"] = int(YELLOW_UPPER[0])
    debug_info["yellow_s_min"] = int(YELLOW_LOWER[1])
    debug_info["yellow_s_max"] = int(YELLOW_UPPER[1])
    debug_info["yellow_v_min"] = int(YELLOW_LOWER[2])
    debug_info["yellow_v_max"] = int(YELLOW_UPPER[2])
    return YELLOW_LOWER, YELLOW_UPPER


def fixed_road_bounds():
    debug_info["road_s_max"] = ROAD_MAX_S
    debug_info["road_v_min"] = ROAD_MIN_V
    debug_info["road_v_max"] = ROAD_MAX_V
    return ROAD_LOWER, ROAD_UPPER


def current_min_mask_area():
    value = max(20, int(round(MIN_MASK_AREA * image_area_scale())))
    debug_info["min_mask_area"] = value
    return value


def current_road_min_area():
    value = max(120, int(round(ROAD_MIN_AREA * image_area_scale())))
    debug_info["road_min_area"] = value
    return value


def current_roi_height():
    value = max(10, int(round(ROI_HEIGHT * image_height_scale())))
    debug_info["roi_height"] = value
    return value


def current_road_row_min_pixels():
    value = max(20, int(round(ROAD_ROW_MIN_PIXELS * image_width_scale())))
    debug_info["road_row_min_pixels"] = value
    return value


def current_curve_min_points():
    value = max(30, int(round(CURVE_MIN_POINTS * image_area_scale())))
    debug_info["curve_min_points"] = value
    return value


def current_curve_split_min_height():
    value = max(6, int(round(CURVE_SPLIT_MIN_HEIGHT * image_height_scale())))
    debug_info["curve_split_min_height"] = value
    return value


def current_max_speed():
    debug_info["tune_max_speed"] = tuned_max_speed
    return tuned_max_speed


def current_err_alpha():
    debug_info["tune_err_alpha"] = tuned_err_alpha
    return tuned_err_alpha


def current_steer_gain_min():
    debug_info["tune_steer_gain_min"] = tuned_steer_gain_min
    return tuned_steer_gain_min


def build_tuned_mode_params(base_params, prefix):
    params = dict(base_params)
    params["target_speed"] = get_tuning_float("{} speed x100".format(prefix), minimum=0.0)
    if prefix in ("S", "T"):
        params["min_speed"] = get_tuning_float("{} min x100".format(prefix), minimum=0.0)
        params["deadband"] = get_tuning_float("{} dead x100".format(prefix), minimum=0.0)
        params["turn_speed_drop"] = get_tuning_float("{} drop x100".format(prefix), minimum=0.0)
        params["steer_rate_limit"] = get_tuning_float("{} rate x100".format(prefix), minimum=0.0)

    params["max_steer"] = get_tuning_float("{} steer x100".format(prefix), minimum=0.0)
    params["pid_kp"] = get_tuning_float("{} kp x100".format(prefix), minimum=0.0)
    if prefix in ("S", "T"):
        params["pid_ki"] = get_tuning_float("{} ki x100".format(prefix), minimum=0.0)
    params["pid_kd"] = get_tuning_float("{} kd x100".format(prefix), minimum=0.0)

    if params["min_speed"] > params["target_speed"]:
        params["min_speed"] = params["target_speed"]

    return params


def write_mode_tuning_debug(params, label):
    debug_info["tune_mode"] = label
    debug_info["tune_target_speed"] = params["target_speed"]
    debug_info["tune_min_speed"] = params["min_speed"]
    debug_info["tune_max_steer"] = params["max_steer"]
    debug_info["tune_deadband"] = params["deadband"]
    debug_info["tune_speed_drop"] = params["turn_speed_drop"]
    debug_info["tune_pid_kp"] = params["pid_kp"]
    debug_info["tune_pid_ki"] = params["pid_ki"]
    debug_info["tune_pid_kd"] = params["pid_kd"]
    debug_info["tune_steer_rate"] = params["steer_rate_limit"]


def rebuild_tuned_controls():
    global tuned_max_speed, tuned_err_alpha, tuned_steer_gain_min
    global tuned_straight_params, tuned_turn_params, tuned_startup_params, tuned_recovery_params
    global tuned_show_masks, tuned_draw_debug, tuned_draw_markers

    tuned_max_speed = get_tuning_float("Max speed x100", minimum=0.0)
    tuned_err_alpha = get_tuning_float("Err alpha x100", minimum=0.0, maximum=1.0)
    tuned_steer_gain_min = get_tuning_float("Steer gain min x100", minimum=0.0, maximum=1.0)
    tuned_straight_params = build_tuned_mode_params(STRAIGHT_PARAMS, "S")
    tuned_turn_params = build_tuned_mode_params(TURN_PARAMS, "T")
    tuned_startup_params = build_tuned_mode_params(STARTUP_PARAMS, "U")

    min_steer = get_tuning_float("Rec min steer x100", minimum=0.0)
    max_steer = get_tuning_float("Rec max steer x100", minimum=min_steer)
    tuned_recovery_params = {
        "speed": get_tuning_float("Rec speed x100", minimum=0.0),
        "min_steer": min_steer,
        "max_steer": max_steer,
        "continue_gain": get_tuning_float("Rec cont x100", minimum=0.0),
        "reverse_gain": get_tuning_float("Rec rev x100", minimum=0.0),
        "frames": max(0, get_tuning_value("Rec frames")),
    }
    tuned_show_masks = get_tuning_value("Show masks") > 0
    tuned_draw_debug = get_tuning_value("Draw debug") > 0
    tuned_draw_markers = get_tuning_value("Draw marks") > 0

    debug_info["tune_max_speed"] = tuned_max_speed
    debug_info["tune_err_alpha"] = tuned_err_alpha
    debug_info["tune_steer_gain_min"] = tuned_steer_gain_min
    debug_info["tune_recovery_speed"] = tuned_recovery_params["speed"]
    debug_info["tune_recovery_min_steer"] = tuned_recovery_params["min_steer"]
    debug_info["tune_recovery_max_steer"] = tuned_recovery_params["max_steer"]
    debug_info["tune_recovery_continue_gain"] = tuned_recovery_params["continue_gain"]
    debug_info["tune_recovery_reverse_gain"] = tuned_recovery_params["reverse_gain"]
    debug_info["tune_recovery_frames"] = tuned_recovery_params["frames"]


def current_straight_params():
    params = dict(tuned_straight_params)
    write_mode_tuning_debug(params, "straight")
    return params


def current_turn_params():
    params = dict(tuned_turn_params)
    write_mode_tuning_debug(params, "turn")
    return params


def current_startup_params():
    params = dict(tuned_startup_params)
    write_mode_tuning_debug(params, "startup")
    return params


def current_recovery_params():
    params = dict(tuned_recovery_params)
    debug_info["tune_recovery_speed"] = params["speed"]
    debug_info["tune_recovery_min_steer"] = params["min_steer"]
    debug_info["tune_recovery_max_steer"] = params["max_steer"]
    debug_info["tune_recovery_continue_gain"] = params["continue_gain"]
    debug_info["tune_recovery_reverse_gain"] = params["reverse_gain"]
    debug_info["tune_recovery_frames"] = params["frames"]
    return params


def current_show_masks():
    return tuned_show_masks


def current_draw_debug():
    return tuned_draw_debug


def current_draw_markers():
    return tuned_draw_markers


def ensure_camera_window():
    global camera_window_initialized

    if camera_window_initialized:
        return

    cv2.namedWindow(CAMERA_WINDOW, 0)
    camera_window_initialized = True


def show_camera_frame(image):
    ensure_tuning_controls()
    ensure_camera_window()
    cv2.imshow(CAMERA_WINDOW, image)
    cv2.waitKey(1)


def roi_bounds(h, ratio):
    search_top = int(ratio * h)
    search_bot = min(h, search_top + current_roi_height())
    return search_top, search_bot


def apply_roi(mask, h, w, ratio):
    search_top, search_bot = roi_bounds(h, ratio)
    roi_mask = numpy.zeros_like(mask)
    if search_bot > search_top:
        roi_mask[search_top:search_bot, 0:w] = mask[search_top:search_bot, 0:w]
    return roi_mask, search_top, search_bot


def threshold_hsv_region(hsv, lower, upper, top, bot, median_kernel=0, gaussian_kernel=0):
    h, w = hsv.shape[:2]
    top = max(0, min(h, top))
    bot = max(top, min(h, bot))
    mask = numpy.zeros((h, w), dtype=numpy.uint8)
    if bot <= top:
        return mask

    region = cv2.inRange(hsv[top:bot, 0:w], lower, upper)
    if median_kernel > 1:
        region = cv2.medianBlur(region, median_kernel)
    if gaussian_kernel > 1:
        region = cv2.GaussianBlur(region, (gaussian_kernel, gaussian_kernel), 0)
    mask[top:bot, 0:w] = region
    return mask


def active_roi_bounds(h, w):
    active_ratio = STARTUP_ROI_TOP_RATIO if startup_active else roi_top_ratio
    search_top, search_bot = roi_bounds(h, active_ratio)
    debug_info["roi_top_ratio"] = active_ratio
    debug_info["roi_source"] = "startup" if startup_active else "dynamic"
    debug_info["roi_top"] = search_top
    debug_info["roi_bot"] = search_bot
    return active_ratio, search_top, search_bot


def set_roi_forward(h, w, mask):
    active_ratio, search_top, search_bot = active_roi_bounds(h, w)
    mask, search_top, search_bot = apply_roi(mask, h, w, active_ratio)
    return mask


def fit_curvature_region(mask_region, w, roi_top, roi_bot, min_points):
    ys, xs = numpy.nonzero(mask_region)
    point_count = len(xs)
    if point_count < min_points:
        return False, 0.0, point_count

    ys = ys.astype(float) + roi_top
    y_span = max(1.0, float(roi_bot - roi_top))
    y_norm = 2.0 * (ys - roi_top) / y_span - 1.0
    x_norm = (xs.astype(float) - (w / 2.0)) / (w / 2.0)

    try:
        curve_a, curve_b, _ = numpy.polyfit(y_norm, x_norm, 2)
    except (TypeError, ValueError, numpy.linalg.LinAlgError):
        return False, 0.0, point_count

    curvature = (2.0 * curve_a) / ((1.0 + curve_b * curve_b) ** 1.5)
    return True, float(curvature), point_count


def fit_curvature(mask, w, roi_top, roi_bot, min_points):
    h = mask.shape[0]
    roi_top = max(0, min(h, roi_top))
    roi_bot = max(roi_top, min(h, roi_bot))
    return fit_curvature_region(mask[roi_top:roi_bot, 0:w], w, roi_top, roi_bot, min_points)


def estimate_canvas_curve_layers(canvas_mask, w):
    h = canvas_mask.shape[0]
    curve_min_points = current_curve_min_points()
    split_min_height = current_curve_split_min_height()
    layer_curves = []
    curve_layers = []
    fast_layers = 0
    segment_bot = h
    segment_top = h // 2

    while segment_top >= 0 and segment_bot - segment_top >= split_min_height:
        segment_height = max(1, segment_bot - segment_top)
        valid_layer, layer_curve, layer_points = fit_curvature(
            canvas_mask,
            w,
            segment_top,
            segment_bot,
            max(12, min(curve_min_points, int(curve_min_points * segment_height / max(1, h)))),
        )
        if not valid_layer:
            break

        layer_curves.append(layer_curve)
        curve_layers.append({
            "top": segment_top,
            "bot": segment_bot,
            "curve": layer_curve,
            "points": layer_points,
        })
        if abs(layer_curve) >= FAST_MAX_CURVE:
            break

        fast_layers += 1
        segment_bot = segment_top
        segment_top //= 2

    return fast_layers, layer_curves, curve_layers


def estimate_mode_curvature(canvas_mask, w):
    h = canvas_mask.shape[0]
    roi_top = h // 2
    roi_bot = h

    valid, curvature, point_count = fit_curvature(canvas_mask, w, roi_top, roi_bot, current_curve_min_points())
    debug_info["mode_curve_valid"] = valid
    debug_info["mode_curvature"] = curvature if valid else 0.0
    debug_info["mode_curve_points"] = point_count
    return debug_info["mode_curvature"]


def estimate_line_curvature(mask, w, canvas_mask=None):
    roi_top = debug_info.get("roi_top", 0)
    roi_bot = debug_info.get("roi_bot", mask.shape[0])
    valid, curvature, point_count = fit_curvature(mask, w, roi_top, roi_bot, current_curve_min_points())
    debug_info["curve_points"] = point_count
    if not valid:
        debug_info["curve_valid"] = False
        debug_info["line_curvature"] = 0.0
        debug_info["fast_curve_layers"] = 0
        debug_info["fast_curve_required"] = False
        debug_info["fast_layer_curves"] = []
        debug_info["curve_layers"] = []
        return 0.0

    if canvas_mask is None:
        canvas_mask = mask
    fast_layers, layer_curves, curve_layers = estimate_canvas_curve_layers(canvas_mask, w)

    debug_info["curve_valid"] = True
    debug_info["line_curvature"] = curvature
    if startup_active:
        debug_info["fast_curve_layers"] = 0
        debug_info["fast_curve_required"] = False
        debug_info["fast_layer_curves"] = []
        debug_info["curve_layers"] = []
        return curvature

    debug_info["fast_curve_layers"] = fast_layers
    debug_info["fast_curve_required"] = fast_layers > 0
    debug_info["fast_layer_curves"] = layer_curves
    debug_info["curve_layers"] = curve_layers
    return curvature


def estimate_road_features(hsv, h, w):
    lower_road, upper_road = fixed_road_bounds()
    _active_ratio, roi_top, roi_bot = active_roi_bounds(h, w)
    road_mask = threshold_hsv_region(hsv, lower_road, upper_road, roi_top, roi_bot, median_kernel=5)
    kernel = numpy.ones((5, 5), numpy.uint8)
    if roi_bot > roi_top:
        road_mask[roi_top:roi_bot, 0:w] = cv2.morphologyEx(
            road_mask[roi_top:roi_bot, 0:w],
            cv2.MORPH_CLOSE,
            kernel,
        )

    road_area = cv2.countNonZero(road_mask)
    debug_info["road_area"] = road_area
    if road_area < current_road_min_area():
        debug_info["road_valid"] = False
        debug_info["road_rows"] = 0
        debug_info["road_err"] = 0.0
        debug_info["road_curvature"] = 0.0
        debug_info["road_heading"] = 0.0
        return road_mask

    roi_top = debug_info.get("roi_top", 0)
    roi_bot = debug_info.get("roi_bot", h)
    row_y = []
    row_x = []
    row_min_pixels = current_road_row_min_pixels()
    for y in range(roi_top, roi_bot):
        xs = numpy.flatnonzero(road_mask[y])
        if len(xs) >= row_min_pixels:
            row_y.append(float(y))
            row_x.append(0.5 * float(xs[0] + xs[-1]))

    debug_info["road_rows"] = len(row_y)
    if len(row_y) < ROAD_MIN_ROWS:
        debug_info["road_valid"] = False
        debug_info["road_err"] = 0.0
        debug_info["road_curvature"] = 0.0
        debug_info["road_heading"] = 0.0
        return road_mask

    row_y = numpy.array(row_y)
    row_x = numpy.array(row_x)
    y_span = max(1.0, float(roi_bot - roi_top))
    y_norm = 2.0 * (row_y - roi_top) / y_span - 1.0
    x_norm = (row_x - (w / 2.0)) / (w / 2.0)

    try:
        curve_a, curve_b, _ = numpy.polyfit(y_norm, x_norm, 2)
    except (TypeError, ValueError, numpy.linalg.LinAlgError):
        debug_info["road_valid"] = False
        debug_info["road_err"] = 0.0
        debug_info["road_curvature"] = 0.0
        debug_info["road_heading"] = 0.0
        return road_mask

    tail_count = min(5, len(x_norm))
    road_err = float(numpy.mean(x_norm[-tail_count:]))
    road_curvature = float((2.0 * curve_a) / ((1.0 + curve_b * curve_b) ** 1.5))
    road_heading = float(2.0 * curve_a + curve_b)
    debug_info["road_valid"] = True
    debug_info["road_err"] = road_err
    debug_info["road_curvature"] = road_curvature
    debug_info["road_heading"] = road_heading
    return road_mask


def update_control_curvature():
    yellow_valid = debug_info.get("curve_valid", False)
    road_valid = debug_info.get("road_valid", False)
    yellow_curve = debug_info.get("line_curvature", 0.0)
    road_curve = debug_info.get("road_curvature", 0.0)

    if yellow_valid and road_valid:
        if abs(yellow_curve) < CURVE_EXIT or yellow_curve * road_curve >= 0.0:
            control_curve = (1.0 - ROAD_CURVE_BLEND) * yellow_curve + ROAD_CURVE_BLEND * road_curve
            curve_source = "yellow+road"
        else:
            control_curve = yellow_curve
            curve_source = "yellow"
    elif yellow_valid:
        control_curve = yellow_curve
        curve_source = "yellow"
    elif road_valid:
        control_curve = road_curve
        curve_source = "road"
    else:
        control_curve = 0.0
        curve_source = "none"

    debug_info["control_curvature"] = control_curve
    debug_info["control_curve_valid"] = yellow_valid or road_valid
    debug_info["curve_source"] = curve_source
    return control_curve


def update_roi_top_ratio(line_found, curvature):
    global roi_top_ratio, roi_stable_target_ratio, roi_candidate_target_ratio, roi_candidate_count

    if startup_active:
        target_ratio = roi_top_ratio
        roi_top_ratio = target_ratio
        roi_stable_target_ratio = target_ratio
        roi_candidate_target_ratio = target_ratio
        roi_candidate_count = 0
        debug_info["roi_target_ratio"] = target_ratio
        debug_info["roi_curve_intensity"] = 0.0
        debug_info["roi_target_source"] = "startup"
        debug_info["roi_layer_target_y"] = -1
        return

    if not line_found:
        target_ratio = ROI_TOP_RATIO_MIN
        curve_intensity = 0.0
        debug_info["roi_target_source"] = "lost"
        debug_info["roi_layer_target_y"] = -1
    else:
        curve_ratio = min(1.0, abs(curvature) / CURVE_ROI_SCALE)
        curve_intensity = numpy.sqrt(curve_ratio)
        target_ratio = ROI_TOP_RATIO_MIN + curve_intensity * (ROI_TOP_RATIO_MAX - ROI_TOP_RATIO_MIN)
        debug_info["roi_target_source"] = "nonlinear"
        debug_info["roi_layer_target_y"] = -1

        target_ratio = max(ROI_TOP_RATIO_MIN, min(ROI_TOP_RATIO_MAX, target_ratio))

    if abs(target_ratio - roi_stable_target_ratio) > ROI_TARGET_DEADBAND:
        if abs(target_ratio - roi_candidate_target_ratio) <= ROI_TARGET_DEADBAND:
            roi_candidate_count += 1
        else:
            roi_candidate_target_ratio = target_ratio
            roi_candidate_count = 1

        if roi_candidate_count >= ROI_TARGET_CONFIRM_FRAMES:
            roi_stable_target_ratio = roi_candidate_target_ratio
            roi_candidate_count = 0
    else:
        roi_stable_target_ratio = target_ratio
        roi_candidate_target_ratio = target_ratio
        roi_candidate_count = 0

    target_ratio = roi_stable_target_ratio
    roi_top_ratio = (1.0 - ROI_RATIO_ALPHA) * roi_top_ratio + ROI_RATIO_ALPHA * target_ratio
    debug_info["roi_target_ratio"] = target_ratio
    debug_info["roi_candidate_ratio"] = roi_candidate_target_ratio
    debug_info["roi_candidate_count"] = roi_candidate_count
    debug_info["roi_curve_intensity"] = curve_intensity if line_found else 0.0


def estimate_lane_error(image):
    global debug_info, vision_frame_count

    vision_frame_count += 1

    h, w = image.shape[:2]
    debug_info["image_h"] = h
    debug_info["image_w"] = w

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow, upper_yellow = fixed_yellow_bounds()
    _active_ratio, roi_top, roi_bot = active_roi_bounds(h, w)
    mask = threshold_hsv_region(
        hsv,
        lower_yellow,
        upper_yellow,
        roi_top,
        roi_bot,
        median_kernel=5,
        gaussian_kernel=5,
    )
    curve_canvas_top = int(CURVE_CANVAS_TOP_RATIO * h)
    debug_info["curve_canvas_top"] = curve_canvas_top
    yellow_canvas_mask = threshold_hsv_region(
        hsv,
        lower_yellow,
        upper_yellow,
        curve_canvas_top,
        h,
        median_kernel=5,
        gaussian_kernel=5,
    )
    estimate_mode_curvature(yellow_canvas_mask, w)

    M = cv2.moments(mask)
    debug_info["mask_area"] = M["m00"] / 255.0
    show_masks = current_show_masks()
    yellow_reliable = debug_info["mask_area"] >= current_min_mask_area()
    force_road_update = show_masks or not yellow_reliable
    road_update_due = vision_frame_count % max(1, ROAD_FEATURE_INTERVAL) == 0
    road_mask = None
    if force_road_update or road_update_due:
        road_mask = estimate_road_features(hsv, h, w)
        debug_info["road_feature_updated"] = True
    else:
        debug_info["road_feature_updated"] = False

    if show_masks:
        cv2.namedWindow("region of interest", 0)
        cv2.imshow("region of interest", mask)
        if road_mask is not None:
            cv2.namedWindow("road mask", 0)
            cv2.imshow("road mask", road_mask)

    if not yellow_reliable:
        debug_info["curve_valid"] = False
        debug_info["line_curvature"] = 0.0
        debug_info["fast_curve_layers"] = 0
        debug_info["fast_layer_curves"] = []
        debug_info["curve_layers"] = []
        control_curvature = update_control_curvature()

        update_roi_top_ratio(debug_info.get("road_valid", False), control_curvature)

        if debug_info.get("road_valid", False):
            raw_err = debug_info.get("road_err", 0.0)
            err = raw_err - LINE_TARGET_ERR
            debug_info["line_found"] = True
            debug_info["vision_source"] = "road"
            debug_info["reject_reason"] = "road_fallback"
            debug_info["cx"] = int((raw_err * 0.5 + 0.5) * w)
            debug_info["cy"] = -1
            debug_info["roi_y_norm"] = 1.0
            debug_info["roi_y_gain"] = 1.0
            debug_info["raw_err"] = raw_err
            debug_info["line_target_err"] = LINE_TARGET_ERR
            debug_info["line_err"] = err
            return err, w

        debug_info["line_found"] = False
        debug_info["vision_source"] = "none"
        debug_info["reject_reason"] = "small_area"
        debug_info["cx"] = -1
        update_roi_top_ratio(False, 0.0)
        return None, w

    curvature = estimate_line_curvature(mask, w, yellow_canvas_mask)
    control_curvature = update_control_curvature()

    cx = int(M["m10"] / M["m00"])
    raw_err = (cx - (w / 2)) / (w / 2)
    err = raw_err - LINE_TARGET_ERR
    debug_info["vision_source"] = "yellow"
    cy = int(M["m01"] / M["m00"])
    roi_top = debug_info.get("roi_top", 0)
    roi_bot = debug_info.get("roi_bot", h)
    roi_y_span = max(1.0, float(roi_bot - roi_top))
    roi_y_norm = (float(cy) - roi_top) / roi_y_span
    roi_y_norm = max(0.0, min(1.0, roi_y_norm))
    roi_y_gain = ROI_Y_GAIN_MIN + (ROI_Y_GAIN_MAX - ROI_Y_GAIN_MIN) * roi_y_norm

    update_roi_top_ratio(True, control_curvature)
    debug_info["line_found"] = True
    debug_info["reject_reason"] = "ok"
    debug_info["cx"] = cx
    debug_info["cy"] = cy
    debug_info["roi_y_norm"] = roi_y_norm
    debug_info["roi_y_gain"] = roi_y_gain
    debug_info["raw_err"] = raw_err
    debug_info["line_target_err"] = LINE_TARGET_ERR
    debug_info["line_err"] = err
    return err, w


def limit_steer_rate(target_steer, params):
    if abs(target_steer) < 1e-6:
        return 0.45 * prev_steer

    steer_delta = target_steer - prev_steer
    steer_rate_limit = params["steer_rate_limit"]
    steer_delta = max(-steer_rate_limit, min(steer_rate_limit, steer_delta))
    return prev_steer + steer_delta


def recovery_steering(params):
    if abs(prev_steer) > RECOVERY_SOURCE_STEER_MIN:
        edge_lost = abs(last_raw_err) > RECOVERY_EDGE_RAW_ERR or abs(last_line_err) > RECOVERY_EDGE_LINE_ERR
        if edge_lost and lost_count <= RECOVERY_EDGE_FRAMES:
            recover_steer = prev_steer * params["continue_gain"]
            recovery_mode = "continue_edge"
        else:
            recover_steer = -prev_steer * params["reverse_gain"]
            recovery_mode = "reverse"
        if abs(recover_steer) < params["min_steer"]:
            recover_steer = params["min_steer"] if recover_steer > 0.0 else -params["min_steer"]
    elif abs(prev_err) > RECOVERY_ERR_FALLBACK:
        recover_steer = params["min_steer"] if prev_err > 0.0 else -params["min_steer"]
        recovery_mode = "reverse_err"
    else:
        return None, "stop"

    recover_steer = max(-params["max_steer"], min(params["max_steer"], recover_steer))
    return recover_steer, recovery_mode


def straight_speed_steer_gain(speed_cmd, params):
    speed_span = max(1e-6, params["target_speed"] - params["min_speed"])
    speed_ratio = (speed_cmd - params["min_speed"]) / speed_span
    speed_ratio = max(0.0, min(1.0, speed_ratio))
    min_gain = current_steer_gain_min()
    gain = 1.0 - (1.0 - min_gain) * speed_ratio
    debug_info["speed_steer_gain"] = gain
    return gain


def apply_steer_speed_coupling(speed_cmd, target_speed, steer, params):
    steer_ratio = abs(steer) / max(1e-6, params["max_steer"])
    steer_ratio = max(0.0, min(1.0, steer_ratio))
    speed_scale = 1.0 - STEER_SPEED_COUPLING * steer_ratio
    coupled_speed = speed_cmd * speed_scale
    coupled_speed = max(params["min_speed"], min(target_speed, coupled_speed))
    debug_info["steer_speed_ratio"] = steer_ratio
    debug_info["steer_speed_scale"] = speed_scale
    debug_info["steer_speed_coupling"] = STEER_SPEED_COUPLING
    return coupled_speed


def pid_steering(err, params, steer_gain=1.0):
    global pid_integral, pid_prev_err, debug_info

    dt = control_dt
    if abs(err) < params["deadband"]:
        err = 0.0
        pid_integral *= 0.5
    else:
        pid_integral += err * dt
        limit = params["pid_integral_limit"]
        pid_integral = max(-limit, min(limit, pid_integral))

    d_err = (err - pid_prev_err) / dt
    pid_prev_err = err

    p_term = params["pid_kp"] * err
    i_term = params["pid_ki"] * pid_integral
    d_term = params["pid_kd"] * d_err
    steer = -(p_term + i_term + d_term) * steer_gain
    steer = max(-params["max_steer"], min(params["max_steer"], steer))

    debug_info["control"] = "pid"
    debug_info["pid_p"] = p_term
    debug_info["pid_i"] = i_term
    debug_info["pid_d"] = d_term
    return steer


def update_startup_state(raw_err):
    global startup_active, startup_stable_count, startup_target_err, pid_integral, pid_prev_err

    if not startup_active:
        debug_info["startup"] = False
        debug_info["startup_stable"] = startup_stable_count
        debug_info["startup_target_err"] = LINE_TARGET_ERR
        return

    yellow_confidence = (
        debug_info.get("vision_source", "") == "yellow"
    )
    if startup_target_err is None:
        if yellow_confidence:
            startup_target_err = raw_err
        else:
            startup_target_err = LINE_TARGET_ERR

    if startup_target_err > LINE_TARGET_ERR:
        startup_target_err = max(LINE_TARGET_ERR, startup_target_err - STARTUP_TARGET_STEP)
    elif startup_target_err < LINE_TARGET_ERR:
        startup_target_err = min(LINE_TARGET_ERR, startup_target_err + STARTUP_TARGET_STEP)

    target_ready = abs(startup_target_err - LINE_TARGET_ERR) <= STARTUP_TARGET_DONE_ERR
    line_ready = yellow_confidence and abs(raw_err - LINE_TARGET_ERR) <= STARTUP_EXIT_RAW_ERR
    if target_ready and line_ready:
        startup_stable_count += 1
    else:
        startup_stable_count = 0

    if startup_stable_count >= 3:
        startup_active = False
        startup_target_err = LINE_TARGET_ERR
        pid_integral = 0.0
        pid_prev_err = 0.0
        debug_info["fast_curve_layers"] = 0
        debug_info["fast_curve_required"] = False
        debug_info["fast_layer_curves"] = []
        debug_info["curve_layers"] = []

    debug_info["startup"] = startup_active
    debug_info["startup_stable"] = startup_stable_count
    debug_info["startup_target_err"] = startup_target_err


def current_line_target_err():
    if startup_active and startup_target_err is not None:
        return startup_target_err
    return LINE_TARGET_ERR


def blend_tracking_tasks(position_err):
    curve = debug_info.get("control_curvature", 0.0)
    curve_ratio = min(1.0, abs(curve) / GEOMETRY_CURVE_SCALE)
    geometry_weight = GEOMETRY_WEIGHT_MAX * curve_ratio
    position_weight = 1.0 - geometry_weight

    geometry_err = GEOMETRY_ERR_GAIN * numpy.arctan(L * curve)
    geometry_err = max(-GEOMETRY_ERR_LIMIT, min(GEOMETRY_ERR_LIMIT, geometry_err))
    ctrl_err = position_weight * position_err + geometry_weight * geometry_err

    debug_info["position_task_err"] = position_err
    debug_info["geometry_task_err"] = geometry_err
    debug_info["position_task_weight"] = position_weight
    debug_info["geometry_task_weight"] = geometry_weight
    return ctrl_err


def select_control_params(curvature):
    global control_mode, pid_integral, pid_prev_err

    if startup_active:
        control_mode = "straight"
        debug_info["mode"] = "startup"
        return current_startup_params()

    if not debug_info.get("mode_curve_valid", False):
        debug_info["mode"] = control_mode
        if control_mode == "turn":
            return current_turn_params()
        return current_straight_params()

    abs_curve = abs(curvature)
    mode_exit_err = max(abs(debug_info.get("raw_err", 0.0)), abs(debug_info.get("filtered_err", 0.0)))
    debug_info["mode_exit_err"] = mode_exit_err

    if control_mode == "straight" and abs_curve > CURVE_ENTER:
        control_mode = "turn"
        pid_integral = 0.0
        pid_prev_err = 0.0
    elif control_mode == "turn" and abs_curve < CURVE_EXIT and mode_exit_err < TURN_EXIT_MAX_ERR:
        control_mode = "straight"
        pid_integral = 0.0
        pid_prev_err = 0.0

    debug_info["mode"] = control_mode
    if control_mode == "turn":
        return current_turn_params()
    return current_straight_params()


def speed_target_for(params, err):
    info = debug_info
    fast_curve_layers = info.get("fast_curve_layers", 0)
    raw_err = info.get("raw_err", err)
    line_err = info.get("line_err", err)
    abs_err = max(abs(err), abs(line_err), abs(raw_err))
    abs_curve = abs(info.get("control_curvature", 0.0))
    max_speed = current_max_speed()
    target_speed = params["target_speed"]

    fast_confidence = False
    if (
        not startup_active
        and control_mode == "straight"
        and fast_curve_layers > 0
        and abs_err < FAST_MAX_ERR
        and abs_curve < FAST_CONTROL_CURVE_LIMIT
        and info.get("vision_source", "") == "yellow"
        and info.get("road_valid", False)
        and info.get("normal_frame_count", 0) >= STARTUP_FAST_BLOCK_FRAMES
    ):
        fast_confidence = True
        target_speed = min(
            max_speed,
            target_speed + FAST_SPEED_BOOST + FAST_CURVE_LAYER_STEP * max(0, fast_curve_layers - 1),
        )

    target_speed = min(target_speed, max_speed)
    vision_source = info.get("vision_source", "")
    if vision_source == "road":
        target_speed = min(target_speed, ROAD_FALLBACK_MAX_SPEED)
        if control_mode == "turn":
            target_speed = min(target_speed, ROAD_FALLBACK_TURN_MAX_SPEED)
    if control_mode == "straight" and abs_curve >= FAST_CONTROL_CURVE_LIMIT:
        target_speed = min(target_speed, params["target_speed"])
    if abs_err >= SEVERE_ERR_THRESHOLD:
        target_speed = min(target_speed, SEVERE_ERR_SPEED_LIMIT)
    elif abs_err >= LARGE_ERR_THRESHOLD:
        target_speed = min(target_speed, LARGE_ERR_SPEED_LIMIT)

    params["min_speed"] = min(params["min_speed"], target_speed)
    info["fast_confidence"] = fast_confidence
    info["fast_curve_layers"] = fast_curve_layers
    info["speed_target"] = target_speed
    return target_speed


def draw_debug(image):
    if not current_draw_debug():
        return

    lines = [
        "err raw={:.2f} filt={:.2f} ctrl={:.2f} target={:.2f}".format(
            debug_info.get("raw_err", 0.0),
            debug_info.get("filtered_err", 0.0),
            debug_info.get("ctrl_err", 0.0),
            debug_info.get("line_target_err", 0.0),
        ),
        "speed={:.2f}/{:.2f} steer={:.2f} raw={:.2f}".format(
            debug_info.get("speed_cmd", 0.0),
            debug_info.get("speed_target", 0.0),
            debug_info.get("steer_cmd", 0.0),
            debug_info.get("raw_steer", 0.0),
        ),
        "mode={} ctrl={} vision={} reject={} fast={}".format(
            debug_info.get("mode", ""),
            debug_info.get("control", ""),
            debug_info.get("vision_source", ""),
            debug_info.get("reject_reason", ""),
            debug_info.get("fast_confidence", False),
        ),
        "curve={:.2f} src={} layers={} roi={:.2f}->{:.2f} {}".format(
            debug_info.get("control_curvature", 0.0),
            debug_info.get("curve_source", ""),
            debug_info.get("fast_curve_layers", 0),
            debug_info.get("roi_top_ratio", ROI_TOP_RATIO_BASE),
            debug_info.get("roi_target_ratio", ROI_TOP_RATIO_BASE),
            debug_info.get("roi_target_source", ""),
        ),
        "area={:.0f} cx={} cy={} startup={} lost={}".format(
            debug_info.get("mask_area", 0.0),
            debug_info.get("cx", -1),
            debug_info.get("cy", -1),
            debug_info.get("startup", False),
            lost_count,
        ),
        "tune {} vmax={:.2f} alpha={:.2f} gain_min={:.2f}".format(
            debug_info.get("tune_mode", ""),
            debug_info.get("tune_max_speed", 0.0),
            debug_info.get("tune_err_alpha", 0.0),
            debug_info.get("tune_steer_gain_min", 0.0),
        ),
        "pid kp={:.2f} ki={:.2f} kd={:.2f} dead={:.2f}".format(
            debug_info.get("tune_pid_kp", 0.0),
            debug_info.get("tune_pid_ki", 0.0),
            debug_info.get("tune_pid_kd", 0.0),
            debug_info.get("tune_deadband", 0.0),
        ),
        "limit v={:.2f}/{:.2f} steer={:.2f} rate={:.2f} drop={:.2f}".format(
            debug_info.get("tune_min_speed", 0.0),
            debug_info.get("tune_target_speed", 0.0),
            debug_info.get("tune_max_steer", 0.0),
            debug_info.get("tune_steer_rate", 0.0),
            debug_info.get("tune_speed_drop", 0.0),
        ),
        "rec v={:.2f} steer={:.2f}-{:.2f} gain={:.2f}/{:.2f} frames={}".format(
            debug_info.get("tune_recovery_speed", 0.0),
            debug_info.get("tune_recovery_min_steer", 0.0),
            debug_info.get("tune_recovery_max_steer", 0.0),
            debug_info.get("tune_recovery_continue_gain", 0.0),
            debug_info.get("tune_recovery_reverse_gain", 0.0),
            debug_info.get("tune_recovery_frames", 0),
        ),
    ]

    for idx, text in enumerate(lines):
        y = 22 + idx * 20
        cv2.putText(image, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1)


def log_debug():
    global last_debug_log_time

    if not DEBUG_OUTPUT:
        return

    now = rospy.Time.now().to_sec()
    if now <= 0.0:
        now = rospy.get_time()
    if last_debug_log_time > 0.0 and now - last_debug_log_time < DEBUG_PERIOD:
        return
    last_debug_log_time = now

    rospy.loginfo(
        "mode={} vision={} reject={} err={:.3f}/{:.3f} speed={:.3f}/{:.3f} "
        "steer={:.3f} curve={:.3f} road={} fast={} lost={} pid={:.2f}/{:.2f}/{:.2f}".format(
            debug_info.get("mode", ""),
            debug_info.get("vision_source", ""),
            debug_info.get("reject_reason", ""),
            debug_info.get("raw_err", 0.0),
            debug_info.get("ctrl_err", 0.0),
            debug_info.get("speed_cmd", 0.0),
            debug_info.get("speed_target", 0.0),
            debug_info.get("steer_cmd", 0.0),
            debug_info.get("control_curvature", 0.0),
            debug_info.get("road_valid", False),
            debug_info.get("fast_confidence", False),
            lost_count,
            debug_info.get("pid_p", 0.0),
            debug_info.get("pid_i", 0.0),
            debug_info.get("pid_d", 0.0),
        )
    )


def follow_line(image):
    global pub, prev_err, prev_steer, lost_count, last_raw_err, last_line_err, startup_stable_count, startup_target_err, normal_frame_count

    ensure_tuning_controls()
    refresh_tuning_cache()
    err, w = estimate_lane_error(image)
    akm = AckermannDriveStamped()

    if err is None:
        lost_count += 1
        recovery_params = current_recovery_params()
        recover_steer, recovery_mode = recovery_steering(recovery_params)
        if lost_count <= recovery_params["frames"] and recover_steer is not None:
            akm.drive.speed = recovery_params["speed"]
            akm.drive.steering_angle = recover_steer
            debug_info["recovery"] = True
            debug_info["recovery_mode"] = recovery_mode
        else:
            akm.drive.speed = 0.0
            akm.drive.steering_angle = 0.0
            debug_info["recovery"] = False
            debug_info["recovery_mode"] = "stop"
        debug_info["recovery_steer"] = akm.drive.steering_angle
        debug_info["speed_cmd"] = akm.drive.speed
        debug_info["raw_steer"] = akm.drive.steering_angle
        debug_info["steer_cmd"] = akm.drive.steering_angle
        debug_info["raw_err"] = 0.0
        debug_info["line_target_err"] = LINE_TARGET_ERR
        debug_info["line_err"] = 0.0
        debug_info["filtered_err"] = prev_err
        debug_info["ctrl_err"] = 0.0
        debug_info["mode"] = "recovery" if debug_info["recovery"] else "lost"
        debug_info["sharp_turn"] = False
        debug_info["control"] = "recovery" if debug_info["recovery"] else "stop"
        debug_info["pid_p"] = 0.0
        debug_info["pid_i"] = 0.0
        debug_info["pid_d"] = 0.0
        debug_info["speed_steer_gain"] = 1.0
        debug_info["position_task_err"] = 0.0
        debug_info["geometry_task_err"] = 0.0
        debug_info["position_task_weight"] = 1.0
        debug_info["geometry_task_weight"] = 0.0
        if startup_active:
            startup_stable_count = 0
            startup_target_err = None
        debug_info["startup"] = startup_active
        debug_info["startup_stable"] = startup_stable_count
        debug_info["startup_target_err"] = startup_target_err if startup_target_err is not None else LINE_TARGET_ERR
        log_debug()
        pub.publish(akm)
        draw_debug(image)
        show_camera_frame(image)
        return

    lost_count = 0
    debug_info["recovery"] = False
    debug_info["recovery_mode"] = "none"
    debug_info["recovery_steer"] = 0.0
    last_raw_err = debug_info.get("raw_err", 0.0)
    update_startup_state(last_raw_err)
    if startup_active:
        normal_frame_count = 0
    else:
        normal_frame_count += 1
    debug_info["normal_frame_count"] = normal_frame_count
    target_err = current_line_target_err()
    err = last_raw_err - target_err
    debug_info["line_target_err"] = target_err
    debug_info["line_err"] = err
    if startup_active:
        debug_info["position_task_err"] = err
        debug_info["geometry_task_err"] = 0.0
        debug_info["position_task_weight"] = 1.0
        debug_info["geometry_task_weight"] = 0.0
    else:
        err *= debug_info.get("roi_y_gain", 1.0)
        err = blend_tracking_tasks(err)
    last_line_err = err
    err_alpha = current_err_alpha()
    err = err_alpha * err + (1.0 - err_alpha) * prev_err
    prev_err = err
    debug_info["filtered_err"] = err
    params = select_control_params(debug_info.get("mode_curvature", 0.0))

    if abs(err) < params["deadband"]:
        err = 0.0
    debug_info["ctrl_err"] = err

    target_speed = speed_target_for(params, err)
    speed_cmd = target_speed - params["turn_speed_drop"] * abs(err)
    speed_cmd = max(params["min_speed"], min(target_speed, speed_cmd))

    steer_gain = straight_speed_steer_gain(speed_cmd, params)
    steer = pid_steering(err, params, steer_gain)

    steer = max(-params["max_steer"], min(params["max_steer"], steer))
    if abs(err) > params["sharp_turn_err"] and abs(steer) < params["sharp_turn_steer"]:
        steer = params["sharp_turn_steer"] if err < 0.0 else -params["sharp_turn_steer"]
        debug_info["sharp_turn"] = True
    else:
        debug_info["sharp_turn"] = False
    debug_info["raw_steer"] = steer
    speed_cmd = apply_steer_speed_coupling(speed_cmd, target_speed, steer, params)
    steer = limit_steer_rate(steer, params)
    prev_steer = steer
    debug_info["speed_cmd"] = speed_cmd
    debug_info["steer_cmd"] = steer

    akm.drive.speed = speed_cmd
    akm.drive.steering_angle = steer
    pub.publish(akm)
    log_debug()

    if current_draw_markers():
        target_x = int((debug_info.get("line_target_err", LINE_TARGET_ERR) * 0.5 + 0.5) * w)
        cv2.line(image, (target_x, 0), (target_x, image.shape[0]), (255, 0, 0), 2)
        point_cx = debug_info.get("cx", target_x)
        point_cy = debug_info.get("cy", -1)
        if point_cy < 0:
            point_cy = image.shape[0] - 30
        cv2.circle(image, (point_cx, point_cy), 8, (0, 0, 255), -1)
    draw_debug(image)
    show_camera_frame(image)


def image_callback(msg):
    global bridge

    update_control_dt(msg.header.stamp)
    if bridge is None:
        bridge = cv_bridge.CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    frame = resize_for_processing(frame)
    follow_line(frame)


def stop_car():
    if pub is None:
        return

    akm = AckermannDriveStamped()
    akm.drive.speed = 0.0
    akm.drive.steering_angle = 0.0
    pub.publish(akm)
    rospy.sleep(0.1)


if __name__ == "__main__":
    rospy.init_node("follower")
    rospy.loginfo("follow_perf version: {}".format(DEBUG_VERSION))
    pub = rospy.Publisher("/car1/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=10)
    rospy.on_shutdown(stop_car)
    rospy.Subscriber("/car1/camera/zed_left/image_rect_color_left", Image, image_callback)
    rospy.spin()
