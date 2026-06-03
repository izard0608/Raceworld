#! /usr/bin/env python3
import cv2
import cv_bridge
import numpy
import rospy
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped

# 全局速度与MPC搜索参数
MAX_SPEED = 0.80  # 全局最高速度上限
STEER_STEP = 0.04  # MPC枚举转角的步长，越小越细但计算越慢

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

# 曲率与多ROI参数
CURVE_MIN_POINTS = 80  # 曲率拟合所需最少mask点数
CURVE_SPLIT_MIN_HEIGHT = 8  # 分层曲率评估的最小层高，避免过薄ROI误判
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
FAST_SPEED_BOOST = 0.15  # 下1/2曲率接近0时的基础高速加成
FAST_MAX_ERR = 0.12  # 允许进入高速模式的最大横向误差
FAST_MAX_CURVE = 0.03  # 允许进入高速模式的最大曲率
FAST_CONTROL_CURVE_LIMIT = 0.04  # 控制曲率超过该值时禁止直道高速
STARTUP_FAST_BLOCK_FRAMES = 12  # 缓启动退出后暂时禁止高速，避免刚入正轨就冲弯
STRAIGHT_STEER_SPEED_GAIN_MIN = 0.68  # 直道最高速时PID转角倍率下限

# 控制模型参数
MPC_HORIZON = 7  # MPC预测步数
DT = 0.08  # 控制周期估计值
L = 0.32  # 车辆轴距，用于简化自行车模型

# 误差处理参数
LINE_TARGET_ERR = 0.0  # 目标横向误差，0表示让线位于图像中心
ERR_ALPHA = 0.3  # 横向误差低通滤波系数，越大越相信当前帧
STARTUP_TARGET_STEP = 0.015  # 缓启动阶段每帧把目标位置推向图像中心的步长
STARTUP_TARGET_DONE_ERR = 0.02  # 缓启动目标位置接近中心到该阈值内后退出
STARTUP_EXIT_RAW_ERR = 0.18  # 缓启动退出时，黄线实际位置也必须接近目标，避免目标到中心但车还没跟上
ROAD_FALLBACK_MAX_SPEED = 0.24  # 只靠路面兜底时的最高速度，防止黄线丢失后继续高速外冲
ROAD_FALLBACK_TURN_MAX_SPEED = 0.20  # 弯道且只靠路面兜底时的最高速度
LARGE_ERR_SPEED_LIMIT = 0.30  # 大横向误差时的最高速度
SEVERE_ERR_SPEED_LIMIT = 0.20  # 严重横向误差时的最高速度
LARGE_ERR_THRESHOLD = 0.35  # 进入大误差限速的误差阈值
SEVERE_ERR_THRESHOLD = 0.60  # 进入严重误差限速的误差阈值

STRAIGHT_PARAMS = {
    "target_speed": 0.52,  # 直道目标速度
    "min_speed": 0.20,  # 直道最低速度
    "max_steer": 0.28,  # 直道最大转角
    "deadband": 0.08,  # 直道误差死区，小于该值按0处理
    "turn_speed_drop": 0.20,  # 误差越大速度越低的降速系数
    "pid_kp": 0.76,  # 直道PID比例系数
    "pid_ki": 0.02,  # 直道PID积分系数
    "pid_kd": 0.10,  # 直道PID微分系数
    "pid_integral_limit": 0.60,  # PID积分限幅，防止积分饱和
    "predict_err_gain": 5.0,  # MPC横向误差预测增益
    "steer_cost": 0.80,  # MPC转角惩罚，越大越不愿大幅打角
    "steer_change_cost": 1.20,  # MPC转角变化惩罚，越大转向越平滑
    "steer_rate_limit": 0.08,  # 单帧转角变化限制
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
    "predict_err_gain": 5.0,  # 保持参数结构一致，缓启动阶段不进入MPC
    "steer_cost": 0.90,  # 保持参数结构一致，缓启动阶段不进入MPC
    "steer_change_cost": 1.40,  # 保持参数结构一致，缓启动阶段不进入MPC
    "steer_rate_limit": 0.06,  # 缓启动转向变化限制，起步更平滑
    "sharp_turn_err": 0.60,  # 缓启动大误差强制补转向阈值
    "sharp_turn_steer": 0.16,  # 缓启动大误差时的最小转角
}

TURN_PARAMS = {
    "target_speed": 0.38,  # 弯道目标速度
    "min_speed": 0.10,  # 弯道最低速度
    "max_steer": 0.40,  # 弯道最大转角
    "deadband": 0.03,  # 弯道误差死区
    "turn_speed_drop": 0.12,  # 弯道误差降速系数
    "pid_kp": STRAIGHT_PARAMS["pid_kp"],
    "pid_ki": STRAIGHT_PARAMS["pid_ki"],
    "pid_kd": STRAIGHT_PARAMS["pid_kd"],
    "pid_integral_limit": STRAIGHT_PARAMS["pid_integral_limit"],
    "predict_err_gain": 7.0,  # 弯道MPC横向误差预测增益
    "steer_cost": 0.65,  # 弯道MPC转角惩罚
    "steer_change_cost": 0.90,  # 弯道MPC转角变化惩罚
    "steer_rate_limit": 0.14,  # 弯道单帧转角变化限制
    "sharp_turn_err": 0.55,  # 弯道大误差强制补转向阈值
    "sharp_turn_steer": 0.30,  # 弯道大误差时的最小转角
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
DEBUG_DRAW = True  # 是否在图像上绘制5行关键调试文字
DEBUG_SHOW_MASKS = False  # 是否显示ROI和路面mask调试窗口，跑速度时应关闭
DEBUG_DRAW_MARKERS = True  # 是否在camera画面上绘制目标线和检测质心
DEBUG_VERSION = "direct_line_near_curve_multiroi_v10_turn_speed_up3"  # 当前调试版本标识

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
control_mode = "straight"
lost_count = 0
startup_active = True
startup_stable_count = 0
startup_target_err = None
normal_frame_count = 0
debug_info = {}


def roi_bounds(h, ratio):
    search_top = int(ratio * h)
    search_bot = min(h, search_top + ROI_HEIGHT)
    return search_top, search_bot


def apply_roi(mask, h, w, ratio):
    search_top, search_bot = roi_bounds(h, ratio)
    roi_mask = mask.copy()
    roi_mask[0:search_top, 0:w] = 0
    roi_mask[search_bot:h, 0:w] = 0
    return roi_mask, search_top, search_bot


def set_roi_forward(h, w, mask):
    active_ratio = STARTUP_ROI_TOP_RATIO if startup_active else roi_top_ratio
    mask, search_top, search_bot = apply_roi(mask, h, w, active_ratio)
    debug_info["roi_top_ratio"] = active_ratio
    debug_info["roi_source"] = "startup" if startup_active else "dynamic"
    debug_info["roi_top"] = search_top
    debug_info["roi_bot"] = search_bot
    return mask


def fit_curvature(mask, w, roi_top, roi_bot, min_points):
    ys, xs = numpy.nonzero(mask)
    point_count = len(xs)
    if point_count < min_points:
        return False, 0.0, point_count

    y_span = max(1.0, float(roi_bot - roi_top))
    y_norm = 2.0 * (ys.astype(float) - roi_top) / y_span - 1.0
    x_norm = (xs.astype(float) - (w / 2.0)) / (w / 2.0)

    try:
        curve_a, curve_b, _ = numpy.polyfit(y_norm, x_norm, 2)
    except (TypeError, ValueError, numpy.linalg.LinAlgError):
        return False, 0.0, point_count

    curvature = (2.0 * curve_a) / ((1.0 + curve_b * curve_b) ** 1.5)
    return True, float(curvature), point_count


def estimate_canvas_curve_layers(canvas_mask, w):
    h = canvas_mask.shape[0]
    layer_curves = []
    curve_layers = []
    fast_layers = 0
    segment_bot = h
    segment_top = h // 2

    while segment_top >= 0 and segment_bot - segment_top >= CURVE_SPLIT_MIN_HEIGHT:
        layer_mask = canvas_mask.copy()
        layer_mask[0:segment_top, 0:w] = 0
        layer_mask[segment_bot:h, 0:w] = 0
        segment_height = max(1, segment_bot - segment_top)
        valid_layer, layer_curve, layer_points = fit_curvature(
            layer_mask,
            w,
            segment_top,
            segment_bot,
            max(12, min(CURVE_MIN_POINTS, int(CURVE_MIN_POINTS * segment_height / max(1, h)))),
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
    lower_mask = canvas_mask.copy()
    lower_mask[0:roi_top, 0:w] = 0

    valid, curvature, point_count = fit_curvature(lower_mask, w, roi_top, roi_bot, CURVE_MIN_POINTS)
    debug_info["mode_curve_valid"] = valid
    debug_info["mode_curvature"] = curvature if valid else 0.0
    debug_info["mode_curve_points"] = point_count
    return debug_info["mode_curvature"]


def estimate_line_curvature(mask, w, canvas_mask=None):
    roi_top = debug_info.get("roi_top", 0)
    roi_bot = debug_info.get("roi_bot", mask.shape[0])
    valid, curvature, point_count = fit_curvature(mask, w, roi_top, roi_bot, CURVE_MIN_POINTS)
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
    lower_road = numpy.array([0, 0, ROAD_MIN_V])
    upper_road = numpy.array([179, ROAD_MAX_S, ROAD_MAX_V])
    road_mask = cv2.inRange(hsv, lower_road, upper_road)
    road_mask = set_roi_forward(h, w, road_mask)
    road_mask = cv2.medianBlur(road_mask, 5)
    kernel = numpy.ones((5, 5), numpy.uint8)
    road_mask = cv2.morphologyEx(road_mask, cv2.MORPH_CLOSE, kernel)

    road_area = cv2.countNonZero(road_mask)
    debug_info["road_area"] = road_area
    if road_area < ROAD_MIN_AREA:
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
    for y in range(roi_top, roi_bot):
        xs = numpy.flatnonzero(road_mask[y])
        if len(xs) >= ROAD_ROW_MIN_PIXELS:
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
    global debug_info

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([26, 43, 46])
    upper_yellow = numpy.array([34, 255, 255])
    yellow_base_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    yellow_base_mask = cv2.medianBlur(yellow_base_mask, 5)

    h, w = yellow_base_mask.shape
    debug_info["image_h"] = h
    mask = set_roi_forward(h, w, yellow_base_mask)
    mask = cv2.GaussianBlur(mask, (5, 5), 0)
    yellow_canvas_mask = cv2.GaussianBlur(yellow_base_mask, (5, 5), 0)
    estimate_mode_curvature(yellow_canvas_mask, w)
    road_mask = estimate_road_features(hsv, h, w)

    if DEBUG_SHOW_MASKS:
        cv2.namedWindow("region of interest", 0)
        cv2.imshow("region of interest", mask)
        cv2.namedWindow("road mask", 0)
        cv2.imshow("road mask", road_mask)

    M = cv2.moments(mask)
    debug_info["mask_area"] = M["m00"] / 255.0
    if debug_info["mask_area"] < MIN_MASK_AREA:
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


def mpc_steering(err, speed_cmd, params):
    global debug_info

    v = max(0.05, speed_cmd)
    best_delta = 0.0
    best_cost = float("inf")
    candidate_count = 0

    # Hand-written receding-horizon MPC: test constant steering candidates.
    max_steer = params["max_steer"]
    for delta in numpy.arange(-max_steer, max_steer + 1e-6, STEER_STEP):
        candidate_count += 1
        y = err
        psi = 0.0
        cost = 0.0
        for _ in range(MPC_HORIZON):
            y += params["predict_err_gain"] * DT * v * psi
            psi += DT * (v / L) * numpy.tan(delta)
            cost += 16.0 * y * y + 1.5 * psi * psi + params["steer_cost"] * delta * delta

        cost += 25.0 * y * y
        cost += params["steer_change_cost"] * (delta - prev_steer) * (delta - prev_steer)
        if cost < best_cost:
            best_cost = cost
            best_delta = float(delta)

    debug_info["mpc_cost"] = best_cost
    debug_info["mpc_candidates"] = candidate_count
    return best_delta


def limit_steer_rate(target_steer, params):
    if abs(target_steer) < 1e-6:
        return 0.45 * prev_steer

    steer_delta = target_steer - prev_steer
    steer_rate_limit = params["steer_rate_limit"]
    steer_delta = max(-steer_rate_limit, min(steer_rate_limit, steer_delta))
    return prev_steer + steer_delta


def recovery_steering():
    if abs(prev_steer) > RECOVERY_SOURCE_STEER_MIN:
        edge_lost = abs(last_raw_err) > RECOVERY_EDGE_RAW_ERR or abs(last_line_err) > RECOVERY_EDGE_LINE_ERR
        if edge_lost and lost_count <= RECOVERY_EDGE_FRAMES:
            recover_steer = prev_steer * RECOVERY_CONTINUE_GAIN
            recovery_mode = "continue_edge"
        else:
            recover_steer = -prev_steer * RECOVERY_REVERSE_GAIN
            recovery_mode = "reverse"
        if abs(recover_steer) < RECOVERY_MIN_STEER:
            recover_steer = RECOVERY_MIN_STEER if recover_steer > 0.0 else -RECOVERY_MIN_STEER
    elif abs(prev_err) > RECOVERY_ERR_FALLBACK:
        recover_steer = RECOVERY_MIN_STEER if prev_err > 0.0 else -RECOVERY_MIN_STEER
        recovery_mode = "reverse_err"
    else:
        return None, "stop"

    recover_steer = max(-RECOVERY_MAX_STEER, min(RECOVERY_MAX_STEER, recover_steer))
    return recover_steer, recovery_mode


def straight_speed_steer_gain(speed_cmd, params):
    speed_span = max(1e-6, params["target_speed"] - params["min_speed"])
    speed_ratio = (speed_cmd - params["min_speed"]) / speed_span
    speed_ratio = max(0.0, min(1.0, speed_ratio))
    gain = 1.0 - (1.0 - STRAIGHT_STEER_SPEED_GAIN_MIN) * speed_ratio
    debug_info["speed_steer_gain"] = gain
    return gain


def pid_steering(err, params, steer_gain=1.0):
    global pid_integral, pid_prev_err, debug_info

    if abs(err) < params["deadband"]:
        err = 0.0
        pid_integral *= 0.5
    else:
        pid_integral += err * DT
        limit = params["pid_integral_limit"]
        pid_integral = max(-limit, min(limit, pid_integral))

    d_err = (err - pid_prev_err) / DT
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
    debug_info["mpc_cost"] = 0.0
    debug_info["mpc_candidates"] = 0
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
        return STARTUP_PARAMS

    if not debug_info.get("mode_curve_valid", False):
        debug_info["mode"] = control_mode
        if control_mode == "turn":
            return TURN_PARAMS
        return STRAIGHT_PARAMS

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
        return TURN_PARAMS
    return STRAIGHT_PARAMS


def speed_target_for(params, err):
    fast_curve_layers = debug_info.get("fast_curve_layers", 0)
    abs_err = max(abs(err), abs(debug_info.get("line_err", err)), abs(debug_info.get("raw_err", err)))
    abs_curve = abs(debug_info.get("control_curvature", 0.0))
    fast_confidence = (
        control_mode == "straight"
        and debug_info.get("vision_source", "") == "yellow"
        and debug_info.get("road_valid", False)
        and debug_info.get("normal_frame_count", 0) >= STARTUP_FAST_BLOCK_FRAMES
        and abs_err < FAST_MAX_ERR
        and abs_curve < FAST_CONTROL_CURVE_LIMIT
        and fast_curve_layers > 0
    )

    target_speed = params["target_speed"]
    if startup_active:
        fast_confidence = False
    elif fast_confidence:
        target_speed = min(
            MAX_SPEED,
            target_speed + FAST_SPEED_BOOST + FAST_CURVE_LAYER_STEP * max(0, fast_curve_layers - 1),
        )

    if debug_info.get("vision_source", "") == "road":
        target_speed = min(target_speed, ROAD_FALLBACK_MAX_SPEED)
        if control_mode == "turn":
            target_speed = min(target_speed, ROAD_FALLBACK_TURN_MAX_SPEED)
    if control_mode == "straight" and abs_curve >= FAST_CONTROL_CURVE_LIMIT:
        target_speed = min(target_speed, params["target_speed"])
    if abs_err >= SEVERE_ERR_THRESHOLD:
        target_speed = min(target_speed, SEVERE_ERR_SPEED_LIMIT)
    elif abs_err >= LARGE_ERR_THRESHOLD:
        target_speed = min(target_speed, LARGE_ERR_SPEED_LIMIT)

    debug_info["fast_confidence"] = fast_confidence
    debug_info["fast_curve_layers"] = fast_curve_layers
    debug_info["speed_target"] = target_speed
    return target_speed


def draw_debug(image):
    if not DEBUG_DRAW:
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
    ]

    for idx, text in enumerate(lines):
        y = 22 + idx * 20
        cv2.putText(image, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1)


def log_debug():
    if not DEBUG_OUTPUT:
        return

    rospy.loginfo_throttle(
        DEBUG_PERIOD,
        "line={} raw_err={:.3f} filt_err={:.3f} ctrl_err={:.3f} speed={:.3f} "
        "steer_raw={:.3f} steer={:.3f} steer_gain={:.3f} speed_target={:.3f} "
        "line_target={:.3f} line_err={:.3f} "
        "curve={:.3f} curve_valid={} curve_pts={} mode_curve={:.3f} mode_curve_valid={} mode_curve_pts={} "
        "fast_layers={} layer_curves={} "
        "roi_ratio={:.2f} roi_target={:.2f} roi_src={} roi_layer_y={} "
        "road_valid={} road_area={} road_err={:.3f} road_curve={:.3f} road_heading={:.3f} "
        "pos_w={:.3f} pos_err={:.3f} geo_w={:.3f} geo_err={:.3f} "
        "area={:.0f} cx={} cy={} roi_y={:.3f} y_gain={:.3f} mpc_cost={:.3f} "
        "candidates={} mode={} startup={} startup_stable={} startup_target={:.3f} "
        "control={} vision={} curve_src={} fast={} "
        "pid_p={:.3f} pid_i={:.3f} pid_d={:.3f} "
        "sharp_turn={} recovery={} reject={} version={}".format(
            debug_info.get("line_found", False),
            debug_info.get("raw_err", 0.0),
            debug_info.get("filtered_err", 0.0),
            debug_info.get("ctrl_err", 0.0),
            debug_info.get("speed_cmd", 0.0),
            debug_info.get("raw_steer", 0.0),
            debug_info.get("steer_cmd", 0.0),
            debug_info.get("speed_steer_gain", 1.0),
            debug_info.get("speed_target", 0.0),
            debug_info.get("line_target_err", 0.0),
            debug_info.get("line_err", 0.0),
            debug_info.get("control_curvature", 0.0),
            debug_info.get("control_curve_valid", False),
            debug_info.get("curve_points", 0),
            debug_info.get("mode_curvature", 0.0),
            debug_info.get("mode_curve_valid", False),
            debug_info.get("mode_curve_points", 0),
            debug_info.get("fast_curve_layers", 0),
            debug_info.get("fast_layer_curves", []),
            debug_info.get("roi_top_ratio", ROI_TOP_RATIO_BASE),
            debug_info.get("roi_target_ratio", ROI_TOP_RATIO_BASE),
            debug_info.get("roi_target_source", ""),
            debug_info.get("roi_layer_target_y", -1),
            debug_info.get("road_valid", False),
            debug_info.get("road_area", 0),
            debug_info.get("road_err", 0.0),
            debug_info.get("road_curvature", 0.0),
            debug_info.get("road_heading", 0.0),
            debug_info.get("position_task_weight", 1.0),
            debug_info.get("position_task_err", 0.0),
            debug_info.get("geometry_task_weight", 0.0),
            debug_info.get("geometry_task_err", 0.0),
            debug_info.get("mask_area", 0.0),
            debug_info.get("cx", -1),
            debug_info.get("cy", -1),
            debug_info.get("roi_y_norm", 1.0),
            debug_info.get("roi_y_gain", 1.0),
            debug_info.get("mpc_cost", 0.0),
            debug_info.get("mpc_candidates", 0),
            debug_info.get("mode", ""),
            debug_info.get("startup", False),
            debug_info.get("startup_stable", 0),
            debug_info.get("startup_target_err", LINE_TARGET_ERR),
            debug_info.get("control", ""),
            debug_info.get("vision_source", ""),
            debug_info.get("curve_source", ""),
            debug_info.get("fast_confidence", False),
            debug_info.get("pid_p", 0.0),
            debug_info.get("pid_i", 0.0),
            debug_info.get("pid_d", 0.0),
            debug_info.get("sharp_turn", False),
            debug_info.get("recovery", False),
            debug_info.get("reject_reason", ""),
            DEBUG_VERSION,
        ),
    )


def follow_line(image):
    global pub, prev_err, prev_steer, lost_count, last_raw_err, last_line_err, startup_stable_count, startup_target_err, normal_frame_count

    err, w = estimate_lane_error(image)
    akm = AckermannDriveStamped()

    if err is None:
        lost_count += 1
        recover_steer, recovery_mode = recovery_steering()
        if lost_count <= RECOVERY_FRAMES and recover_steer is not None:
            akm.drive.speed = RECOVERY_SPEED
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
        debug_info["mpc_cost"] = 0.0
        debug_info["mpc_candidates"] = 0
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
        rospy.loginfo_throttle(
            DEBUG_PERIOD,
            "line lost: recovery={} mode={} lost_count={} speed={:.3f} steer={:.3f} "
            "last_raw={:.3f} last_line={:.3f}".format(
                debug_info["recovery"],
                debug_info["recovery_mode"],
                lost_count,
                akm.drive.speed,
                akm.drive.steering_angle,
                last_raw_err,
                last_line_err,
            ),
        )
        pub.publish(akm)
        draw_debug(image)
        cv2.namedWindow("camera", 0)
        cv2.imshow("camera", image)
        cv2.waitKey(1)
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
    err = ERR_ALPHA * err + (1.0 - ERR_ALPHA) * prev_err
    prev_err = err
    debug_info["filtered_err"] = err
    params = select_control_params(debug_info.get("mode_curvature", 0.0))

    if abs(err) < params["deadband"]:
        err = 0.0
    debug_info["ctrl_err"] = err

    target_speed = speed_target_for(params, err)
    speed_cmd = target_speed - params["turn_speed_drop"] * abs(err)
    speed_cmd = max(params["min_speed"], min(target_speed, speed_cmd))

    if control_mode in ("straight", "turn"):
        steer_gain = straight_speed_steer_gain(speed_cmd, params)
        steer = pid_steering(err, params, steer_gain)
    else:
        debug_info["speed_steer_gain"] = 1.0
        steer = mpc_steering(err, speed_cmd, params)
        debug_info["control"] = "mpc"
        debug_info["pid_p"] = 0.0
        debug_info["pid_i"] = 0.0
        debug_info["pid_d"] = 0.0

    steer = max(-params["max_steer"], min(params["max_steer"], steer))
    if abs(err) > params["sharp_turn_err"] and abs(steer) < params["sharp_turn_steer"]:
        steer = params["sharp_turn_steer"] if err < 0.0 else -params["sharp_turn_steer"]
        debug_info["sharp_turn"] = True
    else:
        debug_info["sharp_turn"] = False
    debug_info["raw_steer"] = steer
    speed_cmd -= 0.06 * abs(steer)
    speed_cmd = max(params["min_speed"], min(target_speed, speed_cmd))
    steer = limit_steer_rate(steer, params)
    prev_steer = steer
    debug_info["speed_cmd"] = speed_cmd
    debug_info["steer_cmd"] = steer

    akm.drive.speed = speed_cmd
    akm.drive.steering_angle = steer
    pub.publish(akm)
    log_debug()

    if DEBUG_DRAW_MARKERS:
        target_x = int((debug_info.get("line_target_err", LINE_TARGET_ERR) * 0.5 + 0.5) * w)
        cv2.line(image, (target_x, 0), (target_x, image.shape[0]), (255, 0, 0), 2)
        point_cx = debug_info.get("cx", target_x)
        point_cy = debug_info.get("cy", -1)
        if point_cy < 0:
            point_cy = image.shape[0] - 30
        cv2.circle(image, (point_cx, point_cy), 8, (0, 0, 255), -1)
    draw_debug(image)
    cv2.namedWindow("camera", 0)
    cv2.imshow("camera", image)
    cv2.waitKey(1)


def image_callback(msg):
    bridge = cv_bridge.CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
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
