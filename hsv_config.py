"""
霍夫变换巡线模块 - HSV参数配置
- 黄色线巡线参数
- 蓝色线巡线参数
- 屏蔽背景色参数

使用方法:
    from config import YELLOW_HSV, BLUE_HSV, MASK_HSV
"""

# ============ OpenCV HSV 参数 (H:0-180, S:0-255, V:0-255) ============
# 蓝色线 HSV 参数 (2025-05-19 标定)
BLUE_HSV = {
    'lower': (30, 25, 50),       # OpenCV格式
    'upper': (150, 199, 242),    # SDK格式: H*2, S/2.55, V/2.55
}

# 黄色线 HSV 参数 (2025-05-19 标定)
YELLOW_HSV = {
    'lower': (0, 59, 153),       # OpenCV格式
    'upper': (46, 255, 255),    # SDK格式: H*2, S/2.55, V/2.55
}

# ============ 屏蔽色 HSV 参数 (背景中需要排除的颜色) ============
# 地面灰色屏蔽 (去除地面反光和阴影)
FLOOR_GRAY = {
    'lower': (0, 0, 80),
    'upper': (180, 50, 200),
}

# 白色干扰屏蔽 (去除白色标记或反光)
WHITE_NOISE = {
    'lower': (0, 0, 200),
    'upper': (180, 30, 255),
}

# 绿色植物屏蔽 (如果场地上有绿植)
GREEN_PLANTS = {
    'lower': (35, 30, 30),
    'upper': (85, 255, 255),
}

# 红色干扰屏蔽 (其他红色标记)
RED_NOISE = {
    'lower1': (0, 50, 50),
    'upper1': (10, 255, 255),
    'lower2': (170, 50, 50),
    'upper2': (180, 255, 255),
}

# 所有屏蔽颜色列表
MASK_COLORS = [
    FLOOR_GRAY,
    WHITE_NOISE,
    GREEN_PLANTS,
]


def create_mask(hsv, color_config):
    """根据HSV配置创建mask"""
    if 'lower1' in color_config:
        # 红色特殊处理 (H值跨越0度)
        mask1 = cv2.inRange(hsv,
                           np.array(color_config['lower1']),
                           np.array(color_config['upper1']))
        mask2 = cv2.inRange(hsv,
                           np.array(color_config['lower2']),
                           np.array(color_config['upper2']))
        return cv2.bitwise_or(mask1, mask2)
    else:
        return cv2.inRange(hsv,
                          np.array(color_config['lower']),
                          np.array(color_config['upper']))

def create_exclusion_mask(hsv):
    """创建综合屏蔽mask (去除背景干扰)"""
    masks = []
    for color in MASK_COLORS:
        masks.append(create_mask(hsv, color))
    # 合并所有屏蔽区域
    exclusion = masks[0]
    for m in masks[1:]:
        exclusion = cv2.bitwise_or(exclusion, m)
    return exclusion

# 导入numpy和cv2 (供外部使用)
import numpy as np
import cv2
