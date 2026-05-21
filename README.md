# GomerX 智能小车控制系统

基于 GomerX 机器人的智能小车控制系统，实现 AprilTag 视觉定位与试管抓取运输功能。

## 功能模块

### 核心模块
| 文件 | 功能 |
|------|------|
| `detect_tag.py` | AprilTag 识别与对准 |
| `patrol_hough.py` | 基于霍夫变换的巡线 |
| `nav_action.py` | 坐标导航动作 |
| `sweep_left.py` / `sweep_right.py` | 横扫扫描目标 |
| `catch.py` | 试管抓取 |
| `release.py` | 试管释放 |
| `hsv_config.py` | HSV 颜色配置 |

### 旋转模块
| 文件 | 功能 |
|------|------|
| `turn_180.py` | 原地180°旋转 |
| `turn_90_ccw.py` | 左转90° |
| `turn_90_cw.py` | 右转90° |

## 快速开始

### 1. 环境要求
- Python 3.9+
- GomerX SDK 1.0.0
- OpenCV
- NumPy

### 2. 安装依赖
```bash
pip install gomerx opencv-python numpy
```

### 3. 连接配置
在脚本中设置机器人名称：
```python
ROBOT_NAME = 'GomerX_你的机器人名称'
```

## 使用方法

### 单独测试各模块

```bash
# AprilTag 扫描与对准
python test_apriltag.py
# 交互命令: scan / align <id> / quit

# 巡线测试
python patrol_hough.py --color yellow  # 黄色线
python patrol_hough.py --color blue    # 蓝色线

# 抓取测试
python test_catch.py

# 释放测试
python test_release.py

# 旋转测试
python turn_180.py      # 180°掉头
python turn_90_ccw.py   # 左转90°
python turn_90_cw.py    # 右转90°

# 横扫测试
python sweep_right.py <target_id>  # 右扫指定ID
python sweep_left.py <target_id>   # 左扫指定ID
```

### 完整流程测试

```bash
python test_demo_full.py
```

完整流程包含20个步骤：
1. 起点扫描对准 ID=5 → 掉头
2. 巡黄线 → 扫描对准 ID=1 → 右扫找试管 → 对准试管
3. 抓取 → 左转90°
4. 巡黄线 → 扫描对准 ID=4 → 右扫找 ID=7 → 对准 ID=7
5. 释放 → 左转90°
6. 巡蓝线 → 终点确认 ID=5

## 项目结构

```
dddddd/
├── README.md
├── .gitignore
├── 核心模块/
│   ├── detect_tag.py      # AprilTag检测
│   ├── patrol_hough.py    # 巡线
│   ├── nav_action.py      # 坐标导航
│   ├── sweep_*.py         # 横扫扫描
│   ├── catch.py           # 抓取
│   ├── release.py         # 释放
│   └── hsv_config.py      # HSV配置
├── 测试脚本/
│   ├── test_apriltag.py   # AprilTag测试
│   ├── test_catch.py      # 抓取测试
│   ├── test_release.py     # 释放测试
│   └── test_demo_full.py  # 完整流程
└── 旋转模块/
    ├── turn_180.py
    ├── turn_90_ccw.py
    └── turn_90_cw.py
```

## AprilTag ID 映射

| ID | 位置 |
|----|------|
| 0 | 试管 |
| 1 | A点起点 (Astart) |
| 2 | A点终点 (Aend) |
| 3 | B点终点 (Bend) |
| 4 | B点起点 (Bstart) |
| 5 | 起点/终点 |
| 7 | 放置区 |

## HSV 参数配置

### 蓝色巡线
- H: 98-116
- S: 51-143
- V: 74-242

### 黄色巡线
- H: 10-40
- S: 50-255
- V: 80-255

## 注意事项

1. **网络稳定性**: 长时间运行可能出现网络断开，系统会自动重试（最多3次）
2. **坐标系**: 使用全局坐标系，c朝+y方向
3. **底盘移动**: x/y最小1cm，最大150cm，超出需分割移动
4. **避障**: 抓取/释放后自动移动到避障位 (13, 19)

## 许可证

MIT License
