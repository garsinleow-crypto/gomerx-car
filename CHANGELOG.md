# 更新日志

## [当前版本] 2026-05-19

### 新增功能
- 完整流程测试脚本 `test_demo_full.py`（20步自动执行）
- 自动重试机制（网络断开时最多重试3次）
- 指定ID横扫模式（替换原有的排除ID模式）

### 模块列表
- `detect_tag.py` - AprilTag识别与对准
- `patrol_hough.py` - 霍夫变换巡线
- `nav_action.py` - 坐标导航
- `sweep_left.py` / `sweep_right.py` - 横扫扫描
- `catch.py` - 试管抓取
- `release.py` - 试管释放
- `turn_180.py` / `turn_90_ccw.py` / `turn_90_cw.py` - 旋转

### 已知问题
- 长时间运行(约20步后)可能出现网络断开，已添加自动重试
- 误差0.3~1cm且移动量<1cm时底盘可能死循环

## [历史版本]

### 2026-05-17
- 实现抓取/释放模块
- 标定蓝色巡线HSV参数
