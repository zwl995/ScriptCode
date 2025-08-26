# ScriptCode

## 功能列表

### interpolate_camera_poses.py - 相机姿态插值工具

该脚本用于从JSON文件中读取相机姿态，对连续的姿态进行插值，并将结果姿态写入新的JSON文件。这对于生成平滑的相机移动以进行3D可视化或视频渲染特别有用。

主要特性：
- 从JSON文件加载相机姿态
- 在连续姿态之间进行插值
- 支持自定义帧率和时间间隔
- 将插值结果保存到新的JSON文件

This code is used to accumulate some commonly used wheel code that cannot be reproduced quickly.