"""从相机轨迹JSON文件中插值相机姿态。

该脚本从JSON文件中读取相机姿态，对连续的姿态进行插值，
并将结果姿态写入新的JSON文件。这对于生成平滑的相机移动以进行3D可视化
或视频渲染特别有用。

此版本使用文件内配置而不是命令行参数。
只需修改main()函数顶部的配置部分即可自定义行为。
"""

import json
import math
from typing import List, Dict

import numpy as np
import pycolmap
from dataclasses import dataclass, asdict


@dataclass
class CameraJson:
    """表示JSON格式的相机参数的数据类。"""
    id: int
    img_name: str
    width: int
    height: int
    position: List[float]
    rotation: List[List[float]]
    fy: float
    fx: float


def fov_to_focal(fov: float, pixels: int) -> float:
    """将视野角转换为焦距。

    Args:
        fov: 以弧度表示的视野角。
        pixels: 对应轴上的像素数。

    Returns:
        以像素为单位的焦距。
    """
    return pixels / (2 * math.tan(fov / 2))


def focal_to_fov(focal: float, pixels: int) -> float:
    """将焦距转换为视野角。

    Args:
        focal: 以像素为单位的焦距。
        pixels: 对应轴上的像素数。

    Returns:
        以弧度表示的视野角。
    """
    return 2 * math.atan(pixels / (2 * focal))


def load_camera_poses(json_path: str) -> Dict[str, np.ndarray]:
    """从JSON文件加载相机姿态。

    Args:
        json_path: 输入JSON文件的路径。

    Returns:
        将图像名称映射到相机到世界变换矩阵的字典。
    """
    with open(json_path, 'r') as file:
        data = json.load(file)

    name_to_tcw = {}
    for item in data:
        img_name = item.get("img_name")
        position = item.get("position")
        rotation = item.get("rotation")
        
        twc = np.eye(4)
        twc[:3, :3] = np.array(rotation)
        twc[:3, 3] = np.array(position)
        tcw = np.linalg.inv(twc)
        name_to_tcw[img_name] = tcw
        
    return dict(sorted(name_to_tcw.items(), key=lambda item: item[0]))


def interpolate_poses(poses: List[np.ndarray], 
                      dt: float, 
                      fps: float) -> List[pycolmap.Rigid3d]:
    """在连续的相机姿态之间进行插值。

    Args:
        poses: 表示相机姿态的4x4变换矩阵列表。
        dt: 相邻帧之间的时间间隔。
        fps: 插值的帧率。

    Returns:
        插值后的Rigid3d变换列表。
    """
    interpolated_poses = []
    
    # 添加第一个姿态
    r0 = poses[0][:3, :3]
    t0 = poses[0][:3, 3]
    first_pose = pycolmap.Rigid3d(rotation=r0, translation=t0)
    interpolated_poses.append(first_pose)
    
    # 在连续的姿态之间进行插值
    for i in range(len(poses) - 1):
        num_frames = int(dt * fps)
        r1 = poses[i][:3, :3]
        t1 = poses[i][:3, 3]
        r2 = poses[i+1][:3, :3]
        t2 = poses[i+1][:3, 3]
        
        pre_pose = pycolmap.Rigid3d(rotation=r1, translation=t1)
        next_pose = pycolmap.Rigid3d(rotation=r2, translation=t2)
        
        for j in range(1, num_frames):
            cur_pose = pycolmap.Rigid3d.interpolate(pre_pose, next_pose, j/num_frames)
            interpolated_poses.append(cur_pose)
            
        interpolated_poses.append(next_pose)
        
    return interpolated_poses


def save_camera_poses(cameras: List[CameraJson], output_path: str) -> None:
    """将相机姿态保存到JSON文件。

    Args:
        cameras: CameraJson对象列表。
        output_path: 输出JSON文件的路径。
    """
    with open(output_path, 'w') as file:
        json.dump(cameras, file, indent=2)


def main() -> None:
    """用于插值相机姿态的主函数，使用文件内配置。
    
    要自定义行为，请修改下面的配置变量：
    """
    # === 配置部分 ===
    # 输入和输出文件路径
    input_json = "/ws/zwl/Data/zwj/home_build/521_fire_gs/3dgs/cameras.json"
    output_json = "/ws/zwl/Data/zwj/home_build/521_fire_gs/3dgs/cameras_interp.json"
    
    # 参考视频前缀（设置为空字符串以使用所有帧）
    ref_video_prefix = ""
    
    # 图像尺寸
    width = 1440
    height = 1920
    
    # 相机视野角
    fov_degrees = 80.0
    
    # 插值参数
    dt = 0.5  # 相邻帧之间的时间间隔
    fps = 30.0  # 插值的帧率
    # === 配置部分结束 ===
    
    # 将视野角转换为弧度并计算焦距
    fov = np.deg2rad(fov_degrees)
    focal_length_x = fov_to_focal(fov, width)
    focal_length_y = focal_length_x

    # 加载相机姿态
    name_to_tcw = load_camera_poses(input_json)
    
    # 根据参考视频前缀筛选姿态
    ref_video_poses = [pose for name, pose in name_to_tcw.items() 
                       if name.startswith(ref_video_prefix)]
    
    # 执行插值
    interpolated_poses = interpolate_poses(ref_video_poses, dt, fps)
    
    # 将姿态转换为相机JSON格式
    cameras = []
    for idx, pose in enumerate(interpolated_poses):
        t_wc = pose.inverse().matrix()
        camera = CameraJson(
            id=idx,
            img_name=f"{idx:06d}",
            width=width,
            height=height,
            position=t_wc[:, 3].tolist(),
            rotation=t_wc[:3, :3].tolist(),
            fy=focal_length_y,
            fx=focal_length_x,
        )
        cameras.append(asdict(camera))

    # 保存到输出文件
    save_camera_poses(cameras, output_json)
    print(f"已保存 {len(cameras)} 个插值后的相机姿态到 {output_json}")


if __name__ == "__main__":
    main()