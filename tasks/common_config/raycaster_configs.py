# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
Public raycaster sensor configuration.
Includes base configuration and presets for MultiMeshRayCaster lidar sensors.
"""

from isaaclab.sensors import MultiMeshRayCasterCfg, patterns
from isaaclab.utils import configclass
from typing import List, Tuple


@configclass
class RayCasterBaseCfg:
    """Raycaster base configuration class.

    Provides default configuration for MultiMeshRayCaster sensors,
    supporting scene-specific parameter customization.
    """

    @classmethod
    def get_raycaster_config(
        cls,
        prim_path: str = "/World/envs/env_.*/Robot/d435_link",
        mesh_prim_paths: list = None,
        update_period: float = 0.1,
        resolution: float = 0.02,
        size: Tuple[float, float] = (2.5, 2.5),
        direction: Tuple[float, float, float] = (0.0, 0.0, -1.0),
        pos_offset: Tuple[float, float, float] = (0.0, 0.0, 0.1),
        debug_vis: bool = False,
    ) -> MultiMeshRayCasterCfg:
        """Get a MultiMeshRayCaster configuration with GridPattern (height-map style).

        Args:
            prim_path: Prim path where the sensor is attached.
            mesh_prim_paths: List of RaycastTargetCfg entries for mesh targets.
            update_period: Sensor update period in seconds.
            resolution: Grid resolution.
            size: Total size of the grid.
            direction: Direction of the rays.
            pos_offset: Position offset from the attachment link (x, y, z).
            debug_vis: Enable debug visualization.

        Returns:
            MultiMeshRayCasterCfg: Raycaster configuration.
        """
        if mesh_prim_paths is None:
            mesh_prim_paths = []

        return MultiMeshRayCasterCfg(
            prim_path=prim_path,
            update_period=update_period,
            offset=MultiMeshRayCasterCfg.OffsetCfg(pos=pos_offset),
            mesh_prim_paths=mesh_prim_paths,
            ray_alignment="base",
            pattern_cfg=patterns.GridPatternCfg(
                resolution=resolution,
                size=size,
                direction=direction,
            ),
            debug_vis=debug_vis,
        )

    @classmethod
    def get_lidar_config(
        cls,
        prim_path: str = "/World/envs/env_.*/Robot/d435_link",
        mesh_prim_paths: list = None,
        update_period: float = 0.1,
        channels: int = 16,
        vertical_fov_range: Tuple[float, float] = (-15.0, 15.0),
        horizontal_fov_range: Tuple[float, float] = (0.0, 360.0),
        horizontal_res: float = 0.5,
        pos_offset: Tuple[float, float, float] = (0.0, 0.0, 0.1),
        debug_vis: bool = False,
    ) -> MultiMeshRayCasterCfg:
        """Get a MultiMeshRayCaster configuration with LidarPattern.

        Args:
            prim_path: Prim path where the sensor is attached.
            mesh_prim_paths: List of RaycastTargetCfg entries for mesh targets.
            update_period: Sensor update period in seconds.
            channels: Number of vertical beams.
            vertical_fov_range: Vertical FOV range in degrees (min, max).
            horizontal_fov_range: Horizontal FOV range in degrees (min, max).
            horizontal_res: Horizontal angular resolution in degrees.
            pos_offset: Position offset from the attachment link (x, y, z).
            debug_vis: Enable debug visualization.

        Returns:
            MultiMeshRayCasterCfg: Raycaster configuration.
        """
        if mesh_prim_paths is None:
            mesh_prim_paths = []

        return MultiMeshRayCasterCfg(
            prim_path=prim_path,
            update_period=update_period,
            offset=MultiMeshRayCasterCfg.OffsetCfg(pos=pos_offset),
            mesh_prim_paths=mesh_prim_paths,
            ray_alignment="base",
            pattern_cfg=patterns.LidarPatternCfg(
                channels=channels,
                vertical_fov_range=vertical_fov_range,
                horizontal_fov_range=horizontal_fov_range,
                horizontal_res=horizontal_res,
            ),
            debug_vis=debug_vis,
        )


@configclass
class RayCasterPresets:
    """Raycaster preset configuration collection.

    Includes common raycaster configuration presets for different scenes.
    """

    @classmethod
    def g1_multi_box_heightmap(cls) -> MultiMeshRayCasterCfg:
        """MultiMeshRayCaster using GridPattern (downward rays, height-map style).

        Attached to the robot d435_link (head area), scans 4 detectable boxes.
        """
        return RayCasterBaseCfg.get_raycaster_config(
            prim_path="/World/envs/env_.*/Robot/d435_link",
            mesh_prim_paths=[
                MultiMeshRayCasterCfg.RaycastTargetCfg(
                    prim_expr="/World/envs/env_.*/lidar_box_1",
                    track_mesh_transforms=True,
                ),
                MultiMeshRayCasterCfg.RaycastTargetCfg(
                    prim_expr="/World/envs/env_.*/lidar_box_2",
                    track_mesh_transforms=True,
                ),
                MultiMeshRayCasterCfg.RaycastTargetCfg(
                    prim_expr="/World/envs/env_.*/lidar_box_3",
                    track_mesh_transforms=True,
                ),
                MultiMeshRayCasterCfg.RaycastTargetCfg(
                    prim_expr="/World/envs/env_.*/lidar_box_4",
                    track_mesh_transforms=True,
                ),
            ],
            resolution=0.02,
            size=(2.5, 2.5),
            direction=(0.0, 0.0, -1.0),
            pos_offset=(0.0, 0.0, 0.1),
            update_period=0.1,
            debug_vis=False,
        )

    @classmethod
    def g1_multi_box_lidar(cls) -> MultiMeshRayCasterCfg:
        """MultiMeshRayCaster using LidarPattern targeting 4 box primitives.

        16-channel LiDAR with 360-degree horizontal and +/-15 degree vertical FOV.
        Attached to the robot d435_link (head area).
        """
        return RayCasterBaseCfg.get_lidar_config(
            prim_path="/World/envs/env_.*/Robot/d435_link",
            mesh_prim_paths=[
                MultiMeshRayCasterCfg.RaycastTargetCfg(
                    prim_expr="/World/envs/env_.*/lidar_box_1",
                    track_mesh_transforms=True,
                ),
                MultiMeshRayCasterCfg.RaycastTargetCfg(
                    prim_expr="/World/envs/env_.*/lidar_box_2",
                    track_mesh_transforms=True,
                ),
                MultiMeshRayCasterCfg.RaycastTargetCfg(
                    prim_expr="/World/envs/env_.*/lidar_box_3",
                    track_mesh_transforms=True,
                ),
                MultiMeshRayCasterCfg.RaycastTargetCfg(
                    prim_expr="/World/envs/env_.*/lidar_box_4",
                    track_mesh_transforms=True,
                ),
            ],
            channels=16,
            vertical_fov_range=(-30.0, 10.0),
            horizontal_fov_range=(0.0, 360.0),
            horizontal_res=0.5,
            pos_offset=(0.0, 0.0, 0.1),
            update_period=0.1,
            debug_vis=False,
        )

    @classmethod
    def g1_room_lidar(cls) -> MultiMeshRayCasterCfg:
        """MultiMeshRayCaster using LidarPattern targeting floor, 4 walls, and 4 boxes.

        16-channel LiDAR with 360-degree horizontal and -30/+10 degree vertical FOV.
        Attached to the robot d435_link (head area).
        """
        return RayCasterBaseCfg.get_lidar_config(
            prim_path="/World/envs/env_.*/Robot/d435_link",
            mesh_prim_paths=[
                MultiMeshRayCasterCfg.RaycastTargetCfg(
                    prim_expr="/World/envs/env_.*/ground_floor",
                    track_mesh_transforms=False,
                ),
                MultiMeshRayCasterCfg.RaycastTargetCfg(
                    prim_expr="/World/envs/env_.*/lidar_box_1",
                    track_mesh_transforms=True,
                ),
                MultiMeshRayCasterCfg.RaycastTargetCfg(
                    prim_expr="/World/envs/env_.*/lidar_box_2",
                    track_mesh_transforms=True,
                ),
                MultiMeshRayCasterCfg.RaycastTargetCfg(
                    prim_expr="/World/envs/env_.*/lidar_box_3",
                    track_mesh_transforms=True,
                ),
                MultiMeshRayCasterCfg.RaycastTargetCfg(
                    prim_expr="/World/envs/env_.*/lidar_box_4",
                    track_mesh_transforms=True,
                ),
            ],
            channels=8,
            vertical_fov_range=(-30.0, 10.0),
            horizontal_fov_range=(0.0, 360.0),
            horizontal_res=1.0,
            pos_offset=(0.0, 0.0, 0.1),
            update_period=0.1,
            debug_vis=False,
        )
