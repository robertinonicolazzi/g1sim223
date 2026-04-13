# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
import tempfile
import torch
from dataclasses import MISSING

from pink.tasks import FrameTask

import isaaclab.envs.mdp as base_mdp
import isaaclab.sim as sim_utils
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.utils import configclass
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.sensors import ContactSensorCfg
from . import mdp
# use Isaac Lab native event system

from tasks.common_config import  G1RobotPresets, CameraPresets, RayCasterPresets  # isort: skip
from tasks.common_event.event_manager import SimpleEvent, SimpleEventManager

# import public scene configuration
from tasks.common_scene.base_scene_pickplace_cylindercfg_wholebody import TableCylinderSceneCfgWH

##
# Scene definition
##

@configclass
class ObjectTableSceneCfg(TableCylinderSceneCfgWH):
    """object table scene configuration class
    inherits from G1SingleObjectSceneCfg, gets the complete G1 robot scene configuration
    can add task-specific scene elements or override default configurations here
    """
    
    # Override: remove warehouse USD from base scene
    room_walls = None

    # Ground floor — large thin cuboid at z=0
    ground_floor = AssetBaseCfg(
        prim_path="/World/envs/env_.*/ground_floor",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.05), rot=(1.0, 0.0, 0.0, 0.0)),
        spawn=sim_utils.CuboidCfg(
            size=(20.0, 20.0, 0.1),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.5, 0.5, 0.5), metallic=0.0),
        ),
    )

    # 4 walls forming a 10m x 10m room
    wall_north = AssetBaseCfg(
        prim_path="/World/envs/env_.*/wall_north",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 5.0, 1.5), rot=(1.0, 0.0, 0.0, 0.0)),
        spawn=sim_utils.CuboidCfg(
            size=(10.0, 0.1, 3.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.7, 0.7, 0.7), metallic=0.0),
        ),
    )

    wall_south = AssetBaseCfg(
        prim_path="/World/envs/env_.*/wall_south",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, -5.0, 1.5), rot=(1.0, 0.0, 0.0, 0.0)),
        spawn=sim_utils.CuboidCfg(
            size=(10.0, 0.1, 3.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.7, 0.7, 0.7), metallic=0.0),
        ),
    )

    wall_east = AssetBaseCfg(
        prim_path="/World/envs/env_.*/wall_east",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(5.0, 0.0, 1.5), rot=(1.0, 0.0, 0.0, 0.0)),
        spawn=sim_utils.CuboidCfg(
            size=(0.1, 10.0, 3.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.7, 0.7, 0.7), metallic=0.0),
        ),
    )

    wall_west = AssetBaseCfg(
        prim_path="/World/envs/env_.*/wall_west",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(-5.0, 0.0, 1.5), rot=(1.0, 0.0, 0.0, 0.0)),
        spawn=sim_utils.CuboidCfg(
            size=(0.1, 10.0, 3.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.7, 0.7, 0.7), metallic=0.0),
        ),
    )

    # Humanoid robot w/ arms higher
    # 5. humanoid robot configuration
    robot: ArticulationCfg = G1RobotPresets.g1_29dof_dex1_wholebody(init_pos=(-3.9, -2.81811, 0.8),
        init_rot=(1, 0, 0, 0))

    contact_forces = ContactSensorCfg(prim_path="/World/envs/env_.*/Robot/.*", history_length=10, track_air_time=True, debug_vis=False)
    # 6. add camera configuration 
    front_camera = CameraPresets.g1_front_camera()
    left_wrist_camera = None
    right_wrist_camera = None
    robot_camera = None

    # 7. lidar target boxes
    lidar_box_1 = RigidObjectCfg(
        prim_path="/World/envs/env_.*/lidar_box_1",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(-2.9, -2.8, 0.84),
            rot=(1, 0, 0, 0),
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.3, 0.3, 0.4),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 1.0), metallic=0.2),
        ),
    )

    lidar_box_2 = RigidObjectCfg(
        prim_path="/World/envs/env_.*/lidar_box_2",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(-3.9, -1.8, 0.84),
            rot=(1, 0, 0, 0),
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.25, 0.25, 0.5),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0), metallic=0.2),
        ),
    )

    lidar_box_3 = RigidObjectCfg(
        prim_path="/World/envs/env_.*/lidar_box_3",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(-4.9, -2.8, 0.4),
            rot=(1, 0, 0, 0),
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.4, 0.4, 0.8),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 0.0), metallic=0.2),
        ),
    )

    lidar_box_4 = RigidObjectCfg(
        prim_path="/World/envs/env_.*/lidar_box_4",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(-3.9, -3.8, 0.3),
            rot=(1, 0, 0, 0),
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.35, 0.35, 0.6),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.5, 0.0), metallic=0.2),
        ),
    )

    # 8. lidar sensor configuration (floor + walls + boxes)
    lidar = RayCasterPresets.g1_room_lidar()
##
# MDP settings
##
@configclass
class ActionsCfg:
    """defines the action configuration related to robot control, using direct joint angle control
    """
    joint_pos = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=1.0, use_default_offset=True)



@configclass
class ObservationsCfg:
    """
    defines all available observation information
    """
    @configclass
    class PolicyCfg(ObsGroup):
        """policy group observation configuration class
        defines all state observation values for policy decision
        inherit from ObsGroup base class 
        """

        robot_joint_state = ObsTerm(func=mdp.get_robot_boy_joint_states)
        robot_gipper_state = ObsTerm(func=mdp.get_robot_gipper_joint_states)
        camera_image = ObsTerm(func=mdp.get_camera_image)

        def __post_init__(self):
            """post initialization function
            set the basic attributes of the observation group
            """
            self.enable_corruption = False  # disable observation value corruption
            self.concatenate_terms = False  # disable observation item connection

    # observation groups
    # create policy observation group instance
    policy: PolicyCfg = PolicyCfg()


@configclass
class TerminationsCfg:
    pass
    # check if the object is out of the working range
    # success = DoneTerm(func=mdp.reset_object_estimate)# use task completion check function

@configclass
class RewardsCfg:
    reward = RewTerm(func=mdp.compute_reward,weight=1.0)

@configclass
class EventCfg:
    pass
    # reset_object = EventTermCfg(
    #     func=mdp.reset_root_state_uniform,  # use uniform distribution reset function
    #     mode="reset",   # set event mode to reset
    #     params={
    #         # position range parameter
    #         "pose_range": {
    #             "x": [-0.05, 0.05],  # x axis position range: -0.05 to 0.0 meter
    #             "y": [-0.05, 0.05],   # y axis position range: 0.0 to 0.05 meter
    #         },
    #         # speed range parameter (empty dictionary means using default value)
    #         "velocity_range": {},
    #         # specify the object to reset
    #         "asset_cfg": SceneEntityCfg("object"),
    #     },
    # )


@configclass
class MoveCylinderG129Dex1WholebodyEnvCfg(ManagerBasedRLEnvCfg):
    """
    inherits from ManagerBasedRLEnvCfg, defines all configuration parameters for the entire environment
    """

    # 1. scene settings
    scene: ObjectTableSceneCfg = ObjectTableSceneCfg(num_envs=1, # environment number: 1
                                                     env_spacing=2.5, # environment spacing: 2.5 meter
                                                     replicate_physics=True # enable physics replication
                                                     )
    # basic settings
    observations: ObservationsCfg = ObservationsCfg()   # observation configuration
    actions: ActionsCfg = ActionsCfg()                  # action configuration
    # MDP settings
        
    terminations: TerminationsCfg = TerminationsCfg()    # termination configuration
    events = EventCfg()                                  # event configuration
    commands = None # command manager
    rewards: RewardsCfg = RewardsCfg()  # reward manager
    curriculum = None # curriculum manager
    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 4
        self.episode_length_s = 20.0
        # simulation settings
        self.sim.dt = 0.005
        self.scene.contact_forces.update_period = self.sim.dt
        self.sim.render_interval = 8
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625

                # 物理材料属性设置 / Physics material properties
        self.sim.physics_material.static_friction = 1.0  # 静摩擦系数 / Static friction
        self.sim.physics_material.dynamic_friction = 1.0  # 动摩擦系数 / Dynamic friction
        self.sim.physics_material.friction_combine_mode = "max"  # 摩擦力合并模式 / Friction combine mode
        self.sim.physics_material.restitution_combine_mode = "max"  # 恢复系数合并模式 / Restitution combine mode
        # create event manager
        self.event_manager = SimpleEventManager()

        # register "reset object" event
        self.event_manager.register("reset_object_self", SimpleEvent(
            func=lambda env: base_mdp.reset_root_state_uniform(
                env,
                torch.arange(env.num_envs, device=env.device),
                pose_range={"x": [-0.05, 0.05], "y": [0.0, 0.05]},
                velocity_range={},
                asset_cfg=SceneEntityCfg("object"),
            )
        ))
        
        self.event_manager.register("reset_all_self", SimpleEvent(
            func=lambda env: base_mdp.reset_scene_to_default(
                env,
                torch.arange(env.num_envs, device=env.device))
        ))
