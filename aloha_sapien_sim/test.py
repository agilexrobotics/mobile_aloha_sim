import sapien as sapien
from sapien.utils import Viewer
import numpy as np
import mplib
import sapien.physx as sapienp




def create_box(
    scene: sapien.Scene,
    pose: sapien.Pose,
    half_size,
    color=None,
    name="",
) -> sapien.Entity:
    entity = sapien.Entity()
    entity.set_name(name)
    entity.set_pose(pose)

    # create PhysX dynamic rigid body
    rigid_component = sapien.physx.PhysxRigidDynamicComponent()
    rigid_component.attach(
        sapien.physx.PhysxCollisionShapeBox(
            half_size=half_size, material=sapien.physx.get_default_material()
        )
    )

    # create render body for visualization
    render_component = sapien.render.RenderBodyComponent()
    render_component.attach(

        sapien.render.RenderShapeBox(
            half_size, sapien.render.RenderMaterial(base_color=[*color[:3], 1])
        )
    )

    entity.add_component(rigid_component)
    entity.add_component(render_component)
    entity.set_pose(pose)

    scene.add_entity(entity)

    return entity

def create_table(
    scene: sapien.Scene,
    pose: sapien.Pose,
    length: float,
    width: float,
    height: float,
    thickness=0.1,
    color=(0.8, 0.6, 0.4),
    name="table",
) -> sapien.Entity:
    """Create a table with specified dimensions."""
    builder = scene.create_actor_builder()

    # Tabletop
    tabletop_pose = sapien.Pose([0.0, 0.0, -thickness / 2])  # Center the tabletop at z=0
    tabletop_half_size = [length / 2, width / 2, thickness / 2]
    builder.add_box_collision(pose=tabletop_pose, half_size=tabletop_half_size)
    builder.add_box_visual(
        pose=tabletop_pose, half_size=tabletop_half_size, material=color
    )

    # Table legs (x4)
    leg_spacing = 0.1 # 距离桌角的距离
    for i in [-1, 1]:
        for j in [-1, 1]:
            x = i * (length / 2 - leg_spacing / 2)  # 计算桌腿的x坐标
            y = j * (width / 2 - leg_spacing / 2)  # 计算桌腿的y坐标
            table_leg_pose = sapien.Pose([x, y, -height / 2])
            table_leg_half_size = [thickness / 2, thickness / 2, height / 2]
            builder.add_box_collision(
                pose=table_leg_pose, half_size=table_leg_half_size
            )
            builder.add_box_visual(
                pose=table_leg_pose, half_size=table_leg_half_size, material=color
            )

    table = builder.build(name=name)
    table.set_pose(pose)
    return table

def left_plan_to_pose(pose,planner,active_joints,robot,scene,viewer):
    left_arm_joint_id = [6,14,18,22,26,30]
    joint_pose = robot.get_qpos()
    qpos=[]
    for i in range(6):
        qpos.append(joint_pose[left_arm_joint_id[i]])
    result = planner.plan_screw(
        target_pose=pose,
        qpos=qpos,
        time_step=1 / 250,
        use_point_cloud=False,
        use_attach=False,
    )
    print("plan ",result["status"])
    if result["status"] == "Success":
        n_step = result["position"].shape[0]
        for i in range(n_step):
            # 平衡重力
            qf = robot.compute_passive_force(
                gravity=True, coriolis_and_centrifugal=True
            )
            robot.set_qf(qf)
            
            for j in range(len(left_arm_joint_id)):
                n_j = left_arm_joint_id[j]
                active_joints[n_j].set_drive_target(result["position"][i][j])
                active_joints[n_j].set_drive_velocity_target(result["velocity"][i][j])

            scene.step()
            if i % 5==0:
                scene.update_render()
                viewer.render()
    
        
def set_left_gripper(active_joints,target):
    for joint in active_joints[34:36]:
        joint.set_drive_target(target)
        joint.set_drive_velocity_target(0.05)
    

def main():
    # 初始化render和scene
    engine = sapien.Engine()
    renderer = sapien.SapienRenderer()
    engine.set_renderer(renderer)

    scene = engine.create_scene()
    scene.set_timestep(1 / 100.0)

    
    # 添加光线
    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([-1, -1, -1], [0.5, 0.5, 0.5])
    
    
    # 创建可视化窗口
    viewer = scene.create_viewer()
    # 设置初始观察相机
    viewer.set_camera_xyz(x=-2, y=0, z=2.5)
    viewer.set_camera_rpy(r=0, p=-np.arctan2(2, 2), y=0)
    viewer.window.set_camera_parameters(near=0.05, far=100, fovy=1)
    
    # 光追（默认不开启）
    # sapien.render.set_viewer_shader_dir("rt")
    # sapien.render.set_ray_tracing_samples_per_pixel(32)
    # sapien.render.set_ray_tracing_path_depth(8)
    # sapien.render.set_ray_tracing_denoiser("optix")
    
    # 添加地面
    scene.add_ground(altitude=0)  # The ground is in fact a special actor.
    # 添加桌子
    create_table(
        scene,
        sapien.Pose(p=[0, 0, 0.75]),
        length=1.2,
        width=0.7,
        height=0.74,
        thickness=0.05,
    )
    # 添加方块
    create_box(
        scene,
        sapien.Pose(p=[-0.1, 0, 0.78]),
        half_size=[0.02, 0.02, 0.02],
        color=(0, 0, 1), 
        name='box',
    )
    
    # 添加aloha机器人
    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    robot = loader.load("./urdf/aloha_sapien_sim.urdf")
    robot.set_root_pose(sapien.Pose([0, -0.65, 0], [1, 0, 0, 1]))
    # 获取活动关节
    active_joints = robot.get_active_joints()
    
    # 设置关节阻尼和刚性
    for joint in active_joints:
        joint.set_drive_property(
            stiffness=1000,
            damping=200,
        )
    
    
    # 初始化aloha机械臂的planner（左臂）
    robot_pose_in_world = [0,-0.65,0,1,0,0,1]
    planner = mplib.Planner(
        urdf="./urdf/aloha_sapien_sim.urdf",
        srdf="./srdf/aloha_sapien_sim.srdf",
        move_group="fl_link6",
    )
    planner.set_base_pose(robot_pose_in_world)
    
    # 打开夹爪
    set_left_gripper(active_joints,0.045)
    
    # 移动到方块上方
    pose = [-0.1,0,0.95,-0.5,0.5,-0.5,-0.5]
    left_plan_to_pose(pose,planner,active_joints,robot,scene,viewer)
    
    # 向下
    pose[2]-=0.03
    left_plan_to_pose(pose,planner,active_joints,robot,scene,viewer)
    
    # 关闭夹爪
    set_left_gripper(active_joints,0)
    
    # 抓起方块
    pose[2]+=0.05
    left_plan_to_pose(pose,planner,active_joints,robot,scene,viewer)
                
    while not viewer.closed:
        # 重力平衡 balance the passive force
        for _ in range(4):  # render every 4 steps
            qf = robot.compute_passive_force(
                gravity=True,
                coriolis_and_centrifugal=True,
            )
            robot.set_qf(qf)
                
        scene.update_render()
        viewer.render()

        scene.step()
if __name__ == "__main__":
    main()