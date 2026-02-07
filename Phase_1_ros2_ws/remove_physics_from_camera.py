import omni.usd
from pxr import Usd, UsdPhysics

stage = omni.usd.get_context().get_stage()
camera_path = "/World/nova_carter_empty/chassis_link/Realsense/RSD455"
camera_prim = stage.GetPrimAtPath(camera_path)

for prim in Usd.PrimRange(camera_prim):
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        print("Rigid body inside camera subtree:", prim.GetPath())


'''
expected output: 
Removed collision APIs from camera subtree

Then, run again the validating_physics_applied_on_robot.py and get output as:

Rigid bodies found:
/World/nova_carter_empty/chassis_link
/World/nova_carter_empty/wheel_left
/World/nova_carter_empty/wheel_right
/World/nova_carter_empty/caster_frame_base
/World/nova_carter_empty/caster_swivel_left
/World/nova_carter_empty/caster_swivel_right
/World/nova_carter_empty/caster_wheel_left
/World/nova_carter_empty/caster_wheel_right


'''
