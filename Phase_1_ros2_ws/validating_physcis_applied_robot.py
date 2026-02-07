from pxr import UsdPhysics
import omni.usd

stage = omni.usd.get_context().get_stage()

rigid_bodies = []
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        rigid_bodies.append(str(prim.GetPath()))

print("Rigid bodies found:")
print("\n".join(rigid_bodies))


'''
Expected output:

Rigid bodies found:
/World/nova_carter_empty/chassis_link
/World/nova_carter_empty/chassis_link/Realsense/RSD455 ## should have physics applied.
/World/nova_carter_empty/wheel_left
/World/nova_carter_empty/wheel_right
/World/nova_carter_empty/caster_frame_base
/World/nova_carter_empty/caster_swivel_left
/World/nova_carter_empty/caster_swivel_right
/World/nova_carter_empty/caster_wheel_left
/World/nova_carter_empty/caster_wheel_right

'''
