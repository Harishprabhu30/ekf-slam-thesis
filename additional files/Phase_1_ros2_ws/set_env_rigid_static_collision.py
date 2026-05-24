import omni.usd
from pxr import UsdPhysics, Sdf

stage = omni.usd.get_context().get_stage()
env_path = "/World/sm_warehouse_a60_h10m_straight_01"

env = stage.GetPrimAtPath(env_path)

if env.IsValid():
    # Apply rigid body if missing
    if not env.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(env)

    # Create the bodyType attribute manually
    attr = env.CreateAttribute("physxRigidBody:bodyType", Sdf.ValueTypeNames.Token)
    attr.Set("static")

    # Apply collision
    if not env.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(env)

    print("Warehouse set to STATIC + COLLISION.")
else:
    print("Warehouse not found")


''' 
change the env_path for whatever using
expected output:

Warehouse set to STATIC + COLLISION.
Warehouse set to STATIC + COLLISION.
Warehouse set to STATIC + COLLISION.


'''
