import omni.usd
from pxr import UsdPhysics

stage = omni.usd.get_context().get_stage()
env_path = "/World/sm_warehouse_a06_h10m_straight_01"
env = stage.GetPrimAtPath(env_path)

print("Warehouse valid:", env.IsValid())

if env.IsValid():
    print("Has RigidBodyAPI:", env.HasAPI(UsdPhysics.RigidBodyAPI))
    print("Has CollisionAPI:", env.HasAPI(UsdPhysics.CollisionAPI))

    body_type_attr = env.GetAttribute("physxRigidBody:bodyType")
    print("BodyType attr exists:", bool(body_type_attr))
    print("BodyType value:", body_type_attr.Get() if body_type_attr else "N/A")


'''
expected output:

Warehouse valid: False
Warehouse valid: True
Has RigidBodyAPI: True
Has CollisionAPI: True
BodyType attr exists: True
BodyType value: static

'''
