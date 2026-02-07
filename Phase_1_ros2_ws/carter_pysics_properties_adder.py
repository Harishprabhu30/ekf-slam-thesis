import omni.usd
import omni.timeline
from pxr import UsdPhysics, Sdf

stage = omni.usd.get_context().get_stage()
timeline = omni.timeline.get_timeline_interface()

if timeline.is_playing():
    print("Please stop simulation first!")
else:
    parts_config = {
        "chassis_link": {"mass": 15.0, "kinematic": True},
        "wheel_left": {"mass": 1.5, "kinematic": False},
        "wheel_right": {"mass": 1.5, "kinematic": False},
        "caster_frame_base": {"mass": 0.5, "kinematic": False},
        "caster_swivel_left": {"mass": 0.3, "kinematic": False},
        "caster_swivel_right": {"mass": 0.3, "kinematic": False},
        "caster_wheel_left": {"mass": 0.5, "kinematic": False},
        "caster_wheel_right": {"mass": 0.5, "kinematic": False}
    }

    for part_name, config in parts_config.items():
        part_path = f"/World/nova_carter_empty/{part_name}"
        part_prim = stage.GetPrimAtPath(part_path)

        if part_prim and part_prim.IsValid():
            if not part_prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI.Apply(part_prim)

            mass_api = UsdPhysics.MassAPI.Apply(part_prim)
            mass_api.CreateMassAttr(config["mass"])

            if config["kinematic"]:
                part_prim.CreateAttribute(
                    "physxRigidBody:kinematicEnabled",
                    Sdf.ValueTypeNames.Bool
                ).Set(True)

            print(f"{part_name}: mass={config['mass']}, kinematic={config['kinematic']}")

    print("Robot physics setup complete!")


'''
Expected output"

chassis_link: mass=15.0, kinematic=True
wheel_left: mass=1.5, kinematic=False
wheel_right: mass=1.5, kinematic=False
caster_frame_base: mass=0.5, kinematic=False
caster_swivel_left: mass=0.3, kinematic=False
caster_swivel_right: mass=0.3, kinematic=False
caster_wheel_left: mass=0.5, kinematic=False
caster_wheel_right: mass=0.5, kinematic=False
Robot physics setup complete!
chassis_link: mass=15.0, kinematic=True
wheel_left: mass=1.5, kinematic=False
wheel_right: mass=1.5, kinematic=False
caster_frame_base: mass=0.5, kinematic=False
caster_swivel_left: mass=0.3, kinematic=False
caster_swivel_right: mass=0.3, kinematic=False
caster_wheel_left: mass=0.5, kinematic=False
caster_wheel_right: mass=0.5, kinematic=False
Robot physics setup complete!


'''
