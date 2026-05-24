# Warehouse Floor Physics Setup in Isaac Sim

This document summarizes the setup process for enabling **realistic physics and friction** for a warehouse environment in NVIDIA Isaac Sim.  

It covers the steps for using the hidden **Ground Plane** as the physics floor and configuring **Physics Materials** for realistic robot traction.

---

## 1. Problem Description

- Warehouse contains ~30 visual floor meshes (XForms) for rendering.  
- Robot passed through floors when physics/friction was not enabled.  
- Clicking the stage sometimes selected materials/meshes accidentally.  
- Goal: Enable realistic floor friction without editing 30 individual floor meshes.

---

## 2. Key Observations

1. **Ground Plane exists under warehouse** (`/World/Warehouse/GroundPlane`)  
   - This handles the actual collisions for the robot.  
   - Visual floor meshes above it are just for rendering.  
2. **CollisionPlane** type is used for the floor  
   - No approximation dropdown is available (normal for infinite planes).  
   - Friction and physics are controlled via a **Rigid Body Physics Material**.

---

## 3. Step-by-Step Setup

### Step 1 — Create a Rigid Body Physics Material

1. Menu: `Create → Physics → Physics Material`  
2. Choose **Rigid Body Material** (ignore Deformable or PBD options).  
3. Configure properties for realistic warehouse concrete:

| Property         | Value  |
|-----------------|--------|
| Static Friction  | 0.9    |
| Dynamic Friction | 0.8    |
| Restitution      | 0.05   |
| Combine Mode     | Multiply |

4. Name the material: `Floor_Concrete_Mat`

> **Note:** Density is **not required** for the floor since it is static.

---

### Step 2 — Assign Physics Material to Ground Plane

1. Select the ground plane in Stage: `/World/Warehouse/GroundPlane/CollisionPlane`  
2. Ensure **Physics → Collider → Collision Enabled = True**  
3. Drag and drop `Floor_Concrete_Mat` onto the ground plane, or use **Add Target** in the Collider component.

> This applies friction to the floor without modifying visual meshes.

---

### Step 3 — Configure Robot Wheels

1. Ensure each wheel has a **rigid body collider**.  
2. Assign a **Physics Material** (rubber):

| Property         | Value  |
|-----------------|--------|
| Static Friction  | 1.1–1.2 |
| Dynamic Friction | 1.0    |
| Restitution      | 0.05   |

> Friction depends on **floor × wheel × combine mode**.

---

### Step 4 — Test Simulation

1. Press **Play** in Isaac Sim.  
2. Drive robot across the warehouse floor.  

Expected behavior:

- Robot stays on floor  
- Collides with shelves and walls  
- Moves with realistic traction (does not slip excessively)

---

## 4. Notes and Best Practices

- Visual floor meshes **do not participate in physics** — only the ground plane matters.  
- Lock the warehouse hierarchy (`Lock Selected Hierarchy`) to avoid accidentally selecting visual meshes during simulation.  
- Use Physics Debug (`Window → Simulation → Physics Debug`) to visualize colliders and contacts.  

---

## 5. Summary

- **CollisionPlane under warehouse** is the physics floor.  
- **Rigid Body Physics Material** enables realistic friction.  
- **Robot wheels** must also have physics materials for proper traction.  
- **Density for static floor** is not needed.  
- Friction combine mode = **Multiply** for realistic behavior.  

After following this setup, the warehouse environment is fully usable for robotics simulation with correct collisions and realistic floor friction.


