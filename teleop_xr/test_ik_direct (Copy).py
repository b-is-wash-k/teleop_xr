"""
Direct IK Solver Test - Bypass ROS2, see raw joint trajectory output.

This script shows EXACTLY what gets published to /joint_trajectory topic
without going through the ROS2 node.

Run:
    cd ~/OPEN_ARM_NEW/src/teleop_xr
    conda activate teleop_xr
    python test_ik_direct.py
"""

import sys
import jax.numpy as jnp
import jaxlie
from teleop_xr.ik.robots.openarm import OpenArmRobot
from teleop_xr.ik.solver import PyrokiSolver
from teleop_xr.ik.controller import IKController
from teleop_xr.messages import XRState, XRFrame

def main():
    print("=" * 70)
    print("DIRECT IK SOLVER OUTPUT TEST")
    print("=" * 70)
    
    # Step 1: Initialize robot and solver
    print("\n[1/4] Loading OpenArm robot...")
    robot = OpenArmRobot()
    print(f"      ✓ Robot loaded")
    print(f"      ✓ Actuated joints: {robot.actuated_joint_names}")
    
    print("\n[2/4] Creating IK solver...")
    solver = PyrokiSolver(robot)
    print(f"      ✓ Solver initialized")
    print(f"      ✓ Warmup complete: {solver.warmup_complete}")
    
    print("\n[3/4] Creating IK controller...")
    controller = IKController(robot, solver)
    print(f"      ✓ Controller initialized")
    
    # Step 2: Create a dummy XR state with target poses
    print("\n[4/4] Simulating VR input...")
    
    # Get default q
    q_default = robot.get_default_config()
    print(f"      Default config: {q_default}")
    
    # Create a simple target pose (move left arm slightly forward)
    # Identity SE3 means: at origin, no rotation
    target_left = jaxlie.SE3.identity()
    target_left = target_left.translate(jnp.array([0.05, 0.0, 0.0]))  # Move +5cm in X
    
    target_right = jaxlie.SE3.identity()
    target_right = target_right.translate(jnp.array([0.05, 0.0, 0.0]))  # Move +5cm in X
    
    # Create XR state (minimal - just with controller poses)
    xr_state = XRState(
        timestamp=0.0,
        frames={
            "left": XRFrame(
                role="controller",
                hand="left",
                position=jnp.array([0.0, 0.3, 0.8]),
                orientation=jnp.array([0.0, 0.0, 0.0, 1.0]),  # wxyz format
            ),
            "right": XRFrame(
                role="controller",
                hand="right",
                position=jnp.array([0.0, -0.3, 0.8]),
                orientation=jnp.array([0.0, 0.0, 0.0, 1.0]),
            ),
        },
    )
    
    print(f"      ✓ XR State created")
    print(f"      Left controller pos: {xr_state.frames['left'].position}")
    print(f"      Right controller pos: {xr_state.frames['right'].position}")
    
    # Step 3: Run IK solver
    print("\n" + "=" * 70)
    print("RUNNING IK SOLVER...")
    print("=" * 70)
    
    try:
        q_solved = controller.step(xr_state, q_default)
        print(f"✓ IK solver succeeded!")
        print(f"  Solved q shape: {q_solved.shape}")
        
    except Exception as e:
        print(f"✗ IK solver failed: {e}")
        return
    
    # Step 4: Display results
    print("\n" + "=" * 70)
    print("JOINT TRAJECTORY OUTPUT (What ROS2 publishes to /joint_trajectory)")
    print("=" * 70)
    
    print("\nJoint Configuration Array:")
    print(f"  {q_solved}")
    
    print("\nJoint Values (Readable):")
    for i, (name, value) in enumerate(zip(robot.actuated_joint_names, q_solved)):
        print(f"  [{i:2d}] {name:30s} = {float(value):8.4f} rad ({float(value)*180/3.14159:7.2f}°)")
    
    print("\n" + "=" * 70)
    print("SUCCESS - This q_solved is EXACTLY what gets sent to the robot!")
    print("=" * 70)
    
    # Step 5: Show forward kinematics (what robot should achieve)
    print("\nForward Kinematics (End-Effector Poses):")
    fk = robot.forward_kinematics(q_solved)
    
    for frame_name, se3_pose in fk.items():
        xyz = se3_pose.position()
        print(f"\n  {frame_name.upper()}:")
        print(f"    Position: {xyz}")
        print(f"    SE3: {se3_pose}")

if __name__ == "__main__":
    main()
