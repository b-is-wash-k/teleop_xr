#!/usr/bin/env python3
"""
Minimal IK solver output test - no ROS2, no complex imports.
Just shows what joint angles the solver computes.
"""

import sys
import numpy as np

# Add workspace
sys.path.insert(0, "/home/air-lab-ncsu/OPEN_ARM_NEW/src/teleop_xr")

from teleop_xr.ik.robots.openarm import OpenArmRobot
from teleop_xr.ik.solver import PyrokiSolver

def main():
    print("=" * 70)
    print("MINIMAL IK OUTPUT TEST")
    print("=" * 70)
    
    # Step 1: Load robot
    print("\n[1] Loading OpenArm robot...")
    robot = OpenArmRobot()
    print(f"    Joints: {len(robot.actuated_joint_names)}")
    
    # Step 2: Create solver
    print("\n[2] Initializing solver (JAX JIT compile)...")
    solver = PyrokiSolver(robot)
    print(f"    Warmup complete: {solver.warmup_complete}")
    
    # Step 3: Get default config
    print("\n[3] Getting default joint configuration...")
    q_default = robot.get_default_config()
    
    print("\nDEFAULT JOINT ANGLES:")
    print("-" * 70)
    for name, angle in zip(robot.actuated_joint_names, q_default):
        print(f"  {name:30s}: {angle:7.3f} rad ({np.degrees(angle):7.1f}°)")
    
    # Step 4: Test IK solve with default target
    print("\n[4] Running IK solver...")
    print("    Target: Both arms at default positions")
    
    try:
        # Get default FK (this is the target)
        fk_default = robot.forward_kinematics(q_default)
        print("\n    Default FK:")
        for frame, pose in fk_default.items():
            pos = pose.translation()
            print(f"      {frame}: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
        
        # Solve IK with default target
        q_solved = solver.solve(
            target_L=fk_default["left"],
            target_R=fk_default["right"],
            target_Head=None,
            q_current=q_default
        )
        
        print("\nSOLVED JOINT ANGLES:")
        print("-" * 70)
        for name, angle in zip(robot.actuated_joint_names, q_solved):
            print(f"  {name:30s}: {angle:7.3f} rad ({np.degrees(angle):7.1f}°)")
        
        # Verify with FK
        print("\nVERIFY (Forward Kinematics of solved config):")
        fk_solved = robot.forward_kinematics(q_solved)
        for frame, pose in fk_solved.items():
            pos = pose.translation()
            print(f"  {frame}: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
        
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
        return
    
    print("\n" + "=" * 70)
    print("✅ DONE")
    print("=" * 70)
    print("\n📌 The SOLVED angles above are EXACTLY what gets published to")
    print("   /joint_trajectory in ROS2 mode.")

if __name__ == "__main__":
    main()
