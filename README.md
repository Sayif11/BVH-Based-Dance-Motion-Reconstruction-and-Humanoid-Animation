# BVH-Based Dance Motion Reconstruction and Humanoid Animation

This repository contains a MATLAB-based pipeline to extract joint angles from
BVH motion-capture files and reconstruct full-body humanoid motion through
kinematic optimization and animation.

The project demonstrates how real dance motion data can be mapped onto a
simplified articulated humanoid model and visualized through smooth 3D
animation.

---

## Project Overview

Motion capture data stored in BVH (Biovision Hierarchy) format provides joint
rotations in a skeletal hierarchy. This project performs the following steps:

1. Parse a BVH file from an online motion-capture repository
2. Extract joint rotations relative to the neck frame
3. Convert motion data into joint-angle trajectories
4. Fit a simplified humanoid kinematic model using nonlinear optimization
5. Animate the reconstructed dance motion in MATLAB

The pipeline is designed to be modular and extensible for other BVH datasets
and humanoid models.

---

## Features

### BVH Parsing
- Custom BVH parser implemented from scratch
- Supports hierarchical joint structure and motion channels
- Extracts frame-wise joint rotations and timing

### Relative Joint Angle Extraction
- Computes joint rotations relative to the neck reference frame
- Converts BVH Euler rotations into rotation matrices
- Exports joint-angle data to CSV for reuse

### Humanoid Kinematic Model
- Simplified articulated humanoid with torso, arms, and legs
- Link lengths inferred from motion-capture data
- Forward kinematics implemented using rotation matrices

### Motion Reconstruction
- Nonlinear least-squares optimization (`lsqnonlin`)
- Fits humanoid pose to measured joint positions
- Ensures consistent limb lengths and body proportions

### Animation
- Smooth interpolation between poses
- Full 3D humanoid animation
- Adjustable camera, view, and playback speed

---

## Code Structure

- **BVH processing**
  - BVH file parsing
  - Joint hierarchy handling
  - Relative angle computation

- **Optimization**
  - Pose fitting using least-squares minimization
  - Error defined between measured and modeled joint positions

- **Visualization**
  - Forward kinematics plotting
  - Patch-based torso rendering
  - Line-based limb visualization

---

## How to Run

1. Place a BVH file in the project directory  
2. Update the BVH filename in the main script:
   ```matlab
   filename = 'Capoeira_Theodoros_v2.bvh';
