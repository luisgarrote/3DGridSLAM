# 3DGridSLAM - Exploiting 3D Grids for Indoor SLAM in Featureless Scenarios

This will be the repository for the 3DGridSLAM and 3DGridSLAM_Localization ROS packages.
Due to reasons outside the scope of this work, the final code release has been delayed until July. We include here a early development version of the code.

## Abstract

Accurate multi-sensor localization is a challenging task in the navigation of AMRs. Precise localization strategies are essential for AMRs to be able to perform with safety their missions in their surrounding environments. This work proposes a novel ROS-based modular 3D grid-based particle filter-based framework that can be used for Simultaneous Localization and Mapping (SLAM) or as a standalone robust localization strategy.
The framework uses odometry and 3D LiDAR data as inputs for localization and SLAM. To further improve localization and representation alignment, a pose refinement stage is employed using Levenberg-Marquardt minimization. The refinement stage considers keypoints in the environment to improve localization and uses the raw 3D point cloud for map maintenance. A pyramid-like 3D grid resolution is used to aid the refinement of the representation, improving pose estimates in featureless scenarios. Experimental validation was carried out with data acquired using an in-house platform, in a set of indoor and semi-structured scenarios comprised of critical featureless areas. The obtained results highlight the robustness of the proposed framework in both SLAM and localization tasks.

## Running 

```rosrun 3DGridSLAM PF2_final2```

## Citation

If you think this work is useful for your research, please consider citing:

```
@ARTICLE{garrote2024,
  author={L. Garrote and U. Reverendo and U. J. Nunes},
  journal={IEEE Access}, 
  title={Exploiting 3D Grids for Indoor SLAM in Featureless Scenarios}, 
  year={2024},
  volume={},
  number={},
  pages={}}

```
 
