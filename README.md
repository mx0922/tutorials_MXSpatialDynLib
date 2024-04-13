# tutorials_MXSpatialDynLib
A simple tutorial of spatial dynamics library written and developed by [**Xiang Meng**](https://scholar.google.com/citations?user=iQAvBl0AAAAJ&hl=zh-TW) (Ph.D. candicate in BIT).

This is an **ultra-lightweight (32.0 KB)** and **user-friendly robot dynamics library**, which includes solutions for kinematics, dynamics, centroidal dynamics, and various matrix computations, facilitating the research process of model-based motion planning and control for legged robots. This dynamics library does not rely on any external solver libraries. Users only need to refer to the robot construction examples provided in the **`setupRobotSpatialModel_myRobot_fixedBase.m`** function to build their own legged robots, such as **humanoid robots, quadruped robots**, etc. Users do not need to concern with the solving process and they can directly obtain the desired matrices referring to the **`test_example.m`** file.

The advantages of this dynamics library are as follows:

1. Users do **not** need to construct robot models using URDF or XML files; they only need to provide simple topological descriptions of the robot.
2. Despite being lightweight, it is **fully functional**, capable of performing common kinematic computations such as Jacobian matrices, dynamic computations such as the inertia matrix \(M\), and centroid dynamics-related matrices such as Centroidal Momentum Matrix \(CMM\).
3. Users can **easily download** and include it in their matlab test programs, with clear function calls that are ready to use immediately.
4. Users only need to have **basic knowledge of robot dynamics and introductory knowledge of spatial dynamics algorithms** to use this library effortlessly.
