# Extended Position Based Dynamics for Soft Body Simulation

![Project Visualization](assets/demo.gif)

This project was done as part of the course **"252-0546-00L Physically-Based Simulation in Computer Graphics"** at ETH Zürich
during the **HS2024** semester. We have implemented **XPBD (Extended Position Based Dynamics)** using advanced simulation methods and built it on a modern **MVC (Model-View-Controller)** architecture, created from the ground up.

---

## Overview

Our implementation of XPBD is inspired by the following sources:
- The original **["Extended Position Based Dynamics"](https://diglib.eg.org/server/api/core/bitstreams/e057a383-9937-4204-850a-97d851d0eebd/content)**
- **["XPBD: Position-Based Simulation of Compliant Constrained Dynamics"](https://dl.acm.org/doi/10.1145/2994258.2994272)**

The architecture is heavily inspired by:
- The **Homework framework** provided in the course.
- **[Engineering Tool 2021 Git Repository](https://gitlab.ethz.ch/mathiebr/engtool21/-/tree/master)** (partially derived from the same code base).

For geometric mesh-processing, we used a custom fork [libigl](https://libigl.github.io/), which supports **directional shadows** that are camera independent.
- The tool is also written in such a way to make it possible to overide the **vertex** and **fragment** shaders of both dynamic and static objects.

---

## Features

Our implementation of XPBD supports the following constraints:

1. **Distance Constraint**  
   Enforces a constant distance between two vertices, allowing for robust spring-like behavior.

2. **Tetrahedral Volume Constraint**  
   Preserves the volume of tetrahedral meshes, ensuring realistic deformation of 3D objects.

3. **Shell Volume Constraint**  
   Preserves the volume of non-tetrahedral meshes, using only the face vertices.

4. **Kinematic Friction**  
   Simulates the energy dissipation when two objects slide against each other.

5. **Static Friction**  
   Prevents motion between objects at rest relative to each other, up to a threshold.

6. **Static Collisions**  
   Resolves collisions between objects by preventing interpenetration and maintaining separation.

---

## How to Build

This project is built using **CMake**. To build the project:

1. Clone the repository:
    ```bash
    git clone git@github.com:ucinereo/PBS-soft-body.git
    ```

2. Pull assets using Git LFS:
   ```bash
   git lfs pull
   ```

3. Create a build directory and configure the project:
   ```bash
   mkdir build
   cd build
   cmake -DCMAKE_BUILD_TYPE=Release ..
   ```

4. Build the project:
   ```bash
   cmake --build .
   ```

---

## Usage

In the viewer there is a dropdown menu where the user can choose between three scenes: Palm Trees and Cuboids without
and with BVH acceleration. The Palm Trees scene was shown during the live demo and the Cuboid scenes can be used to see
the performance gains when using BVH acceleration. Note that the Cuboid scene without BVH might be running either extremely
slow or not at all, that's because the cuboids were heavily subdivided, resulting in 150k static triangles. Thus, this
scene serves as a stress test for our implementation.

The user can also play around with several simulation parameters, such as constraint compliance values, volume constraint
pressure and friction coefficients.

---

## Acknowledgments
- Thanks to [Matthias Müller](https://www.youtube.com/@matthimf) for the awesome YouTube videos, which helped alot implementing XPBD.
- Thanks to our course instructors and TAs for the nice lecture slides and explanations.

## License
Our software is available as open source under the terms of the [GPL-3.0](LICENSE.txt) license.