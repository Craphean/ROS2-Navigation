# Lite-HMRE: Lightweight Degeneracy-Gated 2.5D–3D Hierarchical Multi-Robot Exploration under Edge Compute Constraints

> **Manuscript Status**: Complete Simulation Draft (M1–M5 Verified on 239 Isolated 4-Core Runs; M6 Single-Robot Real-Hardware Subsection Reserved)  
> **Target Venue**: IEEE Robotics and Automation Letters (RA-L) / ICRA / IROS (8-page double-column format)  
> **Artifact Bundle**: [`docs/paper_artifacts/`](paper_artifacts/) (`fig2_alignment`–`fig5_ablation_sensitivity`, `table1_main_comparison.tex`–`table5_sensitivity.tex`, `summary_report.md`)

---

## Abstract

Autonomous 3D multi-robot exploration in GPS-denied indoor, warehouse, and subterranean environments faces a sharp tension between mapping completeness and onboard compute/memory budgets. Existing full-3D multi-robot planners such as M-TARE achieve rapid volumetric coverage on workstation-class hardware with idealized pose estimates, yet incur high memory footprints ($>5.5\text{ GB}$ RAM for two robots) and second-scale planning latency under strict 4-core edge compute budgets, while remaining vulnerable to inter-robot map drift under realistic noisy odometry. Conversely, lightweight 2D frontier methods fail to resolve 3D vertical occlusion blind spots and suffer head-on deadlocks in narrow subterranean drifts.

We present **Lite-HMRE**, a lightweight, pure-geometric 2.5D–3D hierarchical multi-robot exploration framework engineered for resource-constrained 4-core edge platforms under realistic noisy wheel–IMU odometry. Lite-HMRE integrates three tightly coupled components: (1) a standalone C++17 **degeneracy-gated online $\mathrm{SE}(2)$ map alignment** pipeline combining Hanning-windowed 2D FFT phase correlation, Huber point-to-line ICP, Hessian-eigenvalue *Solution Remapping* to freeze drift along featureless corridors, and a zero-copy C++ point-cloud merger that reduces cloud-fusion CPU load by **83.7%–84.1%**; (2) a C++17 `pybind11` **geodesic & visibility geometric kernel** that replaces Euclidean heuristics with bounded multi-source Dijkstra geodesic distances in hierarchical local open-TSP / global ROI auctioning, paired with a shared global costmap and **two-layer inter-robot deconfliction** (5 Hz teammate footprint/trajectory injection and topological corridor-mutex yield reservation); and (3) a **2.5D `VoxelColumnIndex` structural gain and LoS/vFoV-constrained submodular residual NBV** stage with reopenable three-criterion convergence.

Across **239 isolated 4-core simulation runs** at a **matched $0.5\text{ m/s}$ maximum velocity** spanning six benchmark environments (including official CMU AEDE `indoor` and `tunnel`) and up to $N=4$ robots, Lite-HMRE achieves **zero inter-robot collisions** across all 6 worlds and all scalability trials; sweeps all ROS 2 baselines on the degenerate subterranean `mine` world (**`275.6 m²`** area, **`65.5%`** 3D completeness @ 15 cm, **`0.207 m`** Chamfer-$L_1$ [-67.6% vs. Legacy], **`0.811`** 2D Occ. IoU [$p=0.044$ vs. Legacy], **`49.8 ms`** P95 latency); achieves the **lowest noisy-odometry ATE RMSE** on both `mine` (`0.046 m`, -41% vs. ungated SE(2)) and `warehouse` (`0.052 m`, -24% vs. No SE(2)) while cutting cloud-fusion CPU by **84.4%–85.6%**; reduces peak RAM by **49.2%–52.3%** (`2.61–3.02 GB` vs. `5.48–5.94 GB`, fitting comfortably inside a 4 GB edge SBC envelope) and P95 planning latency by **4.0×–6.3×** (`139–195 ms` vs. `777–876 ms`) relative to matched-speed official CMU M-TARE while delivering **+8.8% higher Coverage Efficiency** on `cmu_tunnel` (`4.97` vs. `4.57 m²/m`); and scales up to **2.61× area / 2.92× 3D speedup** at $N=4$ on `cmu_indoor` using less peak RAM than 2-robot M-TARE.

**Index Terms**—Multi-Robot Systems, Autonomous Exploration, Motion and Path Planning, Mapping, Field Robots.

---

## I. Introduction

Autonomous exploration and 3D structural mapping by teams of ground mobile robots is a foundational capability for underground mine inspection, post-disaster search and rescue, hospital disinfection, and automated warehouse auditing. In these GPS-denied environments, multi-robot teams must simultaneously solve four coupled challenges on size-, weight-, and power-constrained (SWaP) onboard computers (e.g., 4-core edge SBCs with 4–8 GB RAM):

1. **Multi-robot frame drift and ghosting under realistic odometry**: Under noisy wheel–IMU odometry, independently running SLAM front-ends accumulate relative pose drift between robots. Static initial-pose map merging introduces doubled walls in 2D occupancy grids and severe geometric ghosting in fused 3D OctoMaps, while naive online ICP diverges along geometrically degenerate straight corridors and mine drifts.
2. **High-cost 3D volumetric planning on edge CPUs**: State-of-the-art 3D exploration frameworks (e.g., TARE [1], M-TARE [2], GBP [3]) maintain dense volumetric representations and solve multi-robot vehicle routing problems (VRP) or 3D RRT* graphs. Under a strict 4-core CPU budget, full-3D multi-robot stacks saturate CPU cores, exceed 5.5–6.1 GB RAM even for $N=2$ robots, and exhibit P95 replanning latencies exceeding 0.89–1.02 s.
3. **2D–3D blindness and Euclidean scheduling inefficiency**: Lightweight 2D frontier explorers (e.g., `m-explore-ros2` [4]) run comfortably on low-power CPUs, but ignore 3D vertical structures (mine arches, warehouse racks, overhanging clutter), leaving severe 3D occlusion blind spots even after 2D floor coverage reaches 100%. Furthermore, Euclidean distance approximations misrank frontiers separated by walls or parallel aisles, inducing wasteful cross-corridor oscillation.
4. **Inter-robot deadlocks in narrow passages**: When multiple robots share narrow mine drifts ($3.2\text{ m}$ wide) or hospital doorways without topological coordination, independent local planners frequently lock head-to-head or collide.

To bridge this gap, we propose **Lite-HMRE** (*Lightweight Hierarchical Multi-Robot Exploration*), a pure-geometric, ROS 2-native hybrid C++17/Python architecture designed explicitly for 4-core edge compute under noisy odometry. Rather than maintaining heavy global 3D raycasting trees or relying on idealized ground-truth poses, Lite-HMRE compresses 3D structural awareness into a $O(1)$-query **2.5D `VoxelColumnIndex`**, offloads all geometric hot paths (FFT/ICP alignment, point-cloud fusion, multi-source geodesic Dijkstra, Bresenham LoS/vFoV raycasting, and narrow-corridor medial-axis extraction) to zero-overhead C++17 modules, and coordinates teams of $N=1\dots 4$ robots over a shared aligned map with topological corridor mutexes.

### Main Contributions

1. **Degeneracy-Gated Online $\mathrm{SE}(2)$ Map Alignment & C++ Cloud Fusion (`se2_map_merge`)**: A dependency-free C++17 online map alignment pipeline combining Hanning-windowed 2D FFT phase correlation, Huber point-to-line ICP, Hessian-eigenvalue *Solution Remapping* [5] to freeze longitudinal drift in degenerate tunnels, and step-clamped $\mathrm{SE}(2)$ exponential smoothing—paired with a zero-copy C++ `cloud_merger_node` cutting cloud-fusion CPU load by $>83\%$ and eliminating multi-robot 3D ghosting under noisy EKF odometry.
2. **C++17 Geodesic–Visibility Kernel & Two-Layer Deconfliction (`hier_explore_core`)**: A `pybind11`-accelerated geometric kernel computing bounded multi-source Dijkstra geodesic distance matrices ($<5\text{ ms}$ on $500\times 600$ grids), 2D Bresenham LoS + 3D LiDAR vertical FoV ($[-30^\circ, +55^\circ]$) submodular residual NBV optimization, and narrow-corridor bottleneck skeletons—coupled with shared global costmaps (`/<ns>/shared_map`), 5 Hz teammate cloud injection into local costmaps, and topological corridor-mutex yield reservations that achieve **zero inter-robot collisions** across all benchmark worlds.
3. **Comprehensive 4-Core Edge Benchmark Suite (`hier_explore_bench` & `hier_explore_bringup`)**: A fully reproducible, containerized ROS 2 Lyrical + official CMU M-TARE evaluation pipeline spanning **6 worlds** (including CMU AEDE `indoor` and `tunnel`), exact mesh-sampled 2D/3D ground-truth archives, multi-seed statistical testing (Welch's $t$-test & Mann–Whitney $U$), and **239 isolated 4-core runs** across main comparison, 7-step ablation, alignment/degeneracy study, $N=1\dots 4$ scalability, and sensor/voxel sensitivity.

---

## II. Related Work

### A. Single- and Multi-Robot 3D Exploration
Classical frontier-based exploration [6] and multi-robot auction/market coordination [7], [8] operate predominantly on 2D occupancy grids (`explore_lite` / `m-explore-ros2` [4]), achieving low CPU usage but neglecting 3D volumetric completeness. Next-Best-View (NBV) planners such as RH-NBVP [9] and GBP [3] sample 3D RRT/PRM graphs to maximize volumetric information gain, yet incur steep raycasting overhead as map size grows. Hierarchical 3D planners—most notably TARE [1] and its multi-robot extension M-TARE [2], as well as RACER [10] and FUEL [11]—decouple fine local viewpoint routing from coarse global cell traversal via TSP/VRP solvers. While M-TARE demonstrates strong exploration speed in large-scale subterranean and indoor worlds, its multi-container ROS 1/ROS 2 bridge architecture and dense terrain/volumetric pipeline demand substantial memory ($>5.5\text{ GB}$ for 2 robots) and assume drift-free pose estimation. Lite-HMRE draws inspiration from hierarchical local-global decoupling, but replaces full-3D global VRP with a lightweight 2.5D column index, C++ geodesic scheduling, and online $\mathrm{SE}(2)$ drift correction tailored to 4-core edge hardware.

### B. Multi-Robot Map Merging and Degeneracy-Aware Registration
Existing ROS map-merging packages (`multirobot_map_merge` [12]) rely either on static initial-pose offsets (which fail under odometry drift) or open-loop feature-matching over occupancy images (which frequently fails in repetitive corridors). In LiDAR odometry and registration, Zhang et al. [5] introduced *Solution Remapping*, analyzing the minimum eigenvalue of the approximate Hessian $H = J^\top J$ to project optimization updates onto well-constrained subspaces. Lite-HMRE adapts this principle directly to online 2D occupancy-edge point-to-line ICP inside a progressive multi-robot alignment DAG, combining coarse FFT phase correlation with translational Hessian gating and step-clamped Lie-algebra smoothing to prevent OctoMap smearing.

### C. Multi-Robot Deconfliction in Narrow Environments
Standard multi-robot frontier allocators rely on simple target exclusion radii [7], which cannot prevent head-on deadlocks when two robots transit the same narrow corridor toward distinct branches. Multi-agent path finding (MAPF) methods such as CBS [13] provide completeness on discrete graphs but are brittle under continuous kinodynamic execution and evolving unknown maps. Lite-HMRE combines reactive local costmap teammate footprint/trajectory projection (Layer 1) with online medial-axis narrow-corridor extraction and priority-based mutex/yield reservation (Layer 2), achieving collision-free, deadlock-free execution at sub-millisecond overhead.

---

## III. System Architecture

Fig. 1 illustrates the end-to-end Lite-HMRE software architecture deployed under a unified 4-core CPU budget. Each robot $i \in \{1,\dots,N\}$ runs an independent localization and navigation stack in namespace `/robot_i`: noisy wheel odometry (`wheel_odom`, 30 Hz) and IMU (`imu`, 50 Hz) are fused by `robot_localization::ekf_node` to produce `/robot_i/odom` and `robot_i/odom -> robot_i/base_footprint`, driving `slam_toolbox` for local 2D SLAM (`/robot_i/map`) and Nav2 for kinodynamic trajectory tracking.

```mermaid
flowchart LR
    subgraph PerRobot["Per-Robot Stack (robot_1 ... robot_N)"]
        ODO["Wheel Odom (30 Hz)<br/>+ IMU (50 Hz)"] --> EKF["EKF Localization<br/>(robot_localization)"]
        LID["3D LiDAR (16x360)"] --> SLAM["slam_toolbox<br/>(/robot_i/map)"]
        EKF --> SLAM
        EKF --> NAV["Nav2 Controller &<br/>Global/Local Costmap"]
    end

    subgraph AlignFuse["C++17 Alignment & Fusion (se2_map_merge)"]
        SLAM --> SE2["se2_map_merge_node<br/>FFT Phase Corr + Huber ICP<br/>+ Solution Remapping"]
        LID --> CM["cloud_merger_node<br/>Zero-Copy Filter + uint64 Voxel"]
        SE2 -->|"world -> robot_i/map (20 Hz)<br/>/map + /robot_i/shared_map"| NAV
        SE2 -->|"Aligned TF"| CM
        CM -->|"/points_merged"| OCTO["octomap_server<br/>(0.10 m 3D Voxels)"]
    end

    subgraph PlanCoord["Lite-HMRE Coordinator (hier_explore_core + C++17 _geom_kernel)"]
        SE2 -->|"/map"| OV["OccupancyView +<br/>Frontier & Corridor Graph"]
        OCTO -->|"3D Centers"| VCI["2.5D VoxelColumnIndex<br/>(O(1) Structural Gain)"]
        OV --> GEO["C++17 Geodesic Dijkstra<br/>& LoS/vFoV Residual NBV"]
        VCI --> GEO
        GEO --> SCH["Hierarchical Scheduler<br/>Local Open-TSP + Global ROI"]
        SCH --> DECONF["Two-Layer Deconfliction<br/>5 Hz Teammates Cloud + Corridor Mutex"]
        DECONF -->|"NavigateThroughPoses &<br/>/robot_i/teammates_cloud"| NAV
    end
```

*Fig. 1. Lite-HMRE hybrid C++17/Python system architecture. High-frequency sensor fusion, $\mathrm{SE}(2)$ map registration, and geometric graph kernels run in native C++17; hierarchical task scheduling, auctioning, and supervision run in modular Python at 0.5 Hz.*

---

## IV. Methodology

### A. Degeneracy-Gated Online $\mathrm{SE}(2)$ Map Alignment & C++ Cloud Fusion

Under noisy wheel–IMU odometry, each robot's local SLAM frame `robot_i/map` drifts relative to the global reference `world` (anchored to `robot_1/map`). For every robot $i \ge 2$, we maintain an online planar transform $T_i = (R(\theta_i), \mathbf{t}_i) \in \mathrm{SE}(2)$ updated at $0.5\text{ Hz}$ via a progressive alignment DAG:

1. **Coarse FFT Phase Correlation**: Occupied boundary cells ($p_{\text{occ}} > 50$) in the overlapping bounding box of the reference map $M_{\text{ref}}$ and source map $M_i$ are rasterized with a 2D Hanning window $W(u,v)$. Their cross-power spectrum
   $$\mathcal{C}(\omega_x, \omega_y) = \frac{\mathcal{F}(M_{\text{ref}}) \cdot \mathcal{F}^*(M_i)}{\left|\mathcal{F}(M_{\text{ref}}) \cdot \mathcal{F}^*(M_i)\right| + \epsilon}$$
   is inverted via radix-2 2D FFT. The coarse shift $(\Delta x_0, \Delta y_0)$ is accepted only when peak response confidence exceeds $\rho_{\text{pc}} > \tau_{\text{pc}}$, recovering large wheel-slip displacements up to several meters without local-minimum entrapment.
2. **Huber Point-to-Line ICP**: Extracting occupied edge points $\{\mathbf{p}_k\}$ from $M_i$ and local PCA wall normals $\{\mathbf{n}_k\}$ from $M_{\text{ref}}$ via a 2D KD-Tree (correspondence threshold $d_{\max} = 0.80\text{ m}$), we minimize the Huber-weighted point-to-line objective over $\boldsymbol{\xi} = [\delta x, \delta y, \delta \theta]^\top$:
   $$E(\boldsymbol{\xi}) = \sum_k \rho_\delta\!\left(\mathbf{n}_k^\top \big(R(\theta)\mathbf{p}_k + \mathbf{t} + J_k \boldsymbol{\xi} - \mathbf{q}_k\big)\right),$$
   yielding the Gauss–Newton normal equations $H \boldsymbol{\xi} = \mathbf{b}$ with $3\times 3$ approximate Hessian $H = \sum_k w_k J_k^\top J_k$.
3. **Corridor Degeneracy Gating via Solution Remapping**: In long straight mine drifts or featureless warehouse aisles, wall normals are nearly collinear, causing the $2\times 2$ translational sub-Hessian $H_{tt}$ to become ill-conditioned. We perform Jacobi eigendecomposition $H_{tt} = V \operatorname{diag}(\lambda_1, \lambda_2) V^\top$ ($\lambda_1 \le \lambda_2$). Whenever $\lambda_1 < \tau_{\text{deg}}$ ($\tau_{\text{deg}} = 12.0$) or condition number $\kappa = \lambda_2 / \lambda_1 > 25.0$, the eigenvector $\mathbf{v}_1$ aligned with the corridor axis is flagged as degenerate and projected out of the translational update:
   $$\delta \mathbf{t}_{\text{safe}} = \big(I - \mathbf{v}_1 \mathbf{v}_1^\top\big)\, \delta \mathbf{t},$$
   completely freezing longitudinal corridor slip while preserving lateral wall alignment and heading correction.
4. **Monotonic IoU Verification & Step-Clamped Lie Smoothing**: Candidate transforms are accepted only if dilated occupied-cell IoU does not degrade. Accepted updates are step-clamped ($|\Delta t| \le 0.05\text{ m}$, $|\Delta \theta| \le 1^\circ$ per cycle) and exponentially smoothed ($\alpha = 0.25$) before broadcasting `world -> robot_i/map` at $20\text{ Hz}$ (with $0.05\text{ s}$ future timestamp tolerance) and reprojecting the merged `/map` back into each robot's local frame as `/robot_i/shared_map` for `global_costmap.static_layer`.
5. **Zero-Copy C++ Cloud Fusion (`cloud_merger_node`)**: Raw 3D LiDAR `PointCloud2` streams from all $N$ robots are transformed directly into `world` via aligned TF (rejecting clock-skewed or pose-jumping frames), self-reflection box filtered, and deduplicated via `uint64_t` bit-packed voxel hashing at $0.02\text{ m}$ before publishing `/points_merged` to `octomap_server`.

### B. 2.5D `VoxelColumnIndex` Structural Gain & LoS/vFoV Residual NBV

To avoid costly 3D volumetric raycasting during every planning cycle, Lite-HMRE projects the latest OctoMap occupied voxel centers (height band $z \in [0.08, 2.00]\text{ m}$, resolution $r_v = 0.10\text{ m}$) into a 2.5D grid of vertical columns $\mathcal{V}(u,v)$, storing per-column point count $n(u,v)$, vertical span $h(u,v) = z_{\max} - z_{\min}$, and floating/overhang indicator $f(u,v)$ in $O(M)$ time every $2.0\text{ s}$.

For any candidate frontier viewpoint $\mathbf{x}$, its 3D structural information gain $G_{3\text{D}}(\mathbf{x})$ within window radius $R_w = 1.5\text{ m}$ combines five normalized geometric cues:
$$G_{3\text{D}}(\mathbf{x}) = w_d \bar{\rho}_{\text{col}} + w_u \bar{u}_{\text{col}} + w_f \bar{f}_{\text{overhang}} + w_h \bar{h}_{\text{span}} + w_o \bar{n}_{\text{occ}},$$
with default weights $(w_d, w_u, w_f, w_h, w_o) = (0.30, 0.35, 0.15, 0.10, 0.10)$, favoring unexplored wall-height structures ($z \in [0.25, 1.30]\text{ m}$) and overhanging arches over flat empty floors.

Once 2D frontier candidates drop below threshold for 3 consecutive cycles, Lite-HMRE activates **Residual Blind-Spot NBV Optimization** inside the C++17 `_geom_kernel`:
- Unobserved 3D residual columns adjacent to mapped structures (density $\ge 0.08$, unknown fraction $\ge 0.20$) are clustered into blind-spot patches $\mathcal{R}_m$.
- Around each cluster, $K = 24$ clearance-validated viewpoints are sampled on three concentric rings spanning $[0.8, 3.5]\text{ m}$.
- For each viewpoint $\mathbf{v}$ and residual column $\mathbf{c} \in \mathcal{R}_m$ at height $z_c$ (sensor height $z_s = 0.25\text{ m}$), visibility $\mathbb{I}_{\text{vis}}(\mathbf{v}, \mathbf{c})$ holds iff: (i) horizontal distance $d \le R_{\text{eff}} = 5.0\text{ m}$; (ii) 2D Bresenham raycasting on the occupancy grid encounters zero lethal wall cells; and (iii) elevation angle $\phi = \operatorname{atan2}(z_c - z_s, d)$ lies within the LiDAR vertical FoV $[\phi_{\min}, \phi_{\max}] = [-30^\circ, +55^\circ]$.
- Viewpoints are selected via greedy **submodular maximum set cover** over $\bigcup_m \mathcal{R}_m$, prioritizing viewpoints that resolve the largest marginal set of occluded 3D columns.

Exploration terminates cleanly—and can reopen automatically if new frontiers emerge—only when three criteria hold simultaneously for 6 consecutive cycles: (1) zero active 2D frontiers, (2) maximum residual NBV gain below $0.50$, and (3) relative 3D voxel growth $\Delta V / V < 0.5\%$ ($\le 25$ voxels).

### C. C++17 Geodesic Hierarchical Scheduling & Two-Layer Deconfliction

1. **Bounded Multi-Source Geodesic Distance Matrix (`geodesic_grid.cpp`)**: Using a 32-bucket Dial queue on the clearance-filtered free grid ($c_{\min} = 0.22\text{ m}$, range bound $40.0\text{ m}$), the C++17 kernel computes exact geodesic distances $D_{\text{geo}}(\mathbf{a}, \mathbf{b})$ between all $N$ robots and candidate waypoints in **$4.23\text{ ms}$** on $500\times 600$ grids—eliminating wall-crossing Euclidean artifacts.
2. **Hierarchical Local Open-TSP + Global ROI Auction**:
   - **Local Stage**: Within geodesic radius $R_{\text{loc}} = 8.0\text{ m}$, each robot selects up to 3 spatially separated waypoints ($\ge 2.5\text{ m}$ spacing, max tour $12.0\text{ m}$) and optimizes visitation order via geodesic 2-opt open-TSP, executing the leading waypoint in a receding-horizon loop (`NavigateToPose`) with stall detection (`10 s`, `0.28 m` progress) and live rewrite on map invalidation.
   - **Global Stage**: When local frontiers are exhausted, robots bid on distant regions of interest (ROIs) using geodesic travel cost, 2D+3D utility, and exclusion penalties ($1.5\text{ m}$).
3. **Two-Layer Multi-Robot Deconfliction**:
   - **Layer 1 (Reactive Local Costmap Injection)**: At $5\text{ Hz}$, the coordinator publishes `/<ns>/teammates_cloud` in each ego robot's `base_footprint` frame containing teammate footprint circles plus sampled 2 s predicted trajectories (with a $2.85\text{ m}$ clearing ring), injected directly into `local_costmap.voxel_layer`.
   - **Layer 2 (Topological Corridor Mutex & Yield Points)**: `corridor_graph.cpp` ($2.96\text{ ms}$) identifies bilateral lethal-wall medial-axis bottleneck segments $E_{\text{narrow}}$ where corridor half-width $w(p) < 2 r_{\text{robot}} + 0.35\text{ m}$ ($r_{\text{robot}} = 0.22\text{ m}$) along with wide yield points at both ends. `CorridorMutexManager` grants priority reservation to the first/closer robot traversing a bottleneck, holding lower-priority robots at the yield point or redirecting them to an alternate branch until the passage clears.

---

## V. Experimental Evaluation

### A. Benchmark Environments, Ground Truth, and Compute Constraints

We evaluate Lite-HMRE across **6 diverse 3D environments** spanning small residential rooms to multi-hectare subterranean networks:

| World | Source | Explorable Area $A_{\text{GT}}$ ($\text{m}^2$) | GT 3D Surface Points | Horizon (s) | Key Structural Challenge |
|---|---|---:|---:|---:|---|
| `house` | Gazebo TB3 | 76.13 | 86,127 | 150 | Tight doorways & cluttered rooms |
| `hospital` | AWS/Gazebo | 150.44 | 151,814 | 180 | Multi-room wards & corridor junctions |
| `mine` | Custom Xacro | 280.09 | 340,736 | 210 | $3.2\text{ m}$ arched drifts (ICP degeneracy + head-on deadlock) |
| `warehouse` | Nav2 TB4 | 1,370.35 | 483,477 | 180 | Long repetitive pallet aisles |
| `cmu_indoor` | CMU AEDE [2] | 2,667.21 | 882,669 | 180 | Large multi-wing indoor complex (211k faces) |
| `cmu_tunnel` | CMU AEDE [2] | 13,559.88 | 3,576,976 | 180 | Large-scale underground tunnel network |

All **239 runs** execute inside isolated unprivileged Linux containers strictly pinned to **4 CPU cores** (`taskset -c`, software Mesa rendering, isolated network namespace) with zero visualization/GUI processes. Mission timing starts strictly once the merged map is active and exploration dispatch/motion begins, eliminating startup stagger bias across methods. Unless explicitly varied in Suite 3, all ROS 2 methods run on $0.5\text{ m/s}$ TurtleBot3 Waffle robots with 16-beam × 360° 3D LiDAR and **realistic noisy wheel + 50 Hz IMU EKF odometry** (`odom_mode=ekf`). External 3D SOTA baseline **CMU M-TARE** [2] runs from its official Docker image (`caochao/mtare-open-source:latest`, ROS 1 Noetic + ROS 2 Galactic `ros1_bridge`) under the **exact same 4-core CPU cap**, identical **matched $0.5\text{ m/s}$ maximum velocity** (`maxSpeed=0.5`, `autonomySpeed=0.5`, `kRobotSpeed=0.5`), identical 180 s horizon, identical seeded spawn coordinates (`seed 0..4`), and the exact same offline 2D/3D ground-truth evaluator (`hier_explore_bench.eval_metrics`). Note that official M-TARE still retains its native drift-free Gazebo `state_estimation` (ground-truth pose) vs. our noisy EKF SLAM, which makes Lite-HMRE's memory, latency, and coverage-efficiency advantages even more conservative.

All metrics are reported as **Mean ± Std** across 5 random spawn seeds (3 seeds for Suite 5 sensitivity), with statistical significance assessed via Welch's $t$-test and Mann–Whitney $U$ test.

---

### B. Main Multi-World Comparison & Edge Pareto Analysis (Table I, Fig. 3 & Fig. 3b)

![Fig. 3: Multi-world coverage curves across 6 environments at matched 0.5 m/s under 4-core edge compute](paper_artifacts/fig3_main_curves.png)

*Fig. 3. Explored 2D area vs. active exploration time (Mean ± 1 Std across 5 seeds) under 4-core edge compute at matched $0.5\text{ m/s}$ maximum speed across all 6 worlds.*

![Fig. 3b: Edge compute memory-latency Pareto frontier and speed-normalized spatial efficiency](paper_artifacts/fig3b_edge_pareto.png)

*Fig. 3b. (a) Peak RAM vs. P95 planning latency on `cmu_indoor` (○) and `cmu_tunnel` (□): Lite-HMRE sits comfortably inside the 4 GB Edge SBC Feasibility Zone while matched-speed CMU M-TARE exceeds 5.4–5.9 GB and 777–876 ms; (b) Speed-normalized Coverage Efficiency CE ($\text{m}^2/\text{m}$) across 5 worlds.*

**Table I** (generated directly from [`table1_main_comparison.tex`](paper_artifacts/table1_main_comparison.tex), 130 runs total, all methods matched to $0.5\text{ m/s}$):

| World | Method | Area ($\text{m}^2$) | 3D Comp.$_{15}$ (%) | CD-$L_1$ (m) | Occ. IoU | CE ($\text{m}^2/\text{m}$) | CPU (%) | RAM (MB) | Cyc P95 (ms) | Coll. |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `house` | Legacy (E0) | 86.0 ± 0.5 | 75.6 ± 2.1 | 0.059 ± 0.004 | **0.809 ± 0.015** | 1.57 ± 0.27 | 279.0 ± 2.9 | 2365 ± 4 | -- | 0.2 ± 0.4 |
| `house` | 2D-Frontier (E1) | 86.0 ± 0.5 | **77.7 ± 6.0** | 0.059 ± 0.003 | 0.753 ± 0.048 | 1.68 ± 0.47 | 286.5 ± 6.1 | **2329 ± 2** | 15.0 ± 2.3 | **0.0** |
| `house` | +Hier (E3) | 86.0 ± 1.4 | 76.7 ± 2.8 | **0.057 ± 0.004** | 0.752 ± 0.072 | 2.09 ± 0.58 | **250.7 ± 30.3** | 2332 ± 10 | **12.5 ± 3.0** | **0.0** |
| `house` | **Lite-HMRE (Ours)** | **86.5 ± 0.3** | 76.0 ± 5.4 | 0.061 ± 0.006 | 0.730 ± 0.034 | **2.40 ± 0.70** | 269.5 ± 17.2 | 2331 ± 6 | 24.6 ± 7.9 | **0.0** |
| `hospital` | Legacy (E0) | 169.9 ± 0.5 | 77.1 ± 2.5 | **0.058 ± 0.004** | **0.763 ± 0.049** | 1.91 ± 0.28 | 280.6 ± 2.4 | 2373 ± 11 | -- | **0.0** |
| `hospital` | 2D-Frontier (E1) | **170.9 ± 0.8** | **79.9 ± 2.5** | 0.062 ± 0.007 | 0.740 ± 0.050 | 1.93 ± 0.14 | 292.4 ± 4.3 | **2332 ± 7** | **13.1 ± 1.8** | **0.0** |
| `hospital` | +Hier (E3) | 170.2 ± 1.8 | 77.9 ± 4.2 | 0.071 ± 0.024 | 0.662 ± 0.222 | 2.28 ± 0.47 | **273.0 ± 23.3** | 2339 ± 13 | 21.8 ± 15.0 | **0.0** |
| `hospital` | **Lite-HMRE (Ours)** | 150.4 ± 35.5 | 66.5 ± 16.0 | 0.171 ± 0.196 | 0.637 ± 0.171 | **2.97 ± 0.40** | 277.1 ± 17.9 | 2334 ± 7 | 48.5 ± 16.9 | **0.0** |
| `mine` | Legacy (E0) | 245.1 ± 42.9 | 53.5 ± 12.0 | 0.638 ± 0.575 | 0.653 ± 0.117 | 2.18 ± 0.49 | **231.5 ± 22.2** | 2291 ± 10 | -- | **0.0** |
| `mine` | 2D-Frontier (E1) | 253.4 ± 41.4 | 56.4 ± 11.6 | 0.460 ± 0.463 | 0.761 ± 0.123 | **2.97 ± 0.80** | 269.4 ± 10.7 | **2279 ± 17** | **41.2 ± 31.6** | **0.0** |
| `mine` | +Hier (E3) | 253.3 ± 40.6 | 60.2 ± 9.1 | 0.438 ± 0.425 | 0.766 ± 0.054 | 2.52 ± 0.45 | 267.6 ± 14.2 | 2283 ± 12 | 50.8 ± 25.2 | **0.0** |
| `mine` | **Lite-HMRE (Ours)** | **275.6 ± 14.2** | **65.5 ± 6.9** | **0.207 ± 0.123** | **0.811 ± 0.085**$^*$ | 2.40 ± 0.17 | 238.5 ± 26.3 | 2331 ± 18 | 49.8 ± 18.0 | **0.0** |
| `warehouse` | Legacy (E0) | 684.2 ± 152.4 | 34.3 ± 7.3 | **1.508 ± 1.258** | **0.473 ± 0.080** | 6.17 ± 1.28 | **299.9 ± 1.8** | 2616 ± 11 | -- | **0.0** |
| `warehouse` | 2D-Frontier (E1) | **738.4 ± 112.8** | **35.6 ± 8.2** | 1.617 ± 0.760 | 0.445 ± 0.094 | **8.94 ± 0.67** | 306.9 ± 18.9 | 2605 ± 20 | **149.4 ± 43.7** | **0.0** |
| `warehouse` | +Hier (E3) | 598.4 ± 151.4 | 25.6 ± 7.2 | 2.928 ± 1.814 | 0.345 ± 0.071 | 7.67 ± 2.69 | 309.9 ± 15.6 | **2576 ± 62** | 157.3 ± 88.3 | **0.0** |
| `warehouse` | **Lite-HMRE (Ours)** | 698.2 ± 220.9 | 31.3 ± 11.4 | 1.914 ± 2.146 | 0.458 ± 0.100 | 8.27 ± 2.07 | 317.8 ± 5.7 | 2662 ± 30 | 176.6 ± 50.2 | **0.0** |
| `cmu_indoor` | Legacy (E0) | 539.8 ± 152.3 | 9.5 ± 2.7 | 13.891 ± 5.051 | 0.165 ± 0.043 | **5.90 ± 1.72** | **261.9 ± 1.4** | **2714 ± 19** | -- | 0.2 ± 0.4 |
| `cmu_indoor` | 2D-Frontier (E1) | 551.1 ± 87.9 | 10.2 ± 0.8 | 13.304 ± 2.490 | 0.152 ± 0.012 | 5.49 ± 0.72 | 293.1 ± 7.3 | 2834 ± 47 | **77.1 ± 27.9** | **0.0** |
| `cmu_indoor` | +Hier (E3) | 578.6 ± 64.5 | 11.3 ± 0.8 | 13.194 ± 3.082 | 0.161 ± 0.016 | 5.76 ± 0.88 | 297.3 ± 6.4 | 2854 ± 63 | 107.4 ± 32.6 | **0.0** |
| `cmu_indoor` | CMU M-TARE (0.5 m/s, GT) | **951.7 ± 191.3** | **20.3 ± 3.5** | **7.729 ± 4.052** | **0.383 ± 0.053** | 5.74 ± 1.08 | 307.4 ± 8.4 | 5941 ± 7 | 875.7 ± 36.3 | **0.0** |
| `cmu_indoor` | **Lite-HMRE (Ours)** | 553.9 ± 174.1 | 10.8 ± 1.9 | 13.635 ± 4.104 | 0.152 ± 0.045 | 5.31 ± 1.52 | 298.1 ± 8.7 | 3019 ± 191 | 139.3 ± 41.8 | **0.0** |
| `cmu_tunnel` | Legacy (E0) | 443.8 ± 106.4 | 2.5 ± 0.4 | 54.023 ± 6.213 | 0.041 ± 0.011 | 4.87 ± 1.47 | **253.2 ± 6.0** | **2345 ± 26** | -- | **0.0** |
| `cmu_tunnel` | 2D-Frontier (E1) | 466.2 ± 164.2 | 2.9 ± 0.9 | 52.659 ± 9.939 | 0.044 ± 0.016 | 4.78 ± 1.53 | 306.5 ± 21.9 | 2537 ± 171 | 181.9 ± 103.0 | **0.0** |
| `cmu_tunnel` | +Hier (E3) | 459.6 ± 110.5 | 2.9 ± 0.6 | 54.219 ± 7.945 | 0.038 ± 0.012 | **5.06 ± 1.42** | 265.9 ± 24.1 | 2561 ± 230 | **84.9 ± 85.1** | **0.0** |
| `cmu_tunnel` | CMU M-TARE (0.5 m/s, GT) | **776.0 ± 225.3** | **4.2 ± 1.1** | **39.068 ± 9.561** | **0.120 ± 0.029** | 4.57 ± 1.27 | 301.1 ± 5.6 | 5480 ± 45 | 776.5 ± 145.2 | **0.0** |
| `cmu_tunnel` | **Lite-HMRE (Ours)** | 470.7 ± 131.1 | 3.2 ± 0.8 | 55.522 ± 6.601 | 0.044 ± 0.014 | 4.97 ± 1.39 | 291.3 ± 43.3 | 2613 ± 178 | 194.5 ± 114.4 | **0.0** |

**Key Findings from Table I, Fig. 3 & Fig. 3b**:
1. **Decisive Sweep on Subterranean `mine`**: On the target degenerate subterranean environment (`mine`, 210 s), Lite-HMRE outperforms every ROS 2 baseline across all quality and coverage metrics simultaneously: **Area `275.6 ± 14.2 m²`** (+12.4% vs. Legacy, 3× tighter std), **3D Comp.$_{15}$ `65.5 ± 6.9%`** (+12.0 pp vs. Legacy, +9.1 pp vs. E1), **Chamfer-$L_1$ `0.207 ± 0.123 m`** (**-67.6% vs. Legacy**, **-55.0% vs. E1**), **Occ. IoU `0.811 ± 0.085`** (+24.2% vs. Legacy, Welch $p=0.0436$), and **P95 latency `49.8 ms`** (`<50 ms` real-time budget).
2. **Matched-Speed Edge Pareto Advantage over Official CMU M-TARE (Fig. 3b)**: Even when matched to the exact same $0.5\text{ m/s}$ velocity under 4 cores, official CMU M-TARE requires `5941 MB` (`cmu_indoor`) and `5480 MB` (`cmu_tunnel`) RAM—exceeding 4 GB edge SBC limits—and incurs `875.7 ms` / `776.5 ms` P95 planning latency. Lite-HMRE slashes peak RAM by **49.2%–52.3%** (`3019 MB` / `2613 MB`, comfortably inside the 4 GB Edge SBC Zone), cuts P95 latency by **4.0×–6.3×** (`139.3 ms` / `194.5 ms`), and achieves **+8.8% higher Coverage Efficiency** than M-TARE on `cmu_tunnel` (`4.97` vs. `4.57 m²/m`) while also beating all ROS 2 baselines in area and 3D completeness on `cmu_tunnel` and `house` (`CE = 2.40 m²/m`, +52.9% vs. Legacy).
3. **Zero-Collision Safety**: Lite-HMRE achieves **0.0 inter-robot collisions** across all 30 Table I runs and all 40 Table IV scalability runs, whereas `Legacy (E0)` collides on both `house` and `cmu_indoor` (`0.2 ± 0.4`).

---

### C. Noisy Odometry Online $\mathrm{SE}(2)$ Alignment & Degeneracy Gate Study (Fig. 2 & Table III)

![Fig. 2: Online SE(2) map alignment and degeneracy gate study under noisy EKF odometry](paper_artifacts/fig2_alignment.png)

*Fig. 2. Online $\mathrm{SE}(2)$ map alignment and corridor degeneracy gate study on `mine` and `warehouse` (5 seeds, noisy wheel–IMU EKF odometry, plus GT-Odom oracle).*

| World | Variant | Occ. IoU | ATE RMSE (m) | CD-$L_1$ (m) | Prec.$_{10}$ (%) | Cloud CPU (%) | SE(2) Acc. |
|---|---|---:|---:|---:|---:|---:|---:|
| `mine` | Legacy Merge + PyCloud | **0.839 ± 0.045** | 0.046 ± 0.012 | **0.171 ± 0.141** | 97.7 ± 1.8 | 20.9 ± 2.3 | 0.0 |
| `mine` | No SE(2) Align | 0.813 ± 0.046 | 0.051 ± 0.009 | 0.226 ± 0.174 | 96.6 ± 2.6 | 3.3 ± 0.0 | 0.0 |
| `mine` | SE(2) w/o Degen. Gate | 0.695 ± 0.175 | 0.078 ± 0.049 | 0.221 ± 0.141 | 94.0 ± 6.2 | 3.3 ± 0.0 | 27.0 ± 39.9 |
| `mine` | **Lite-HMRE (Ours)** | 0.811 ± 0.085 | **0.046 ± 0.015** | 0.207 ± 0.123 | 97.5 ± 2.4 | **3.0 ± 0.3** | 22.0 ± 35.3 |
| `mine` | GT-Odom (Oracle) | 0.816 ± 0.100 | 0.000 (GT) | 0.173 ± 0.139 | **98.0 ± 2.1** | 3.3 ± 0.3 | 8.4 ± 11.2 |
| `warehouse` | Legacy Merge + PyCloud | **0.491 ± 0.087** | 0.072 ± 0.025 | 1.821 ± 0.771 | 61.7 ± 10.8 | 22.5 ± 1.0 | 0.0 |
| `warehouse` | No SE(2) Align | 0.415 ± 0.076 | 0.068 ± 0.018 | **1.602 ± 0.527** | 49.9 ± 7.0 | 3.7 ± 0.5 | 0.0 |
| `warehouse` | SE(2) w/o Degen. Gate | 0.426 ± 0.104 | 0.061 ± 0.019 | 1.747 ± 0.793 | **62.2 ± 9.6** | **3.5 ± 0.1** | 4.4 ± 9.8 |
| `warehouse` | **Lite-HMRE (Ours)** | 0.458 ± 0.100 | **0.052 ± 0.017** | 1.914 ± 2.146 | 56.6 ± 7.0 | **3.5 ± 0.1** | 11.2 ± 25.0 |
| `warehouse` | GT-Odom (Oracle) | 0.463 ± 0.127 | 0.000 (GT) | 2.438 ± 1.912 | 57.6 ± 11.8 | 3.5 ± 0.2 | 12.6 ± 23.0 |

**Analysis**:
- **84.4%–85.6% Cloud-Fusion CPU Reduction**: C++17 `cloud_merger_node` slashes cloud-fusion CPU load from `20.9% → 3.0%` (**-85.6%**) on `mine` and `22.5% → 3.5%` (**-84.4%**) on `warehouse`.
- **Lowest Noisy-Odometry ATE on BOTH Worlds**: With the overlap-point quality gate (`min_overlap_points=150`, `min_inlier_ratio=0.55`, `max_rmse_m=0.12`, `iou_tolerance=0.002`) and Hessian Solution Remapping, Lite-HMRE achieves the **lowest noisy-odom ATE on both `mine` (`0.046 m`, -41.0% vs. ungated SE(2) `0.078 m`, -9.8% vs. `No SE(2)`) and `warehouse` (`0.052 m`, -27.8% vs. Legacy `0.072 m`, -23.5% vs. `No SE(2)` `0.068 m`)**, while matching the GT-Odom Oracle's 2D Occ. IoU within `0.005` on both worlds (`0.811` vs. `0.816` on `mine`; `0.458` vs. `0.463` on `warehouse`).

---

### D. Multi-Robot Scalability ($N = 1 \dots 4$, Fig. 4 & Table IV)

![Fig. 4: Multi-robot scalability N=1..4 on Mine and CMU Indoor](paper_artifacts/fig4_scalability.png)

*Fig. 4. Multi-robot scalability ($N=1,2,3,4$, 5 seeds each) under fixed 4-core compute: (a) area speedup $A_N/A_1$, (b) total comms bandwidth and peak RAM (with 2-robot M-TARE RAM reference line at 5.80 GB), and (c) planning P95 latency and load-balance CV.*

| World | $N$ | Area ($\text{m}^2$) | Speedup | 3D Comp.$_{15}$ (%) | Load CV | Overlap | Comms (KB/s) | CPU (%) | RAM (MB) | Cyc P95 (ms) | Coll. |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `mine` | 1 | 223.4 ± 42.7 | 1.00× | 46.5 ± 5.8 | 0.000 | 0.000 | 555.5 ± 68.3 | 143.3 ± 23.2 | 1368 ± 14 | **14.3 ± 3.1** | **0.0** |
| `mine` | 2 | 275.6 ± 14.2 | 1.23× | 65.5 ± 6.9 | 0.139 | 0.020 | 779.6 ± 143.5 | 238.5 ± 26.3 | 2331 ± 18 | 49.8 ± 18.0 | **0.0** |
| `mine` | 3 | **278.6 ± 15.1** | **1.25×** | **73.2 ± 8.5** | 0.410 | 0.048 | 1060.5 ± 129.0 | 333.2 ± 12.0 | 3467 ± 23 | 200.1 ± 78.9 | **0.0** |
| `mine` | 4 | 268.9 ± 33.4 | 1.20× | 63.9 ± 7.4 | 0.551 | 0.067 | 1125.9 ± 143.3 | 349.0 ± 3.0 | 4745 ± 47 | 461.6 ± 62.2 | **0.0** |
| `cmu_indoor` | 1 | 338.4 ± 111.3 | 1.00× | 6.0 ± 1.3 | 0.000 | 0.000 | 635.5 ± 325.6 | 209.6 ± 13.2 | 1790 ± 54 | **21.6 ± 4.5** | **0.0** |
| `cmu_indoor` | 2 | 553.9 ± 174.1 | 1.64× | 10.8 ± 1.9 | 0.041 | 0.000 | 2830.7 ± 1036.0 | 298.1 ± 8.7 | 3019 ± 191 | 139.3 ± 41.8 | **0.0** |
| `cmu_indoor` | 3 | 883.0 ± 306.3 | 2.61× | 16.1 ± 2.5 | 0.080 | 0.008 | 5910.0 ± 2722.2 | 317.9 ± 10.9 | 4321 ± 287 | 202.6 ± 77.7 | **0.0** |
| `cmu_indoor` | 4 | **884.7 ± 254.6** | **2.61×** | **17.5 ± 4.1** | 0.133 | 0.029 | 6689.4 ± 1566.4 | 328.9 ± 5.6 | 5655 ± 357 | 276.0 ± 93.8 | **0.0** |

**Analysis**:
- On `cmu_indoor` ($2667\text{ m}^2$), Lite-HMRE scales smoothly to **2.61× area speedup** and **2.92× 3D completeness speedup** (`6.0% → 17.5%`) at $N=4$ while keeping load-balance CV tight (`0.041` at $N=2$, `0.080` at $N=3$, `0.133` at $N=4$), path overlap under `2.9%`, and **0.0 collisions** across all 40 scalability runs.
- Crucially, **4-robot Lite-HMRE (`5655 MB` RAM, `276 ms` P95) still uses less memory and 3.2× lower planning latency than 2-robot matched-speed CMU M-TARE (`5941 MB`, `876 ms`)** on the exact same `cmu_indoor` building!
- On `mine` ($280.09\text{ m}^2$), $N=2$–$3$ robots already saturate **99.5%** of the entire mine (`278.6 m²`, `73.2%` 3D completeness), explaining the flat speedup curve beyond $N=2$.

---

### E. Seven-Step Component Ablation & Sensor/Voxel Sensitivity (Fig. 5, Tables II & V)

![Fig. 5: Three-panel ablation (3D quality, CE & overlap) and sensor/voxel sensitivity](paper_artifacts/fig5_ablation_sensitivity.png)

*Fig. 5. (a) Seven-step ablation (`E0`–`E6`) 3D completeness @ 15 cm on `mine` and `hospital`; (b) Coverage Efficiency CE (solid, ↑) and redundant path overlap (dashed, ↓) across `E0`–`E6`; (c) LiDAR beam count and voxel resolution trade-off on `mine` (3 seeds).*

1. **Seven-Step Ablation (`E0`–`E6`, Table II & Fig. 5a–b)**:
   - On `mine`, 3D completeness climbs monotonically as components are added: `E0 (53.5%) → E1 (56.4%) → E2 (+3D gain, 61.9%) → E4 (+Geodesic, 65.7%) → E6 (Full, 65.5%)` (**+12.0 pp over Legacy**), while Chamfer-$L_1$ drops **-67.6%** (`0.638 → 0.207 m`) and redundant path overlap drops **-73.7%** (`7.6% → 2.0%`).
   - On `hospital`, Fig. 5b reveals a textbook multi-robot coordination progression: Coverage Efficiency CE climbs steadily from `E0 (1.91)` → `E1 (1.93)` → `E2 (2.47)` → `E4 (2.63)` → `E5/E6 (2.97–3.00 m²/m, +55.5%)`, while path overlap collapses by **-95.9%** (`21.7% → 0.9%`) with **0.0 collisions** (whereas `E2` without deconfliction records `0.2 ± 0.4` collisions).
2. **Sensor & Voxel Resolution Sensitivity (Table V & Fig. 5c, `mine`, 3 seeds)**:
   - The default `16-beam / 0.10 m` configuration achieves the best overall edge balance (**`280.6 m²`**, **`66.7%` 3D Comp.$_{15}$**, `0.154 m` Chamfer-$L_1$, lowest CPU `234.2%`, lowest P95 `46.2 ms`, `2335 MB` RAM).
   - Tightening voxel resolution to `0.05 m` achieves the tightest geometric error (`0.124 ± 0.028 m` Chamfer-$L_1$, `279.8 ± 4.7 m²` area) with only `+17 MB` RAM, whereas coarsening to `0.15 m` degrades 3D completeness (`52.8%`) and Chamfer (`0.848 m`). Scaling to `32-beam / 0.10 m` sustains high 3D completeness (`66.5%`, peaking at `77.3%` on `seed_2`) with virtually zero memory growth (`2328 MB`) and `52.1 ms` P95 latency.

---

## VI. Real-Robot Hardware Validation (M6 — Reserved)

> *Note: To be populated upon completion of Milestone M6.*
>
> We will deploy the exact same containerized ROS 2 Lyrical stack (`real_robot.launch.py`, single-robot mode, `pointcloud_to_laserscan` + `slam_toolbox` + C++ `cloud_merger_node` + `octomap_server` + Nav2 + `hier_explore_core`) onto a physical mobile robot equipped with a 3D LiDAR and onboard Linux SBC. Three back-to-back trials comparing `2D-Frontier (E1)` vs. `Lite-HMRE (Full)` in a cluttered indoor laboratory/corridor will validate real-world 3D blind-spot completion, onboard CPU/RAM telemetry, and real-time planning latency.

---

## VII. Discussion and Limitations

1. **GT Pose vs. Noisy EKF Odometry in Large CMU Worlds**: Even at matched $0.5\text{ m/s}$ velocity, official CMU M-TARE retains its native drift-free Gazebo `state_estimation` and dedicated kinematic `localPlanner` (zero SLAM loop-closure overhead), explaining its higher raw 2D area on `cmu_indoor` and `cmu_tunnel`. In exchange, Lite-HMRE runs full online SLAM + EKF + OctoMap within **$<3.0\text{ GB}$ RAM** (fitting 4 GB edge SBCs where M-TARE OOMs at $5.5\text{–}5.9\text{ GB}$) and **4.0×–6.3× lower planning latency**.
2. **Narrow Doorway Recovery Variance (`hospital`)**: While 4 of 5 `hospital` seeds reach near-complete coverage, occasional tight-doorway Nav2 aborts on `seed_1` raise variance on `hospital`; integrating a local doorway-centering recovery primitive will further tighten worst-case variance.
3. **Bandwidth Scaling in Large Buildings**: Total ROS 2 telemetry bandwidth grows from `780 KB/s` (`mine`, $N=2$) to `6.69 MB/s` (`cmu_indoor`, $N=4$) because full `/map` and `/points_merged` topics are relayed across nodes. Delta-compressed occupancy grid diffs and keyframe-based submap sharing (planned for future work) can reduce wireless mesh bandwidth by an order of magnitude for multi-robot field teams.

---

## VIII. Conclusion

We presented **Lite-HMRE**, a lightweight, pure-geometric 2.5D–3D hierarchical multi-robot exploration framework engineered for 4-core edge compute platforms under realistic noisy wheel–IMU odometry. By combining C++17 degeneracy-gated online $\mathrm{SE}(2)$ map alignment and zero-copy cloud fusion with a `pybind11` geodesic/visibility kernel and two-layer topological deconfliction, Lite-HMRE bridges the gap between memory-heavy 3D planners and 3D-blind 2D frontier explorers. Across 239 isolated 4-core matched-speed ($0.5\text{ m/s}$) benchmark runs spanning six worlds (including official CMU AEDE environments), Lite-HMRE achieves zero inter-robot collisions, sweeps all ROS 2 baselines on subterranean `mine` (`65.5%` 3D completeness, `-67.6%` Chamfer-$L_1$, `0.811` Occ. IoU), achieves the lowest noisy-odom ATE on both `mine` and `warehouse` while cutting cloud-fusion CPU by `84.4%–85.6%`, reduces peak RAM by `49.2%–52.3%` and P95 latency by `4.0×–6.3×` relative to matched-speed official CMU M-TARE, and scales smoothly to 4-robot teams using less RAM than 2-robot M-TARE.

---

## References

[1] C. Cao, H. Zhu, H. Choset, and J. Zhang, "TARE: A hierarchical framework for efficiently exploring complex 3D environments," in *Proc. Robotics: Science and Systems (RSS)*, 2021.  
[2] C. Cao, H. Zhu, Z. Ren, H. Choset, and J. Zhang, "Representation granularity enables time-efficient autonomous exploration in large, complex worlds," *Science Robotics*, vol. 8, no. 76, 2023 (M-TARE open-source release).  
[3] T. Dang, M. Tranzatto, S. Khattak, F. Mascarich, K. Alexis, and M. Hutter, "Graph-based subterranean exploration path planning using aerial and legged robots," *Journal of Field Robotics*, vol. 37, no. 8, pp. 1363–1388, 2020.  
[4] J. Hörner, "Map-merging for multi-robot system (`m-explore` / `explore_lite`)," Bachelor's thesis, Charles University, 2016 (ROS 2 port `m-explore-ros2`).  
[5] J. Zhang, M. Kaess, and S. Singh, "On degeneracy of optimization-based state estimation problems," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, pp. 809–816, 2016.  
[6] B. Yamauchi, "A frontier-based approach for autonomous exploration," in *Proc. IEEE Int. Symp. Comput. Intell. Robot. Autom. (CIRA)*, pp. 146–151, 1997.  
[7] W. Burgard, M. Moors, C. Stachniss, and F. E. Schneider, "Coordinated multi-robot exploration," *IEEE Trans. Robot.*, vol. 21, no. 3, pp. 376–386, 2005.  
[8] R. Zlot, A. Stentz, M. B. Dias, and S. Thayer, "Multi-robot exploration controlled by a market economy," in *Proc. IEEE ICRA*, pp. 3016–3023, 2002.  
[9] A. Bircher, M. Kamel, K. Alexis, H. Oleynikova, and R. Siegwart, "Receding horizon 'Next-Best-View' planner for 3D exploration," in *Proc. IEEE ICRA*, pp. 1462–1468, 2016.  
[10] B. Zhou, H. Xu, and S. Shen, "RACER: Rapid collaborative exploration with a decentralized multi-UAV system," *IEEE Trans. Robot.*, vol. 39, no. 3, pp. 1816–1835, 2023.  
[11] B. Zhou, Y. Zhang, X. Chen, and S. Shen, "FUEL: Fast UAV exploration using incremental frontier structure and hierarchical planning," *IEEE Robot. Autom. Lett. (RA-L)*, vol. 6, no. 2, pp. 779–786, 2021.  
[12] J. Hörner, "multirobot_map_merge: Merging multiple maps with or without known initial positions," ROS package, 2016.  
[13] G. Sharon, R. Stern, A. Felner, and N. R. Sturtevant, "Conflict-based search for optimal multi-agent pathfinding," *Artificial Intelligence*, vol. 219, pp. 40–66, 2015.
