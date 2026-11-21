# M5 Benchmark Quantitative Summary (4-Core Edge Profile)

## Suite 1: Main Multi-World Benchmark (Table I / Fig. 3)

| World | Method | N | Area (m²) | 3D Comp@15cm (%) | Chamfer-L1 (m) | Occ IoU | CE (m²/m) | CPU (%) | RAM (MB) | Cyc P95 (ms) | Coll |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| house | Legacy (E0) | 5 | 86.0 ± 0.5 | 75.6 ± 2.1 | 0.059 ± 0.004 | 0.809 ± 0.015 | 1.57 ± 0.27 | 279.0 ± 2.9 | 2365 ± 4 | -- | 0.2 ± 0.4 |
| house | 2D-Frontier (E1) | 5 | 86.0 ± 0.5 | 77.7 ± 6.0 | 0.059 ± 0.003 | 0.753 ± 0.048 | 1.68 ± 0.47 | 286.5 ± 6.1 | 2329 ± 2 | 15.0 ± 2.3 | 0.0 ± 0.0 |
| house | +Hier (E3) | 5 | 86.0 ± 1.4 | 76.7 ± 2.8 | 0.057 ± 0.004 | 0.752 ± 0.072 | 2.09 ± 0.58 | 250.7 ± 30.3 | 2332 ± 10 | 12.5 ± 3.0 | 0.0 ± 0.0 |
| house | Lite-HMRE (Ours) | 5 | 86.5 ± 0.3 | 76.0 ± 5.4 | 0.061 ± 0.006 | 0.730 ± 0.034 | 2.40 ± 0.70 | 269.5 ± 17.2 | 2331 ± 6 | 24.6 ± 7.9 | 0.0 ± 0.0 |
| hospital | Legacy (E0) | 5 | 169.9 ± 0.5 | 77.1 ± 2.5 | 0.058 ± 0.004 | 0.763 ± 0.049 | 1.91 ± 0.28 | 280.6 ± 2.4 | 2373 ± 11 | -- | 0.0 ± 0.0 |
| hospital | 2D-Frontier (E1) | 5 | 170.9 ± 0.8 | 79.9 ± 2.5 | 0.062 ± 0.007 | 0.740 ± 0.050 | 1.93 ± 0.14 | 292.4 ± 4.3 | 2332 ± 7 | 13.1 ± 1.8 | 0.0 ± 0.0 |
| hospital | +Hier (E3) | 5 | 170.2 ± 1.8 | 77.9 ± 4.2 | 0.071 ± 0.024 | 0.662 ± 0.222 | 2.28 ± 0.47 | 273.0 ± 23.3 | 2339 ± 13 | 21.8 ± 15.0 | 0.0 ± 0.0 |
| hospital | Lite-HMRE (Ours) | 5 | 150.4 ± 35.5 | 66.5 ± 16.0 | 0.171 ± 0.196 | 0.637 ± 0.171 | 2.97 ± 0.40 | 277.1 ± 17.9 | 2334 ± 7 | 48.5 ± 16.9 | 0.0 ± 0.0 |
| mine | Legacy (E0) | 5 | 245.1 ± 42.9 | 53.5 ± 12.0 | 0.638 ± 0.575 | 0.653 ± 0.117 | 2.18 ± 0.49 | 231.5 ± 22.2 | 2291 ± 10 | -- | 0.0 ± 0.0 |
| mine | 2D-Frontier (E1) | 5 | 253.4 ± 41.4 | 56.4 ± 11.6 | 0.460 ± 0.463 | 0.761 ± 0.123 | 2.97 ± 0.80 | 269.4 ± 10.7 | 2279 ± 17 | 41.2 ± 31.6 | 0.0 ± 0.0 |
| mine | +Hier (E3) | 5 | 253.3 ± 40.6 | 60.2 ± 9.1 | 0.438 ± 0.425 | 0.766 ± 0.054 | 2.52 ± 0.45 | 267.6 ± 14.2 | 2283 ± 12 | 50.8 ± 25.2 | 0.0 ± 0.0 |
| mine | Lite-HMRE (Ours) | 5 | 275.6 ± 14.2 | 65.5 ± 6.9 | 0.207 ± 0.123 | 0.811 ± 0.085 | 2.40 ± 0.17 | 238.5 ± 26.3 | 2331 ± 18 | 49.8 ± 18.0 | 0.0 ± 0.0 |
| warehouse | Legacy (E0) | 5 | 684.2 ± 152.4 | 34.3 ± 7.3 | 1.508 ± 1.258 | 0.473 ± 0.080 | 6.17 ± 1.28 | 299.9 ± 1.8 | 2616 ± 11 | -- | 0.0 ± 0.0 |
| warehouse | 2D-Frontier (E1) | 5 | 738.4 ± 112.8 | 35.6 ± 8.2 | 1.617 ± 0.760 | 0.445 ± 0.094 | 8.94 ± 0.67 | 306.9 ± 18.9 | 2605 ± 20 | 149.4 ± 43.7 | 0.0 ± 0.0 |
| warehouse | +Hier (E3) | 5 | 598.4 ± 151.4 | 25.6 ± 7.2 | 2.928 ± 1.814 | 0.345 ± 0.071 | 7.67 ± 2.69 | 309.9 ± 15.6 | 2576 ± 62 | 157.3 ± 88.3 | 0.0 ± 0.0 |
| warehouse | Lite-HMRE (Ours) | 5 | 698.2 ± 220.9 | 31.3 ± 11.4 | 1.914 ± 2.146 | 0.458 ± 0.100 | 8.27 ± 2.07 | 317.8 ± 5.7 | 2662 ± 30 | 176.6 ± 50.2 | 0.0 ± 0.0 |
| cmu_indoor | Legacy (E0) | 5 | 539.8 ± 152.3 | 9.5 ± 2.7 | 13.891 ± 5.051 | 0.165 ± 0.043 | 5.90 ± 1.72 | 261.9 ± 1.4 | 2714 ± 19 | -- | 0.2 ± 0.4 |
| cmu_indoor | 2D-Frontier (E1) | 5 | 551.1 ± 87.9 | 10.2 ± 0.8 | 13.304 ± 2.490 | 0.152 ± 0.012 | 5.49 ± 0.72 | 293.1 ± 7.3 | 2834 ± 47 | 77.1 ± 27.9 | 0.0 ± 0.0 |
| cmu_indoor | +Hier (E3) | 5 | 578.6 ± 64.5 | 11.3 ± 0.8 | 13.194 ± 3.082 | 0.161 ± 0.016 | 5.76 ± 0.88 | 297.3 ± 6.4 | 2854 ± 63 | 107.4 ± 32.6 | 0.0 ± 0.0 |
| cmu_indoor | CMU M-TARE | 5 | 951.7 ± 191.3 | 20.3 ± 3.5 | 7.729 ± 4.052 | 0.383 ± 0.053 | 5.74 ± 1.08 | 307.4 ± 8.4 | 5941 ± 7 | 875.7 ± 36.3 | 0.0 ± 0.0 |
| cmu_indoor | Lite-HMRE (Ours) | 5 | 553.9 ± 174.1 | 10.8 ± 1.9 | 13.635 ± 4.104 | 0.152 ± 0.045 | 5.31 ± 1.52 | 298.1 ± 8.7 | 3019 ± 191 | 139.3 ± 41.8 | 0.0 ± 0.0 |
| cmu_tunnel | Legacy (E0) | 5 | 443.8 ± 106.4 | 2.5 ± 0.4 | 54.023 ± 6.213 | 0.041 ± 0.011 | 4.87 ± 1.47 | 253.2 ± 6.0 | 2345 ± 26 | -- | 0.0 ± 0.0 |
| cmu_tunnel | 2D-Frontier (E1) | 5 | 466.2 ± 164.2 | 2.9 ± 0.9 | 52.659 ± 9.939 | 0.044 ± 0.016 | 4.78 ± 1.53 | 306.5 ± 21.9 | 2537 ± 171 | 181.9 ± 103.0 | 0.0 ± 0.0 |
| cmu_tunnel | +Hier (E3) | 5 | 459.6 ± 110.5 | 2.9 ± 0.6 | 54.219 ± 7.945 | 0.038 ± 0.012 | 5.06 ± 1.42 | 265.9 ± 24.1 | 2561 ± 230 | 84.9 ± 85.1 | 0.0 ± 0.0 |
| cmu_tunnel | CMU M-TARE | 5 | 776.0 ± 225.3 | 4.2 ± 1.1 | 39.068 ± 9.561 | 0.120 ± 0.029 | 4.57 ± 1.27 | 301.1 ± 5.6 | 5480 ± 45 | 776.5 ± 145.2 | 0.0 ± 0.0 |
| cmu_tunnel | Lite-HMRE (Ours) | 5 | 470.7 ± 131.1 | 3.2 ± 0.8 | 55.522 ± 6.601 | 0.044 ± 0.014 | 4.97 ± 1.39 | 291.3 ± 43.3 | 2613 ± 178 | 194.5 ± 114.4 | 0.0 ± 0.0 |

## Suite 2: 7-Step Component Ablation (Table II)

| World | Config | N | Area (m²) | 3D Comp@15cm (%) | Chamfer-L1 (m) | CE (m²/m) | Overlap | Cyc P95 (ms) | Coll |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| mine | Legacy (E0) | 5 | 245.1 ± 42.9 | 53.5 ± 12.0 | 0.638 ± 0.575 | 2.18 ± 0.49 | 0.076 ± 0.040 | -- | 0.0 ± 0.0 |
| mine | 2D-Frontier (E1) | 5 | 253.4 ± 41.4 | 56.4 ± 11.6 | 0.460 ± 0.463 | 2.97 ± 0.80 | 0.013 ± 0.028 | 41.2 ± 31.6 | 0.0 ± 0.0 |
| mine | +3D-Gain (E2) | 5 | 277.2 ± 8.4 | 61.9 ± 2.4 | 0.195 ± 0.093 | 2.34 ± 0.37 | 0.045 ± 0.066 | 40.8 ± 18.0 | 0.0 ± 0.0 |
| mine | +Hier (E3) | 5 | 253.3 ± 40.6 | 60.2 ± 9.1 | 0.438 ± 0.425 | 2.52 ± 0.45 | 0.039 ± 0.041 | 50.8 ± 25.2 | 0.0 ± 0.0 |
| mine | +Geodesic (E4) | 5 | 271.1 ± 34.7 | 65.7 ± 9.9 | 0.254 ± 0.377 | 2.38 ± 0.33 | 0.042 ± 0.064 | 61.0 ± 10.8 | 0.0 ± 0.0 |
| mine | +Deconflict (E5) | 5 | 259.3 ± 48.1 | 60.5 ± 12.5 | 0.393 ± 0.534 | 2.67 ± 0.31 | 0.031 ± 0.062 | 77.0 ± 12.2 | 0.0 ± 0.0 |
| mine | Lite-HMRE (Ours) | 5 | 275.6 ± 14.2 | 65.5 ± 6.9 | 0.207 ± 0.123 | 2.40 ± 0.17 | 0.020 ± 0.033 | 49.8 ± 18.0 | 0.0 ± 0.0 |
| hospital | Legacy (E0) | 5 | 169.9 ± 0.5 | 77.1 ± 2.5 | 0.058 ± 0.004 | 1.91 ± 0.28 | 0.217 ± 0.073 | -- | 0.0 ± 0.0 |
| hospital | 2D-Frontier (E1) | 5 | 170.9 ± 0.8 | 79.9 ± 2.5 | 0.062 ± 0.007 | 1.93 ± 0.14 | 0.029 ± 0.018 | 13.1 ± 1.8 | 0.0 ± 0.0 |
| hospital | +3D-Gain (E2) | 5 | 170.7 ± 1.3 | 79.6 ± 3.4 | 0.063 ± 0.002 | 2.47 ± 0.21 | 0.015 ± 0.016 | 16.4 ± 9.1 | 0.2 ± 0.4 |
| hospital | +Hier (E3) | 5 | 170.2 ± 1.8 | 77.9 ± 4.2 | 0.071 ± 0.024 | 2.28 ± 0.47 | 0.031 ± 0.027 | 21.8 ± 15.0 | 0.0 ± 0.0 |
| hospital | +Geodesic (E4) | 5 | 168.4 ± 5.5 | 77.7 ± 5.9 | 0.067 ± 0.020 | 2.63 ± 0.24 | 0.003 ± 0.005 | 30.7 ± 7.2 | 0.0 ± 0.0 |
| hospital | +Deconflict (E5) | 5 | 157.3 ± 20.4 | 68.3 ± 12.9 | 0.113 ± 0.078 | 3.00 ± 0.70 | 0.019 ± 0.043 | 45.5 ± 8.0 | 0.0 ± 0.0 |
| hospital | Lite-HMRE (Ours) | 5 | 150.4 ± 35.5 | 66.5 ± 16.0 | 0.171 ± 0.196 | 2.97 ± 0.40 | 0.009 ± 0.016 | 48.5 ± 16.9 | 0.0 ± 0.0 |

## Suite 3: Noisy Odometry SE(2) Alignment & Degeneracy Study (Fig. 2)

| World | Variant | N | Occ IoU | ATE RMSE (m) | Chamfer-L1 (m) | Prec@10cm (%) | Cloud CPU (%) | SE(2) Accepts | Degen Gates |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| mine | Legacy Merge + PyCloud | 5 | 0.839 ± 0.045 | 0.046 ± 0.012 | 0.171 ± 0.141 | 97.7 ± 1.8 | 20.9 ± 2.3 | 0.0 ± 0.0 | 0.0 ± 0.0 |
| mine | No SE(2) Align | 5 | 0.813 ± 0.046 | 0.051 ± 0.009 | 0.226 ± 0.174 | 96.6 ± 2.6 | 3.3 ± 0.0 | 0.0 ± 0.0 | 0.0 ± 0.0 |
| mine | SE(2) w/o Degen. Gate | 5 | 0.695 ± 0.175 | 0.078 ± 0.049 | 0.221 ± 0.141 | 94.0 ± 6.2 | 3.3 ± 0.0 | 27.0 ± 39.9 | 0.0 ± 0.0 |
| mine | Lite-HMRE (Ours) | 5 | 0.811 ± 0.085 | 0.046 ± 0.015 | 0.207 ± 0.123 | 97.5 ± 2.4 | 3.0 ± 0.3 | 22.0 ± 35.3 | 0.0 ± 0.0 |
| mine | GT-Odom (Oracle) | 5 | 0.816 ± 0.100 | -- | 0.173 ± 0.139 | 98.0 ± 2.1 | 3.3 ± 0.3 | 8.4 ± 11.2 | 0.0 ± 0.0 |
| warehouse | Legacy Merge + PyCloud | 5 | 0.491 ± 0.087 | 0.072 ± 0.025 | 1.821 ± 0.771 | 61.7 ± 10.8 | 22.5 ± 1.0 | 0.0 ± 0.0 | 0.0 ± 0.0 |
| warehouse | No SE(2) Align | 5 | 0.415 ± 0.076 | 0.068 ± 0.018 | 1.602 ± 0.527 | 49.9 ± 7.0 | 3.7 ± 0.5 | 0.0 ± 0.0 | 0.0 ± 0.0 |
| warehouse | SE(2) w/o Degen. Gate | 5 | 0.426 ± 0.104 | 0.061 ± 0.019 | 1.747 ± 0.793 | 62.2 ± 9.6 | 3.5 ± 0.1 | 4.4 ± 9.8 | 0.4 ± 0.5 |
| warehouse | Lite-HMRE (Ours) | 5 | 0.458 ± 0.100 | 0.052 ± 0.017 | 1.914 ± 2.146 | 56.6 ± 7.0 | 3.5 ± 0.1 | 11.2 ± 25.0 | 0.2 ± 0.4 |
| warehouse | GT-Odom (Oracle) | 5 | 0.463 ± 0.127 | -- | 2.438 ± 1.912 | 57.6 ± 11.8 | 3.5 ± 0.2 | 12.6 ± 23.0 | 0.0 ± 0.0 |

## Suite 4: Multi-Robot Scalability N = 1..4 (Fig. 4)

| World | N Robots | Runs | Area (m²) | 3D Comp@15cm (%) | Load CV | Overlap | Comms (KB/s) | CPU (%) | RAM (MB) | Cyc P95 (ms) | Coll |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| mine | 1 | 5 | 223.4 ± 42.7 | 46.5 ± 5.8 | 0.000 ± 0.000 | 0.000 ± 0.000 | 555.5 ± 68.3 | 143.3 ± 23.2 | 1368 ± 14 | 14.3 ± 3.1 | 0.0 ± 0.0 |
| mine | 2 | 5 | 275.6 ± 14.2 | 65.5 ± 6.9 | 0.139 ± 0.104 | 0.020 ± 0.033 | 779.6 ± 143.5 | 238.5 ± 26.3 | 2331 ± 18 | 49.8 ± 18.0 | 0.0 ± 0.0 |
| mine | 3 | 5 | 278.6 ± 15.1 | 73.2 ± 8.5 | 0.410 ± 0.327 | 0.048 ± 0.050 | 1060.5 ± 129.0 | 333.2 ± 12.0 | 3467 ± 23 | 200.1 ± 78.9 | 0.0 ± 0.0 |
| mine | 4 | 5 | 268.9 ± 33.4 | 63.9 ± 7.4 | 0.551 ± 0.474 | 0.067 ± 0.062 | 1125.9 ± 143.3 | 349.0 ± 3.0 | 4745 ± 47 | 461.6 ± 62.2 | 0.0 ± 0.0 |
| cmu_indoor | 1 | 5 | 338.4 ± 111.3 | 6.0 ± 1.3 | 0.000 ± 0.000 | 0.000 ± 0.000 | 635.5 ± 325.6 | 209.6 ± 13.2 | 1790 ± 54 | 21.6 ± 4.5 | 0.0 ± 0.0 |
| cmu_indoor | 2 | 5 | 553.9 ± 174.1 | 10.8 ± 1.9 | 0.041 ± 0.040 | 0.000 ± 0.000 | 2830.7 ± 1036.0 | 298.1 ± 8.7 | 3019 ± 191 | 139.3 ± 41.8 | 0.0 ± 0.0 |
| cmu_indoor | 3 | 5 | 883.0 ± 306.3 | 16.1 ± 2.5 | 0.080 ± 0.039 | 0.008 ± 0.017 | 5910.0 ± 2722.2 | 317.9 ± 10.9 | 4321 ± 287 | 202.6 ± 77.7 | 0.0 ± 0.0 |
| cmu_indoor | 4 | 5 | 884.7 ± 254.6 | 17.5 ± 4.1 | 0.133 ± 0.068 | 0.029 ± 0.045 | 6689.4 ± 1566.4 | 328.9 ± 5.6 | 5655 ± 357 | 276.0 ± 93.8 | 0.0 ± 0.0 |

## Suite 5: Sensor & Voxel Resolution Sensitivity on Mine (Table V)

| Config | Runs | Area (m²) | 3D Comp@15cm (%) | Chamfer-L1 (m) | CPU (%) | RAM (MB) | Cyc P95 (ms) |
|---|---:|---:|---:|---:|---:|---:|---:|
| 16-beam / 0.05 m | 3 | 279.8 ± 4.7 | 62.5 ± 5.8 | 0.124 ± 0.028 | 244.6 ± 24.9 | 2352 ± 21 | 69.6 ± 28.5 |
| 16-beam / 0.10 m (Default) | 3 | 280.6 ± 16.9 | 66.7 ± 8.8 | 0.154 ± 0.139 | 234.2 ± 23.8 | 2335 ± 23 | 46.2 ± 23.6 |
| 16-beam / 0.15 m | 3 | 226.2 ± 56.0 | 52.8 ± 13.5 | 0.848 ± 0.709 | 244.6 ± 29.0 | 2325 ± 19 | 67.1 ± 24.2 |
| 32-beam / 0.10 m | 3 | 244.3 ± 60.7 | 66.5 ± 14.7 | 0.611 ± 0.889 | 247.3 ± 30.7 | 2328 ± 31 | 52.1 ± 25.1 |

## Statistical Significance (Lite-HMRE vs. Baselines)

| World | Comparison | Metric | Delta Mean | Welch p | Mann-Whitney U p |
|---|---|---|---:|---:|---:|
| house | full vs legacy | final_area_m2 | +0.4846 | 0.08333 | 0.1508 |
| house | full vs legacy | completeness_15cm | +0.0045 | 0.87 | 0.6905 |
| house | full vs legacy | chamfer_l1_m | +0.0019 | 0.5701 | 1 |
| house | full vs legacy | occupied_iou_tol1 | -0.0791 | 0.003588 | 0.007937 |
| house | full vs e1_2d | final_area_m2 | +0.4494 | 0.1002 | 0.07491 |
| house | full vs e1_2d | completeness_15cm | -0.0174 | 0.644 | 0.5476 |
| house | full vs e1_2d | chamfer_l1_m | +0.0019 | 0.5489 | 1 |
| house | full vs e1_2d | occupied_iou_tol1 | -0.0232 | 0.406 | 0.3095 |
| house | full vs e3_hier | final_area_m2 | +0.4934 | 0.4719 | 0.8413 |
| house | full vs e3_hier | completeness_15cm | -0.0071 | 0.8036 | 0.8413 |
| house | full vs e3_hier | chamfer_l1_m | +0.0040 | 0.2773 | 0.2222 |
| house | full vs e3_hier | occupied_iou_tol1 | -0.0220 | 0.5623 | 0.4206 |
| hospital | full vs legacy | final_area_m2 | -19.5714 | 0.2848 | 1 |
| hospital | full vs legacy | completeness_15cm | -0.1060 | 0.2143 | 0.5476 |
| hospital | full vs legacy | chamfer_l1_m | +0.1125 | 0.2678 | 0.09524 |
| hospital | full vs legacy | occupied_iou_tol1 | -0.1260 | 0.1786 | 0.3095 |
| hospital | full vs e1_2d | final_area_m2 | -20.5664 | 0.2645 | 0.2222 |
| hospital | full vs e1_2d | completeness_15cm | -0.1335 | 0.136 | 0.03175 |
| hospital | full vs e1_2d | chamfer_l1_m | +0.1091 | 0.2807 | 0.3095 |
| hospital | full vs e1_2d | occupied_iou_tol1 | -0.1036 | 0.254 | 0.4206 |
| hospital | full vs e3_hier | final_area_m2 | -19.8530 | 0.2791 | 0.3095 |
| hospital | full vs e3_hier | completeness_15cm | -0.1134 | 0.1921 | 0.4206 |
| hospital | full vs e3_hier | chamfer_l1_m | +0.1002 | 0.3174 | 0.4206 |
| hospital | full vs e3_hier | occupied_iou_tol1 | -0.0248 | 0.8485 | 0.8413 |
| mine | full vs legacy | final_area_m2 | +30.5464 | 0.1925 | 0.2222 |
| mine | full vs legacy | completeness_15cm | +0.1196 | 0.09855 | 0.09524 |
| mine | full vs legacy | chamfer_l1_m | -0.4307 | 0.1708 | 0.2222 |
| mine | full vs legacy | occupied_iou_tol1 | +0.1579 | 0.04356 | 0.09524 |
| mine | full vs e1_2d | final_area_m2 | +22.2038 | 0.3082 | 0.6905 |
| mine | full vs e1_2d | completeness_15cm | +0.0909 | 0.1788 | 0.2222 |
| mine | full vs e1_2d | chamfer_l1_m | -0.2529 | 0.2957 | 0.5476 |
| mine | full vs e1_2d | occupied_iou_tol1 | +0.0501 | 0.4766 | 0.6905 |
| mine | full vs e3_hier | final_area_m2 | +22.3458 | 0.298 | 0.5476 |
| mine | full vs e3_hier | completeness_15cm | +0.0526 | 0.3367 | 0.3095 |
| mine | full vs e3_hier | chamfer_l1_m | -0.2301 | 0.3003 | 0.3095 |
| mine | full vs e3_hier | occupied_iou_tol1 | +0.0448 | 0.3527 | 0.4206 |
| warehouse | full vs legacy | final_area_m2 | +13.9644 | 0.9106 | 0.6905 |
| warehouse | full vs legacy | completeness_15cm | -0.0295 | 0.6423 | 0.5476 |
| warehouse | full vs legacy | chamfer_l1_m | +0.4061 | 0.7268 | 0.8413 |
| warehouse | full vs legacy | occupied_iou_tol1 | -0.0147 | 0.8048 | 1 |
| warehouse | full vs e1_2d | final_area_m2 | -40.1902 | 0.7296 | 1 |
| warehouse | full vs e1_2d | completeness_15cm | -0.0434 | 0.5118 | 0.4206 |
| warehouse | full vs e1_2d | chamfer_l1_m | +0.2971 | 0.7822 | 0.6905 |
| warehouse | full vs e1_2d | occupied_iou_tol1 | +0.0128 | 0.8396 | 0.8413 |
| warehouse | full vs e3_hier | final_area_m2 | +99.7500 | 0.4321 | 0.4206 |
| warehouse | full vs e3_hier | completeness_15cm | +0.0571 | 0.3776 | 0.5476 |
| warehouse | full vs e3_hier | chamfer_l1_m | -1.0137 | 0.4438 | 0.1508 |
| warehouse | full vs e3_hier | occupied_iou_tol1 | +0.1133 | 0.07665 | 0.05556 |
| cmu_indoor | full vs legacy | final_area_m2 | +14.0476 | 0.8954 | 1 |
| cmu_indoor | full vs legacy | completeness_15cm | +0.0130 | 0.4086 | 0.5476 |
| cmu_indoor | full vs legacy | chamfer_l1_m | -0.2561 | 0.9321 | 1 |
| cmu_indoor | full vs legacy | occupied_iou_tol1 | -0.0128 | 0.6567 | 0.8413 |
| cmu_indoor | full vs e1_2d | final_area_m2 | +2.8050 | 0.9754 | 0.6905 |
| cmu_indoor | full vs e1_2d | completeness_15cm | +0.0061 | 0.537 | 0.5476 |
| cmu_indoor | full vs e1_2d | chamfer_l1_m | +0.3314 | 0.882 | 0.8413 |
| cmu_indoor | full vs e1_2d | occupied_iou_tol1 | +0.0005 | 0.9817 | 1 |
| cmu_indoor | full vs e3_hier | final_area_m2 | -24.7146 | 0.7777 | 0.4206 |
| cmu_indoor | full vs e3_hier | completeness_15cm | -0.0050 | 0.6154 | 0.3095 |
| cmu_indoor | full vs e3_hier | chamfer_l1_m | +0.4415 | 0.8526 | 0.8413 |
| cmu_indoor | full vs e3_hier | occupied_iou_tol1 | -0.0093 | 0.684 | 0.6905 |
| cmu_indoor | full vs mtare | final_area_m2 | -397.8258 | 0.008944 | 0.01587 |
| cmu_indoor | full vs mtare | completeness_15cm | -0.0944 | 0.001736 | 0.007937 |
| cmu_indoor | full vs mtare | chamfer_l1_m | +5.9067 | 0.05127 | 0.09524 |
| cmu_indoor | full vs mtare | occupied_iou_tol1 | -0.2308 | 8.373e-05 | 0.007937 |
| cmu_tunnel | full vs legacy | final_area_m2 | +26.8622 | 0.7316 | 0.8413 |
| cmu_tunnel | full vs legacy | completeness_15cm | +0.0077 | 0.1108 | 0.1508 |
| cmu_tunnel | full vs legacy | chamfer_l1_m | +1.4994 | 0.7211 | 0.8413 |
| cmu_tunnel | full vs legacy | occupied_iou_tol1 | +0.0039 | 0.633 | 0.6905 |
| cmu_tunnel | full vs e1_2d | final_area_m2 | +4.5190 | 0.9629 | 1 |
| cmu_tunnel | full vs e1_2d | completeness_15cm | +0.0032 | 0.5878 | 0.8413 |
| cmu_tunnel | full vs e1_2d | chamfer_l1_m | +2.8633 | 0.6083 | 0.5476 |
| cmu_tunnel | full vs e1_2d | occupied_iou_tol1 | +0.0004 | 0.9634 | 1 |
| cmu_tunnel | full vs e3_hier | final_area_m2 | +11.0936 | 0.8887 | 1 |
| cmu_tunnel | full vs e3_hier | completeness_15cm | +0.0037 | 0.4533 | 0.3095 |
| cmu_tunnel | full vs e3_hier | chamfer_l1_m | +1.3036 | 0.7852 | 1 |
| cmu_tunnel | full vs e3_hier | occupied_iou_tol1 | +0.0068 | 0.4298 | 0.4206 |
| cmu_tunnel | full vs mtare | final_area_m2 | -305.3522 | 0.03718 | 0.09524 |
| cmu_tunnel | full vs mtare | completeness_15cm | -0.0093 | 0.1756 | 0.09524 |
| cmu_tunnel | full vs mtare | chamfer_l1_m | +16.4542 | 0.01546 | 0.05556 |
| cmu_tunnel | full vs mtare | occupied_iou_tol1 | -0.0752 | 0.002377 | 0.007937 |
