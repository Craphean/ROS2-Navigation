# M5 Benchmark Quantitative Summary (4-Core Edge Profile)

## Suite 1: Main Multi-World Benchmark (Table I / Fig. 3)

| World | Method | N | Area (m²) | 3D Comp@15cm (%) | Chamfer-L1 (m) | Occ IoU | CE (m²/m) | CPU (%) | RAM (MB) | Cyc P95 (ms) | Coll |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| house | Legacy (E0) | 5 | 86.2 ± 0.6 | 77.7 ± 3.6 | 0.058 ± 0.004 | 0.800 ± 0.010 | 1.47 ± 0.24 | 262.3 ± 20.4 | 2365 ± 4 | -- | 0.2 ± 0.4 |
| house | 2D-Frontier (E1) | 5 | 86.0 ± 0.7 | 76.2 ± 3.2 | 0.062 ± 0.006 | 0.720 ± 0.061 | 1.48 ± 0.26 | 241.7 ± 6.5 | 2334 ± 7 | 8.0 ± 1.9 | 0.0 ± 0.0 |
| house | +Hier (E3) | 5 | 86.0 ± 0.5 | 77.5 ± 3.4 | 0.061 ± 0.007 | 0.718 ± 0.055 | 1.78 ± 0.40 | 240.7 ± 4.5 | 2331 ± 7 | 8.6 ± 1.6 | 0.0 ± 0.0 |
| house | Lite-HMRE (Ours) | 5 | 86.3 ± 0.4 | 78.4 ± 0.9 | 0.060 ± 0.004 | 0.726 ± 0.090 | 2.01 ± 0.39 | 270.0 ± 13.3 | 2333 ± 2 | 23.9 ± 6.9 | 0.0 ± 0.0 |
| hospital | Legacy (E0) | 5 | 170.1 ± 1.5 | 78.8 ± 2.1 | 0.063 ± 0.011 | 0.742 ± 0.059 | 1.89 ± 0.19 | 234.3 ± 3.0 | 2370 ± 9 | -- | 0.2 ± 0.4 |
| hospital | 2D-Frontier (E1) | 5 | 159.1 ± 21.8 | 70.9 ± 9.9 | 0.115 ± 0.091 | 0.665 ± 0.096 | 3.23 ± 0.90 | 239.3 ± 7.0 | 2326 ± 11 | 16.3 ± 4.4 | 0.0 ± 0.0 |
| hospital | +Hier (E3) | 5 | 167.1 ± 4.9 | 72.3 ± 5.5 | 0.078 ± 0.018 | 0.625 ± 0.177 | 2.75 ± 0.89 | 268.4 ± 20.7 | 2332 ± 5 | 17.2 ± 7.4 | 0.0 ± 0.0 |
| hospital | Lite-HMRE (Ours) | 5 | 156.6 ± 33.4 | 70.6 ± 16.4 | 0.141 ± 0.169 | 0.654 ± 0.130 | 3.26 ± 0.76 | 283.1 ± 4.2 | 2333 ± 5 | 41.2 ± 6.3 | 0.0 ± 0.0 |
| mine | Legacy (E0) | 5 | 260.2 ± 43.4 | 53.9 ± 9.1 | 0.444 ± 0.512 | 0.729 ± 0.157 | 3.04 ± 0.83 | 186.9 ± 5.1 | 2280 ± 8 | -- | 0.0 ± 0.0 |
| mine | 2D-Frontier (E1) | 5 | 249.8 ± 50.9 | 57.8 ± 10.3 | 0.501 ± 0.523 | 0.697 ± 0.157 | 2.91 ± 0.31 | 206.8 ± 6.9 | 2268 ± 28 | 10.0 ± 9.3 | 0.0 ± 0.0 |
| mine | +Hier (E3) | 5 | 191.2 ± 52.2 | 42.1 ± 10.1 | 1.550 ± 1.067 | 0.631 ± 0.031 | 3.25 ± 0.54 | 234.7 ± 27.3 | 2251 ± 21 | 31.1 ± 15.4 | 0.0 ± 0.0 |
| mine | Lite-HMRE (Ours) | 5 | 238.2 ± 54.0 | 51.8 ± 10.9 | 0.621 ± 0.631 | 0.687 ± 0.084 | 3.08 ± 0.54 | 261.2 ± 5.1 | 2317 ± 24 | 67.3 ± 22.4 | 0.0 ± 0.0 |
| warehouse | Legacy (E0) | 5 | 637.7 ± 176.6 | 26.4 ± 8.9 | 2.115 ± 1.138 | 0.418 ± 0.090 | 7.24 ± 1.43 | 255.2 ± 7.4 | 2609 ± 9 | -- | 0.0 ± 0.0 |
| warehouse | 2D-Frontier (E1) | 5 | 752.9 ± 156.9 | 30.1 ± 8.7 | 1.425 ± 0.774 | 0.414 ± 0.062 | 9.69 ± 1.12 | 281.2 ± 17.8 | 2609 ± 25 | 122.0 ± 84.9 | 0.0 ± 0.0 |
| warehouse | +Hier (E3) | 5 | 616.8 ± 144.9 | 28.6 ± 5.8 | 2.559 ± 1.829 | 0.392 ± 0.089 | 7.04 ± 1.88 | 288.4 ± 18.9 | 2598 ± 35 | 122.0 ± 62.9 | 0.0 ± 0.0 |
| warehouse | Lite-HMRE (Ours) | 5 | 652.2 ± 207.5 | 30.0 ± 8.1 | 2.417 ± 1.981 | 0.405 ± 0.132 | 7.18 ± 2.53 | 309.9 ± 3.7 | 2649 ± 34 | 204.1 ± 102.7 | 0.0 ± 0.0 |
| cmu_indoor | Legacy (E0) | 5 | 546.6 ± 141.4 | 9.8 ± 2.0 | 14.013 ± 4.207 | 0.155 ± 0.018 | 6.04 ± 1.87 | 264.5 ± 4.2 | 2696 ± 7 | -- | 0.0 ± 0.0 |
| cmu_indoor | 2D-Frontier (E1) | 5 | 500.0 ± 98.6 | 9.1 ± 1.2 | 14.584 ± 3.849 | 0.138 ± 0.016 | 5.79 ± 1.36 | 287.2 ± 5.7 | 2803 ± 82 | 86.4 ± 21.8 | 0.0 ± 0.0 |
| cmu_indoor | +Hier (E3) | 5 | 509.5 ± 120.6 | 9.9 ± 2.6 | 13.790 ± 2.388 | 0.141 ± 0.019 | 5.48 ± 1.23 | 290.9 ± 3.7 | 2822 ± 48 | 113.3 ± 41.8 | 0.0 ± 0.0 |
| cmu_indoor | CMU M-TARE | 5 | 1807.1 ± 209.8 | 36.7 ± 4.0 | 2.002 ± 1.720 | 0.583 ± 0.047 | 3.06 ± 0.39 | 319.9 ± 1.2 | 6110 ± 17 | 1020.1 ± 16.3 | 0.0 ± 0.0 |
| cmu_indoor | Lite-HMRE (Ours) | 5 | 443.0 ± 94.0 | 8.5 ± 1.7 | 14.064 ± 3.282 | 0.123 ± 0.023 | 4.78 ± 1.53 | 291.3 ± 3.6 | 2949 ± 80 | 133.3 ± 39.0 | 0.0 ± 0.0 |
| cmu_tunnel | Legacy (E0) | 5 | 500.0 ± 156.9 | 2.8 ± 0.7 | 53.962 ± 6.775 | 0.044 ± 0.011 | 5.76 ± 2.00 | 202.6 ± 9.6 | 2354 ± 44 | -- | 0.0 ± 0.0 |
| cmu_tunnel | 2D-Frontier (E1) | 5 | 391.6 ± 89.9 | 2.5 ± 0.5 | 54.541 ± 9.013 | 0.034 ± 0.007 | 5.28 ± 1.79 | 242.2 ± 24.7 | 2522 ± 195 | 161.4 ± 125.3 | 0.2 ± 0.4 |
| cmu_tunnel | +Hier (E3) | 5 | 488.4 ± 158.5 | 2.7 ± 0.6 | 52.911 ± 8.770 | 0.044 ± 0.014 | 5.34 ± 1.82 | 291.6 ± 44.2 | 2542 ± 161 | 239.8 ± 131.8 | 0.0 ± 0.0 |
| cmu_tunnel | CMU M-TARE | 5 | 2752.1 ± 690.3 | 13.7 ± 2.6 | 15.830 ± 11.161 | 0.319 ± 0.048 | 4.27 ± 1.07 | 314.3 ± 1.4 | 5664 ± 27 | 890.5 ± 102.5 | 0.2 ± 0.4 |
| cmu_tunnel | Lite-HMRE (Ours) | 5 | 424.2 ± 108.3 | 2.8 ± 0.7 | 53.120 ± 9.501 | 0.039 ± 0.012 | 4.79 ± 1.34 | 302.1 ± 17.1 | 2734 ± 310 | 195.5 ± 139.9 | 0.0 ± 0.0 |

## Suite 2: 7-Step Component Ablation (Table II)

| World | Config | N | Area (m²) | 3D Comp@15cm (%) | Chamfer-L1 (m) | CE (m²/m) | Overlap | Cyc P95 (ms) | Coll |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| mine | Legacy (E0) | 5 | 260.2 ± 43.4 | 53.9 ± 9.1 | 0.444 ± 0.512 | 3.04 ± 0.83 | 0.047 ± 0.051 | -- | 0.0 ± 0.0 |
| mine | 2D-Frontier (E1) | 5 | 249.8 ± 50.9 | 57.8 ± 10.3 | 0.501 ± 0.523 | 2.91 ± 0.31 | 0.020 ± 0.028 | 10.0 ± 9.3 | 0.0 ± 0.0 |
| mine | +3D-Gain (E2) | 5 | 217.9 ± 53.6 | 50.3 ± 9.4 | 0.829 ± 0.734 | 2.58 ± 0.39 | 0.011 ± 0.026 | 40.7 ± 15.2 | 0.0 ± 0.0 |
| mine | +Hier (E3) | 5 | 191.2 ± 52.2 | 42.1 ± 10.1 | 1.550 ± 1.067 | 3.25 ± 0.54 | 0.009 ± 0.020 | 31.1 ± 15.4 | 0.0 ± 0.0 |
| mine | +Geodesic (E4) | 5 | 222.6 ± 51.8 | 48.1 ± 11.3 | 0.764 ± 0.649 | 3.17 ± 0.21 | 0.018 ± 0.041 | 36.8 ± 6.1 | 0.0 ± 0.0 |
| mine | +Deconflict (E5) | 5 | 217.6 ± 54.0 | 50.6 ± 10.5 | 0.991 ± 0.732 | 2.73 ± 0.66 | 0.003 ± 0.006 | 41.2 ± 15.7 | 0.0 ± 0.0 |
| mine | Lite-HMRE (Ours) | 5 | 238.2 ± 54.0 | 51.8 ± 10.9 | 0.621 ± 0.631 | 3.08 ± 0.54 | 0.040 ± 0.040 | 67.3 ± 22.4 | 0.0 ± 0.0 |
| hospital | Legacy (E0) | 5 | 170.1 ± 1.5 | 78.8 ± 2.1 | 0.063 ± 0.011 | 1.89 ± 0.19 | 0.145 ± 0.028 | -- | 0.2 ± 0.4 |
| hospital | 2D-Frontier (E1) | 5 | 159.1 ± 21.8 | 70.9 ± 9.9 | 0.115 ± 0.091 | 3.23 ± 0.90 | 0.036 ± 0.051 | 16.3 ± 4.4 | 0.0 ± 0.0 |
| hospital | +3D-Gain (E2) | 5 | 168.3 ± 5.1 | 74.4 ± 5.7 | 0.075 ± 0.013 | 2.64 ± 0.67 | 0.002 ± 0.003 | 14.0 ± 8.4 | 0.0 ± 0.0 |
| hospital | +Hier (E3) | 5 | 167.1 ± 4.9 | 72.3 ± 5.5 | 0.078 ± 0.018 | 2.75 ± 0.89 | 0.012 ± 0.018 | 17.2 ± 7.4 | 0.0 ± 0.0 |
| hospital | +Geodesic (E4) | 5 | 156.6 ± 30.0 | 64.4 ± 12.7 | 0.145 ± 0.109 | 3.00 ± 0.41 | 0.023 ± 0.033 | 19.9 ± 6.0 | 0.0 ± 0.0 |
| hospital | +Deconflict (E5) | 5 | 162.9 ± 9.1 | 70.5 ± 6.4 | 0.080 ± 0.017 | 2.97 ± 0.26 | 0.024 ± 0.034 | 34.9 ± 14.5 | 0.0 ± 0.0 |
| hospital | Lite-HMRE (Ours) | 5 | 156.6 ± 33.4 | 70.6 ± 16.4 | 0.141 ± 0.169 | 3.26 ± 0.76 | 0.000 ± 0.000 | 41.2 ± 6.3 | 0.0 ± 0.0 |

## Suite 3: Noisy Odometry SE(2) Alignment & Degeneracy Study (Fig. 2)

| World | Variant | N | Occ IoU | ATE RMSE (m) | Chamfer-L1 (m) | Prec@10cm (%) | Cloud CPU (%) | SE(2) Accepts | Degen Gates |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| mine | Legacy Merge + PyCloud | 5 | 0.694 ± 0.099 | 0.262 ± 0.120 | 0.560 ± 0.643 | 94.9 ± 4.8 | 20.3 ± 3.2 | 0.0 ± 0.0 | 0.0 ± 0.0 |
| mine | No SE(2) Align | 5 | 0.734 ± 0.077 | 0.314 ± 0.036 | 0.613 ± 0.592 | 98.2 ± 1.4 | 2.6 ± 0.3 | 0.0 ± 0.0 | 0.0 ± 0.0 |
| mine | SE(2) w/o Degen. Gate | 5 | 0.666 ± 0.019 | 0.296 ± 0.048 | 0.593 ± 0.515 | 97.9 ± 1.2 | 2.6 ± 0.2 | 28.6 ± 40.9 | 0.4 ± 0.5 |
| mine | Lite-HMRE (Ours) | 5 | 0.687 ± 0.084 | 0.215 ± 0.069 | 0.621 ± 0.631 | 97.2 ± 1.9 | 3.3 ± 0.3 | 47.4 ± 30.3 | 1.2 ± 2.7 |
| mine | GT-Odom (Oracle) | 5 | 0.760 ± 0.061 | -- | 0.767 ± 0.528 | 99.9 ± 0.0 | 2.9 ± 0.3 | 32.8 ± 32.6 | 0.4 ± 0.9 |
| warehouse | Legacy Merge + PyCloud | 5 | 0.473 ± 0.113 | 0.311 ± 0.262 | 1.929 ± 2.195 | 57.0 ± 5.7 | 21.4 ± 1.2 | 0.0 ± 0.0 | 0.0 ± 0.0 |
| warehouse | No SE(2) Align | 5 | 0.425 ± 0.093 | 0.177 ± 0.044 | 2.201 ± 2.145 | 56.5 ± 3.2 | 3.0 ± 0.0 | 0.0 ± 0.0 | 0.0 ± 0.0 |
| warehouse | SE(2) w/o Degen. Gate | 5 | 0.441 ± 0.037 | 0.257 ± 0.058 | 1.874 ± 0.609 | 56.6 ± 8.6 | 3.9 ± 1.7 | 20.2 ± 36.0 | 1.4 ± 1.9 |
| warehouse | Lite-HMRE (Ours) | 5 | 0.405 ± 0.132 | 0.312 ± 0.074 | 2.417 ± 1.981 | 55.7 ± 8.4 | 3.4 ± 0.2 | 19.8 ± 41.0 | 1.6 ± 1.8 |
| warehouse | GT-Odom (Oracle) | 5 | 0.432 ± 0.078 | -- | 2.882 ± 2.108 | 63.8 ± 4.5 | 3.2 ± 0.2 | 21.4 ± 38.1 | 3.2 ± 4.7 |

## Suite 4: Multi-Robot Scalability N = 1..4 (Fig. 4)

| World | N Robots | Runs | Area (m²) | 3D Comp@15cm (%) | Load CV | Overlap | Comms (KB/s) | CPU (%) | RAM (MB) | Cyc P95 (ms) | Coll |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| mine | 1 | 5 | 161.6 ± 12.6 | 33.9 ± 4.1 | 0.000 ± 0.000 | 0.000 ± 0.000 | 385.6 ± 53.9 | 129.5 ± 19.3 | 1345 ± 9 | 8.7 ± 2.5 | 0.0 ± 0.0 |
| mine | 2 | 5 | 238.2 ± 54.0 | 51.8 ± 10.9 | 0.207 ± 0.113 | 0.040 ± 0.040 | 725.5 ± 113.1 | 261.2 ± 5.1 | 2317 ± 24 | 67.3 ± 22.4 | 0.0 ± 0.0 |
| mine | 3 | 5 | 267.3 ± 49.4 | 67.7 ± 11.9 | 0.258 ± 0.112 | 0.068 ± 0.076 | 897.7 ± 146.8 | 305.7 ± 16.5 | 3448 ± 23 | 102.8 ± 55.5 | 0.0 ± 0.0 |
| mine | 4 | 5 | 280.1 ± 11.1 | 69.7 ± 6.0 | 0.319 ± 0.083 | 0.039 ± 0.037 | 1029.3 ± 101.9 | 344.1 ± 4.1 | 4744 ± 32 | 237.0 ± 123.3 | 0.0 ± 0.0 |
| cmu_indoor | 1 | 5 | 280.0 ± 170.4 | 4.3 ± 1.6 | 0.000 ± 0.000 | 0.000 ± 0.000 | 637.8 ± 379.0 | 164.3 ± 20.7 | 1774 ± 69 | 36.5 ± 25.3 | 0.0 ± 0.0 |
| cmu_indoor | 2 | 5 | 443.0 ± 94.0 | 8.5 ± 1.7 | 0.050 ± 0.062 | 0.001 ± 0.002 | 2732.7 ± 783.6 | 291.3 ± 3.6 | 2949 ± 80 | 133.3 ± 39.0 | 0.0 ± 0.0 |
| cmu_indoor | 3 | 5 | 659.7 ± 141.5 | 13.2 ± 1.4 | 0.171 ± 0.113 | 0.021 ± 0.026 | 5212.4 ± 2939.8 | 311.9 ± 13.3 | 4211 ± 334 | 182.8 ± 112.7 | 0.0 ± 0.0 |
| cmu_indoor | 4 | 5 | 828.7 ± 229.3 | 16.3 ± 3.7 | 0.233 ± 0.134 | 0.028 ± 0.040 | 6783.4 ± 2852.5 | 325.4 ± 11.8 | 5669 ± 376 | 208.3 ± 82.7 | 0.0 ± 0.0 |

## Suite 5: Sensor & Voxel Resolution Sensitivity on Mine (Table V)

| Config | Runs | Area (m²) | 3D Comp@15cm (%) | Chamfer-L1 (m) | CPU (%) | RAM (MB) | Cyc P95 (ms) |
|---|---:|---:|---:|---:|---:|---:|---:|
| 16-beam / 0.05 m | 3 | 231.7 ± 51.8 | 50.1 ± 11.9 | 0.773 ± 0.823 | 213.8 ± 4.1 | 2296 ± 12 | 30.4 ± 6.9 |
| 16-beam / 0.10 m (Default) | 3 | 214.5 ± 60.8 | 46.9 ± 11.8 | 0.886 ± 0.724 | 261.5 ± 6.3 | 2306 ± 26 | 55.3 ± 9.2 |
| 16-beam / 0.15 m | 3 | 235.3 ± 42.8 | 50.8 ± 7.9 | 0.586 ± 0.552 | 230.0 ± 32.9 | 2318 ± 31 | 37.6 ± 26.0 |
| 32-beam / 0.10 m | 3 | 245.4 ± 44.0 | 65.8 ± 13.7 | 0.403 ± 0.337 | 235.7 ± 37.7 | 2333 ± 18 | 45.5 ± 29.8 |

## Statistical Significance (Lite-HMRE vs. Baselines)

| World | Comparison | Metric | Delta Mean | Welch p | Mann-Whitney U p |
|---|---|---|---:|---:|---:|
| house | full vs legacy | final_area_m2 | +0.1118 | 0.7371 | 0.5476 |
| house | full vs legacy | completeness_15cm | +0.0071 | 0.6931 | 0.6905 |
| house | full vs legacy | chamfer_l1_m | +0.0017 | 0.531 | 0.5476 |
| house | full vs legacy | occupied_iou_tol1 | -0.0744 | 0.1372 | 0.05556 |
| house | full vs e1_2d | final_area_m2 | +0.3340 | 0.3755 | 0.5476 |
| house | full vs e1_2d | completeness_15cm | +0.0229 | 0.1875 | 0.1508 |
| house | full vs e1_2d | chamfer_l1_m | -0.0027 | 0.4391 | 0.6905 |
| house | full vs e1_2d | occupied_iou_tol1 | +0.0052 | 0.9173 | 0.8413 |
| house | full vs e3_hier | final_area_m2 | +0.3326 | 0.2874 | 0.4206 |
| house | full vs e3_hier | completeness_15cm | +0.0095 | 0.5801 | 0.6905 |
| house | full vs e3_hier | chamfer_l1_m | -0.0011 | 0.7677 | 1 |
| house | full vs e3_hier | occupied_iou_tol1 | +0.0077 | 0.8749 | 0.8413 |
| hospital | full vs legacy | final_area_m2 | -13.4724 | 0.4183 | 0.5476 |
| hospital | full vs legacy | completeness_15cm | -0.0823 | 0.3271 | 0.1508 |
| hospital | full vs legacy | chamfer_l1_m | +0.0779 | 0.3612 | 0.1508 |
| hospital | full vs legacy | occupied_iou_tol1 | -0.0877 | 0.2223 | 0.2222 |
| hospital | full vs e1_2d | final_area_m2 | -2.4434 | 0.8949 | 0.5476 |
| hospital | full vs e1_2d | completeness_15cm | -0.0030 | 0.9727 | 0.8413 |
| hospital | full vs e1_2d | chamfer_l1_m | +0.0255 | 0.7761 | 0.5476 |
| hospital | full vs e1_2d | occupied_iou_tol1 | -0.0111 | 0.8824 | 1 |
| hospital | full vs e3_hier | final_area_m2 | -10.4840 | 0.5241 | 0.4206 |
| hospital | full vs e3_hier | completeness_15cm | -0.0171 | 0.8343 | 0.6905 |
| hospital | full vs e3_hier | chamfer_l1_m | +0.0621 | 0.4583 | 0.5476 |
| hospital | full vs e3_hier | occupied_iou_tol1 | +0.0291 | 0.7749 | 0.8413 |
| mine | full vs legacy | final_area_m2 | -21.9870 | 0.4988 | 0.5476 |
| mine | full vs legacy | completeness_15cm | -0.0205 | 0.7544 | 1 |
| mine | full vs legacy | chamfer_l1_m | +0.1766 | 0.6408 | 0.8413 |
| mine | full vs legacy | occupied_iou_tol1 | -0.0421 | 0.616 | 0.8413 |
| mine | full vs e1_2d | final_area_m2 | -11.6430 | 0.7346 | 0.4206 |
| mine | full vs e1_2d | completeness_15cm | -0.0597 | 0.3985 | 0.4206 |
| mine | full vs e1_2d | chamfer_l1_m | +0.1201 | 0.7519 | 0.8413 |
| mine | full vs e1_2d | occupied_iou_tol1 | -0.0098 | 0.9058 | 0.6905 |
| mine | full vs e3_hier | final_area_m2 | +47.0040 | 0.1991 | 0.3095 |
| mine | full vs e3_hier | completeness_15cm | +0.0977 | 0.1792 | 0.4206 |
| mine | full vs e3_hier | chamfer_l1_m | -0.9288 | 0.1411 | 0.2222 |
| mine | full vs e3_hier | occupied_iou_tol1 | +0.0554 | 0.2248 | 0.4206 |
| warehouse | full vs legacy | final_area_m2 | +14.4958 | 0.9083 | 1 |
| warehouse | full vs legacy | completeness_15cm | +0.0363 | 0.5199 | 0.6905 |
| warehouse | full vs legacy | chamfer_l1_m | +0.3022 | 0.7768 | 0.6905 |
| warehouse | full vs legacy | occupied_iou_tol1 | -0.0134 | 0.8568 | 0.8413 |
| warehouse | full vs e1_2d | final_area_m2 | -100.7040 | 0.4137 | 0.6905 |
| warehouse | full vs e1_2d | completeness_15cm | -0.0009 | 0.9869 | 1 |
| warehouse | full vs e1_2d | chamfer_l1_m | +0.9920 | 0.3432 | 0.5476 |
| warehouse | full vs e1_2d | occupied_iou_tol1 | -0.0092 | 0.893 | 0.8413 |
| warehouse | full vs e3_hier | final_area_m2 | +35.4294 | 0.7632 | 0.8413 |
| warehouse | full vs e3_hier | completeness_15cm | +0.0146 | 0.7517 | 0.6905 |
| warehouse | full vs e3_hier | chamfer_l1_m | -0.1419 | 0.9093 | 0.8413 |
| warehouse | full vs e3_hier | occupied_iou_tol1 | +0.0127 | 0.8642 | 1 |
| cmu_indoor | full vs legacy | final_area_m2 | -103.5842 | 0.215 | 0.3095 |
| cmu_indoor | full vs legacy | completeness_15cm | -0.0128 | 0.3132 | 0.3095 |
| cmu_indoor | full vs legacy | chamfer_l1_m | +0.0516 | 0.9833 | 1 |
| cmu_indoor | full vs legacy | occupied_iou_tol1 | -0.0312 | 0.04635 | 0.05556 |
| cmu_indoor | full vs e1_2d | final_area_m2 | -56.9874 | 0.3771 | 0.6905 |
| cmu_indoor | full vs e1_2d | completeness_15cm | -0.0066 | 0.5041 | 0.8413 |
| cmu_indoor | full vs e1_2d | chamfer_l1_m | -0.5200 | 0.8241 | 1 |
| cmu_indoor | full vs e1_2d | occupied_iou_tol1 | -0.0142 | 0.2949 | 0.5476 |
| cmu_indoor | full vs e3_hier | final_area_m2 | -66.4790 | 0.361 | 0.3095 |
| cmu_indoor | full vs e3_hier | completeness_15cm | -0.0144 | 0.3314 | 0.2222 |
| cmu_indoor | full vs e3_hier | chamfer_l1_m | +0.2743 | 0.884 | 1 |
| cmu_indoor | full vs e3_hier | occupied_iou_tol1 | -0.0174 | 0.2352 | 0.4206 |
| cmu_indoor | full vs mtare | final_area_m2 | -1364.0816 | 2.064e-05 | 0.007937 |
| cmu_indoor | full vs mtare | completeness_15cm | -0.2823 | 1.507e-05 | 0.007937 |
| cmu_indoor | full vs mtare | chamfer_l1_m | +12.0623 | 0.0003312 | 0.007937 |
| cmu_indoor | full vs mtare | occupied_iou_tol1 | -0.4593 | 1.451e-06 | 0.007937 |
| cmu_tunnel | full vs legacy | final_area_m2 | -75.7732 | 0.4032 | 0.1508 |
| cmu_tunnel | full vs legacy | completeness_15cm | -0.0005 | 0.9079 | 1 |
| cmu_tunnel | full vs legacy | chamfer_l1_m | -0.8416 | 0.8763 | 1 |
| cmu_tunnel | full vs legacy | occupied_iou_tol1 | -0.0054 | 0.4932 | 0.4206 |
| cmu_tunnel | full vs e1_2d | final_area_m2 | +32.5972 | 0.6189 | 0.3095 |
| cmu_tunnel | full vs e1_2d | completeness_15cm | +0.0024 | 0.5608 | 0.5476 |
| cmu_tunnel | full vs e1_2d | chamfer_l1_m | -1.4210 | 0.8144 | 0.6905 |
| cmu_tunnel | full vs e1_2d | occupied_iou_tol1 | +0.0046 | 0.4893 | 0.5476 |
| cmu_tunnel | full vs e3_hier | final_area_m2 | -64.2572 | 0.4783 | 0.2222 |
| cmu_tunnel | full vs e3_hier | completeness_15cm | +0.0003 | 0.9378 | 0.8413 |
| cmu_tunnel | full vs e3_hier | chamfer_l1_m | +0.2096 | 0.972 | 1 |
| cmu_tunnel | full vs e3_hier | occupied_iou_tol1 | -0.0055 | 0.5334 | 0.4206 |
| cmu_tunnel | full vs mtare | final_area_m2 | -2327.9306 | 0.001433 | 0.007937 |
| cmu_tunnel | full vs mtare | completeness_15cm | -0.1099 | 0.0004243 | 0.007937 |
| cmu_tunnel | full vs mtare | chamfer_l1_m | +37.2906 | 0.0005043 | 0.007937 |
| cmu_tunnel | full vs mtare | occupied_iou_tol1 | -0.2799 | 0.0001092 | 0.007937 |
