## Time-Optimal Trajectory Planning and Tracking for Autonomous Vehicles
TOTPT solves time-optimal raceline using NLP with direct collocation, and performs NMPC path tracking in Simulink.


## Installation
### Check Prerequisites
* Matlab and Simulink (tested on R2017a and R2022a with Win11)
* Install [CasADi v3.5.5](https://web.casadi.org/get/)
* Install [acados for Matlab](https://docs.acados.org/installation/index.html#windows-for-use-with-matlab)

[Download this repository](https://github.com/zlijunting/TOTPT/archive/refs/heads/main.zip) and unzip at your local directory

### Check Dependencies
*


## Folders
* `racetrack-database`: racelines and track widths of race tracks, available on (https://github.com/TUMFTM/racetrack-database)
* `functions`: helper functions for track processing
* `params`: track smoothing parameters and vehicle parameters
* `tro`: offline time-optimal trajectory optimization framework
* `nmpc`: online NMPC trajectory tracking framework





<!-- 這句看不見，一句話的註解 -->
<!-- Introduction -->


## Workflow
1. run `add_path`
2. run live script `tro_main` in `tro` folder to generate smoothed time-optimal raceline
3. navigate to `nmpc` folder, run `nmpc_gen` to generate NMPC s-function and automatically copy it to `sim` folder
5. run `racing_sim`

<!-- run `acados_env_variables_windows` before nmpc codegen --!>

<!--
## Workflow
### 1. Track Smoothing
...
### 2. Offline Trajectory Optimization
...
### 3. Online Tracjectory Tracking
--!>

