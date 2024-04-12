## Time-Optimal Trajectory Planning and Tracking for Autonomous Vehicles
TOTPT solves time-optimal raceline using NLP with direct collocation, and performs NMPC path tracking in Simulink.


## Installation
### Check Prerequisites
* Matlab and Simulink (tested on R2017a and R2022a with Win11)
* Install [CasADi v3.5.5](https://github.com/casadi/casadi/releases/tag/3.5.5)
* Install [acados for Matlab](https://docs.acados.org/installation/index.html#windows-for-use-with-matlab)

[Download TOTPT](https://github.com/zlijunting/TOTPT/archive/refs/heads/main.zip) and unzip at your local directory

### Check Dependencies
* Run `totpt_env_variables.m` to add `functions` and `params` folders to Matlab search path
* Open `acados_env_variables_windows` in `nmpc` folder, fill acados and casadi installation paths in the two lines
```
acados_dir = 'A:\path\to\acados';
casadi_dir = 'B:\path\to\casadi-windows-matlabR2016a-v3.5.5';
```

## Workflow
1. navigate to `tro` folder, run live script `tro_main` to generate smoothed time-optimal raceline (1~3 min, depend on problem size)
2. navigate to `nmpc` folder, run `nmpc_gen` to generate NMPC mex function and automatically copy it to `sim` folder
3. navigate to `sim` folder, run live script `racing_sim` to simulate racing scenario

   

## Folders
* `racetrack-database`: racelines and track widths of race tracks, available on (https://github.com/TUMFTM/racetrack-database)
* `functions`: helper functions for track processing
* `params`: track smoothing parameters and vehicle parameters
* `tro`: offline time-optimal trajectory optimization framework
* `nmpc`: online NMPC trajectory tracking framework


