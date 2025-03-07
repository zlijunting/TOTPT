## Time-Optimal Trajectory Planning and Tracking for Autonomous Vehicles
TOTPT solves time-optimal raceline using NLP with direct collocation approach, and performs NMPC path tracking within the racing scenario implemented in Simulink.

## Installation
### Prerequisites
* Matlab and Simulink (tested on R2017a and R2022a on Win11)
* Install [CasADi](https://web.casadi.org/get/)
* Install [acados for Matlab](https://docs.acados.org/installation/index.html#windows-for-use-with-matlab)

[Download TOTPT](https://github.com/zlijunting/TOTPT/archive/refs/heads/main.zip) and unzip at local directory

### Dependencies
* Run `totpt_env_variables.m` to add subfolders to Matlab search path
* Open `acados_env_variables_windows.m` in `nmpc` folder, fill acados and casadi installation paths in the two lines
```
acados_dir = 'A:\path\to\acados';
casadi_dir = 'B:\path\to\casadi-3.6.5';
```

## Workflow
1. navigate to `tro` folder, run live script `tro_main` to generate smoothed time-optimal raceline (1~3 min, depend on problem size) 
2. navigate to `nmpc` folder, run `nmpc_gen` to generate NMPC mex function and automatically copy it to `sim` folder
3. navigate to `sim` folder, run live script `racing_sim` to simulate racing scenario

![tracking](./imgs_doc/tro_results_doc.png)
   


   

## Folders
* `racetrack-database`: racelines and track widths of race tracks, forked from (https://github.com/TUMFTM/racetrack-database)
* `functions`: helper functions for track processing
* `params`: track smoothing parameters and vehicle parameters
* `tro`: offline time-optimal trajectory optimization framework
* `nmpc`: online NMPC trajectory tracking framework

## Citing
```
@article{li2024time,
  title={Time-Optimal Trajectory Planning and Tracking for Autonomous Vehicles},
  author={Li, Jun-Ting and Chen, Chih-Keng and Ren, Hongbin},
  journal={Sensors},
  volume={24},
  number={11},
  pages={3281},
  year={2024},
  publisher={MDPI}
}
```

