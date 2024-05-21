# CMR-Sims

## How to Run

To install requirements: `pip3 install -r requirements.txt`

To run: `python3 ./lapsim.py <PATH_TO_EXPERIMENT_CONFIG>`

### Options

`-h` Help

`--recalc` Re-precalculate value set for cornering model

- Required whenever **vehicle configs** are changed (except accumulator config, which are computed on runtime)

- May take upwards of half an hour with current codebase

`--overwrite` Overwrite any previous output data with same file names (unsafe, but will ensure new data is output)

`--drs` Run DRS

## Configurations

All config files are in Google's Protocol Buffers format `.pbtxt`

- Vehicle configs

    - Each vehicle config file corresponds to a differently configured vehicle, e.g. 22e with 5P
    
    - Includes accumulator-related settings; other settings are in experiment configs

- Track configs

    - Each track is modelled as a sequence of zero-width `STRAIGHT`s and `CORNER`s

- Experiment configs

    - Each Lapsim run uses a single experiment config, which can use only a single vehicle

    - Each experiment config file includes one or more `tasks`, defined by different tracks and policies
