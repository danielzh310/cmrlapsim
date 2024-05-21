import argparse
import os

from proto.experiment_config_pb2 import ExperimentConfig
from proto.vehicle_config_pb2 import VehicleConfig
from utils import *


def main(args):
    # Get template configs
    experimentConfig = readProtoText(args.config_template, ExperimentConfig)
    vehicleConfig = readProtoText(experimentConfig.vehicleConfigPath, VehicleConfig)

    # Get directories to store new configs
    experimentDir = os.path.dirname(args.config_template)
    vehicleDir = os.path.dirname(experimentConfig.vehicleConfigPath)

    for v in args.values_to_assign:
        # Change the variable in config
        exec(f"{args.variable_to_change} = {v}")

        # Change experiment config to link to this new vehicle config file
        experimentConfig.name = str(v)
        experimentConfig.vehicleConfigPath = f"{vehicleDir}/{v}.pbtxt"

        # Save new config files
        experimentFilename = os.path.join(experimentDir, f"{v}.pbtxt")
        vehicleFilename = os.path.join(vehicleDir, f"{v}.pbtxt")
        writeProtoText(experimentFilename, experimentConfig)
        writeProtoText(vehicleFilename, vehicleConfig)

    return


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--config_template",
        type=str,
        help="Experiment config template to run, use local paths for ease of future use"
    )
    parser.add_argument(
        "--variable_to_change",
        type=str,
    )
    parser.add_argument(
        "--values_to_assign",
        type=int,
        nargs="+",
        help="Range of values to assign to the variable_to_change",
    )
    args = parser.parse_args()

    main(args)
