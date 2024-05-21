import argparse
import shutil
import time
import os
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
#os.environ["GIT_PYTHON_GIT_EXECUTABLE"] = "C:\\Program Files\\Git\\bin\\git.exe"
import git
from joblib import dump, load
import pandas as pd

from plot import plotDrs, plotThermalOutput, plotAccel
from car import Car, func
from controller import Policy, Controller
from utils import writeProtoText, readProtoText
from corner_approximator import generateLatAccels
from visualization import generate_visualizations

from proto.vehicle_config_pb2 import VehicleConfig
from proto.track_pb2 import Track, SegmentType
from proto.experiment_config_pb2 import ExperimentConfig, Task
from proto.policy_config_pb2 import DrsPolicy
from proto.visualization_config_pb2 import VisualizationConfig


def simulate(track, controller, nLaps, lats, rads, drs_lats, drs_rad):

	for j in range(nLaps):
		t = time.time()
		nSegments = len(track.segment)
		print('Lap {}'.format(j+1))
		for i, segment in enumerate(track.segment):
			controller.car.setSegmentNumber(i)
			controller.car.setSegmentType(segment.type)
			if segment.type is SegmentType.STRAIGHT:
				length = segment.length
				if i < nSegments-1:
					nextSegment = track.segment[i+1]
					assert(nextSegment.type is SegmentType.CORNER)
					controller.straight(length, nextSegment.radius, lats, rads, drs_lats, drs_rad)
				elif j < nLaps-1:  # We are at the last segment of a lap. So next segment is track.segment[0]
					nextLength = track.segment[0].length
					nextCornerRadius = track.segment[1].radius
					controller.straight(length + nextLength, nextCornerRadius, lats, rads, drs_lats, drs_rad)
				else:
					controller.straight(length, 1e9, lats, rads, drs_lats, drs_rad)

			else: # CORNER
				controller.corner(segment.radius, segment.angle, lats, rads, drs_lats, drs_rad)

		print('{:.2f}s'.format(time.time()-t))

	car = controller.car

	print('\n\nPARAMETERS\n')
	print('Course: ' + track.name)
	print('Laps: {}'.format(nLaps))
	print('Accel Power Limit: {:.2f} kW'.format(controller.policy.accelPower/1e3))
	print('Regen Power Limit: {:.2f} kW'.format(controller.policy.regenPower/1e3))

def create_results_df():
	columns = ['Name', 'Distance', 'Time', 'Motor Loss', 'Inverter Loss', 'Accumulator Loss', 'Mech Brake Loss', 'Recovered', 'Remaining Energy']
	return pd.DataFrame(columns=columns)

def log_task_results(car, results_df, task_name):
	row = []

	row.append(task_name)

	distance = car.state.position/1000
	row.append(distance)

	time = car.state.timestamp
	row.append(time)

	motorHeatLoss = car.powertrain.motorHeatLoss/3.6e6
	inverterHeatLoss = car.powertrain.inverterHeatLoss/3.6e6
	acHeatLoss = car.powertrain.accumulator.heatLoss/3.6e6
	row.append(motorHeatLoss)
	row.append(inverterHeatLoss)
	row.append(acHeatLoss)

	mechBrakeLoss = car.powertrain.brakeHeatLoss/3.6e6
	row.append(mechBrakeLoss)

	recovered = car.powertrain.accumulator.recovered/3600/1e3
	row.append(recovered)

	remainingEnergy = car.powertrain.accumulator.getEnergy()/3.6e6
	row.append(remainingEnergy)

	results_df.loc[results_df.shape[0]] = row

def main(args):
	for exp in args.experiments:

		experimentConfig = readProtoText(exp, ExperimentConfig)

		# Create experiment directory
		path = os.path.join(experimentConfig.baseDir, experimentConfig.name)
		print("PATH: " + path)
		if os.path.exists(path):
			if args.overwrite:
				shutil.rmtree(path)
			else:
				if args.exception:
					raise Exception(
						"Experiment already exists. Use another name or add --overwrite argument"
					)
				print("Experiment " + exp + " already exists. Continuing with next experiment...")
				continue
		os.makedirs(path)
		
		repo = git.Repo(search_parent_directories=True)
		experimentConfig.gitSha = repo.head.object.hexsha

		filename = os.path.join(path, "experiment_config.pbtxt")
		writeProtoText(filename, experimentConfig)

		vehicleConfig = readProtoText(experimentConfig.vehicleConfigPath, VehicleConfig)
		filename = os.path.join(path, "vehicle_config.pbtxt")
		writeProtoText(filename, vehicleConfig)
		
		car = Car(vehicleConfig)
		print(car)

		# Generates path for current or future cornering data file - named the same as car_config file but with ".pkl"
		corneringPath = experimentConfig.vehicleConfigPath[:-6] + ".pkl"
		corneringPathDrs = ""
		if (args.drs):
			corneringPathDrs = experimentConfig.vehicleConfigPath[:-6] + "_drs.pkl"
		else:
			corneringPathDrs = corneringPath

		# Calculate cornering data if it doesn't exist or recalc is requested.
		if not os.path.exists(corneringPath) or args.recalc:
			print("Calculating cornering data")
			#TODO create model
			velocities = np.linspace(1, 70, 25)
			swangles = np.linspace(1, 180, 40)

			lats, rad = generateLatAccels(car, velocities, swangles, False)
			dump((lats, rad), corneringPath) # puts data into .pkl file
			print("Data calculated and dumped to " + str(corneringPath))
		else:
			print("Loading cornering data from path " + str(corneringPath))
			(lats, rad) = load(corneringPath)
			print("Data loaded!")

		if args.drs and (not os.path.exists(corneringPathDrs) or args.recalc):
			print("Calculating DRS cornering data")
			#TODO create model
			velocities = np.linspace(1, 70, 25)
			swangles = np.linspace(1, 180, 40)
			start = time.time()
			drs_lats, drs_rad = generateLatAccels(car, velocities, swangles, True)
			dump((drs_lats, drs_rad), corneringPathDrs) # puts data into .pkl file
			print("Data calculated and dumped to " + str(corneringPathDrs))
			print(f"seconds elapsed for DRS generateLatAccels(): {time.time() - start}")
		else:
			print("Loading cornering data from path " + str(corneringPathDrs))
			(drs_lats, drs_rad) = load(corneringPathDrs)
			print("Data loaded!")

		results_df = create_results_df()

		for task in experimentConfig.task:
			car = Car(vehicleConfig)
			taskPath = os.path.join(path, task.name)
			os.makedirs(taskPath)

			track = readProtoText(task.trackPath, Track)
			nLaps = task.nLaps

			policy = Policy(task.policyConfig)
			
			#print("Fitting model to function")
			#popt, pcov = curve_fit(func, rad, lats)
			#perr = np.sqrt(np.diag(pcov))
			#print("Fit function with average parameter error: " + str(sum(perr)/len(perr)))

			controller = Controller(car, policy)

			#plt.plot(rad, func(rad, *popt))
			print('\n\n-----', task.name, '-----\n')
			simulate(track, controller, nLaps, lats, rad, drs_lats, drs_rad)

			# plotDrs(car.log, taskPath, task.name)
			# plotAccel(car.log, taskPath, task.name)
			# plotThermalOutput(car.log, taskPath, task.name)
			car.saveLog(os.path.join(taskPath, "vehicle_state_log.pbtxt"))
			car.saveLogAsCSV(os.path.join(taskPath, "vehicle_state_log.csv"))

			log_task_results(car, results_df, task.name)

			if args.visualization is not None:
				if nLaps <= 1:
					car_data = pd.read_csv(os.path.join(taskPath, "vehicle_state_log.csv"), index_col=False)
					vis_config = readProtoText(args.visualization, VisualizationConfig)
					vis_path = os.path.join(taskPath, "Visualizations")
					generate_visualizations(car_data, track, vis_config, vis_path)
				else:
					print("Multilap task: skipping visualization")
	
		with open(os.path.join(path, "results.csv"), "w+") as f:
			# don't print row #s since order isn't important
			f.write(results_df.to_csv(index=False))  

if __name__ == "__main__":
	parser = argparse.ArgumentParser()

	#parser.add_argument(
	#	"--experiment_config",
	#	type=str,
	#	required=True,
	#	help="Path to experiment config pbtxt file. Relative paths encouraged to ease sharing",
	#)

	parser.add_argument(
		"experiments",
		type=str,
		nargs="+",
		metavar="experiment_config",
		help="Experiments to run, use local paths for ease of future use"
	)

	parser.add_argument(
		"--overwrite",
		action="store_true",
		dest="overwrite",
		default=False,
		help="Flag to override existing experiment of same name",
	)

	parser.add_argument(
		"--recalculate",
		action="store_true",
		dest="recalc",
		default=False,
		help="Flag to recalculate corner model",
	)

	parser.add_argument(
		"--exception",
		action="store_true",
		dest="exception",
		default=False,
		help="Flag to raise an exception instead of print an error message when an experiment already exists, " +
		"not recommended for long experiment queues"
	)

	parser.add_argument(
		"--drs",
		action="store_true",
		dest="drs",
		default=False,
		help="Flag to calculate DRS cornering model"
	)

	parser.add_argument(
		"--vis_config",
		dest="visualization",
		help="Path to visualization config. If omitted, no visualization is generated."
	)

	args = parser.parse_args()
	main(args)
