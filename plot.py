import matplotlib.pyplot as plt
import numpy as np
import os

# Accepts statelog as NewStateLog() pbtxt
def plotDrs(stateLog, path, taskName):
    downForce = []
    dragForce = []
    drs = []
    timestamp = []

    for state in stateLog.vehicleState:
        if (state.timestamp > 0.1):
            downForce.append(state.downForce)
            dragForce.append(state.dragForce)
            drs.append(1500*state.drs)
            timestamp.append(state.timestamp)

    fig, ax = plt.subplots()
    ax.plot(timestamp, downForce, color='blue', label='dragforce')
    ax.plot(timestamp, dragForce, color='red', label='downforce')
    ax.fill_between(timestamp, drs, color='green', step='pre', label='drs')
    #ax.plot(timestamp, drs, color='green', label='drs')
    plt.figtext(.8, .95, "Time: " + str(timestamp[-1]))
    plt.legend(loc="upper left")
    plt.title(taskName + ': DRS')
    ax.set_xlabel('time (s)')
    ax.grid()

    fig.savefig(os.path.join(path, "drs_graph"))
    
    plt.show()


def plotAccel(stateLog, path, taskName):
    accel = []
    timestamp = []

    for state in stateLog.vehicleState:
        if (state.timestamp > 0.1):
            accel.append(state.accel)
            timestamp.append(state.timestamp)

    fig, ax = plt.subplots()
    ax.plot(timestamp, accel, color='red', label='Longitudinal Acceleration m/s^2')
    plt.legend(loc="upper left")
    plt.title(taskName + ': Acceleration')

    
    ax.set_xlabel('time (s)')
    ax.set_ylabel('a (m/s^2)')
    ax.grid()
    

    fig.savefig(os.path.join(path, "accel_graph"))
    #plt.show()



def plotThermalOutput(stateLog, path, taskName):
    inverter = []
    motor = []
    timestamp = []

    for state in stateLog.vehicleState:
        if (state.timestamp > 0.1):
            inverter.append(state.inverterHeat)
            motor.append(state.motorHeatTotal)
            timestamp.append(state.timestamp)

    inverterAvg = sum(inverter) / len(inverter)
    motorAvg = sum(motor) / len(motor)


    fig, ax = plt.subplots()
    ax.plot(timestamp, motor, color='red', label='Motor Heat (all 4 motors)')
    ax.plot(timestamp, inverter, color='blue', label='Inverter Heat')
    plt.legend(loc="upper left")
    plt.title(taskName + ': Thermal Output')
    plt.figtext(.8, .95, "Inverter Avg: " + str(inverterAvg))
    plt.figtext(.8, .9, "Motor Avg: " + str(motorAvg))

    
    ax.set_xlabel('time (s)')
    ax.set_ylabel('heat generated (W)')
    ax.grid()
    

    fig.savefig(os.path.join(path, "heat_graph"))
    #plt.show()


def plotOthers(stateLog, path, taskName):
    accel = []
    timestamp = []

    for state in stateLog.vehicleState:
        if (state.timestamp > 0.1):
            accel.append(state.accel)
            timestamp.append(state.timestamp)

    fig, ax = plt.subplots()
    ax.plot(timestamp, accel, color='red', label='Longitudinal Acceleration m/s^2')
    plt.legend(loc="upper left")
    plt.title(taskName + ': Acceleration')

    
    ax.set_xlabel('time (s)')
    ax.set_ylabel('a (m/s^2)')
    ax.grid()
    

    fig.savefig(os.path.join(path, "accel_graph"))
    #plt.show()


