#!/usr/bin/env python

import numpy as np

import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook


MAX_VOLTAGE = 12.0 # volts
FREE_SPIN_SPEED = 6000.0 # RPM

STALL_FORCE = 6.28 # in-lbs
WHEEL_INERTIA = 0.1 # in-lb

BACK_EMF = MAX_VOLTAGE / FREE_SPIN_SPEED

def simulationLoop(controller_func,
                   ordered_ball_times,
                   simulation_step_time,
                   sim_steps_per_controller_step,
                   total_time):
    controller_step_time = simulation_step_time * sim_steps_per_controller_step
    num_sim_steps = int(total_time / simulation_step_time)
    num_controller_steps = int(num_sim_steps / sim_steps_per_controller_step)

    # system state
    wheel_velocity = 0

    # logged values
    log_values = {
        "time": np.arange(0, total_time, simulation_step_time),
        "velocity": [0.0] * num_sim_steps,
        "eff_v": [0.0] * num_sim_steps,
        "output": [0.0] * num_sim_steps,

        "controller_time": np.arange(0, total_time, controller_step_time),
        "integrator": [0.0] * num_controller_steps,
    }

    def step_num(i, j):
        return i * sim_steps_per_controller_step + j

    for i in range(num_controller_steps):
        output_voltage = max(-12.0,
                             min(12.0,
                                 controller_func(wheel_velocity,
                                                 controller_step_time,
                                                 log_values,
                                                 i)))

        for j in range(sim_steps_per_controller_step):
            effective_voltage = output_voltage - (wheel_velocity * BACK_EMF)
            force = effective_voltage / MAX_VOLTAGE * STALL_FORCE
            wheel_velocity += force / WHEEL_INERTIA * simulation_step_time * 60.0 / (2.0 * np.pi)
            log_values["velocity"][step_num(i, j)] = wheel_velocity
            log_values["eff_v"][step_num(i, j)] = effective_voltage
            log_values["output"][step_num(i, j)] = output_voltage
    return log_values

def makeController():
    setpoint = 3000.0

    kFv = MAX_VOLTAGE / FREE_SPIN_SPEED
    kI = 0.001

    integrator = 0.0
    def controllerFunc(speed, delta_time, log_values, log_idx):
        nonlocal integrator
        error = setpoint - speed
        integrator += error * delta_time
        log_values["integrator"][log_idx] = integrator
        return kI * integrator + kFv * setpoint
    return controllerFunc

if __name__ == '__main__':
    data = simulationLoop(makeController(), [], 0.001, 5, 30)
    fig = plt.figure()

    num_plots = 4
    plot = fig.add_subplot(num_plots, 1, 1)
    plot.plot(data['time'], data['velocity'])

    plot = fig.add_subplot(num_plots, 1, 2)
    plot.plot(data['time'], data['eff_v'])

    plot = fig.add_subplot(num_plots, 1, 3)
    plot.plot(data['time'], data['output'])

    plot = fig.add_subplot(num_plots, 1, 4)
    plot.plot(data['controller_time'], data['integrator'])

    plt.show()

