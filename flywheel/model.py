#!/usr/bin/env python

import numpy as np

import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook

MAX_VOLTAGE = 12.0 # volts
FREE_SPIN_SPEED = 6000.0 # RPM

STALL_FORCE = 8.520 # N-m
WHEEL_INERTIA = 1040e-6 # Kg-m

BACK_EMF = MAX_VOLTAGE / FREE_SPIN_SPEED

BALL_INERTIA = 479e-6 # Kg-m
BALL_CONTACT_DISTANCE = 0.2 # fraction of a rotation

def simulationLoop(controller_func,
                   ordered_ball_times,
                   simulation_step_time,
                   sim_steps_per_controller_step,
                   total_time):
    controller_step_time = simulation_step_time * sim_steps_per_controller_step
    num_sim_steps = int(total_time / simulation_step_time)
    num_controller_steps = int(num_sim_steps / sim_steps_per_controller_step)
    times = np.arange(0, total_time, simulation_step_time)

    # system state
    wheel_velocity = 0 # rpm
    # logged values
    log_values = {
        "time": times,
        "velocity": [0.0] * num_sim_steps,
        "eff_v": [0.0] * num_sim_steps,
        "output": [0.0] * num_sim_steps,
        "system_inertia":  [0.0] * num_sim_steps,
        "ball_fire_speed": [0.0] * num_sim_steps,
        "energy": [0.0] * num_sim_steps,

        "controller_time": np.arange(0, total_time, controller_step_time),
        "integrator": [0.0] * num_controller_steps,
        "setpoint": [0.0] * num_controller_steps,
    }

    def step_num(i, j):
        return i * sim_steps_per_controller_step + j

    next_ball_idx = 0
    ball_distances = []
    def systemInertia():
        return WHEEL_INERTIA + BALL_INERTIA * len(ball_distances)

    for i in range(num_controller_steps):
        output_voltage = max(-12.0,
                             min(12.0,
                                 controller_func(wheel_velocity,
                                                 controller_step_time,
                                                 log_values,
                                                 i)))

        for j in range(sim_steps_per_controller_step):
            cur_time = times[step_num(i, j)]
            ball_fire_speed = 0.0
            if next_ball_idx < len(ordered_ball_times) \
               and ordered_ball_times[next_ball_idx] < cur_time:
                next_ball_idx += 1
                system_momentum = systemInertia() * wheel_velocity
                ball_distances.append(0.0)
                wheel_velocity = system_momentum / systemInertia()
            for k in range(len(ball_distances)):
                ball_distances[k] += wheel_velocity / 60.0 * simulation_step_time
            if len(ball_distances) and ball_distances[0] > BALL_CONTACT_DISTANCE:
                ball_distances.pop(0)
                ball_fire_speed = wheel_velocity

            effective_voltage = output_voltage - (wheel_velocity * BACK_EMF)
            force = effective_voltage / MAX_VOLTAGE * STALL_FORCE
            wheel_velocity += force / systemInertia() * simulation_step_time * 60.0 / (2.0 * np.pi)
            velocity_rad_per_sec = wheel_velocity * 2.0 * np.pi / 60.0
            log_values["velocity"][step_num(i, j)] = wheel_velocity
            log_values["eff_v"][step_num(i, j)] = effective_voltage
            log_values["output"][step_num(i, j)] = output_voltage
            log_values["system_inertia"][step_num(i, j)] = systemInertia()
            log_values["ball_fire_speed"][step_num(i, j)] = ball_fire_speed
            log_values["energy"][step_num(i, j)] = velocity_rad_per_sec * velocity_rad_per_sec * WHEEL_INERTIA
    return log_values

def makeKifController():
    setpoint = 3000.0

    kFv = MAX_VOLTAGE / FREE_SPIN_SPEED
    kP = 12.0 / 3000.0
    kI = 0.0001

    integrator = 0.0
    def controllerFunc(speed, delta_time, log_values, log_idx):
        nonlocal integrator
        error = setpoint - speed
        integrator += error * delta_time
        log_values["integrator"][log_idx] = integrator
        log_values["setpoint"][log_idx] = setpoint
        return kP * error + kI * integrator + kFv * setpoint
    return controllerFunc

def makeBangBangController():
    setpoint = 3000.0
    high_bang_output = MAX_VOLTAGE
    low_bang_output = MAX_VOLTAGE * setpoint / FREE_SPIN_SPEED

    def controllerFunc(speed, delta_time, log_values, log_idx):
        log_values["setpoint"][log_idx] = setpoint
        if speed < setpoint:
            return high_bang_output
        else:
            return low_bang_output
    return controllerFunc

def makeLaggingBangBangController():
    # controller consts
    setpoint = 3000.0
    high_bang_output = MAX_VOLTAGE
    low_bang_output = MAX_VOLTAGE * setpoint / FREE_SPIN_SPEED
    under_speed_time_threshold = 0.1

    # controller state
    under_speed_time = 0.0

    def controllerFunc(speed, delta_time, log_values, log_idx):
        log_values["setpoint"][log_idx] = setpoint
        nonlocal under_speed_time
        if speed >= setpoint:
            under_speed_time = 0.0
            return low_bang_output
        under_speed_time += delta_time
        if under_speed_time > under_speed_time_threshold:
            return high_bang_output
        else:
            return low_bang_output
    return controllerFunc

if __name__ == '__main__':
    data = simulationLoop(makeLaggingBangBangController(),
                          [10.0, 11.001, 12.002, 13.003, 14.004, 15.005],
                          0.0001, 50, 30)
    # data = simulationLoop(makeBangBangController(), [], 0.0001, 50, 30)
    fig = plt.figure(figsize=(8, 8))

    num_plots = 6
    plot = fig.add_subplot(num_plots, 1, 1, ylabel="velocity rpm")
    plot.plot(data['time'], data['velocity'])
    plot.plot(data['controller_time'], data['setpoint'])

    plot = fig.add_subplot(num_plots, 1, 2, ylabel="output")
    plot.plot(data['time'], data['output'])

    plot = fig.add_subplot(num_plots, 1, 3, ylabel="energy")
    plot.plot(data['time'], data['energy'])

    plot = fig.add_subplot(num_plots, 1, 4, ylabel="error integrator")
    plot.plot(data['controller_time'], data['integrator'])

    plot = fig.add_subplot(num_plots, 1, 5, ylabel="system inertia")
    plot.plot(data['time'], data['system_inertia'])

    plot = fig.add_subplot(num_plots, 1, 6, ylabel="ball fire speed")
    plot.plot(data['time'], data['ball_fire_speed'])

    plt.show()

