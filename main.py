import numpy as np
from math import degrees, radians
from car import Car
from matplotlib import pyplot as plt

car = Car()
timestep = 5e-3

# TODO NO SLIP RATIO FOR NOW
wheel_speeds = []
car_speeds = []
engine_rpm = []
engine_torques = []
force_pos = []
force_neg = []
wheel_torques = []
wheel_rpm = []
gears = []
rear_weight = []
front_weight = []
wheel_speeds_inertia = []

steps = 66500

for i in range(steps):
    total_force = 0
    total_resistance_force = 0
    # get the current torque
    engine_torque = car.get_engine_torque()
    #if i > 3000:
    #    engine_torque = 0
    # iterate over the wheels to get the force on the car
    for wheel in car.wheels:
        if wheel.drive:
            wheel_force = car.get_traction_force(engine_torque, wheel)
            # the wheel could be spinning
            # get the weight on the wheel
            max_traction = wheel.get_max_traction(wheel.get_weight_on_wheel(car))
            rear_weight.append(wheel.get_weight_on_wheel(car))
            # todo handle spin
            total_force += wheel_force
        # apply rolling resistance
        total_resistance_force += wheel.get_rolling_force(car.speed)
        if not wheel.drive:
            front_weight.append(wheel.get_weight_on_wheel(car))
    # apply air resistance just one time
    total_resistance_force += car.get_air_resistance()
    # these forces can't move the car if it is still
    total_force -= total_resistance_force
    # newton second law
    car.acceleration = total_force / car.get_total_mass()
    # euler numeric integration
    car.speed += car.acceleration * timestep
    # compute angular speed from the car speed
    # todo slipping
    for wheel in car.wheels:
        wheel.angular_speed = car.speed / wheel.radius
    # now use one wheel to compute rpm, does not matter which as they are all the same in terms of speed
    car.rpm = car.wheels[0].angular_speed * car.gear_ratios[car.gear] * car.differential_ratio * 60 / (2*np.pi)
    # graphs
    car_speeds.append(car.speed)
    wheel_speeds.append(car.wheels[0].angular_speed)
    engine_rpm.append(car.rpm)
    engine_torques.append(engine_torque)
    force_pos.append(total_force)
    gears.append(car.gear)
    force_neg.append(total_resistance_force)
    wheel_rpm.append(wheel.angular_speed*60/(2*np.pi))

fig, ax = plt.subplots(ncols=8)
ax[0].plot(range(steps), car_speeds)
ax[0].set_title('car_speed')
ax[1].plot(range(steps), wheel_speeds)
ax[1].set_title('wheel_speed')
ax[2].plot(range(steps), engine_rpm)
ax[2].set_title('car_rpm')
ax[3].plot(range(steps), wheel_rpm)
ax[3].set_title('wheel_rpm')
ax[4].plot(range(steps), engine_torques)
ax[4].set_title('car_torque')
ax[5].plot(range(steps), force_pos)
ax[5].set_title('total_forces')
ax[6].plot(range(steps), force_neg)
ax[6].set_title('constrast_forces')
ax[7].plot(range(steps), gears)
ax[7].set_title('current gear')
plt.show()