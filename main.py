import numpy as np
from math import degrees, radians
from car import Car
from matplotlib import pyplot as plt
from matplotlib import gridspec

car = Car()
timestep = 1e-2

# TODO NO SLIP RATIO FOR NOW

steps = 5000
mu_s = 0.7
mu_k = 0.1
car.thrust = 1
for i in range(steps):
    if i == 1000:
        car.thrust = 0
    total_force = 0
    total_resistance_force = 0
    # get the current torque
    engine_torque = car.get_engine_torque()
    car.vis_engine_torques.append(engine_torque)
    # iterate over the wheels to get the force on the car
    for wheel in car.wheels:
        if wheel.drive:
            torque = car.get_traction_torque(engine_torque)
            torque -= wheel.get_rolling_force(car) * wheel.radius
            # get max torque friction
            max_torque_handle = car.get_weight_on_wheel() * mu_s * wheel.radius
            total_resistance_force += wheel.get_rolling_force(car)
            wheel.torque = torque

            wheel.vis_torque.append(torque)

            if torque <= max_torque_handle:
                # all torque is transmitted
                wheel.spinning = False
                total_force += torque / wheel.radius
            else:
                # kinetic force
                max_torque_handle = car.get_weight_on_wheel() * mu_k * wheel.radius
                total_force += max_torque_handle / wheel.radius
                wheel.spinning = True
        else:
            # TODO wheel lock on ice and other things
            # TODO wheel is pushed and will rotate only if friction is high enough
            wheel.vis_torque.append(0)
            wheel.spinning = False

    # apply air resistance just one time
    total_resistance_force += car.get_air_resistance()
    # these forces can't move the car if it is still
    total_force -= total_resistance_force
    # newton second law
    car.acceleration = total_force / car.get_total_mass()
    # euler numeric integration
    car.speed += car.acceleration * timestep
    # iterate again over wheels to get acc speed
    for wheel in car.wheels:
        if not wheel.spinning:
            # wheel is connected to the car
            # if wheel speed is different from the correct one move toward it
            angular_acc = (car.speed / wheel.radius - wheel.angular_speed) / (2)
            # get car speed
        else:
            # wheel is not fully connected
            max_torque_handle = car.get_weight_on_wheel() * mu_s * wheel.radius
            angular_acc = car.acceleration / wheel.radius + (wheel.torque - max_torque_handle) / (wheel.get_inertia() *2)
        wheel.angular_speed += angular_acc * timestep
        # graph for rpm and spinning
        wheel.vis_rpm.append(wheel.angular_speed * 60 / (2*np.pi))
        wheel.vis_spinning.append(1 if wheel.spinning else 0)

    drive_wheel = [w for w in car.wheels if w.drive][0]
    car.rpm = drive_wheel.angular_speed * car.gear_ratios[car.gear] * car.final_ratio * 60 / (2 * np.pi)
    # graphs
    car.vis_speed.append(car.speed * 3.6)
    car.vis_forces.append(total_force)
    car.vis_res_forces.append(total_resistance_force)
    car.vis_engine_rpm.append(car.rpm)
    car.vis_gears.append(car.gear+1)

 # PREPARE SUBPLOTS
f = plt.figure()
f.suptitle('CAR')
els = [car.vis_speed, car.vis_gears, car.vis_engine_rpm, car.vis_forces, car.vis_res_forces, car.vis_engine_torques]
names = ['speed', 'gear', 'engine_RPM', 'total force', 'resistance force', 'engine torque']
gs = gridspec.GridSpec(1, len(els))
for i, (el, name) in enumerate(zip(els, names)):
    p = plt.subplot(gs[0, i])
    p.set_title(name)
    p.plot(range(steps), el)

f_1 = plt.figure()
f_1.suptitle('wheels')
gs = gridspec.GridSpec(len(car.wheels), 3)
for i, wheel in enumerate(car.wheels):
    els = [wheel.vis_rpm, wheel.vis_spinning, wheel.vis_torque]
    names = ['wheel_RPM', 'is_spinning', 'torque']
    for j, (el, name) in enumerate(zip(els, names)):
        p = plt.subplot(gs[i, j])
        p.set_title(name)
        p.plot(range(steps), el)
plt.show()