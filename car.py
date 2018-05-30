import numpy as np
from wheel import  Wheel
from scipy import optimize

class Car(object):
    def __init__(self):
        """

        """
        # engine rpm
        self._rpm = 100 #rev/min
        # vehicle mass
        self.mass = 1500 #kg
        # current longitudinal speed
        self.speed = 0 #m/s
        # current acceleration
        self.acceleration = 0 #m/s**2
        # current gear in the gearbox
        self.gear = 0 #pure
        # drag resistance
        self.drag_r = 0.29
        # current differential on the drive wheels
        self.differential_ratio = 3.42 #pure
        # cg height
        self.cg_height = 1.0 #m
        # wheelbase, distance between the front and rear wheels
        self.wheelbase = 1.0 #m
        # TODO efficiency
        # gears-engine scale factors
        self.gear_ratios = [
            (2.66),
            (1.78),
            (1.3),
            (1.),
            (0.74)
        ] #pure
        # curve for the rpm-torque relation in the engine
        self.engine_rpm_torque_curve_params = optimize.curve_fit(self.rpm_torque_function,
                                                          (100, 2000, 3000, 4000, 5000, 6000),
                                                          (290, 330, 340, 350, 350, 280),
                                                       p0=[-1, 1, 300])[0]
        #self.wheel_slip_longitudinal_force_curve_params = optimize.curve_fit(self.slip_longitudinal_force_function,
        #                                               (-100.1, -20, -10, 0, 10, 20, 100.1),
        #                                               (0, -4000, -6000, 0, 6000, 4000, 0),
        #                                               p0=[6000, 1e-3])[0]
        self.max_engine_rpm = 2000 #rev/min
        self.wheels = [# rear
                       Wheel(self.wheelbase * 0.5, drive=True),
                       Wheel(self.wheelbase * 0.5, drive=True),
                       # front
                       Wheel(self.wheelbase * 0.5, drive=False),
                       Wheel(self.wheelbase * 0.5, drive=False),
                       ]

    def get_engine_torque(self):
        """
        Get the torque on the rear axel from the engine
        :return:
        """
        engine_torque = self.rpm_torque_function(self.rpm, *self.engine_rpm_torque_curve_params)
        return engine_torque

    def get_traction_force(self, engine_torque, wheel:Wheel):
        """
        Get the longitudinal force for the wheel
        :param wheel:
        :return:
        """
        torque = self.get_traction_torque(engine_torque)
        force = torque / wheel.radius
        return force

    def get_traction_torque(self, engine_torque):
        """
        Get the torque on the axel
        :param wheel:
        :return:
        """
        axel_torque = engine_torque * self.gear_ratios[self.gear] * self.differential_ratio
        return axel_torque

    def get_total_weight(self):
        return 9.81 * self.get_total_mass()

    def get_total_mass(self):
        return self.mass + np.sum([w.mass for w in self.wheels])
    #def get_slip_ratio(self, wheel:Wheel):
    #    """
    #    Get the current sleep ratio for the wheel
    #    :return:
    #    """
    #    return (wheel.angular_speed*wheel.radius-self.speed)/self.speed

    def rpm_torque_function(self, x, a, b, c):
        return a*x**2 + b*x + c

    #def slip_longitudinal_force_function(self, x, a, b):
    #    return a * np.sin(b * x)
    def get_air_resistance(self):
        return self.drag_r * self.speed ** 2
    @property
    def rpm(self):
        return self._rpm
    @rpm.setter
    def rpm(self, rpm):
        self._rpm = rpm
        if self.rpm >= self.max_engine_rpm:
            if self.gear + 1 < len(self.gear_ratios):
                self.gear = self.gear + 1

