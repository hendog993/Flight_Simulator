import math as m
import numpy as np
import matplotlib.pyplot as pt

'''
Henry notes:
    Refactored to create a class called Program that handles everything. Eventually you could even
    create two separate classes and have a FlightSim class , with a Program class that
    handles the interface between the plotting and the calculations.
'''


class FlightForces:
    # These variables are not specific to the calculations but need to be encapsulated in the program class
    # - aka class variables
    g = 9.81  # Acceleration of Gravity (m/s^2) 
    PressureAtSeaLevel = 101325  # Pressure at MSL (Pa)
    Rs = 287.052  # Specific Gas constant (J/kg*K)
    rho = 1.225  # density of ground level air (kg/m^3)
    T = 15  # Temperature at MSL (C)

    def __init__(self):
        self.RPM = 0  # only variable I saw that was variable per program - can update later though
        self.TakeOffSpeed = 30  # (m/s)
        self.mass = 950  # Weight of Cessna 172 (kg)
        # NOTE: Differences in gravitational acceleration at various altitudes will be ignored
        self.WingSpan = 11  # Length of wings (m)
        self.Chord = 1.56  # Chord Length (m)
        self.WeightForce = self.g * self.mass  # W = mg (N) WeightForce < Lift Force = LiftOff
        self.DynamicPressure = 0  # Dynamic Pressure acting on the wings of the aircraft
        self.LiftForce = 0  # Initial Lift force acting on the wings (N)
        self.LiftOffForce = self.WeightForce
        self.Area = self.WingSpan * self.Chord  # Wing Area (m^2)
        self.AngleOfAttack = 14  # Angle of Attack
        self.lam = 0.0098  # Lapse rate (DegreeC/m)
        self.PropellerPitch = 53  # Pitch angle of the aircraft's propeller (degrees)
        self.PropellerDiameter = 76  # Propeller Diameter of Cessna 172 (inches) 1.9304 meters
        self.RPM_max = 2700  #
        self.Acceleration_H = 0  # Acceleration horizontal (m/s^2)
        self.Acceleration_V = 0  # Acceleration vertical (m/s^2)
        self.ThrustForce = 0  # Force of Thrust acting on the aircraft (N)
        self.DragForce = 0  # Force of Drag acting on the aircraft (N)
        self.time = 0  # Time counter
        self.velocity_H = 0  # Aircraft's horizontal velocity
        self.velocity_V = 0  # Aircraft's vertical velocity
        self.previous_altitude = 0  # Aircraft's previous altitude
        self.altitude = 0  # Aircraft's altitude
        self.previous_velocity_H = 0  # Last calculation of the aircraft's velocity in the Horizontal Direction
        self.previous_velocity_V = 0  # Last calculation of the aircraft's velocity in the Vertical Direction

    def compute_thrust_force(self):
        thrust_force = \
            (4.392399 * 10 ** -8) * self.RPM * ((self.PropellerDiameter ** 3.5) / m.sqrt(self.PropellerPitch)) * \
            ((4.23333 * 10 ** -4) * self.RPM * self.PropellerPitch - self.velocity_H)
        self.Acceleration_H = thrust_force / self.mass
        return None

    def compute_velocity(self):
        self.velocity_H = self.previous_velocity_H + self.Acceleration_H * self.time  # Todo add wind effects
        self.velocity_V = self.previous_velocity_V + self.Acceleration_V * self.time
        self.previous_velocity_V = self.velocity_V
        self.previous_velocity_H = self.velocity_H
        return None

    def compute_altitude(self):
        if self.LiftForce > self.WeightForce:
            Acceleration_V = (self.LiftForce - self.WeightForce) / self.mass
            change_in_altitude = self.velocity_V * self.time + (0.5 * Acceleration_V * self.time**2)
            self.altitude = self.previous_altitude + change_in_altitude

        elif self.LiftForce < self.WeightForce:
            Acceleration_V = (self.WeightForce - self.LiftForce) / self.mass
            change_in_altitude = self.velocity_V * self.time + (0.5 * Acceleration_V * self.time**2)
            self.altitude = self.previous_altitude - change_in_altitude

        else:
            Acceleration_V = 0
            altitude = self.previous_altitude

        altitude = self.previous_altitude + self.Acceleration_V * self.time

    def compute_dynamic_pressure(self):
        air_pressure = self.PressureAtSeaLevel * (1 - (2.25577 * 10 ** -5) * self.altitude) ** 5.25588
        temperature = self.T - self.lam * self.altitude
        rho_adjusted = air_pressure / (self.Rs * temperature)
        self.DynamicPressure = (0.5 * rho_adjusted * self.velocity_H ** 2)
        return None

    def compute_lift_force(self):
        coefficient_of_lift = (-1.72 * 10 ** -4) * (self.AngleOfAttack ** 3) - \
                              (3.34 * 10 ** -4) * (self.AngleOfAttack ** 2) + 0.129 * self.AngleOfAttack + 0.237
        self.LiftForce = self.DynamicPressure * self.Area * coefficient_of_lift
        return None

    # Note: The changes in the coefficient of drag due to altitude/velocity will be ignored
    # Cd at 0 m/s will be 0
    def compute_drag_force(self):
        if self.velocity_H == 0:
            coefficient_of_drag = 0
        else:
            coefficient_of_drag = (-3.61 * 10 ** -9) * (self.AngleOfAttack ** 6) + (2.9 * 10 * -8) * (
                                  self.AngleOfAttack ** 5) + \
                                  (3.12 * 10 ** -7) * (self.AngleOfAttack ** 4) - (1.83 * 10 ** -5) * (
                                              self.AngleOfAttack ** 3) + \
                                  (8.78 * 10 ** -4) * (self.AngleOfAttack ** 2) + (
                                              4.38 * 10 ** -3) * self.AngleOfAttack + 0.0169
        self.DragForce = self.DynamicPressure * self.Area * coefficient_of_drag
        return None

    def compute_all_forces(self):
        self.compute_thrust_force()
        self.compute_velocity()
        self.compute_altitude()
        self.compute_dynamic_pressure()
        self.compute_lift_force()
        self.compute_drag_force()


def main():
    program1 = FlightForces()
    while program1.time < 20:
        program1.compute_thrust_force()
        program1.compute_velocity()
        mph = program1.velocity_H * 2.236936
        print("RPM:" + str(program1.RPM) + "   Velocity: " + str(mph) + "   Time: " + str(program1.time))
        program1.time += 0.1
        pass
    return None


if __name__ == "__main__":
    main()
