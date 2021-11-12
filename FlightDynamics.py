import math as m
import numpy as np
#import matplotlib.pyplot as pt

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
    lam = 0.0098  # Lapse rate (DegreeC/m)
    T = 15  # Temperature at MSL (C)
    PropellerPitch = 53  # Pitch angle of the aircraft's propeller (degrees)
    PropellerDiameter = 76  # Propeller Diameter of Cessna 172 (inches) 1.9304 meters
    mass = 950  # Weight of Cessna 172 (kg)
    time_step = 0.1
    program_step = 1
    RPM_max = 2700  # MAX RPM
    Chord = 1.56  # Chord Length (m)
    WingSpan = 11  # Length of wings (m)
    Area = WingSpan * Chord  # Wing Area (m^2)
    WeightForce = g * mass  # W = mg (N) WeightForce < Lift Force = LiftOff (acting downward in - direction)
    NormalForce_at_GroundLevel = - WeightForce  # The normal force of the ground acting on the aircraft (flip signs)
    GroundLevel = 0.0  # Altitude value in meters on the ground (m)
    LiftOffForce = WeightForce
    program_counter = 0
    program_size = 100  # Iterations of Calculation

    def __init__(self):
        self.RPM = np.array([], dtype=float)  # only variable I saw that was variable per program - can update later though
        # NOTE: Differences in gravitational acceleration at various altitudes will be ignored
        self.DynamicPressure = np.zeros(self.program_size, dtype=float)  # Dynamic Pressure acting on the wings of the aircraft
        self.LiftForce = np.zeros(self.program_size, dtype=float)  # Initial Lift force acting on the wings (N)
        self.AngleOfAttack = np.zeros(self.program_size, dtype=float)  # Angle of Attack
        self.Acceleration_H = np.zeros(self.program_size, dtype=float)  # Acceleration horizontal (m/s^2)
        self.Acceleration_V = np.zeros(self.program_size, dtype=float)  # Acceleration vertical (m/s^2)
        self.ThrustForce = np.zeros(self.program_size, dtype=float)  # Force of Thrust acting on the aircraft (N)
        self.DragForce = np.zeros(self.program_size, dtype=float)  # Force of Drag acting on the aircraft (N)
        self.time = np.zeros(self.program_size, dtype=float)  # Time counter
        self.velocity_H = np.zeros(self.program_size, dtype=float)  # Aircraft's horizontal velocity
        self.velocity_V = np.zeros(self.program_size, dtype=float)  # Aircraft's vertical velocity
        self.previous_altitude = np.zeros(self.program_size, dtype=float)  # Aircraft's previous altitude
        self.altitude = np.zeros(self.program_size, dtype=float)  # Aircraft's altitude
        self.change_in_altitude = np.zeros(self.program_size, dtype=float)
        self.previous_velocity_H = np.zeros(self.program_size, dtype=float)  # Last calculation of the aircraft's \
        # velocity in the Horizontal Direction
        self.previous_velocity_V = np.zeros(self.program_size, dtype=float)  # Last calculation of the aircraft's \
        # velocity in the Vertical Direction

    def compute_thrust_force(self, x):
        self.ThrustForce[x] = \
            (4.392399 * 10 ** -8) * self.RPM[x] * ((self.PropellerDiameter ** 3.5) / m.sqrt(self.PropellerPitch)) * \
            ((4.23333 * 10 ** -4) * self.RPM[x] * self.PropellerPitch - self.velocity_H[x])
        self.Acceleration_H[x] = self.ThrustForce[x] / self.mass
        return None

    def compute_velocity_h(self, x):
        self.velocity_H[x] = self.previous_velocity_H[x] + self.Acceleration_H[x] * self.time_step  # Todo add wind effects
        if self.program_counter == self.program_size:
            self.previous_velocity_H[x] = self.velocity_H[x]
        else:
            self.previous_velocity_H[x + 1] = self.velocity_H[x]
        return None

    def compute_dynamic_pressure(self, x):
        air_pressure = self.PressureAtSeaLevel * (1 - (2.25577 * 10 ** -5) * self.altitude[x]) ** 5.25588
        temperature = self.T - self.lam * self.altitude[x]
        rho_adjusted = air_pressure / (self.Rs * temperature)
        self.DynamicPressure[x] = (0.5 * rho_adjusted * self.velocity_H[x] ** 2)
        return None

    def compute_lift_force(self, x):
        coefficient_of_lift = (-1.72 * 10 ** -4) * (self.AngleOfAttack[x] ** 3) - \
                              (3.34 * 10 ** -4) * (self.AngleOfAttack[x] ** 2) + 0.129 * self.AngleOfAttack[x] + 0.237
        self.LiftForce[x] = self.DynamicPressure[x] * self.Area * coefficient_of_lift
        return None

    # Note: The changes in the coefficient of drag due to altitude/velocity will be ignored
    # Cd at 0 m/s will be 0
    def compute_drag_force(self, x):
        if self.velocity_H[x] == 0:
            coefficient_of_drag = 0
        else:
            coefficient_of_drag = (-3.61 * 10 ** -9) * (self.AngleOfAttack[x] ** 6) + (2.9 * 10 * -8) * (
                                  self.AngleOfAttack[x] ** 5) + \
                                  (3.12 * 10 ** -7) * (self.AngleOfAttack[x] ** 4) - (1.83 * 10 ** -5) * (
                                              self.AngleOfAttack[x] ** 3) + \
                                  (8.78 * 10 ** -4) * (self.AngleOfAttack[x] ** 2) + (
                                              4.38 * 10 ** -3) * self.AngleOfAttack[x] + 0.0169
        self.DragForce[x] = self.DynamicPressure[x] * self.Area * coefficient_of_drag
        return None

    def compute_altitude(self, x):
        if self.LiftForce[x] > self.WeightForce:
            self.Acceleration_V[x] = (self.LiftForce[x] - self.WeightForce) / self.mass
            self.change_in_altitude[x] = self.velocity_V[x] * self.time_step + (0.5 * self.Acceleration_V[x] * self.time_step**2)
            self.altitude[x] = self.previous_altitude[x] + self.change_in_altitude[x]

        elif self.LiftForce[x] < self.WeightForce and self.altitude[x] == self.GroundLevel:
            self.Acceleration_V[x] = 0.0

        elif self.LiftForce[x] < self.WeightForce:
            self.Acceleration_V[x] = (self.WeightForce - self.LiftForce[x]) / self.mass
            self.change_in_altitude[x] = self.velocity_V[x] * self.time_step + (0.5 * self.Acceleration_V[x] * self.time_step**2)
            self.altitude[x] = self.previous_altitude[x] - self.change_in_altitude[x]

        elif self.LiftForce[x] == self.WeightForce:
            self.Acceleration_V[x] = 0.0
            self.altitude[x] = self.previous_altitude[x]

        self.previous_altitude[x+1] = self.altitude[x]

    def compute_velocity_v(self, x):
        self.velocity_V[x] = self.previous_velocity_V[x] + self.Acceleration_V[x] * self.time_step
        self.previous_velocity_V[x+1] = self.velocity_V[x]
        return None

    def test_sequence1(self, rpm):

        #  Initialize variables
        self.RPM = np.full([self.program_size], rpm)
        self.DynamicPressure[0] = 0.0
        self.LiftForce[0] = 0.0
        self.AngleOfAttack[0] = 0.0
        self.Acceleration_H[0] = 0.0
        self.Acceleration_V[0] = 0.0
        self.ThrustForce[0] = 0.0
        self.DragForce[0] = 0.0
        self.time[0] = 0.0
        self.velocity_H[0] = 0.0
        self.velocity_V[0] = 0.0
        self.previous_altitude[0] = 0.0
        self.altitude[0] = 0.0
        self.change_in_altitude[0] = 0.0
        self.previous_velocity_H[0] = 0.0
        self.previous_velocity_V[0] = 0.0

        while self.program_counter < self.program_size-1:
            self.compute_all_forces(self.program_counter)

            # If the program counter = 0 , then time=0 , else time should increase by time_step
            if self.program_counter == 0:
                self.time[self.program_counter] = 0
            else:
                self.time[self.program_counter] = self.time_step + self.time[self.program_counter - 1]
            self.program_counter += self.program_step

    def compute_all_forces(self, x):
        self.compute_thrust_force(x)
        self.compute_velocity_h(x)
        self.compute_dynamic_pressure(x)
        self.compute_lift_force(x)
        self.compute_drag_force(x)
        self.compute_altitude(x)
        self.compute_velocity_v(x)




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
