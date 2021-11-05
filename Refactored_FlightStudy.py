import math as m
import numpy as np
import matplotlib.pyplot as pt

'''
Henry notes:
    Refactored to create a class called Program that handles everything. Eventually you could even
    create two separate classes and have a FlightSim class , with a Program class that
    handles the interface between the plotting and the calculations.
'''

class Program:

    # These variables are not specific to the calculations but need to be encapsulated in the program class - aka class variables
    g = 9.81  # Acceleration of Gravity (m/s^2) 
    PressureAtSeaLevel = 101325  # Pressure at MSL (Pa)
    Rs = 287.052  # Specific Gas constant (J/kg*K)
    rho = 1.225  # density of ground level air (kg/m^3)
    T = 15  # Temperature at MSL (C)

    def __init__(self, RPMinput):
        self.RPM = RPMinput # only variable I saw that was variable per program - can update later though
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
        # self.RPM = 0  # Revolutions Per Minute
        self.Acceleration_H = 0  # Acceleration horizontal (m/s^2)
        self.ThrustForce = 0  # Force of Thrust acting on the aircraft (N)
        self.time = 0  # Time counter
        self.velocity = 0  # Aircraft's velocity
        self.altitude = 0  # Aircraft's altitude
        self.previous_velocity = 0  # Last calculation of the aircraft's velocity
        return None 

    def compute_thrust_force(self):
        thrust_force = \
            (4.392399*10**-8)*self.RPM*((self.PropellerDiameter**3.5)/m.sqrt(self.PropellerPitch)) * \
            ((4.23333*10**-4)*self.RPM*self.PropellerPitch - self.velocity)
        self.Acceleration_H = thrust_force / self.mass
        return None 

    def compute_velocity(self):
        self.velocity = self.previous_velocity + self.Acceleration_H * self.time  # Todo add wind effects of velocity
        return None 

    def compute_dynamic_pressure(self):
        air_pressure = self.PressureAtSeaLevel * (1 - (2.25577 * 10 ** -5) * self.altitude) ** 5.25588
        temperature = self.T - self.lam * self.altitude
        rho_adjusted = air_pressure / (self.Rs * temperature)
        self.DynamicPressure = (0.5 * rho_adjusted * self.velocity ** 2)
        return None 

    def compute_lift_force(self):
        coefficient_of_lift = (-1.72*10**-4)*(self.AngleOfAttack**3) - \
                            (3.34*10**-4)*(self.AngleOfAttack**2) + 0.129*self.AngleOfAttack + 0.237
        self.LiftForce = self.DynamicPressure * self.Area * coefficient_of_lift
        return None 

    # Note: The changes in the coefficient of drag due to altitude/velocity will be ignored
    # Cd at 0 m/s will be
    def compute_drag_force(self):
        #  coefficient_of_drag = TODO find Cd as a function of angle of attack
        #  DragForce = DynamicPressure * Area * coefficient_of_drag
        pass 

    
def main():
    program1 = Program(2700)
    while program1.time < 20:
        program1.compute_thrust_force()
        program1.compute_velocity()
        mph = program1.velocity * 2.236936
        print("RPM:" + str(program1.RPM) + "   Velocity: " + str(mph) + "   Time: " + str(program1.time) )
        program1.time += 0.1
        pass 
    return None 


if __name__ == "__main__":
    main()
