import math as m
import numpy as np
import matplotlib.pyplot as pt

TakeOffSpeed = 30  # (m/s)
mass = 950  # Weight of Cessna 172 (kg)
g = 9.81  # Acceleration of Gravity (m/s^2) \
# NOTE: Differences in gravitational acceleration at various altitudes will be ignored
WingSpan = 11  # Length of wings (m)
Chord = 1.56  # Chord Length (m)
T = 15  # Temperature at MSL (C)
PressureAtSeaLevel = 101325  # Pressure at MSL (Pa)
Rs = 287.052  # Specific Gas constant (J/kg*K)
rho = 1.225  # density of ground level air (kg/m^3)
WeightForce = g * mass  # W = mg (N) WeightForce < Lift Force = LiftOff
DynamicPressure = 0  # Dynamic Pressure acting on the wings of the aircraft
LiftForce = 0  # Initial Lift force acting on the wings (N)
LiftOffForce = WeightForce
Area = WingSpan * Chord  # Wing Area (m^2)
AngleOfAttack = 14  # Angle of Attack
lam = 0.0098  # Lapse rate (DegreeC/m)
PropellerPitch = 53  # Pitch angle of the aircraft's propeller (degrees)
PropellerDiameter = 76  # Propeller Diameter of Cessna 172 (inches) 1.9304 meters
RPM_max = 2700  #
RPM = 0  # Revolutions Per Minute
Acceleration_H = 0  # Acceleration horizontal (m/s^2)
ThrustForce = 0  # Force of Thrust acting on the aircraft (N)
time = 0  # Time counter
velocity = 0  # Aircraft's velocity
altitude = 0  # Aircraft's altitude
previous_velocity = 0  # Last calculation of the aircraft's velocity


def compute_thrust_force():
    global ThrustForce
    thrust_force = \
        (4.392399*10**-8)*RPM*((PropellerDiameter**3.5)/m.sqrt(PropellerPitch)) * \
        ((4.23333*10**-4)*RPM*PropellerPitch - velocity)

    global Acceleration_H
    Acceleration_H = thrust_force / mass


def compute_velocity():
    global Acceleration_H
    global velocity
    velocity = previous_velocity + Acceleration_H * time  # Todo add wind effects of velocity


def compute_dynamic_pressure():
    global DynamicPressure
    global altitude
    global PressureAtSeaLevel
    air_pressure = PressureAtSeaLevel * (1 - (2.25577 * 10 ** -5) * altitude) ** 5.25588
    temperature = T - lam * altitude
    rho_adjusted = air_pressure / (Rs * temperature)
    DynamicPressure = (0.5 * rho_adjusted * velocity ** 2)


def compute_lift_force():
    global DynamicPressure
    global AngleOfAttack
    global LiftForce
    coefficient_of_lift = (-1.72*10**-4)*(AngleOfAttack**3) - \
                          (3.34*10**-4)*(AngleOfAttack**2) + 0.129*AngleOfAttack + 0.237
    LiftForce = DynamicPressure * Area * coefficient_of_lift


# Note: The changes in the coefficient of drag due to altitude/velocity will be ignored
# Cd at 0 m/s will be
def compute_drag_force():
    global AngleOfAttack
    global DynamicPressure
    #  coefficient_of_drag = TODO find Cd as a function of angle of attack
    #  DragForce = DynamicPressure * Area * coefficient_of_drag
RPM = 2700

while time < 20:
    compute_thrust_force()
    compute_velocity()
    mph = velocity * 2.236936
    print("RPM:" + str(RPM))
    print("Velocity: " + str(mph))
    print("Time: " + str(time))
    time += 0.1


#
# pt.plot(velocity, LiftForce)
# pt.vlines(TakeOffSpeed, 0, 25000, colors='yellow')
# pt.hlines(WeightForce, 0, 40, colors='red')
# pt.xlabel("Velocity (m/s), " + str(TakeOffSpeed) + "m/s is typical for takeoff")
# pt.ylabel("Lift Force , " + str(WeightForce) + "N is required for takeoff")
# pt.show()
# print(WeightForce)







