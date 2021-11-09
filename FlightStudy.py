import math as m
import numpy as np
import matplotlib.pyplot as pt

from FlightDynamics import FlightForces


class FlightSim:

    def __init__(self):
        self.flight_force1 = FlightForces()

    def velocity_vs_lift_force(self):

        pt.plot(self.flight_force1.velocity_H, self.flight_force1.LiftForce)
        pt.vlines(self.flight_force1.TakeOffSpeed, 0, 25000, colors='yellow')
        pt.hlines(self.flight_force1.WeightForce, 0, 40, colors='red')
        pt.xlabel("Velocity (m/s), " + str(self.flight_force1.TakeOffSpeed) + "m/s is typical for takeoff")
        pt.ylabel("Lift Force , " + str(self.flight_force1.WeightForce) + "N is required for takeoff")
        pt.show()
        print(self.flight_force1.WeightForce)


def main():

    flight_study1 = FlightSim()
    flight_study1.velocity_vs_lift_force()


if __name__ == "__main__":
    main()
