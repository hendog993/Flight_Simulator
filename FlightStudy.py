import math as m
import numpy as np
import matplotlib.pyplot as pt

from FlightDynamics import FlightForces


class FlightSim:

    def __init__(self, flight_forces):
        self.flight_force1 = flight_forces

    def velocity_vs_altitude(self):

        pt.plot(self.flight_force1.velocity_H, 'bo', self.flight_force1.altitude)
        fig, ax = pt.subplots()
        ax.plot(self.flight_force1.time, self.flight_force1.velocity_H)
        ax.plot(self.flight_force1.time, self.flight_force1.altitude)
        pt.xlabel("Velocity (m/s) ")
        pt.ylabel("Altitude (m) ")
        pt.show()


def main():

    flight_force1 = FlightForces()
    flight_study1 = FlightSim(flight_force1)
    flight_force1.test_sequence1(2700)
    flight_study1.velocity_vs_altitude()


if __name__ == "__main__":
    main()
