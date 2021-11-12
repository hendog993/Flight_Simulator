import math as m
import numpy as np
import matplotlib.pyplot as pt

from FlightDynamics import FlightForces


class FlightSim:

    def __init__(self, flight_forces):
        self.flight_force1 = flight_forces

    def velocity_vs_altitude(self):

        self.flight_force1.time[self.flight_force1.program_size-1] = self.flight_force1.time[self.flight_force1.program_size-2]
        self.flight_force1.altitude[self.flight_force1.program_size - 1] = self.flight_force1.altitude[
            self.flight_force1.program_size - 2]
        pt.plot(self.flight_force1.time, self.flight_force1.altitude)
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
