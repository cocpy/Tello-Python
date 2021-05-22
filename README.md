# Tello-Python

Control DJI Tello drone with python

# Installation

    pip install tello-python

# Examples


from tello import tello


drone = tello.Tello()


drone.takeoff()


drone.forward(100)


drone.cw(90)


drone.flip('l')


drone.streamon()


drone.land()

# For more commands, please refer to the methods and comments in the source code hello.py file