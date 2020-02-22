import UDPComms
from UDPComms import Subscriber


class Trajectory_Reciever:
    def __init__(port=8830):
        #port is definetly subject to change
        self.pi_subscriber = Subscriber(port)

    def get_trajectory():
        #NOTE: form of messaage: {x_vel: 5, y_vel: 3}
        try:
            msg = self.pi_subscriber.get()
            print ("recieved message: ", msg)
            return msg['x_vel'], msg.['y_vel']
        except:
            print ("error getting the message")
