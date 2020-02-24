import UDPComms
from UDPComms import Subscriber
from UDPComms import Publisher

class Trajectory_Reciever:
    def __init__(self, port=8830):
        #port is definetly subject to change
        self.pi_subscriber = Subscriber(port)

    def get_trajectory(self):
        #NOTE: form of messaage: {x_vel: 5, y_vel: 3}
        try:
            msg = self.pi_subscriber.get()
            print ("recieved message: ", msg)
            return msg['ly'] * 0.5, msg['lx'] * -0.24
        except:
            print ("error getting the message")
            return 0, 0

class StayOrGo:
    #decide UDP port
    def __init__(self, port=3500):
        self.pi_publisher = Publisher(port)
        pass
    def send_value(self):
        self.pi_publisher.send({"go_val": })
        pass