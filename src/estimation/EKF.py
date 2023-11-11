#! /usr/bin/python3

class EKF:
    def __init__(self, dt, z, x0, P0, v, w, gt, acc, yaw, states):
        self.dt = dt
        self.z = z
        self.x0 = x0
        self.P0 = P0
        self.v = v
        self.w = w
        self.gt = gt
        self.acc = acc
        self.yaw = yaw
        self.states = states

    def PDR_linearized(self):
        print("Fk")

    def noise_linearized(self):
        print("Gk")

    def measurement_linearized(self):
        print("Hk")

    def predict(self):
        print("predict")

    def update(self):
        print("update")

    def evaluate(self):
        print("evaluated")

if __name__ == "__main__":
    
    foo = EKF(0,0,0,0,0,0,0,0,0,0)
    foo.evaluate()