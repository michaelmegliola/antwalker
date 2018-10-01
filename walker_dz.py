import time
import numpy as np

import rcpy
import rcpy.clock as clock
import rcpy.servo as servo
import rcpy.mpu9250 as mpu9250

HIP_POS = (-0.4, 0, 0.4)
ANKLE_POS = (-0.7, 0.0)

class Walker:

    def __init__(self):
        # Hips are servos 1 - 4 clockwise from the front right
        # Ankles are servos 5 -8 clockwise from the front right
        rcpy.set_state(rcpy.RUNNING)
        self.servos = [servo.Servo(srv) for srv in range(1,9)]
        self.clocks = [clock.Clock(srv, .02) for srv in self.servos]
        self.reset()

    def start(self):
        servo.enable()
        for i in self.clocks:
            i.start()
        mpu9250.initialize(enable_dmp = True, dmp_sample_rate = 100, enable_fusion = True, enable_magnetometer = True)

    def stop(self):
        for i in self.clocks:
            i.stop()
        servo.disable()

    def get_attitude(self):
        return np.multiply(mpu9250.read()['tb'], 57.29578)

    def discrete_theta(self, theta):
        if theta < -3.0:
            return int(-1)
        elif theta > 3.0:
            return int(1)
        else:
            return int(0)

    def get_discrete_attitude(self):
        xyz = self.get_attitude()
        print(xyz)
        return [self.discrete_theta(theta) for theta in xyz]

    def move_hip(self, joint, position):
        self.servos[joint].set(HIP_POS[position])

    def move_ankle(self, joint, position):
        self.servos[joint+4].set(ANKLE_POS[position])
    
    def twist(self):
        for i in range(4):
            self.move_hip(i, 0)
        time.sleep(1)
        print(self.get_attitude())
        time.sleep(1)
        for i in range(4):
            self.move_hip(i, 1)

    def reset(self):
        for i in range(4):
            self.move_hip(i,1)
            self.move_ankle(i,1)
        time.sleep(1)

    def sample(self):
        return np.random.randint(16)

    def step(self, action):
        z0 = self.get_attitude()[2]
        pos_fr = action & 0b0001 != 0
        pos_rr = action & 0b0010 != 0
        pos_rl = action & 0b0100 != 0
        pos_fl = action & 0b1000 != 0
        print(action, pos_fr, pos_rr, pos_rl, pos_fl)
        if pos_fr:
            self.move_hip(0, 0)
        else:
            self.move_hip(0, 2)
        if pos_rr:
            self.move_hip(1, 0)
        else:
            self.move_hip(1, 2)
        if pos_rl:
            self.move_hip(2, 0)
        else:
            self.move_hip(2, 2)
        if pos_fl:
            self.move_hip(3, 0)
        else:
            self.move_hip(3, 2)
        
        time.sleep(1)
        xyz = self.get_attitude()
        reward = abs(z0 - xyz[2])
        done = reward > 16.5
        return action, reward, done 

    def learn(self):
        alpha = 0.10
        explore = 1.0
        q = np.zeros((16,16))
        for n in range(100):
            w.reset()
            time.sleep(3)
            state = 0
            self.step(state)
            done = False
            while not done:
                if np.random.random() < explore:
                    action = self.sample()
                else:
                    action = np.argmax(q[state])
                obs, reward, done = self.step(action)
                q[state][obs] = alpha * q[state][obs] + (1 - alpha) * (reward + np.max(q[obs]))
                state = obs
                print(reward)
            explore *= .9
            explore = max(explore, .01)
            print(n, reward, np.sum(q), explore)

w = Walker()
w.start()
w.reset()
w.learn()