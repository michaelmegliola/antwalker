import time
import numpy as np

import rcpy
import rcpy.clock as clock
import rcpy.servo as servo
import rcpy.mpu9250 as mpu9250

HIP_POS = (-0.4, 0.4)
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
        mpu9250.initialize(enable_dmp = False, dmp_sample_rate = 100, enable_fusion = False, enable_magnetometer = True)

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
        return [self.discrete_theta(theta) for theta in xyz]

    def move_hip(self, joint, position):
        self.servos[joint].set(HIP_POS[position])

    def move_ankle(self, joint, position):
        self.servos[joint+4].set(ANKLE_POS[position])
    
    def reset(self):
        for i in range(4):
            self.move_hip(i,1)
            self.move_ankle(i,1)
        time.sleep(1)

    def sample(self):
        return np.random.randint(16) * 16

    def get_step(self, hips, ankles):
        action = 0
        for n in range(4):
            action += hips[n] * 2**(n+4)
            action += ankles[n] * 2**n
        return action

    def step(self, action):
        
        #file = open('capture.csv','w')
        
        hip_action = action // 16
        pos_fr = hip_action & 0b0001 != 0
        pos_rr = hip_action & 0b0010 != 0
        pos_rl = hip_action & 0b0100 != 0
        pos_fl = hip_action & 0b1000 != 0

        if pos_fr:
            self.move_hip(0, 0)
        else:
            self.move_hip(0, 1)
        if pos_rr:
            self.move_hip(1, 0)
        else:
            self.move_hip(1, 1)
        if pos_rl:
            self.move_hip(2, 0)
        else:
            self.move_hip(2, 1)
        if pos_fl:
            self.move_hip(3, 0)
        else:
            self.move_hip(3, 1)
        
        ankle_action = action % 16
        pos_fr = ankle_action & 0b0001 != 0
        pos_rr = ankle_action & 0b0010 != 0
        pos_rl = ankle_action & 0b0100 != 0
        pos_fl = ankle_action & 0b1000 != 0

        if pos_fr:
            self.move_ankle(0, 0)
        else:
            self.move_ankle(0, 1)
        if pos_rr:
            self.move_ankle(1, 0)
        else:
            self.move_ankle(1, 1)
        if pos_rl:
            self.move_ankle(2, 0)
        else:
            self.move_ankle(2, 1)
        if pos_fl:
            self.move_ankle(3, 0)
        else:
            self.move_ankle(3, 1)
        
        t0 = time.time() + 0.085
        dx = 0.0
        dy = 0.0
        n = 0
        diff = 0.0
        max_read = np.zeros(3)
        #reading_list = []
        while time.time() < t0:  # assumes each loop takes apx same amount of time
            accel = mpu9250.read()['accel']
            max_read = np.maximum(accel, max_read)
            #reading_list.append(accel)
            diff += accel[0] - accel[1]
            n += 1
        time.sleep(0.25)
        reward = -diff
        print(-diff)
        reward = 0 if reward < 45 else reward
        
        '''      
        for i in reading_list:
            for j in i:
                file.write(str(j))
                file.write(',')
            file.write('\n')
            
        file.close()
        '''        
        done = bool(reward > 45)
        return action, reward, done 

    def getAhrs(self):
        print(mpu9250.read())

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
                elif np.argmax(q[state]) > 0:
                    action = np.argmax(q[state])
                    action *= 16
                else:
                    action = self.sample()
                    
                obs, reward, done = self.step(action)
                obs = int(obs/16)
                q[state][obs] = alpha * q[state][obs] + (1 - alpha) * (reward + np.max(q[obs]))
                state = obs
                
            explore *= .9
            explore = max(explore, .01)
            print(n, reward, np.sum(q), explore)

w = Walker()
w.start()
w.learn()