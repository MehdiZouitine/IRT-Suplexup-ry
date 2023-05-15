# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Demonstrates the gait manager (inverse kinematics + simple ellipsoid path).
"""

from controller import Robot
import sys
sys.path.append('..')
# Eve's locate_opponent() is implemented in this module:
from utils.image_processing import ImageProcessing as IP
from utils.fall_detection import FallDetection
from utils.gait_manager import GaitManager
# from utils.camera import Camera


class Fatima (Robot):
    SMALLEST_TURNING_RADIUS = 0.1
    SAFE_ZONE = 0.75
    TIME_BEFORE_DIRECTION_CHANGE = 200  # 8000 ms / 40 ms

    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

        # self.camera = Camera(self)
        self.fall_detector = FallDetection(self.time_step, self)
        self.gait_manager = GaitManager(self, self.time_step)
        self.heading_angle = 3.14 / 2
        # Time before changing direction to stop the robot from falling off the ring
        self.counter = 0
        self.t = 0
        self.start = True

    def run(self):
        
        n = 120
        while self.step(self.time_step) != -1:
            self.fall_detector.check()
            if self.t < 20:
                radius=0.05
            elif self.t in range(n, n+35):
                radius = -0.05
            elif self.t in range(n+125, n+155):
                radius = -0.05
            elif self.t >n+215:
                print("=ok=")
                radius = -0.05
            else:
                radius = 0
            
            self.gait_manager.update_theta()
            self.gait_manager.command_to_motors(desired_radius=radius)
            self.t += 1
            # self.gait_manager.command_to_motors(desired_radius=radius)
            # if self.start:
                # radius=-0.1
            # else:
                # if self.t < 30:
                    # radius=0.1
                # elif (self.t > 30):
                    # radius=0
            
            # We need to update the internal theta value of the gait manager at every step:
            # self.gait_manager.update_theta()
            # self.gait_manager.command_to_motors(desired_radius=radius)
            # self.t += 1
            # if self.t == 100:
                # self.t = 0
            # if self.t == 30:
                # self.start = False
                

    

    def _get_normalized_opponent_x(self):
        """Locate the opponent in the image and return its horizontal position in the range [-1, 1]."""
        img = self.camera.get_image()
        _, _, horizontal_coordinate = IP.locate_opponent(img)
        if horizontal_coordinate is None:
            return 0
        return horizontal_coordinate * 2 / img.shape[1] - 1


# create the Robot instance and run main loop
wrestler = Fatima()
wrestler.run()
