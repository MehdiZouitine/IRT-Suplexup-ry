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


# class Fatima (Robot):
#     SMALLEST_TURNING_RADIUS = 0.1
#     SAFE_ZONE = 0.75
#     TIME_BEFORE_DIRECTION_CHANGE = 200  # 8000 ms / 40 ms

#     def __init__(self):
#         Robot.__init__(self)
#         self.time_step = int(self.getBasicTimeStep())

#         # self.camera = Camera(self)
#         self.fall_detector = FallDetection(self.time_step, self)
#         self.gait_manager = GaitManager(self, self.time_step)
#         self.heading_angle = 3.14 / 2
#         # Time before changing direction to stop the robot from falling off the ring
#         self.counter = 0
#         self.t = 0
#         self.start = True

#     def run(self):
        
#         n = 180
#         while self.step(self.time_step) != -1:
#             self.fall_detector.check()
#             if self.t < 15:
#                 radius=0.05
#             elif self.t in range(n, n+55):
#                 radius = -0.05
#             elif self.t in range(n+125, n+176):
#                 radius = -0.05
#             elif self.t >n+210:
#                 print("=ok=")
#                 radius = -0.05
#             else:
#                 radius = 0
            
#             self.gait_manager.update_theta()
#             self.gait_manager.command_to_motors(desired_radius=radius)
#             self.t += 1
#             # self.gait_manager.command_to_motors(desired_radius=radius)
#             # if self.start:
#                 # radius=-0.1
#             # else:
#                 # if self.t < 30:
#                     # radius=0.1
#                 # elif (self.t > 30):
#                     # radius=0
            
#             # We need to update the internal theta value of the gait manager at every step:
#             # self.gait_manager.update_theta()
#             # self.gait_manager.command_to_motors(desired_radius=radius)
#             # self.t += 1
#             # if self.t == 100:
#                 # self.t = 0
#             # if self.t == 30:
#                 # self.start = False
                

    

#     def _get_normalized_opponent_x(self):
#         """Locate the opponent in the image and return its horizontal position in the range [-1, 1]."""
#         img = self.camera.get_image()
#         _, _, horizontal_coordinate = IP.locate_opponent(img)
#         if horizontal_coordinate is None:
#             return 0
#         return horizontal_coordinate * 2 / img.shape[1] - 1


# # create the Robot instance and run main loop
# wrestler = Fatima()
# wrestler.run()


from utils.motion_library import MotionLibrary


class Bob (Robot):
    def __init__(self):
        super().__init__()
        # to load all the motions from the motion folder, we use the Motion_library class:
        self.library = MotionLibrary()

        # we initialize the shoulder pitch motors using the Robot.getDevice() function:
        self.RShoulderPitch = self.getDevice("RShoulderPitch")
        self.LShoulderPitch = self.getDevice("LShoulderPitch")
        self.t = 0
        self.nb_motions = 0
        self.time_step = int(self.getBasicTimeStep())
        self.fall_detector = FallDetection(self.time_step, self)

    # def run(self):
    #     # to control a motor, we use the setPosition() function:
    #     self.RShoulderPitch.setPosition(1.3)
    #     self.LShoulderPitch.setPosition(1.3)
    #     # for more motor control functions, see the documentation: https://cyberbotics.com/doc/reference/motor
    #     # to see the list of available devices, see the NAO documentation: https://cyberbotics.com/doc/guide/nao

    #     time_step = int(self.getBasicTimeStep())
    #     while self.step(time_step) != -1:
    #         is_fallen = self.fall_detector.detect_fall()
    #         if is_fallen:
    #             if current_motion != 'Stand':
    #                 self.library.stop(current_motion)
    #             self.fall_detector.check()
    #         if self.t < 1000 and not is_fallen: # We wait a bit for the robot to stabilise
    #             # to play a motion from the library, we use the play() function as follows:
    #             self.library.play('SideStepLeftLoop')
    #             current_motion = 'SideStepLeftLoop'
    #             self.t += 1
    #             # self.nb_motions += 1
    #         if self.t >= 1000 and self.t<1500  and not is_fallen:
    #             self.library.stop('SideStepLeftLoop')
    #             self.library.play('ForwardLoop')
    #             current_motion = 'ForwardLoop'
    #             self.t += 1
    #         if self.t >= 1500 and self.t<1600  and not is_fallen:
    #             self.library.stop('ForwardLoop')
    #             self.library.play("TurnRight20")
    #             current_motion = 'TurnRight20'
    #             self.t += 1
    #         if self.t >= 1600 and self.t<1800  and not is_fallen:
    #             self.library.stop("TurnRight20")
    #             self.library.play("ForwardLoop")
    #             current_motion = 'ForwardLoop'
    #             self.t += 1
    #         if self.t >= 1800 and self.t<2000  and not is_fallen:
    #             self.library.stop("ForwardLoop")
    #             self.library.play("TurnRight60")
    #             current_motion = 'TurnRight60'
    #             self.t += 1
    #         if self.t >= 2000 and self.t<2500 and not is_fallen:
    #             self.library.stop("TurnRight60")
    #             self.library.play("ForwardLoop")
    #             current_motion = 'ForwardLoop'
    #             self.t += 1
    #         if self.t >= 2500 and self.t<2850  and not is_fallen:
    #             self.library.stop("ForwardLoop")
    #             self.library.play("TurnRight60")
    #             current_motion = 'TurnRight60'
    #             self.t += 1
    #         if self.t >= 2850 and self.t<3300  and not is_fallen:
    #             self.library.stop("TurnRight60")
    #             self.library.play("ForwardLoop")
    #             current_motion  = 'ForwardLoop'
    #             self.t += 1
    #         if self.t >= 3300 and self.t<3900  and not is_fallen:
    #             self.library.stop("ForwardLoop")
    #             self.library.play("TurnRight60")
    #             current_motion = 'TurnRight60'
    #             self.t += 1
    #         if self.t >= 3900 and self.t<4300  and not is_fallen:
    #             self.library.stop("TurnRight60")
    #             self.library.play("ForwardLoop")
    #             current_motion = 'ForwardLoop'
    #             self.t += 1
    #         if self.t >= 4300  and not is_fallen:
    #             self.library.stop("ForwardLoop")
    #             self.t += 1
    #             self.library.play("Stand")
    #             current_motion = 'Stand'

    def run(self):
        # to control a motor, we use the setPosition() function:
        self.RShoulderPitch.setPosition(1.3)
        self.LShoulderPitch.setPosition(1.3)
        # for more motor control functions, see the documentation: https://cyberbotics.com/doc/reference/motor
        # to see the list of available devices, see the NAO documentation: https://cyberbotics.com/doc/guide/nao

        time_step = int(self.getBasicTimeStep())
        while self.step(time_step) != -1:
            is_fallen = self.fall_detector.detect_fall()
            if is_fallen:
                if current_motion != 'Stand':
                    self.library.stop(current_motion)
                self.fall_detector.check()
            if self.t < 1500 and not is_fallen: # We wait a bit for the robot to stabilise
                # to play a motion from the library, we use the play() function as follows:
                self.library.play('SideStepLeftLoop')
                current_motion = 'SideStepLeftLoop'
                self.t += 1
                # self.nb_motions += 1

            if self.t >= 1500 and self.t<1550  and not is_fallen:
                self.library.stop('SideStepLeftLoop')
                self.library.play("Stand")
                current_motion = 'Stand'
                self.t += 1
            if self.t >= 1550 and self.t<2300  and not is_fallen:
                self.library.stop("Stand")
                self.library.play("ForwardLoop")
                current_motion = 'ForwardLoop'
                
                self.t += 1
            if self.t >= 2300 and self.t<2500  and not is_fallen:
                self.library.stop("ForwardLoop")
                self.library.play("TurnRight20")
                current_motion = 'TurnRight20'
                self.t += 1
            if self.t >= 2500 and self.t<3200  and not is_fallen:
                self.library.stop("TurnRight20")
                self.library.play("ForwardLoop")
                current_motion = 'ForwardLoop'
                self.t += 1
            # if self.t >= 3200 and self.t<3500  and not is_fallen:
            #     self.library.stop("ForwardLoop")
            #     self.library.play("Backwards")
            #     current_motion = 'Backwards'
            #     self.t += 1
            if self.t >= 3200  and self.t<3450  and not is_fallen:
                self.library.stop("ForwardLoop")
                self.library.play("TurnRight20")
                current_motion = 'TurnRight20'
                self.t += 1
            if self.t >= 3450 and self.t<3900  and not is_fallen:
                self.library.stop("TurnRight20")
                self.library.play("ForwardLoop")
                current_motion = 'ForwardLoop'
                self.t += 1
            if self.t >= 3900 and self.t<4200  and not is_fallen:
                self.library.stop("ForwardLoop")
                self.library.play("TurnRight20")
                current_motion = 'TurnRight20'
                self.t += 1
            if self.t >= 4200 and self.t<4500  and not is_fallen:
                self.library.stop("TurnRight20")
                self.library.play("ForwardLoop")
                current_motion = 'ForwardLoop'
                self.t += 1
            
            if self.t >= 4500  and not is_fallen:
                self.library.stop("ForwardLoop")
                self.t += 1
                self.library.play("Stand")
                current_motion = 'Stand'
            
            # if self.t >= 1800 and self.t<2000  and not is_fallen:
            #     self.library.stop("ForwardLoop")
            #     self.library.play("TurnRight60")
            #     current_motion = 'TurnRight60'
            #     self.t += 1
            # if self.t >= 2000 and self.t<2500 and not is_fallen:
            #     self.library.stop("TurnRight60")
            #     self.library.play("ForwardLoop")
            #     current_motion = 'ForwardLoop'
            #     self.t += 1
            # if self.t >= 2500 and self.t<2850  and not is_fallen:
            #     self.library.stop("ForwardLoop")
            #     self.library.play("TurnRight60")
            #     current_motion = 'TurnRight60'
            #     self.t += 1
            # if self.t >= 2850 and self.t<3300  and not is_fallen:
            #     self.library.stop("TurnRight60")
            #     self.library.play("ForwardLoop")
            #     current_motion  = 'ForwardLoop'
            #     self.t += 1
            # if self.t >= 3300 and self.t<3900  and not is_fallen:
            #     self.library.stop("ForwardLoop")
            #     self.library.play("TurnRight60")
            #     current_motion = 'TurnRight60'
            #     self.t += 1
            # if self.t >= 3900 and self.t<4300  and not is_fallen:
            #     self.library.stop("TurnRight60")
            #     self.library.play("ForwardLoop")
            #     current_motion = 'ForwardLoop'
            #     self.t += 1
            # if self.t >= 4300  and not is_fallen:
            #     self.library.stop("ForwardLoop")
            #     self.t += 1
            #     self.library.play("Stand")
            #     current_motion = 'Stand'
            
                


# create the Robot instance and run main loop
wrestler = Bob()
wrestler.run()