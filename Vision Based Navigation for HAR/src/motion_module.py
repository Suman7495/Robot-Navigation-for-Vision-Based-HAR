#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import qi
import argparse
import sys
import math
import almath
import time
from datetime import datetime
from naoqi import ALProxy

# Import Vision APIs
import vision_definitions
from PIL import Image

# Trajectory Generation
from create_trajectory import *

# Import Cloud API
from google_micro_api import Google_Microsoft_Fun

save_path_name = "SoRo_Images"


class Pepper:
    def __init__(self, arg_session):
        """Initialization"""
        self.session = arg_session
        # the modules commonly need to be used
        self.navigation_service = self.session.service("ALNavigation")
        self.motion_service = self.session.service("ALMotion")
        self.posture_service = self.session.service("ALRobotPosture")
        self.session.service("ALPeoplePerception")
        self.AutonomousLife_service = self.session.service("ALAutonomousLife")
        self.BasicAwareness_service = self.session.service("ALBasicAwareness")
        # the distance that a robot needs to move toward a human (along x-axis)

        # Waypoint target positions
        self.TargetPos = [[0, 0], [0, 0], [0, 0], [0, 0]]

        # Image counter
        self.img_num = 0

        # Pepper's IP. Ideallly, read from arg_session
        self.IP = "130.251.13.194"

        # Social Distance
        self.socialDistance = 2.0

    def StopAutonomousLifeParam(self):
        """
            Initialization procedure.
            Stop previous motions.
        """
        # Wake up robot
        self.motion_service.wakeUp()

        # Send robot to Stand Init
        self.posture_service.goToPosture("StandInit", 0.5)

        # Enable arms control by move algorithm
        self.motion_service.setMoveArmsEnabled(True, True)

        # Foot contact protection
        self.motion_service.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

        self.AutonomousLife_service.setAutonomousAbilityEnabled("AutonomousBlinking", False)
        self.AutonomousLife_service.setAutonomousAbilityEnabled("BackgroundMovement", False)
        self.AutonomousLife_service.setAutonomousAbilityEnabled("BasicAwareness", False)
        self.AutonomousLife_service.setAutonomousAbilityEnabled("SpeakingMovement", False)

    def SetAutonomousLifeParam(self):
        """
            Initialization procedure.
        """
        # Wake up robot
        print("In SetAutonomousLifeParam\n")
        self.motion_service.wakeUp()

        # Send robot to Stand Init
        self.posture_service.goToPosture("StandInit", 0.5)

        # Enable arms control by move algorithm
        self.motion_service.setMoveArmsEnabled(True, True)

        # Food contact protection
        self.motion_service.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

        # Disable the robot to make its eye LEDs blink when it sees someone and when it is interacting.
        self.AutonomousLife_service.setAutonomousAbilityEnabled("AutonomousBlinking", False)
        # Enable slight movements the robot does autonomously when its limbs are not moving.
        self.AutonomousLife_service.setAutonomousAbilityEnabled("BackgroundMovement", True)
        # Allows the robot to react to the environment to establish and keep eye contact with people.
        self.AutonomousLife_service.setAutonomousAbilityEnabled("BasicAwareness", True)
        # Enables to start autonomously movements during the speech of the robot.
        self.AutonomousLife_service.setAutonomousAbilityEnabled("SpeakingMovement", True)

        # set the the tracking mode: "BodyRotation" which uses the head and the rotation of the body
        self.BasicAwareness_service.setTrackingMode("BodyRotation")
        # self.BasicAwareness_service.setTrackingMode("WholeBody")

        # check whether an autonomous ability is enabled.
        # print(AutonomousLife_service.getAutonomousAbilityEnabled("BasicAwareness"))
        # print(AutonomousLife_service.getAutonomousAbilityEnabled("SpeakingMovement"))

    # Disable the previous autonomous ablities

    # The robot goes to a relaxed and safe position and sets Motor off.
    def Rest(self):
        self.motion_service.rest()
        # print motion state
        # print(self.motion_service.getSummary())

    def FaceDectection(self):
        """
            Detects a face.
        """
        print("Detecting face...\n")

        face_service = self.session.service("ALFaceDetection")
        # Sets the image resolution for the extractor
        face_service.setResolution(2)
        face_service.setTrackingEnabled(True)
        period = 100
        face_service.subscribe("Test_Face", period, 0.0)
        memory_service = self.session.service("ALMemory")
        memFaceValue = "FaceDetected"
        val_face = memory_service.getData(memFaceValue, 0)

        # A while loop that reads the val_face and checks whether faces are detected.
        while (val_face == [] or val_face == None):
            val_face = memory_service.getData(memFaceValue, 0)
            # print("can't see face")

        # once the robot detects a face
        print("I found a face\n")
        IP = self.IP
        tts = ALProxy("ALTextToSpeech", IP, 9559)
        # the robot will say "I saw a face!"
        tts.say("I saw a face!")

        # get the angle of headyaw
        names = ["HeadYaw"]
        useSensors = True
        angle = self.motion_service.getAngles(names, useSensors)
        print("HeadYaw angle is:", angle[0])
        face_service.unsubscribe("Test_Face")

        # disable the the robot to react to the environment to establish and keep eye contact with people.
        self.AutonomousLife_service.setAutonomousAbilityEnabled("BasicAwareness", False)
        self.motion_service.setStiffnesses("Head", 1.0)

        # move the wheel toward the direction where the robot detected the face
        self.motion_service.moveTo(0, 0, angle[0], _async=True)
        self.motion_service.waitUntilMoveIsFinished()

        print("Success here\n")
        # make pepper move his head back to the center
        angles = [0]
        fractionMaxSpeed = 0.2
        self.motion_service.setAngles(names, angles, fractionMaxSpeed)

        print("Finished Face Detection\n")
        # face_service.setTrackingEnabled(False)

    def GetDistance(self):
        """
            Get distance to human.
            Obtain trajectory waypoints from CreateTraj()
        """
        print("In GetDistance\n")
        # use ALPeoplePerception module to get the distance between a robot and a human
        start = time.time()
        people_service = self.session.service("ALPeoplePerception")

        period = 100
        people_service.subscribe("Test_People", period, 0.0)
        memory_service = self.session.service("ALMemory")
        # the value history of event "PeoplePerception/PeopleDetected" is stored into ALMemory
        memPeopleValue = "PeoplePerception/PeopleDetected"
        # to access the event value
        val_people = memory_service.getData(memPeopleValue, 0)
        time.sleep(1)
        # A while loop that reads the val_people and checks whether a person is detected.
        while (val_people == [] or val_people == None):
            val_people = memory_service.getData(memPeopleValue, 0)
            # print("Can't find person")
        else:
            # print(val_people)
            # print("find person")
            timeStamp = val_people[0]
            personDataArray = val_people[1]
            for personData in personDataArray:
                personID = personData[0]
                DistanceToCamera = personData[1]
                PitchAngleInImage = personData[2]
                YawAngleInImage = personData[3]
                # print("person ID %f" %(personID))
                print("DistanceToCamera %f" % (DistanceToCamera))
                return DistanceToCamera

    def NaviageToTarget(self):
        """
            Navigate along the waypoints and take pictures.
        """
        # navigation_service = self.session.service("ALNavigation")
        print("\nNavigating to person...\n")

        # Navigate along waypoints
        for i in range(self.TargetPos.shape[0]):
            try:
                x = float(self.TargetPos[i, 0])
                y = float(self.TargetPos[i, 1])
                theta = 0
                # Move to target waypoint
                initRobotPosition = almath.Pose2D(self.motion_service.getRobotPosition(False))
                print("Moving to target position: ", x, y)
                self.motion_service.moveTo(x, y, theta, _async=True)
                self.motion_service.waitUntilMoveIsFinished()
                # self.navigation_service.moveAlong(["Holonomic", ["Line", [x, y]], 0.0, 2.0])

                # Obtain image at the waypoint
                if (i != 3):
                    self.GetImage(self.img_num)
                    self.img_num += 1
                if self.GetDistance() < self.socialDistance:
                    print("Reached inside Social Distane.\n")
                    break
                # time.sleep(0.5)
                # endRobotPosition = almath.Pose2D(self.motion_service.getRobotPosition(False))
                # robotMove = almath.pose2DInverse(initRobotPosition)*endRobotPosition
                # print("Distance change and angle change after moveTo() function:",robotMove)

            except KeyboardInterrupt:
                print ("Interrupted by user, stopping motion.")
                # stop
                self.motion_service.stopMove()
                sys.exit(0)
        print("Completed navigation\n")

    def MoveToTakePics(self):
        """
            Pepper moves it's head to take pictures of surrounding
        """
        print("Moving head to take pictures...\n")
        self.AutonomousLife_service.setAutonomousAbilityEnabled("BasicAwareness", True)
        self.FaceDectection()
        # Get the angle of the Head Yaw
        name = ["HeadYaw"]
        useSensors = True
        headYaw = self.motion_service.getAngles(name[0], useSensors)
        print("HeadYaw is: ", headYaw[0])
        name1 = ["HeadPitch"]
        headPitch = self.motion_service.getAngles(name1[0], useSensors)
        print("HeadPitch is: ", headPitch[0])
        fractionMaxSpeed = 0.2
        headpitch_list = [-0.1]
        self.motion_service.setAngles(name1, headpitch_list, fractionMaxSpeed)
        print("Moving head to take pictures...\n")
        angle_list = [0.52, -0.52, 0]
        for angle in angle_list:
            self.motion_service.setAngles(name, angle, fractionMaxSpeed)
            self.GetImage(self.img_num)
            self.img_num += 1

    def GetImage(self, img_name):
        """
            Obtain an image from Pepper's camera and store image.
        """
        print("Taking pictures...\n")

        # Get the service ALVideoDevice.
        video_service = self.session.service("ALVideoDevice")

        # Register a Generic Video Module
        resolution = 2    # VGA 640*480px
        colorSpace = 11   # RGB
        fps = 15
        # use 3D camera in the eyes
        # video_service.setActiveCamera(2)

        # get the default camera is
        # activeCam =video_service.getActiveCamera()
        # print("Active camera %d\n" %(activeCam))

        nameId = video_service.subscribe("python_client", resolution, colorSpace, fps)
        time.sleep(1)
        print ("Getting image " + str(img_name))

        # Get image remotely and save it as a JPG using ImageDraw package
        naoImage = video_service.getImageRemote(nameId)
        video_service.unsubscribe(nameId)

        # Get the image size and pixel array.
        imageWidth = naoImage[0]
        imageHeight = naoImage[1]
        array = naoImage[6]
        image_string = str(bytearray(array))

        # Create a PIL Image from the pixel array.
        im = Image.frombytes("RGB", (imageWidth, imageHeight), image_string)
        # Save the image in the following directory
        im.save("../image/" + save_path_name + "/" + save_path_name + "_" + str(img_name) + ".jpg", "JPEG")
        im.show()

    def TakeClosePic(self):
        """
            Note: Not used in this version.
            Takes close up pics
        """
        print("In TakeClosePic\n")

        # x – normalized, unitless, velocity along X-axis.
        # +1 and -1 correspond to the maximum velocity in the forward and backward directions, respectively.
        x = 1.0
        y = 0.0
        theta = 0.0
        # index of image
        image_i = 0
        frequency = 0.1
        # get current time
        t0 = time.time()
        initRobotPosition = almath.Pose2D(self.motion_service.getRobotPosition(False))

        #  since now the robot is 1.5 away from the human
        #  let the robot move 0.7 meters forward while taking photos(now the velocity is 0.1 meter per second)
        #  in order to keep 0.8 meter distance away from human

        while ((time.time() - t0) <= 7.0):
            # once 7s has been passed, break the loop
            if (time.time() - t0) > 7.0:
                break
            # print "time difference :" + str((time.time() - t0))
            self.motion_service.moveToward(x, y, theta, [["MaxVelXY", frequency]])
            image_i += 1
            self.GetImage(image_i)

        # stop the robot
        self.motion_service.stopMove()
        endRobotPosition = almath.Pose2D(self.motion_service.getRobotPosition(False))
        robotMove = almath.pose2DInverse(initRobotPosition)*endRobotPosition
        robotMove.theta = almath.modulo2PI(robotMove.theta)
        print ("Distance change and angle change after approaching the human:", robotMove)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    print("Starting...")
    # IP = "192.168.1.5"
    parser.add_argument("--ip", type=str, default="130.251.13.194",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")
    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
        print("Success in connection")
    except RuntimeError or KeyboardInterrupt:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) + ".\n"
               "Please check your script arguments. Run with -h option for help.")
        motion_service = session.service("ALMotion")
        print("Stop")
        motion_service.stopMove()
        sys.exit(1)

    # create an object of Pepper class
    pepper_object = Pepper(session)
    pepper_object.StopAutonomousLifeParam
    pepper_object.Rest()
    pepper_object.SetAutonomousLifeParam()

    # Karen's code
    # pepper_object.FaceDectection()
    # time.sleep(2)
    # pepper_object.GetDistance()
    # pepper_object.NaviageToTarget()
    # pepper_object.TakeClosePic()

    # ADDED BY ADAM, ROHIT AND SUMAN
    pepper_object.FaceDectection()
    DistanceToCamera = pepper_object.GetDistance()
    # ADDED BY ADAM, ROHIT, SUMAN
    if DistanceToCamera < pepper_object.socialDistance:
        pepper_object.TargetPos = np.matrix([(DistanceToCamera-pepper_object.socialDistance), 0])
    else:
        pepper_object.TargetPos = CreateTraj(DistanceToCamera, pepper_object.socialDistance)
    print("Target Positions are: ", pepper_object.TargetPos)
    pepper_object.NaviageToTarget()
    pepper_object.MoveToTakePics()
    # END of added section

    print ("Calling google microsoft API........")
    time.sleep(0.5)

    image_path = "../image/" + save_path_name
    api_path = "../api_result/" + save_path_name + ".py"

    Google_Microsoft_Fun(image_path, api_path)
