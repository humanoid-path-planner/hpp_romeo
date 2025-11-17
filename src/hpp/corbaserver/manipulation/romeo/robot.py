#
# Copyright (c) 2015 CNRS
# Authors: Joseph Mirabel
#
#

from hpp.corbaserver.manipulation.robot import HumanoidRobot as Parent


class Robot(Parent):
    """
    Control of robot Romeo in hpp

    This class implements a client to the corba server implemented in
    hpp-manipulation-corba. It derives from class
    hpp.corbaserver.manipulation.robot.Robot.

    This class is also used to initialize a client to the gepetto corba
    server in order to display configurations of the Romeo robot.

    At creation of an instance, the urdf and srdf files are loaded using
    idl interface hpp::corbaserver::manipulation::Robot::loadRobotModel.
    """

    #  Information to retrieve urdf and srdf files.
    urdfFilename = "package://example-robot-data/robots/romeo_description/urdf/romeo.urdf"
    srdfFilename = "package://example-robot-data/robots/romeo_description/srdf/romeo_moveit.srdf"

    halfSitting = {
        "LEyePitch": 0,
        "LWristYaw": -0.3,
        "root_joint": (0, 0, 0.840252, 0, 0, 0, 1),
        "LEyeYaw": 0,
        "RWristYaw": -0.3,
        "LHipYaw": 0,
        "RHipPitch": -0.3490658,
        "RElbowYaw": 1.05,
        "LShoulderYaw": 0.6,
        "TrunkYaw": 0,
        "RShoulderPitch": 1.5,
        "LShoulderPitch": 1.5,
        "LWristPitch": -0.2,
        "HeadRoll": 0,
        "LKneePitch": 0.6981317,
        "RAnkleRoll": 0,
        "LHipPitch": -0.3490658,
        "LElbowYaw": -1.05,
        "RHipYaw": 0,
        "LAnklePitch": -0.3490658,
        "RAnklePitch": -0.3490658,
        "LToePitch": 0,
        "RKneePitch": 0.6981317,
        "HeadPitch": 0,
        "LWristRoll": -0.4,
        "RShoulderYaw": -0.6,
        "RWristPitch": -0.2,
        "LElbowRoll": -0.5,
        "RWristRoll": -0.4,
        "LAnkleRoll": 0,
        "REyeYaw": 0,
        "NeckPitch": 0,
        "REyePitch": 0,
        "RToePitch": 0,
        "LHipRoll": 0,
        "RHipRoll": 0,
        "RElbowRoll": 0.5,
        "NeckYaw": 0,
        # Here start romeo -full- specifics
        "LHand": 0,
        "LFinger12": 0,
        "LFinger13": 0,
        "LFinger21": 0,
        "LFinger22": 0,
        "LFinger23": 0,
        "LFinger31": 0,
        "LFinger32": 0,
        "LFinger33": 0,
        "LThumb1": 0,
        "LThumb2": 0,
        "LThumb3": 0,
        "RHand": 0,
        "RFinger12": 0,
        "RFinger13": 0,
        "RFinger21": 0,
        "RFinger22": 0,
        "RFinger23": 0,
        "RFinger31": 0,
        "RFinger32": 0,
        "RFinger33": 0,
        "RThumb1": 0,
        "RThumb2": 0,
        "RThumb3": 0,
    }

    leftHandClosed = {
        "LHand": 0.0,
        "LFinger12": 0.0,
        "LFinger13": 0.0,
        "LFinger21": 0.0,
        "LFinger22": 0.0,
        "LFinger23": 0.0,
        "LFinger31": 0.0,
        "LFinger32": 0.0,
        "LFinger33": 0.0,
        "LThumb1": 0.0,
        "LThumb2": 0.0,
        "LThumb3": 0.0,
    }
    leftHandOpen = {
        "LHand": 1,
        "LFinger12": 1.06,
        "LFinger13": 1.06,
        "LFinger21": 1.06,
        "LFinger22": 1.06,
        "LFinger23": 1.06,
        "LFinger31": 1.06,
        "LFinger32": 1.06,
        "LFinger33": 1.06,
        "LThumb1": 0,
        "LThumb2": 1.06,
        "LThumb3": 1.06,
    }
    rightHandClosed = {
        "RHand": 0.0,
        "RFinger12": 0.0,
        "RFinger13": 0.0,
        "RFinger21": 0.0,
        "RFinger22": 0.0,
        "RFinger23": 0.0,
        "RFinger31": 0.0,
        "RFinger32": 0.0,
        "RFinger33": 0.0,
        "RThumb1": 0.0,
        "RThumb2": 0.0,
        "RThumb3": 0.0,
    }
    rightHandOpen = {
        "RHand": 1,
        "RFinger12": 1.06,
        "RFinger13": 1.06,
        "RFinger21": 1.06,
        "RFinger22": 1.06,
        "RFinger23": 1.06,
        "RFinger31": 1.06,
        "RFinger32": 1.06,
        "RFinger33": 1.06,
        "RThumb1": 0,
        "RThumb2": 1.06,
        "RThumb3": 1.06,
    }

    def __init__(self, compositeName, robotName, rootJointType="freeflyer", load=True):
        Parent.__init__(self, compositeName, robotName, rootJointType, load)
        self.tf_root = "base_link"
        self.leftAnkle = "LAnkleRoll"
        self.rightAnkle = "RAnkleRoll"

    def getInitialConfig(self):
        q = self.getCurrentConfig()
        for n in self.jointNames:
            if n.startswith(self.displayName):
                name = n[len(self.displayName) + 1 :]
                dof = self.halfSitting[name]
                if type(dof) is tuple:
                    ndof = len(dof)
                    r = self.rankInConfiguration[n]
                    q[r : r + ndof] = dof
                else:
                    q[self.rankInConfiguration[n]] = dof
        return q

    def getJointDofValue(self, jointName):
        for n in self.jointNames:
            if self.displayName + "/" + n == jointName:
                return self.getCurrentConfig()[self.rankInConfiguration[jointName]]

    def getHandConfig(self, side, conf):
        q = []
        if side == "left":
            if conf == "open":
                q = self.getLeftHandOpenConfig()
            elif conf == "closed":
                q = self.getLeftHandClosedConfig()
        elif side == "right":
            if conf == "open":
                q = self.getRightHandOpenConfig()
            elif conf == "closed":
                q = self.getRightHandClosedConfig()
        elif side == "both":
            if conf == "open":
                q = self.getRightHandOpenConfig()
                self.setCurrentConfig(q)
                q = self.getLeftHandOpenConfig()
            elif conf == "closed":
                q = self.getRightHandClosedConfig()
                self.setCurrentConfig(q)
                q = self.getLeftHandClosedConfig()
        return q

    def getLeftHandClosedConfig(self):
        q = self.getCurrentConfig()
        for n in self.jointNames:
            if n.startswith(self.displayName):
                name = n[len(self.displayName) + 1 :]
                if name in self.leftHandClosed:
                    q[self.rankInConfiguration[n]] = self.leftHandClosed[name]
        return q

    def getLeftHandOpenConfig(self):
        q = self.getCurrentConfig()
        for n in self.jointNames:
            if n.startswith(self.displayName):
                name = n[len(self.displayName) + 1 :]
                if name in self.leftHandOpen:
                    q[self.rankInConfiguration[n]] = self.leftHandOpen[name]
        return q

    def getRightHandClosedConfig(self):
        q = self.getCurrentConfig()
        for n in self.jointNames:
            if n.startswith(self.displayName):
                name = n[len(self.displayName) + 1 :]
                if name in self.rightHandClosed:
                    q[self.rankInConfiguration[n]] = self.rightHandClosed[name]
        return q

    def getRightHandOpenConfig(self):
        q = self.getCurrentConfig()
        for n in self.jointNames:
            if n.startswith(self.displayName):
                name = n[len(self.displayName) + 1 :]
                if name in self.rightHandOpen:
                    q[self.rankInConfiguration[n]] = self.rightHandOpen[name]
        return q
