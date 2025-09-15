import random

from hpp.corbaserver.wholebody_step.client import Client as WholebodyStepClient
from hpp_ros import PathPlayer, ScenePublisher

from hpp import Quaternion  # noqa: F401
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.romeo import Robot

robot = Robot("romeo")
robot.setJointBounds("base_joint_x", [-4, 3])
robot.setJointBounds("base_joint_y", [-5, 3])
robot.setJointBounds("base_joint_z", [0, 2])

r = ScenePublisher(robot)

# Get half-sitting configuration
q = robot.getInitialConfig()
q[0] = -3
q[1] = -4
r(q)

ps = ProblemSolver(robot)

# create static stability constraint
wcl = WholebodyStepClient()
wcl.problem.addStaticStabilityConstraints(
    "balance", q, robot.leftAnkle, robot.rightAnkle
)

# Define quatenion for righthand orientation constraints
# quat = Quaternion()
# quat.fromRPY(-1.55, -1.55, 0.0) # Right hand horizontal, grasping to -z
# quat.fromRPY(-1.55, 0.0, 0.0)   # Right hand vertical, pointing down
# quat.fromRPY(0.0, 0.0, -1.55)   # Right hand vertical, pointing forward
# quat.normalize()

ps.resetConstraints()

# Add constraints to problem solver
ps.setNumericalConstraints(
    "balance",
    [
        "balance/relative-com",
        "balance/relative-orientation",
        "balance/relative-position",
        "balance/orientation-left-foot",
        "balance/position-left-foot",
    ],
)
ps.createPositionConstraint(
    "RightHandPos",
    "RWristPitch",
    "",
    [0.0, 0.0, 0.0],
    [-2.6, -4, 0.75],
    True,
    True,
    True,
)
# ps.createOrientationConstraint(
# "RightHandOr",
# "",
# "RWristPitch",
# [quat.array[0], quat.array[1], quat.array[2], quat.array[3]],
# True,
# True,
# True,
# )
# ps.setNumericalConstraints("eef", ["RightHandPos", "RightHandOr"])
ps.setNumericalConstraints("eef", ["RightHandPos"])

ps.createPositionConstraint(
    "LeftAnklePos",
    robot.leftAnkle,
    "",
    [0.0, 0.0, 0.0],
    [
        robot.getJointPosition(robot.leftAnkle)[0],
        robot.getJointPosition(robot.leftAnkle)[1],
        robot.getJointPosition(robot.leftAnkle)[2],
    ],
    True,
    True,
    False,
)
ps.createOrientationConstraint(
    "LeftAnkleOr",
    "",
    robot.leftAnkle,
    [
        robot.getJointPosition(robot.leftAnkle)[3],
        robot.getJointPosition(robot.leftAnkle)[4],
        robot.getJointPosition(robot.leftAnkle)[5],
        robot.getJointPosition(robot.leftAnkle)[6],
    ],
    False,
    False,
    True,
)
ps.setNumericalConstraints("lock", ["LeftAnklePos", "LeftAnkleOr"])

ps.lockOneDofJoint("LFinger11", robot.getJointDofValue("LFinger11"))
ps.lockOneDofJoint("LFinger12", robot.getJointDofValue("LFinger12"))
ps.lockOneDofJoint("LFinger13", robot.getJointDofValue("LFinger13"))
ps.lockOneDofJoint("LFinger21", robot.getJointDofValue("LFinger21"))
ps.lockOneDofJoint("LFinger22", robot.getJointDofValue("LFinger22"))
ps.lockOneDofJoint("LFinger23", robot.getJointDofValue("LFinger23"))
ps.lockOneDofJoint("LFinger31", robot.getJointDofValue("LFinger31"))
ps.lockOneDofJoint("LFinger32", robot.getJointDofValue("LFinger32"))
ps.lockOneDofJoint("LFinger33", robot.getJointDofValue("LFinger33"))
ps.lockOneDofJoint("LThumb1", robot.getJointDofValue("LThumb1"))
ps.lockOneDofJoint("LThumb2", robot.getJointDofValue("LThumb2"))
ps.lockOneDofJoint("LThumb3", robot.getJointDofValue("LThumb3"))
ps.lockOneDofJoint("RFinger11", robot.getJointDofValue("RFinger11"))
ps.lockOneDofJoint("RFinger12", robot.getJointDofValue("RFinger12"))
ps.lockOneDofJoint("RFinger13", robot.getJointDofValue("RFinger13"))
ps.lockOneDofJoint("RFinger21", robot.getJointDofValue("RFinger21"))
ps.lockOneDofJoint("RFinger22", robot.getJointDofValue("RFinger22"))
ps.lockOneDofJoint("RFinger23", robot.getJointDofValue("RFinger23"))
ps.lockOneDofJoint("RFinger31", robot.getJointDofValue("RFinger31"))
ps.lockOneDofJoint("RFinger32", robot.getJointDofValue("RFinger32"))
ps.lockOneDofJoint("RFinger33", robot.getJointDofValue("RFinger33"))
ps.lockOneDofJoint("RThumb1", robot.getJointDofValue("RThumb1"))
ps.lockOneDofJoint("RThumb2", robot.getJointDofValue("RThumb2"))
ps.lockOneDofJoint("RThumb3", robot.getJointDofValue("RThumb3"))
ps.lockOneDofJoint("NeckYaw", robot.getJointDofValue("NeckYaw"))
ps.lockOneDofJoint("NeckPitch", robot.getJointDofValue("NeckPitch"))
ps.lockOneDofJoint("HeadPitch", robot.getJointDofValue("HeadPitch"))
ps.lockOneDofJoint("HeadRoll", robot.getJointDofValue("HeadRoll"))
ps.lockOneDofJoint("LToePitch", robot.getJointDofValue("LToePitch"))
ps.lockOneDofJoint("RToePitch", robot.getJointDofValue("RToePitch"))
ps.lockOneDofJoint("LEyeYaw", robot.getJointDofValue("LEyeYaw"))
ps.lockOneDofJoint("LEyePitch", robot.getJointDofValue("LEyePitch"))
ps.lockOneDofJoint("REyeYaw", robot.getJointDofValue("REyeYaw"))
ps.lockOneDofJoint("REyePitch", robot.getJointDofValue("REyePitch"))

q_init = robot.getCurrentConfig()
# q_start = robot.getCurrentConfig()

# Get one config wrt constraints
res, q, error, time = ps.applyConstraints(q_init)
full_time = time
while not res:
    full_time += time
    res, q, error, time = ps.applyConstraints(q)

# robot.shootRandomConfig()
# robot.setCurrentConfig(q)
# q = robot.getRightHandOpenConfig()

full_time
r(q)


###################################################
# Test de timing pour trouver une solution IK
# à partir d'une configuration completement random
###################################################
average_timer = 0
average_full_timer = 0
timer = []
full_timer = []
for i in range(0, 10):
    q_tmp = robot.shootRandomConfig()

    res, q, error, time = ps.applyConstraints(q_tmp)
    full_time = time
    timer.append(time)
    while not res:
        full_time += time
        res, q, error, time = ps.applyConstraints(q)

    full_timer.append(full_time)
    average_timer += timer[i]
    average_full_timer += full_timer[i]

average_timer = average_timer / 10
average_full_timer = average_full_timer / 10

###################################################
# Test de timing pour trouver une solution IK
# à partir d'une configuration placée pas très loin de la configuration cible
###################################################

average_timer = 0
average_full_timer = 0
timer = []
full_timer = []
q_tmp = q_init
for i in range(0, 10):
    q_tmp[10] += random.uniform(0, 1.5)

    res, q, error, time = ps.applyConstraints(q_tmp)
    full_time = time
    timer.append(time)
    while not res:
        res, q, error, time = ps.applyConstraints(q)
        full_time += time

    full_timer.append(full_time)
    average_timer += timer[i]
    average_full_timer += full_timer[i]

average_timer = average_timer / 10
average_full_timer = average_full_timer / 10
average_timer
average_full_timer


###################################################
# Path planning tests
###################################################
# floor = robot.getJointPosition(robot.rightAnkle)
# ps.createPositionConstraint(
# "Feet",
# robot.rightAnkle,
# "",
# [0.0, 0.0, 0.0],
# [floor[0], floor[1], floor[2]],
# True,
# True,
# True,
# )
# ps.setNumericalConstraints ("floor", ["Feet"])


q_goal = [
    -3,
    -3,
    0.8,
    1.0,
    0,
    0,
    0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.6,
    -0.3,
    0.7,
    0.5,
    0.7,
    0.3,
    0.3,
]
r(q_goal)

ps.setInitialConfig(q_init)
ps.addGoalConfig(q_goal)
ps.solve()

ps.createPositionConstraint(
    "RightHand", "RWristPitch", "", [0.0, 0.0, 0.0], [0.1, 0.3, -0.25]
)
q_tmp = ps.applyConstraints(q_init)
r(q_tmp)


pp = PathPlayer(robot.client, r)

pp(0)
pp(1)
