import pybullet as p
import pybullet_data

from robot_descriptions.loaders.pybullet import load_robot_description

physicsClient = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setRealTimeSimulation(0)
p.setGravity(0, 0, -9.8)

p.configureDebugVisualizer(p.COV_ENABLE_PLANAR_REFLECTION, 1)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

stadium = p.loadSDF("stadium.sdf")
plane = p.loadURDF("plane.urdf", [0, 0, 0.01])
soccerball = p.loadURDF("soccerball.urdf", [0, 0, 0.21], globalScaling = 0.4)

p.getVisualShapeData(plane)
p.changeVisualShape(plane, -1, rgbaColor=[1, 1, 1, 0])

startPosBall = [0, 0, 0.21]
startOrientationBall = p.getQuaternionFromEuler([0, 0, 0])
p.changeDynamics(soccerball, -1, linearDamping = 0, angularDamping = 0,
                 rollingFriction = 0.001, spinningFriction = 0.001)

robot = load_robot_description("atlas_v4_description")
startPosRobot = [-3, 0, 1.01]
startOrientationRobot = p.getQuaternionFromEuler([0, 0, 0])
p.resetBasePositionAndOrientation(robot, startPosRobot, startOrientationRobot)

num_links = p.getNumJoints(robot)
for link_index in range(num_links):
    p.changeVisualShape(robot, -1, rgbaColor=[1, 0.3, 0.3, 1])
    p.changeVisualShape(robot, link_index, rgbaColor=[1, 0.3, 0.3, 1])

box_shape_X = p.createCollisionShape(p.GEOM_BOX, halfExtents=[26.5, 0.1, 0.5])
box_shape_Y = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 5, 0.5])

boxes_data = [{ 'shape_index': box_shape_X, 'position': [0, 12.5, 0.5] },
    { 'shape_index': box_shape_X, 'position': [0, -12.5, 0.5] },
    { 'shape_index': box_shape_Y, 'position': [-26.5, -7.5, 0.5] },
    { 'shape_index': box_shape_Y, 'position': [26.5, 7.5, 0.5] },
    { 'shape_index': box_shape_Y, 'position': [-26.5, 7.5, 0.5] },
    { 'shape_index': box_shape_Y, 'position': [26.5, -7.5, 0.5] }]

boxes = []

for data in boxes_data:
    box = p.createMultiBody(
        baseInertialFramePosition=[0, 0, 0],
        baseCollisionShapeIndex=data['shape_index'],
        basePosition=data['position'],
        baseOrientation=[0, 0, 0, 1],
        useMaximalCoordinates=False
    )
    p.changeVisualShape(box, -1, rgbaColor=[0.75, 0.75, 0.75, 1])
    boxes.append(box)