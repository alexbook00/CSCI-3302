from controller import Robot, DistanceSensor, Motor

# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDistanceSensor(psNames[i]))
    ps[i].enable(TIME_STEP)

leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

hitFirst = False
turnedRight = False

def turnAround():
    while robot.step(TIME_STEP) != -1:
    
        psValues = []
        for i in range(8):
            psValues.append(ps[i].getValue())
        
        if psValues[3] >= 100 and psValues[4] >= 100:
            leftMotor.setVelocity(.5 * MAX_SPEED)
            rightMotor.setVelocity(.5 * MAX_SPEED)
            return
        
        leftMotor.setVelocity(0.5 * MAX_SPEED)
        rightMotor.setVelocity(-0.5 * MAX_SPEED)
        
def turnRightAndGo():
    while robot.step(TIME_STEP) != -1:
    
        psValues = []
        for i in range(8):
            psValues.append(ps[i].getValue())
            
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = -0.5 * MAX_SPEED
        
        if psValues[5] >= 125:
            leftSpeed = .5 * MAX_SPEED
            rightSpeed = .5 * MAX_SPEED
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            return
        
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)

# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    # detect obstacles
    front_obstacle = psValues[0] >= 100 or psValues[7] >= 100
    
    if turnedRight == True:
        # print(psValues[5])
        if psValues[5] < 100:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break
    
    # modify speeds according to obstacles
    if front_obstacle:
        if hitFirst == False:
            # turn 180 degress clockwise
            turnAround()
            hitFirst = True
        elif hitFirst == True:
            turnRightAndGo()
            turnedRight = True
        
    leftMotor.setVelocity(0.5 * MAX_SPEED)
    rightMotor.setVelocity(0.5 * MAX_SPEED)