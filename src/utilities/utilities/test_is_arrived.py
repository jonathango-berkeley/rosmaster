from rescue_robot import RescueRobot

robot = RescueRobot()

initial_position = robot.get_position()

input("move robot to a different location through rviz and hit enter!")

robot.run_robot(initial_position)

while True:
    if robot.is_arrived():
        print("Robot is Arrived")
        break

