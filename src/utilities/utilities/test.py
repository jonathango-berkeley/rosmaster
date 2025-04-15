from rescue_robot import RescueRobot

robot = RescueRobot()

origin = robot.get_position()

input('enter!')

robot.run_robot(origin)