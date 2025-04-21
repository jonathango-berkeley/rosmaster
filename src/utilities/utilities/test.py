from rescue_robot import RescueRobot

robot = RescueRobot()

while True:
    if robot.current_position is not None:
        print(f"current position: {robot.current_position.transform.translation}")
    else:
        print("no position found")

    try:
        print("----- aruco queue ------")
        for key in robot.aruco_queue:
            loc = robot.aruco_queue[key]["location"]
            print(f"{key} location: {loc.transform.translation}")
    except Exception as e:
        print(f"error: {e}")

