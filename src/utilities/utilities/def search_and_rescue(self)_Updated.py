def search_and_rescue(self):
    # Step 1: Record original point
    original_pose = self.get_position()

    # Assume checkpoints are ordered and can be fetched like this:
    checkpoint_index = 1
    checkpoints = []

    while True:
        # Step 2: Get checkpoint i
        checkpoint = self.get_check_point(checkpoint_index)
        if checkpoint is None:
            break

        checkpoints.append(checkpoint)

        # Step 3: Go to checkpoint i
        self.run_robot(checkpoint)
        self.is_arrived()

        # Step 4: Detect objects (assume aruco_callback is running in the background)
        self.node.get_logger().info(f"Detecting objects at checkpoint {checkpoint_index}")
        rclpy.spin_once(self.node, timeout_sec=3.0)  # allow callback to populate aruco_queue

        # Step 5: Rescue all detected objects at this checkpoint
        for marker_id, data in list(self.aruco_queue.items()):
            if marker_id in self.aruco_saved:
                continue

            found_location = data["found_location"]
            target_location = data["location"]

            # Go to found location
            self.run_robot(found_location)
            self.is_arrived()

            # Activate magnet before going to grab the object
            self.switch_magnet(True)

            # Go to target pose (where the object is)
            self.run_robot(target_location)
            self.is_arrived()
            self.node.get_logger().info("Waiting 3 seconds to grab object")
            rclpy.sleep(3.0)

            # Return to found location
            self.run_robot(found_location)
            self.is_arrived()

            # Return through checkpoints to original
            for cp in reversed(checkpoints[:checkpoint_index]):
                self.run_robot(cp)
                self.is_arrived()

            # Return to original point
            self.run_robot(original_pose)
            self.is_arrived()

            # Deactivate magnet
            self.switch_magnet(False)

            # Mark as saved
            self.remove_object(marker_id)

        checkpoint_index += 1

    # Step 6: After all checkpoints are visited and objects rescued, return to original point
    for cp in reversed(checkpoints):
        self.run_robot(cp)
        self.is_arrived()

    self.run_robot(original_pose)
    self.is_arrived()
    self.node.get_logger().info("Rescue mission completed.")
