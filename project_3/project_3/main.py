#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor
from project_3.person_detector import PersonDetectorNode
from project_3.person_tracker import PersonTrackerNode

def main(args=None):
    rclpy.init(args=args)
    
    detector = PersonDetectorNode()
    tracker = PersonTrackerNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(detector)
    executor.add_node(tracker)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        detector.destroy_node()
        tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
