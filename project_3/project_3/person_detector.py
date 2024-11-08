#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
import numpy as np

class PersonDetectorNode(Node):
    def __init__(self):
        super().__init__('person_detector')
        
        # Subscribe to laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publish clusters as PointCloud
        self.person_pub = self.create_publisher(
            PointCloud,
            '/person_candidates',
            10
        )
        
        # Parameters for person detection
        self.cluster_distance = 0.01  # Distance between points in a cluster
        self.min_cluster_size = 6    # Minimum points to be considered a person
        self.max_cluster_size = 125 # Maximum points to be considered a person
        
    def scan_callback(self, scan_msg):
        points = self.laser_scan_to_points(scan_msg)
        clusters = self.find_clusters(points)
        person_clusters = self.filter_person_clusters(clusters)
        self.publish_clusters(person_clusters, scan_msg.header.stamp)
        
    def laser_scan_to_points(self, scan):
        angles = np.arange(scan.angle_min, scan.angle_max + scan.angle_increment, scan.angle_increment)
        points = []
        
        for i, r in enumerate(scan.ranges):
            # Filter out invalid measurements and points too close or too far
            if r < scan.range_min or r > scan.range_max or not np.isfinite(r):
                continue
                
            x = r * np.cos(angles[i])
            y = r * np.sin(angles[i])
            points.append([x, y])
        
        return np.array(points)
    
    def find_clusters(self, points):
        if len(points) < 2:
            return []
            
        clusters = []
        remaining_points = points.tolist()
        
        while remaining_points:
            current_cluster = [remaining_points.pop(0)]
            i = 0
            
            while i < len(remaining_points):
                for cluster_point in current_cluster:
                    dist = np.sqrt((remaining_points[i][0] - cluster_point[0])**2 + 
                                 (remaining_points[i][1] - cluster_point[1])**2)
                    
                    if dist <= self.cluster_distance:
                        current_cluster.append(remaining_points.pop(i))
                        i -= 1
                        break
                i += 1
            
            if len(current_cluster) >= self.min_cluster_size:
                clusters.append(np.array(current_cluster))
                
        return clusters
    
    def filter_person_clusters(self, clusters):
        person_clusters = []
        
        for cluster in clusters:
            # Calculate cluster properties
            cluster_size = len(cluster)
            if not (self.min_cluster_size <= cluster_size <= self.max_cluster_size):
                continue
                
            min_x, min_y = np.min(cluster, axis=0)
            max_x, max_y = np.max(cluster, axis=0)
            width = max_x - min_x
            depth = max_y - min_y
            
            area = width * depth
            density = cluster_size / area if area > 0 else 0
            aspect_ratio = width / depth if depth > 0 else 0
            
            is_person = (
                (0.1 <= width <= 0.8) or (0.1 <= depth <= 0.8) #and
                #not (width > 3 or depth > 3)                   
                #0.6 <= aspect_ratio <= 3   # People are roughly circular/oval in lidar view
                #density >= 100 and              # Dense enough to be a person
                #cluster_size >= self.min_cluster_size and  # Minimum size check
                #cluster_size <= self.max_cluster_size      # Maximum size check
            )
            
            if is_person:
                person_clusters.append(cluster)
                print("density: ",density)
        
        return person_clusters

    
    def publish_clusters(self, clusters, timestamp):
        cloud = PointCloud()
        cloud.header.frame_id = "laser"
        cloud.header.stamp = timestamp
        
        for cluster in clusters:
            # Use cluster centroid instead of all points
            centroid = np.mean(cluster, axis=0)
            p = Point32()
            p.x = float(centroid[0])
            p.y = float(centroid[1])
            p.z = 0.0
            cloud.points.append(p)
            
        self.person_pub.publish(cloud)

def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
