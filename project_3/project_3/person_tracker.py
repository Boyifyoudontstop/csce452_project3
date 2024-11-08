#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, Point
import numpy as np

class PersonTrackerNode(Node):
    def __init__(self):
        super().__init__('person_tracker')
        
        # Subscribe to person candidates (PointCloud)
        self.candidates_sub = self.create_subscription(
            PointCloud,
            '/person_candidates',
            self.track_callback,
            10
        )
        
        # Publisher for tracked persons
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/person_markers',
            10
        )
       
        self.tracks = {}
        self.next_id = 1
        self.max_tracking_distance = 1.0
        self.track_history_length = 10000
        
        self.get_logger().info('Person tracker initialized')
    
    def clear_markers(self):
        marker_array = MarkerArray()
        
        # Create delete markers for all existing tracks
        for track_id in self.tracks.keys():
            marker = Marker()
            marker.header.frame_id = "laser"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "person_tracks"
            marker.id = track_id
            marker.action = Marker.DELETE  # This tells RViz to delete this marker
            marker_array.markers.append(marker)
        
        # Publish the delete markers
        if marker_array.markers:
            self.marker_pub.publish(marker_array)
        
        # Clear the tracks dictionary
        self.tracks.clear()
        self.next_id = 1
    
    def track_callback(self, pointcloud):
        # Convert PointCloud to clusters
        points = np.array([[p.x, p.y] for p in pointcloud.points])
        if len(points) == 0:
            return
            
        # Simple clustering of received points
        clusters = self.cluster_points(points)
        centers = [np.mean(cluster, axis=0) for cluster in clusters]
        
        # Match clusters to existing tracks
        assignments = self.associate_detections(centers)
        
        # Update tracks with new detections
        self.update_tracks(assignments, centers, pointcloud.header.stamp)
        
        # Publish updated tracks
        self.publish_tracks()
        
    def cluster_points(self, points, distance_threshold=0.2):
        if len(points) == 0:
            return []
            
        clusters = []
        remaining = list(range(len(points)))
        
        while remaining:
            current = [remaining.pop(0)]
            i = 0
            
            while i < len(remaining):
                for c in current:
                    dist = np.linalg.norm(points[remaining[i]] - points[c])
                    if dist <= distance_threshold:
                        current.append(remaining.pop(i))
                        i -= 1
                        break
                i += 1
            
            clusters.append(points[current])
            
        return clusters
        
    def associate_detections(self, centers):
        if not self.tracks or not centers:
            return {}
            
        assignments = {}
        used_detections = set()
        
        for track_id, track in self.tracks.items():
            if not track['positions']:
                continue
                
            current_pos = track['positions'][-1]
            min_dist = float('inf')
            best_detection_idx = -1
            
            for i, center in enumerate(centers):
                if i in used_detections:
                    continue
                    
                dist = np.linalg.norm(current_pos - center)
                if dist < min_dist and dist < self.max_tracking_distance:
                    min_dist = dist
                    best_detection_idx = i
                    
            if best_detection_idx >= 0:
                assignments[track_id] = best_detection_idx
                used_detections.add(best_detection_idx)
                
        return assignments
        
    def update_tracks(self, assignments, centers, timestamp):
        # Update matched tracks
        for track_id, detection_idx in assignments.items():
            if track_id not in self.tracks:
                self.tracks[track_id] = {'positions': [], 'last_update': None}
            
            self.tracks[track_id]['positions'].append(centers[detection_idx])
            if len(self.tracks[track_id]['positions']) > self.track_history_length:
                self.tracks[track_id]['positions'].pop(0)
            self.tracks[track_id]['last_update'] = timestamp
            
        # Create new tracks for unmatched detections
        for i, center in enumerate(centers):
            if i not in assignments.values():
                self.tracks[self.next_id] = {
                    'positions': [center],
                    'last_update': timestamp
                }
                self.next_id += 1
                
    def publish_tracks(self):
        marker_array = MarkerArray()
        
        for track_id, track in self.tracks.items():
            if not track['positions']:
                continue
                
            marker = Marker()
            marker.header.frame_id = "laser"
            marker.header.stamp = track['last_update']
            marker.ns = "person_tracks"
            marker.id = track_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05
            
            # Generate unique color for this track
            marker.color.r = (track_id * 0.3) % 1.0
            marker.color.g = (track_id * 0.7) % 1.0
            marker.color.b = (track_id * 0.5) % 1.0
            marker.color.a = 1.0
            
            for pos in track['positions']:
                p = Point()
                p.x = float(pos[0])
                p.y = float(pos[1])
                p.z = 0.0
                marker.points.append(p)
                
            marker_array.markers.append(marker)
            
        if marker_array.markers:  # Only publish if we have markers
            self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = PersonTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

