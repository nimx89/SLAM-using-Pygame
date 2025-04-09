import pygame
import numpy as np
import math

class LidarSensor:
    def __init__(self, environment, range_max=250, uncertainty=0.5):
        self.environment = environment
        self.range_max = range_max
        self.uncertainty = uncertainty
        self.position = (0, 0)
        self.angle = 0  # Robot heading in radians
        self.num_rays = 120  # Number of rays for 360 degree scan
        self.angle_step = 2 * math.pi / self.num_rays  # Angle between each ray
        
        # For visualization
        self.scan_data = []
        
    def set_position(self, position, angle):
        self.position = position
        self.angle = angle
        
    def _cast_ray(self, angle):
        # Start point of the ray is the sensor position
        start_x, start_y = self.position
        
        # End point is at maximum range, in the direction of the ray
        end_x = start_x + self.range_max * math.cos(angle)
        end_y = start_y + self.range_max * math.sin(angle)
        
        # Ray is defined as a line segment from start to end
        ray = (start_x, start_y, end_x, end_y)
        
        closest_intersection = None
        min_distance = float('inf')
        
        # Check for intersection with each wall
        for wall in self.environment.get_walls():
            intersection = self._get_intersection(ray, wall)
            if intersection:
                # Calculate distance to intersection
                dx = intersection[0] - start_x
                dy = intersection[1] - start_y
                distance = math.sqrt(dx*dx + dy*dy)
                
                # Keep track of the closest intersection
                if distance < min_distance:
                    min_distance = distance
                    closest_intersection = intersection
        
        # Add some uncertainty to the range measurement
        if closest_intersection:
            noise = np.random.normal(0, self.uncertainty)
            min_distance += noise
            
            # Ensure distance is not negative due to noise
            min_distance = max(0, min_distance)
        else:
            min_distance = self.range_max
            
        return min_distance, closest_intersection
    
    def _get_intersection(self, ray, wall):
        # Ray: (x1, y1, x2, y2), Wall: (x3, y3, x4, y4)
        x1, y1, x2, y2 = ray
        x3, y3, x4, y4 = wall
        
        # Calculate denominator
        den = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
        
        # Lines are parallel if denominator is zero
        if den == 0:
            return None
            
        # Calculate intersection parameters
        ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / den
        ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / den
        
        # Check if intersection is within both line segments
        if 0 <= ua <= 1 and 0 <= ub <= 1:
            # Calculate intersection point
            x = x1 + ua * (x2 - x1)
            y = y1 + ua * (y2 - y1)
            return (x, y)
        else:
            return None
    
    def scan(self):
        # Reset scan data
        self.scan_data = []
        
        # Cast rays in all directions
        for i in range(self.num_rays):
            # Calculate the angle of this ray
            scan_angle = self.angle + i * self.angle_step
            
            # Normalize angle to [0, 2π)
            scan_angle = scan_angle % (2 * math.pi)
            
            # Cast the ray and get the distance
            distance, intersection = self._cast_ray(scan_angle)
            
            # Store the result
            self.scan_data.append((distance, intersection, scan_angle))
        
        return self.scan_data
    
    def draw(self, screen):
        # Draw the LiDAR scan points
        for distance, intersection, angle in self.scan_data:
            if intersection:
                # Draw a line from sensor to intersection
                pygame.draw.line(screen, (255, 0, 0), self.position, intersection, 1)
                # Draw a small circle at the intersection
                pygame.draw.circle(screen, (0, 255, 0), (int(intersection[0]), int(intersection[1])), 2)