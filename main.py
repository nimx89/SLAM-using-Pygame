import pygame
import math
import numpy as np
from env import Environment
from sensor import LidarSensor
import random
from queue import PriorityQueue 

# Initialize Pygame
pygame.init()

# Constants
WIDTH, HEIGHT = 800, 600
MAP_DIMENSIONS = (WIDTH, HEIGHT)
FPS = 30

def calc_distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def get_frontier_cells(point_cloud, cell_size=15):
    """Find the frontier cells that are at the boundary of explored and unexplored areas."""

    min_x, min_y = 0, 0
    max_x, max_y = MAP_DIMENSIONS

    grid_width = (max_x - min_x) // cell_size +1
    grid_height = (max_y - min_y) // cell_size +1

    grid = np.zeros((grid_width, grid_height))


    for point in point_cloud:
        grid_x = (point[0] - min_x) // cell_size
        grid_y = (point[1] - min_y) // cell_size
        if 0 <= grid_x < grid_width and 0 <= grid_y < grid_height:
            grid[grid_x][grid_y] = 1

    
    frontier_cells = []
    for x in range(grid_width):
        for y in range(grid_height):
            if grid[x, y] == 1:  # Explored cell
                # Check neighbors
                is_frontier = False
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        nx, ny = x + dx, y + dy
                        if (0 <= nx < grid_width and 0 <= ny < grid_height and 
                            grid[nx, ny] == 0):
                            is_frontier = True
                            break
                    if is_frontier:
                        break
                
                if is_frontier:
                    # Convert back to world coordinates (center of cell)
                    world_x = min_x + (x + 0.5) * cell_size
                    world_y = min_y + (y + 0.5) * cell_size
                    frontier_cells.append((world_x, world_y))
    
    return frontier_cells


def find_path(start, goal, point_cloud, cell_size=15):
    """Find a path using A* algorithm."""
    min_x, min_y = 0, 0
    max_x, max_y = WIDTH, HEIGHT
    
    grid_width = (max_x - min_x) // cell_size + 1
    grid_height = (max_y - min_y) // cell_size + 1
    
    # Mark cells with point cloud points as obstacles with buffer
    obstacles = np.zeros((grid_width, grid_height))
    for point in point_cloud:
        grid_x = (point[0] - min_x) // cell_size
        grid_y = (point[1] - min_y) // cell_size
        if 0 <= grid_x < grid_width and 0 <= grid_y < grid_height:
            obstacles[grid_x, grid_y] = 1
            # Add buffer around obstacles
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    nx, ny = grid_x + dx, grid_y + dy
                    if 0 <= nx < grid_width and 0 <= ny < grid_height:
                        obstacles[nx, ny] = 1
    
    # Convert start and target to grid coordinates
    start_x = int((start[0] - min_x) // cell_size)
    start_y = int((start[1] - min_y) // cell_size)
    target_x = int((goal[0] - min_x) // cell_size)
    target_y = int((goal[1] - min_y) // cell_size)
    
    # Ensure start and target are valid
    if not (0 <= start_x < grid_width and 0 <= start_y < grid_height):
        return []
    if not (0 <= target_x < grid_width and 0 <= target_y < grid_height):
        return []
    
    # A* algorithm
    open_set = PriorityQueue()
    open_set.put((0, (start_x, start_y)))
    came_from = {}
    g_score = {(start_x, start_y): 0}
    
    while not open_set.empty():
        _, current = open_set.get()
        
        if current == (target_x, target_y):
            # Reconstruct path
            path = []
            while current in came_from:
                x, y = current
                path.append((min_x + (x + 0.5) * cell_size, min_y + (y + 0.5) * cell_size))
                current = came_from[current]
            path.reverse()
            return path
        
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                
                neighbor = (current[0] + dx, current[1] + dy)
                nx, ny = neighbor
                
                if not (0 <= nx < grid_width and 0 <= ny < grid_height):
                    continue
                
                if obstacles[nx, ny] == 1:
                    continue
                
                # Calculate g_score (cost from start to neighbor)
                tentative_g = g_score[current] + math.sqrt(dx*dx + dy*dy)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    # This path is better
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + calc_distance(
                        (nx, ny), 
                        (target_x, target_y)
                    )
                    open_set.put((f_score, neighbor))
    
    return []  



# Main function
def main():
    # Create display
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("LiDAR SLAM Simulation")
    clock = pygame.time.Clock()
    
    # Create environment with the floor plan
    environment = Environment(MAP_DIMENSIONS)
    
    # Create LiDAR sensor
    lidar = LidarSensor(environment, range_max=300, uncertainty=2.0)
    
    # Initial robot position and heading
    robot_x, robot_y = 450, 250  # Center of the map
    robot_heading = 0  # Heading in radians
    robot_radius = 15
    robot_speed = 1.5
    rotation_speed = 0.1
    
    # For mapping
    point_cloud = []

    # Add these new variables for automated exploration
    auto_explore = False
    target_position = None
    path = []
    start_position = (robot_x, robot_y)  # Save initial position
    coverage_threshold = 1000  # Adjust based on your map size
    path_update_interval = 50  # Update path every 50 frames
    frame_count = 0
    returning_home = False

    # Game loop
    running = True
    while running:
        frame_count += 1

        # Handle events
        # for event in pygame.event.get():
        #     if event.type == pygame.QUIT:
        #         running = False
            # elif event.type == pygame.KEYDOWN:
            #     if event.key == pygame.K_ESCAPE:
            #         break        
        
            
        # Get keyboard input for robot control
        keys = pygame.key.get_pressed()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_a:  # Press 'A' to toggle auto exploration
                    auto_explore = not auto_explore
                    if auto_explore:
                        target_position = None
                        path = []
                        returning_home = False
        
        # Manual control when auto_explore is off
        if not auto_explore:
            # ... existing manual control code ...
            # Rotate robot
            if keys[pygame.K_LEFT]:
                robot_heading -= rotation_speed
            if keys[pygame.K_RIGHT]:
                robot_heading += rotation_speed
            
            # Move robot forward/backward with collision detection
            if keys[pygame.K_UP]:
                new_x = robot_x + robot_speed * math.cos(robot_heading)
                new_y = robot_y + robot_speed * math.sin(robot_heading)
                if not check_collision(new_x, new_y, environment.get_walls(), robot_radius):
                    robot_x, robot_y = new_x, new_y
            if keys[pygame.K_DOWN]:
                new_x = robot_x - robot_speed * math.cos(robot_heading)
                new_y = robot_y - robot_speed * math.sin(robot_heading)
                if not check_collision(new_x, new_y, environment.get_walls(), robot_radius):
                    robot_x, robot_y = new_x, new_y
        else:
            # Auto exploration logic
            if len(point_cloud) >= coverage_threshold and not returning_home:
                # We've mapped enough, return to start position
                target_position = start_position
                path = find_path((robot_x, robot_y), target_position, point_cloud)
                returning_home = True
            
            # Update path periodically or when we need a new target
            if target_position is None or (frame_count % path_update_interval == 0 and not returning_home):
                # Find frontier cells
                frontier_cells = get_frontier_cells(point_cloud)
                
                if frontier_cells and not returning_home:
                    # Select the best frontier cell (closest or most informative)
                    frontier_cells.sort(key=lambda cell: calc_distance((robot_x, robot_y), cell))
                    target_position = frontier_cells[0]
                    path = find_path((robot_x, robot_y), target_position, point_cloud)
                elif returning_home:
                    # Already returning home
                    pass
                else:
                    # No frontier cells found and not returning home
                    if distance((robot_x, robot_y), start_position) > robot_radius * 2:
                        target_position = start_position
                        path = find_path((robot_x, robot_y), target_position, point_cloud)
                        returning_home = True
                    else:
                        # At home position, exploration complete
                        auto_explore = False
            
            # Follow the path if we have one
            if path:
                # Get the next waypoint
                waypoint = path[0]
                
                # Calculate direction to waypoint
                dx = waypoint[0] - robot_x
                dy = waypoint[1] - robot_y
                target_heading = math.atan2(dy, dx)
                
                # Adjust heading
                heading_diff = (target_heading - robot_heading + math.pi) % (2 * math.pi) - math.pi
                if abs(heading_diff) > 0.1:
                    # Rotate towards the target
                    if heading_diff > 0:
                        robot_heading += min(rotation_speed, heading_diff)
                    else:
                        robot_heading -= min(rotation_speed, -heading_diff)
                else:
                    # Move towards the waypoint
                    new_x = robot_x + robot_speed * math.cos(robot_heading)
                    new_y = robot_y + robot_speed * math.sin(robot_heading)
                    if not check_collision(new_x, new_y, environment.get_walls(), robot_radius):
                        robot_x, robot_y = new_x, new_y
                
                # Check if we've reached the waypoint
                if calc_distance((robot_x, robot_y), waypoint) < robot_radius:
                    path.pop(0)
                    
                    # If we've reached the target and we're returning home
                    if not path and returning_home and calc_distance((robot_x, robot_y), start_position) < robot_radius * 2:
                        auto_explore = False
            else:
                # No path, just explore by rotating and moving forward
                robot_heading += rotation_speed / 2
                new_x = robot_x + robot_speed * math.cos(robot_heading)
                new_y = robot_y + robot_speed * math.sin(robot_heading)
                if not check_collision(new_x, new_y, environment.get_walls(), robot_radius):
                    robot_x, robot_y = new_x, new_y
        
        




        # # Rotate robot
        # if keys[pygame.K_LEFT]:
        #     robot_heading -= rotation_speed
        # if keys[pygame.K_RIGHT]:
        #     robot_heading += rotation_speed
        
        # # # Move robot forward/backward
        # # if keys[pygame.K_UP]:
        # #     robot_x += robot_speed * math.cos(robot_heading)
        # #     robot_y += robot_speed * math.sin(robot_heading)
        # # if keys[pygame.K_DOWN]:
        # #     robot_x -= robot_speed * math.cos(robot_heading)
        # #     robot_y -= robot_speed * math.sin(robot_heading)
        # # Move robot forward/backward with collision detection
        # if keys[pygame.K_UP]:
        #     new_x = robot_x + robot_speed * math.cos(robot_heading)
        #     new_y = robot_y + robot_speed * math.sin(robot_heading)
        #     if not check_collision(new_x, new_y, environment.get_walls(), robot_radius):
        #         robot_x, robot_y = new_x, new_y
                
        # if keys[pygame.K_DOWN]:
        #     new_x = robot_x - robot_speed * math.cos(robot_heading)
        #     new_y = robot_y - robot_speed * math.sin(robot_heading)
        #     if not check_collision(new_x, new_y, environment.get_walls(), robot_radius):
        #         robot_x, robot_y = new_x, new_y

        # Update robot position and get LiDAR scan
        lidar.set_position((robot_x, robot_y), robot_heading)
        scan_data = lidar.scan()
        
        # Add scan points to point cloud
        for distance, intersection, angle in scan_data:
            if intersection:
                # Only add valid scan points and ensure we don't add duplicates
                point = (int(intersection[0]), int(intersection[1]))
                if point not in point_cloud:
                    point_cloud.append(point)
        
        # Draw everything
        # First, draw the environment/map
        screen.blit(environment.get_map(), (0, 0))
        
        # Draw point cloud (the accumulated scan points)
        for point in point_cloud:
            pygame.draw.circle(screen, (150, 50, 150), point, 1)
        
        # Draw LiDAR scan
        lidar.draw(screen)
        
        # Draw the robot
        pygame.draw.circle(screen, (0, 0, 255), (int(robot_x), int(robot_y)), robot_radius)

        # Draw robot direction indicator
        # end_x = robot_x + 20 * math.cos(robot_heading)
        # end_y = robot_y + 20 * math.sin(robot_heading)
        # pygame.draw.line(screen, (255, 0, 0), (robot_x, robot_y), (end_x, end_y), 2)
        
        triangle_size = 10
        point1 = (robot_x, robot_y)
        point2 = (robot_x + triangle_size * math.cos(robot_heading + 2.5), 
                robot_y + triangle_size * math.sin(robot_heading + 2.5))
        point3 = (robot_x + triangle_size * 1.5 * math.cos(robot_heading), 
                robot_y + triangle_size * 1.5 * math.sin(robot_heading))
        point4 = (robot_x + triangle_size * math.cos(robot_heading - 2.5), 
                robot_y + triangle_size * math.sin(robot_heading - 2.5))
        pygame.draw.polygon(screen, (255, 255, 0), [point2, point3, point4])

        if auto_explore:
            # Draw target position if exists
            if target_position:
                pygame.draw.circle(screen, (255, 0, 255), (int(target_position[0]), int(target_position[1])), 5)
            
            # Draw path
            if path and len(path) > 1:
                for i in range(len(path) - 1):
                    pygame.draw.line(screen, (0, 255, 255), 
                                    (int(path[i][0]), int(path[i][1])), 
                                    (int(path[i+1][0]), int(path[i+1][1])), 2)
            
            # Display exploration status
            if returning_home:
                status_text = font.render("Status: Returning Home", True, (0, 0, 0))
            else:
                status_text = font.render(f"Status: Exploring ({len(point_cloud)}/{coverage_threshold} points)", True, (0, 0, 0))
            screen.blit(status_text, (10, 70))
        
        # Display some information
        font = pygame.font.SysFont("Arial", 16)
        pos_text = font.render(f"Position: ({int(robot_x)}, {int(robot_y)})", True, (0, 0, 0))
        heading_text = font.render(f"Heading: {round(robot_heading * 180 / math.pi, 2)}°", True, (0, 0, 0))
        points_text = font.render(f"Map points: {len(point_cloud)}", True, (0, 0, 0))
        
        screen.blit(pos_text, (10, 10))
        screen.blit(heading_text, (10, 30))
        screen.blit(points_text, (10, 50))
        
        # Update display
        pygame.display.flip()
        clock.tick(FPS)

    # Quit Pygame
    pygame.quit()

def check_collision(x, y, walls, robot_radius):
    # Check if the robot would collide with any wall
    for wall in walls:
        x1, y1, x2, y2 = wall
        
        # Calculate the closest point on the line segment to the robot
        line_length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        if line_length == 0:  # If it's a point, not a line
            closest_x, closest_y = x1, y1
        else:
            # Calculate projection of robot position onto wall line
            t = max(0, min(1, ((x - x1) * (x2 - x1) + (y - y1) * (y2 - y1)) / (line_length**2)))
            closest_x = x1 + t * (x2 - x1)
            closest_y = y1 + t * (y2 - y1)
        
        # Calculate distance from the robot to the closest point on the wall
        distance = math.sqrt((x - closest_x)**2 + (y - closest_y)**2)
        
        # If the distance is less than the robot radius, there's a collision
        if distance < robot_radius:
            return True
            
    # Check if the robot is outside the map boundaries
    if x < robot_radius or x > WIDTH - robot_radius or y < robot_radius or y > HEIGHT - robot_radius:
        return True
            
    return False

if __name__ == "__main__":
    main()