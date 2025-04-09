import pygame
import numpy as np
import math
import cv2
from walls import extract_walls

class Environment:
    def __init__(self, map_dimensions):
        # Map dimensions
        self.map_width, self.map_height = map_dimensions
        
        # Colors
        self.white = (255, 255, 255)
        self.black = (0, 0, 0)
        self.gray = (70, 70, 70)
        self.blue = (0, 0, 255)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        
        # Pygame initialization
        self.map_img = pygame.Surface((self.map_width, self.map_height))
        self.map_img.fill(self.white)
        
        # Walls are stored as line segments: [(x1, y1, x2, y2), ...]
        self.walls = self.define_walls()
        
        # Draw walls on the map image
        for wall in self.walls:
            pygame.draw.line(self.map_img, self.black, (wall[0], wall[1]), (wall[2], wall[3]), 2)
    
    
    def define_walls(self):
        # Extract wall segments from the floor plan
        # Each wall is defined by its start and end coordinates: (x1, y1, x2, y2)
        
        # walls = extract_walls('floor_plan.png')

        # #check the dim of walls
        # print("Walls extracted:", len(walls))
        

        walls = [
            # Upper horizontal section
            (200, 140, 320, 140),  # Top horizontal line
            (320, 140, 320, 80),   # Top vertical line
            (320, 80, 520, 80),    # Upper right horizontal line
            (520, 80, 520, 140),   # Upper right vertical line
            (520, 140, 700, 140),  # Upper right horizontal extension
            
            # Left vertical wall
            (200, 140, 200, 430),  # Left wall
            
            # Bottom horizontal section
            (200, 430, 360, 430),  # Bottom left horizontal
            (360, 430, 360, 350),  # Bottom left vertical up
            (360, 350, 440, 350),  # Bottom middle horizontal
            (440, 350, 440, 430),  # Bottom middle vertical down
            (440, 430, 520, 430),  # Bottom right horizontal
            (520, 430, 520, 510),  # Bottom right vertical down
            (520, 510, 700, 510),  # Bottom right horizontal extension
            
            # Right vertical wall
            (700, 140, 700, 510),  # Right wall
            
            # Interior walls/features
            (440, 350, 440, 290),  # Interior wall vertical
            (440, 290, 520, 290),  # Interior wall horizontal
            (520, 290, 520, 430),  # Interior wall vertical down
            
            # Small interior feature/room
            (360, 370, 360, 350),  # Interior feature
            (360, 370, 440, 370),  # Interior feature
            
            # Step in upper right corner
            (650, 140, 650, 180),  # Step vertical
            (650, 180, 700, 180),  # Step horizontal
        ]
        return walls
    




    def get_map(self):
        return self.map_img
    
    def get_walls(self):
        return self.walls