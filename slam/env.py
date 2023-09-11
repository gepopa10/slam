import math
import pygame

class build_environment:
    def __init__(self, map_dimensions):
        pygame.init()
        self.point_cloud = []
        self.external_map = pygame.image.load('slam/map.png')
        self.map_h, self.map_w = map_dimensions
        self.map_window_name = 'SLAM'
        pygame.display.set_caption(self.map_window_name)
        self.map = pygame.display.set_mode((self.map_w, self.map_h))
        self.map.blit(self.external_map,(0,0))
        # Colors
        self.black = (0,0,0)
        self.grey = (70,70,70)
        self.blue = (0,0,255)
        self.green = (0,255,0)
        self.red = (255,0,0)
        self.white = (255,255,255)

    def angle_distance_to_position(self, distance, angle, robot_position):
        x = distance * math.cos(angle) + robot_position[0]
        y = -distance * math.sin(angle) + robot_position[1]
        return (int(x), int(y))
    
    def data_storage(self, data):
        for element in data:
            point = self.angle_distance_to_position(element[0], element[1], element[2])
            if point not in self.point_cloud:
                self.point_cloud.append(point)

    def show_sensor_data(self):
        self.info_map = self.map.copy()
        for point in self.point_cloud:
            self.info_map.set_at((int(point[0]), int(point[1])), (255, 0, 0))
