import random
import pygame

from slam.sensors import laser_sensor
from slam.env import build_environment
from slam.features_detection import features_detection

def random_color():
    levels = range(32, 256, 32)
    return tuple(random.choice(levels) for _ in range(3))

def main():
    feature_map = features_detection()
    environment = build_environment((600, 1200))
    environment.original_map = environment.map.copy()
    laser = laser_sensor(200, environment.original_map, uncertainty=(0.5, 0.01))
    environment.map.fill((255,255,255))
    environment.info_map = environment.map.copy()
    environment.original_map = environment.map.copy()
    
    running = True
    feature_detection = True
    break_point_index = 0

    while running:
        environment.info_map = environment.original_map.copy()
        feature_detection = True
        break_point_index = 0
        endpoints = [0,0]
        sensor_on = False
        predicted_points_to_draw = []
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if pygame.mouse.get_focused():
                sensor_on = True
            elif not pygame.mouse.get_focused():
                sensor_on = False
        
        if sensor_on:
            position = pygame.mouse.get_pos()
            laser.position = position
            sensor_data = laser.sense_obstacles()
            feature_map.laser_points_set(sensor_data)
            while break_point_index < (feature_map.np - feature_map.pmin):
                seed_segment = feature_map.seed_segment_detection(laser.position, break_point_index)
                if seed_segment == False:
                    break
                else:
                    laser_points_of_segment = seed_segment[0]
                    predicted_points_to_draw = seed_segment[1]
                    indices = seed_segment[2]
                    results = feature_map.seed_segment_growing(indices, break_point_index)
                    if results == False:
                        break_point_index = indices[1]
                        continue
                    else:
                        line_eq = results[1]
                        m, b = results[5]
                        line_seg = results[0]
                        outermost = results[2]
                        break_point_index = results[3]

                        endpoints[0] = feature_map.projection_point_to_line(outermost[0], m, b)
                        endpoints[1] = feature_map.projection_point_to_line(outermost[1], m, b)

                        color = random_color()
                        for point in line_seg:
                            environment.info_map.set_at((int(point[0][0]), int(point[0][1])), (0, 255, 0))
                            pygame.draw.circle(environment.info_map, color, (int(point[0][0]), int(point[0][1])), 2.0)
                        pygame.draw.line(environment.info_map, (255, 0, 0), endpoints[0], endpoints[1], 2)

                        environment.data_storage(sensor_data)
            environment.map.blit(environment.info_map, (0,0))
            pygame.display.update()

if __name__ == "__main__":
    main()
