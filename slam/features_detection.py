import numpy as np
import math
from fractions import Fraction
from scipy.odr import * # orthogonal distance regression

class features_detection:
    def __init__(self):
       self.epsilon = 10
       self.delta = 501
       self.snum = 6 # number of points in the seed segment
       self.pmin = 20 # minimum number of points a seed segment should have
       self.gmax = 20
       self.seed_segments = []
       self.line_segments = []
       self.laser_points = []
       self.line_params = None
       self.np = len(self.laser_points) - 1 # nb of lasers points
       self.lmin = 20 # minimum length of a line segment in pixels
       self.lr = 0 #real length of a line segment
       self.pr = 0 # the number of laser points contained in the line segment

    def distance_point_to_point(self, point1, point2):
        px = (point1[0] - point2[0]) ** 2
        py = (point1[1] - point2[1]) ** 2
        return math.sqrt(px+py)

    def distance_point_to_line(self, params, point):
        # Ax+By+C=0
        A, B, C = params
        return abs(A * point[0] + B * point[1] + C) / math.sqrt(A ** 2 + B ** 2)
    
    # extract two points from a line equation
    def line_to_points(self, m, b):
        x = 5
        y = m * x + b
        x2 = 2000
        y2 = m * x2 + b
        return [(x,y), (x2, y2)]
    
    def transform_line_from_general_to_slope_intercept(self, A, B, C):
        m = -A / B
        b = -C / B
        return m, b

    def transform_line_from_slope_intercept_to_general(self, m, b):
        A, B, C = -m, 1, -b
        if A < 0:
            A, B, C = -A, -B, -C
        
        den_a = Fraction(A).limit_denominator(1000).as_integer_ratio()[1]
        den_c = Fraction(C).limit_denominator(1000).as_integer_ratio()[1]
        
        gcd = np.gcd(den_a, den_c)
        lcm = den_a * den_c / gcd

        A = A* lcm
        B = B* lcm
        C = C* lcm

        return A, B, C
    
    def line_intersect_general(self, params1, params2):
        a1, b1, c1 = params1
        a2, b2, c2 = params2

        x = (c1 * b2 - b1 * c2) / (b1 * a2 - a1 * b2)
        y = (a1 * c2 - a2 * c1) / (b1 * a2 - a1 * b2)

        return x, y
    
    def points_to_line(self, point1, point2):
        m, b = 0, 0
        if point2[0] != point1[0]: # we do not support vertical line
            m = (point2[1] - point1[1]) / (point2[0] - point1[0])
            b = point2[1] - m * point2[0]
        return m, b
    
    def projection_point_to_line(self, point, m, b):
        x, y = point
        m2 = -1 / m # slope of a line perpendicular
        b2 = y - m2 * x
        intersection_x = - (b - b2) / (m - m2)
        intersection_y = m2 * intersection_x + b2
        return intersection_x, intersection_y
    
    def angle_distance_to_position(self, distance, angle, robot_position):
        x = distance * math.cos(angle) + robot_position[0]
        y = -distance * math.sin(angle) + robot_position[1]
        return (int(x), int(y))
    
    def laser_points_set(self, data):
        self.laser_points = []
        for point in data:
            coordinates = self.angle_distance_to_position(point[0], point[1], point[2])
            self.laser_points.append([coordinates, point[1]])
        self.np = len(self.laser_points) - 1

    # define a function (linear here) to fit the data with
    def linear_func(self, params, x):
        m, b = params
        return m * x + b
    
    def odr_fit(self, laser_points):
        x = np.array([i[0][0] for i in laser_points])
        y = np.array([i[0][1] for i in laser_points])

        linear_model = Model(self.linear_func)
        data = RealData(x,y)
        odr_model = ODR(data, linear_model, beta0=[0,0])

        out = odr_model.run()
        m, b = out.beta
        return m, b
    
    def predict_point(self, line_params, sensed_point, robot_pos):
        m, b = self.points_to_line(robot_pos, sensed_point) # this is the laser beam linear equation
        params1 = self.transform_line_from_slope_intercept_to_general(m, b)
        return self.line_intersect_general(params1, line_params)
    
    def seed_segment_detection(self, robot_position, break_point_index):
        flag = True
        self.np = max(0, self.np)
        self.seed_segments = []
        for i in range(break_point_index, (self.np - self.pmin)):
            predicted_points_to_draw = []
            j = i + self.snum
            m, b = self.odr_fit(self.laser_points[i:j])
            params = self.transform_line_from_slope_intercept_to_general(m, b)
            
            for k in range(i, j):
                predicted_point = self.predict_point(params, self.laser_points[k][0], robot_position)
                predicted_points_to_draw.append(predicted_point)
                d1 = self.distance_point_to_point(predicted_point, self.laser_points[k][0])
                if d1 > self.delta:
                    flag = False
                    break
                d2 = self.distance_point_to_line(params, self.laser_points[k][0])
                if d2 > self.epsilon:
                    flag = False
                    break
            
            if flag:
                self.line_params  = params
                return [self.laser_points[i:j], predicted_points_to_draw, (i,j)]
        return False
    
    def seed_segment_growing(self, indices, break_point):
        line_eq = self.line_params
        i, j = indices
        # beginning and final points in the line segment
        pb, pf = max(break_point, i - 1), min(j + 1, len(self.laser_points) - 1)
        while self.distance_point_to_line(line_eq, self.laser_points[pf][0]) < self.epsilon:
            if pf > self.np - 1:
                break
            else:
                m, b = self.odr_fit(self.laser_points[pb:pf])
                line_eq = self.transform_line_from_slope_intercept_to_general(m, b)
                point = self.laser_points[pf][0]
            pf = pf + 1
            next_point = self.laser_points[pf][0]

            if self.distance_point_to_point(point, next_point) > self.gmax: # probably a door or window
                break
        
        pf = pf - 1

        while self.distance_point_to_line(line_eq, self.laser_points[pb][0]) < self.epsilon:
            if pb < break_point:
                break
            else:
                m, b = self.odr_fit(self.laser_points[pb:pf])
                line_eq = self.transform_line_from_slope_intercept_to_general(m, b)
                point = self.laser_points[pb][0]
            pb = pb - 1
            next_point = self.laser_points[pb][0]

            if self.distance_point_to_point(point, next_point) > self.gmax: # probably a door or window
                break

        pb = pb + 1

        line_length = self.distance_point_to_point(self.laser_points[pb][0], self.laser_points[pf][0])
        nb_of_points = len(self.laser_points[pb:pf])

        if(line_length >= self.lmin and nb_of_points >= self.pmin):
            self.line_params = line_eq
            m, b = self.transform_line_from_general_to_slope_intercept(line_eq[0], line_eq[1], line_eq[2])
            self.two_points = self.line_to_points(m, b)
            self.line_segments.append((self.laser_points[pb + 1][0], self.laser_points[pf - 1][0]))
            return [self.laser_points[pb:pf], self.two_points, (self.laser_points[pb + 1][0], self.laser_points[pf - 1][0]), pf , line_eq, (m, b)]
        return False




