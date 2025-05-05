#!/usr/bin/env python3

import numpy as np
import random
import cv2
import matplotlib.pyplot as plt

class RRT:
    def __init__(self,start,end,map,max_steps=2000,step_size=7):
        self.start = start
        self.end = end
        self.map = map
        self.max_steps = max_steps
        self.step_size = step_size
        self.V = [start]
        self.V_set = set()
        self.V_set.add(self.start)
        self.E = []
        self.h,self.w = map.shape
        self.map = self.obstacle_clearence()
        self.parent_map = {}


    def obstacle_clearence(self,radius=2):
        map_uint8 = self.map.astype(np.uint8)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*radius + 1, 2*radius + 1))
        inflated = cv2.dilate(map_uint8, kernel)
        return inflated

    def sample_free(self):
        while True:
            x = random.randint(0,self.w-1)
            y = random.randint(0,self.h-1)
            if self.map[y,x] == 0:
                return (x,y)
            
    def nearest(self,x_rand):
        return min(self.V, key=lambda v: np.linalg.norm(np.array(v) - np.array(x_rand)))

    def steer(self,x_near,x_rand):
        dir_vector = np.array(x_rand) - np.array(x_near)
        length = np.linalg.norm(dir_vector)
        if length == 0:
            return x_near
        direction = dir_vector/length * min(self.step_size,length)
        x_new = tuple(np.array(x_near)+direction.astype(int))
        return x_new
    
    def obstacle_free(self,x1,x2):
        x1, x2 = np.array(x1), np.array(x2)
        for alpha in np.linspace(0, 1, num=20):
            point = x1 + alpha * (x2 - x1)
            px, py = int(point[0]), int(point[1])
            if px < 0 or py < 0 or px >= self.w or py >= self.h or self.map[py, px] != 0:
                return False
        return True
    
    def build(self):
        self.parent_map = {}
        for _ in range(self.max_steps):
            x_rand = self.sample_free()
            x_nearest = self.nearest(x_rand)
            x_new = self.steer(x_nearest, x_rand)
            if x_new == x_nearest or x_new in self.V_set:
                continue
            if self.obstacle_free(x_nearest, x_new):
                self.V.append(x_new)
                self.V_set.add(x_new)
                self.E.append((x_nearest, x_new))
                self.parent_map[x_new] = x_nearest
                if np.linalg.norm(np.array(x_new) - np.array(self.end)) < self.step_size:
                    if self.obstacle_free(x_new, self.end) and self.end not in self.parent_map:
                        self.V_set.add(self.end)
                        self.E.append((x_new, self.end))
                        self.V.append(self.end)
                        self.parent_map[self.end] = x_new
                        return True
        return False
    
    def path_finder(self):
        success = self.build()
        if not success or self.end not in self.parent_map:
            print("Path not found.")
            return None
        path = [self.end]
        current = self.end
        visited = set()

        while current != self.start:
            visited.add(current)
            current = self.parent_map[current]
            path.append(current)
        path.reverse()
        return path
        
def draw_shapes(grid, shapes):

    for shape_type, params in shapes.items():
        if shape_type == 'circle':
            for x,y in params:
                cv2.circle(grid, (x, y), 1, color=1, thickness=-1)
            for i in range(len(params) - 1):
                pt1 = params[i]
                pt2 = params[i + 1]
                cv2.line(grid, pt1, pt2, color=1, thickness=1)
    return grid

def display_grid(grid):
    plt.matshow(grid, cmap='gray_r')
    plt.axis('off')
    plt.show()
    

if __name__ == "__main__":
    # grid = np.load('assignment-5-rrt-satvikmetla/map/map.npy')
    # x,y = -0.7,0
    # sx = int((x +3) / 0.05)
    # sy = int((3-y) / 0.05)
    # gx = int((2 +3) / 0.05)
    # gy = int((3-1.8) / 0.05)
    # print(gx,gy)
    # rrt = RRT((sx,sy),(gx,gy),grid)
    # # print(rrt.obstacle_free((96,21),(90,20)))
    # path = rrt.path_finder()
    # print(path)
    # shapes = {
    #     'circle' : path
    # }
    # grid = draw_shapes(grid,shapes)
    # display_grid(grid)
    # display_grid(rrt.map)
    pass
