# -*- coding: utf-8 -*-
"""
2022
@author: Huxm
"""

import random
import numpy as np
from scipy.spatial.distance import squareform, pdist


def angle(x, y, rad=True):
    '''the included angle of two vectors'''
    cos_angle = x.dot(y) / (np.linalg.norm(x) * np.linalg.norm(y) + 1e-7)
    ang = np.arccos(cos_angle)
    if not rad:
        ang = ang * 180 / np.pi
    return ang

def norm(x):
    '''normalization'''
    return x / (np.linalg.norm(x) + 1e-7)

def rotate_angle(x, theta, rad=True):
    '''rotate vector x of theta (positive theta for clockwise rotation)'''
    if not rad:
        theta = theta * np.pi / 180
    rotate_matrix = np.array([[np.cos(theta),np.sin(theta)],[-np.sin(theta),np.cos(theta)]]) 
    y = np.dot(rotate_matrix, x)
    return y

def rotate_towards(x, y, p=0.):
    '''rotate unit vector x towards unit vector y of p*theta, 
       where theta is the included angle between x and y'''
    theta = angle(x, y, rad=True)
    x_ = norm( x + np.sin(p*theta) / np.cos((1/2-p)*theta) * norm(y - x) )
    return x_

def cross(x,y):
    '''cross product'''
    if len(x) == 2:
        x = np.append(x,0)
    if len(y) == 2:
        y = np.append(y,0)
    return np.cross(x,y)

def point_in_cir(point=[0,0], cen=[0,0], r=0):
    '''if a point is in the circle centered on cen with radius r'''
    if np.linalg.norm(np.array(point)-np.array(cen)) < r:
        return True
    else:
        return False
    
def f(v, r, theta, form=2):
    '''calculate f function'''
    if form == 1:
        return (np.linalg.norm(v)**2) / (np.linalg.norm(r) + 1e-7) * abs(np.cos(theta))
    if form == 2:
        return np.linalg.norm(v) / (np.linalg.norm(r)**2 + 1e-7) * abs(np.cos(theta))
    if form == 3:
        return np.linalg.norm(v) * np.exp(-np.linalg.norm(r)) * abs(np.cos(theta))

'''--------------------------------------------------------------------------------'''
class Ped:
    def __init__(self, _id, _loc_x, _loc_y, **kwargs):
        self.id = _id    
        self.r = 0.2
        
        if 'r' in kwargs.keys():
            self.r = kwargs['r']
            
        '''desired velocity''' 
        self.speed_des = random.gauss(1.34, 0.26)
        if 'speed_des' in kwargs.keys():
            self.speed_des = kwargs['speed_des']
             
        if 'destination' in kwargs.keys():
            self.destination = np.array(kwargs['destination'])
        self.dir_des = np.array([1,0])
        if 'dir_des' in kwargs.keys():
            self.dir_des = norm(kwargs['dir_des'])
           
        self.v_des = self.dir_des * self.speed_des
        
        '''position and velocity'''
        self.loc = np.array([_loc_x, _loc_y])
        self.v = self.dir_des * self.speed_des
        
        self.v_next = self.v_des
        self.loc_next = np.array([_loc_x, _loc_y])
         

class PedSet:

    '''----------------initialization for N pedestrians------------------------'''
    def __init__(self, N=60, random_lattice=False, w=10.0, w2=6.0):
        self.peds = []
        
        if not random_lattice:
            for i in range(int(N)):
                xbound = w/2 - 0.25; ybound = w2/2 - 0.25
                xx = random.uniform(-xbound, xbound)
                yy = random.uniform(-ybound, ybound)
                '''no overlap'''
                while self.init_overlap([xx,yy]):
                    xx = random.uniform(-xbound, xbound)
                    yy = random.uniform(-ybound, ybound)               
                self.peds.append( Ped(i, xx, yy, dir_des=[1,0], color='blue') )
    
        else:
            ''''''
            positions = np.zeros((25,15,2))
            for i in range(25):
                for j in range(15):
                    positions[i][j] = [-4.8+0.4*i, -2.8+0.4*j]
            positions = positions.reshape((25*15,-1))
            positions = random.sample(list(positions), N)
            for i in range(len(positions)):
                self.peds.append( Ped(i, positions[i][0], positions[i][1], dir_des=[1,0], color='blue') )
        
        self.num = len(self.peds)
        
    def add_peds(self, N=10, w=10.0, w2=6.0):
        '''add N pedestrians'''
        for i in range(int(N)):
            xbound = w/2 - 0.25; ybound = w2/2 - 0.25
            xx = random.uniform(-xbound, xbound)
            yy = random.uniform(-ybound, ybound)
            
            while self.init_overlap([xx,yy]):
                xx = random.uniform(-xbound, xbound)
                yy = random.uniform(-ybound, ybound) 
            
            index = len(self.peds)
            self.peds.append( Ped(index, xx, yy, dir_des=[1,0], color='blue') )
            
        self.num = len(self.peds)
    
    def del_peds(self, N=10):
        '''remove N pedestrians'''
        self.peds = self.peds[:-N]
        self.num = len(self.peds)
        
    def init_overlap(self, point=[0,0], eps=0.):
        for ped in self.peds:
            if point_in_cir(point=point, cen=[ped.loc[0],ped.loc[1]], r=2*ped.r-eps):
                return True
        return False
    
    def get_loc(self):
        loclist = []
        for ped in self.peds:
            loclist.append(ped.loc)
        return np.array(loclist)
    
    def get_velocity(self):
        vlist = []
        for ped in self.peds:
            vlist.append(ped.v)
        return np.array(vlist)
    
    def get_vx(self):
        vlist = []
        for ped in self.peds:
            vlist.append(ped.v[0])
        return np.array(vlist)
    
    def get_vix(self, i):
        return self.peds[i].v[0]
    
    def get_vi(self, i):
        return np.linalg.norm(self.peds[i].v)
    
    def get_v_avg(self):
        '''Average speed along the corridor'''
        v_avg = 0.0
        for ped in self.peds:
            v_avg = v_avg + ped.v[0]
        v_avg = v_avg / len(self.peds)
        return v_avg
    
    def PBdis(self, a, b, w=10.0, w2=6.0, updown=True, lr=True):
        '''Distance between pedestrians under periodic boundary condition'''
        u = a.copy(); v = b.copy()
        r = min( 0.4+10/np.pi/(self.num/60), 1.5)
        if lr:
            if u[0] > (w/2 - r) and v[0] < (-w/2 + r):
                u[0] = u[0] - w
            if u[0] < (-w/2 + r) and v[0] > (w/2 - r):
                u[0] = u[0] + w
        if updown:    
            if u[1] > (w2/2 - r) and v[1] < (-w2/2 + r):
                u[1] = u[1] - w2
            if u[1] < (-w2/2 + r) and v[1] > (w2/2 - r):
                u[1] = u[1] + w2
        return np.linalg.norm(u-v)
    
    def dist_Matrix(self):
        '''the diatance matrix between all pedestrian pairs'''
        loc_list = self.get_loc()
        self.distMatrix = squareform(pdist(loc_list, self.PBdis))
        return self.distMatrix
    
    def r_ij(self, i, j, w=10.0, w2=6.0, updown=True, lr=True):
        r = min(0.4+10/np.pi/(self.num/60), 1.5)
        u = self.peds[i].loc.copy() 
        v = self.peds[j].loc.copy()     
        if lr:
            if u[0] > (w/2 - r) and v[0] < (-w/2 + r):
                u[0] = u[0] - w
            if u[0] < (-w/2 + r) and v[0] > (w/2 - r):
                u[0] = u[0] + w
        if updown:    
            if u[1] > (w2/2 - r) and v[1] < (-w2/2 + r):
                u[1] = u[1] - w2
            if u[1] < (-w2/2 + r) and v[1] > (w2/2 - r):
                u[1] = u[1] + w2
        return u-v
    
    def v_ij(self, i, j):
        return self.peds[j].v - self.peds[i].v 
    
    def theta_ij(self, i, j, rad=True):
        '''the included angle between v_ij and r_ij'''
        return angle(self.v_ij(i, j), self.r_ij(i, j), rad)
    
    def alpha_ij(self, i, j, rad=True):
        '''the included angle between v_i and r_ji'''
        return angle(self.peds[i].v, self.r_ij(j, i), rad) 
    
    def peds_in_view(self, i, view_angle=150):
        '''other pedestrians in i's vision zone'''
        id_set = []
        view_range = 0.4 + 10/np.pi/(self.num/60)
        
        range_set = np.where( self.distMatrix[i] < view_range )[0]
        
        for j in range_set:
            if j != i and self.alpha_ij(i, j, rad=False) < (view_angle / 2):
                id_set.append(self.peds[j])
        return id_set    
    
    def peds_for_speedchoice(self, i, direction):
        '''pedestrians who may affect i's speed choice'''
        id_set = []
        range_set = np.where( self.distMatrix[i] < 2. )[0]
        
        for j in range_set:
            if j != i and angle(direction, self.r_ij(j, i), rad=False) < 75:
                id_set.append(self.peds[j])
        return id_set
    
    def peds_in_touch(self, i):
        '''other pedestrians in touch with i'''
        id_set = []
        range_set = np.where( self.distMatrix[i] < 2 * self.peds[i].r )[0]
        for j in range_set:
            if j != i:
                id_set.append(self.peds[j])
        return id_set
    
    def update_v_des(self):
        for ped in self.peds:
            if hasattr(ped, "destination"):
                ped.dir_des =  norm(ped.destination - ped.loc) 
                ped.v_des = ped.dir_des * self.speed_des
    
    def updatepos_PB(self, w=10.0, w2=6.0, updown=True, lr=True):
        '''update the position under periodic boundary condition'''
        for ped in self.peds:
            if ped.loc[0] < w/2 and ped.loc_next[0] > w/2 and lr:
                ped.loc[0] = ped.loc_next[0] - w   
            elif ped.loc[0] > -w/2 and ped.loc_next[0] < -w/2 and lr:
                ped.loc[0] = ped.loc_next[0] + w
            elif ped.loc[1] < w2/2 and ped.loc_next[1] > w2/2 and updown:
                ped.loc[1] = ped.loc_next[1] - w2   
            elif ped.loc[1] > -w2/2 and ped.loc_next[1] < -w2/2 and updown:
                ped.loc[1] = ped.loc_next[1] + w2
            else:
                ped.loc = ped.loc_next
    
            
    '''--------------Algorithm for pedestrians' movement--------------------'''
    def move(self, p1 = 0., p2 = 0., eps = 0.):
        
        dt = 0.1 # Time step
        self.dist_Matrix() # Update the distance matrix
        self.update_v_des() # Update the desired velocity if needed
        
        for ped in self.peds: 
            ped_id = ped.id

            touchset = self.peds_in_touch(ped_id)
            viewset = self.peds_in_view(ped_id)
            
            '''---------------------direction sub-model------------------------'''
            
            '''desired direction'''
            d0 = norm(ped.v_des)
            
            '''resolve velocity conflict'''    
            d1 = np.zeros(2)
            for others in viewset:
                rij = self.r_ij(ped_id, others.id)
                vij = self.v_ij(ped_id, others.id)
                theta1 = self.theta_ij(ped_id, others.id, rad=True)
                if theta1 < np.pi/2:
                    value = f(vij, rij, theta1, 2)
                    d1 = d1 + value * norm(np.delete(cross(vij, cross(rij,vij)), -1))
            d1 = norm(d1)
            
            '''resolve space conflict'''    
            d2 = np.zeros(2)
            for others in viewset:
                rij = self.r_ij(ped_id, others.id)
                vi = ped.v
                theta2 = angle(-rij, vi)
                value = f(vi, rij, theta2, 2)
                d2 = d2 + value * norm(np.delete(cross(vi, cross(rij,vi)), -1))
            d2 = norm(d2)
            
            '''repulsion when contact'''
            d_rep1 = np.zeros(2)
            for others in touchset:
                rij = self.r_ij(ped_id, others.id)
                d_rep1 = d_rep1 +  3 * (1 - np.linalg.norm(rij) / 2 / ped.r) * norm(rij)  
    
            
            '''determine the final moving direction'''
            dt1 = np.zeros(2); dt2 = np.zeros(2)
            
            if np.linalg.norm(d1) > 0:
                dt1 = rotate_towards(norm(ped.v), d1, p = p1*dt)
            if np.linalg.norm(d2) > 0:    
                dt2 = rotate_towards(norm(ped.v), d2, p = p2*dt)

            d = norm( d0 + dt1 + dt2 + d_rep1 )
                   
            '''----------------------speed sub-model---------------------------'''   
            
            v = ped.speed_des
            
            newset = self.peds_for_speedchoice(ped.id, direction=d)
            
            for others in newset:
                
                rji = self.r_ij(others.id, ped_id)
                theta3 = angle(rji, d, rad=True)
                
                dij = np.linalg.norm(rji) * abs(np.sin(theta3))
                hij = np.linalg.norm(rji) * abs(np.cos(theta3))
                
                r1 = eps * others.speed_des * dt
                # r1 = eps * 1.34 * dt
                
                if dij <= 2*ped.r + r1:
                    v_cut = hij - np.sqrt( (2*ped.r + r1)**2 - dij**2 )
                    v = min(v, max(0., v_cut / 0.5))  
                    
            '''avoid oscillations due to frequent body contact under very high density and small epsilon'''        
            if angle(ped.v_des, d, rad=False) > 90:
                v = min(v, 0.1)  
                
            ped.v_next = d * v
            ped.loc_next = ped.loc + ped.v_next * dt
        
        # update the position under periodic boundary condition  
        self.updatepos_PB() 
        
        for ped in self.peds:
            ped.v = ped.v_next
        
        self.num = len(self.peds)