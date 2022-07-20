# -*- coding: utf-8 -*-
"""
Created on Wed Jun  9 15:43:42 2021

@author: msommers
"""


import numpy as np        
from shapely.geometry import LineString
from shapely.ops import unary_union
from shapely.prepared import prep



def sort_by_tup_element(tup_list, element):
    """ sorts the list of tuples by a certain element"""
    tup_list.sort(key = lambda x: x[element])
    return tup_list




class CoverCenter(object):
    def __init__(self, TCA_poly, num_to_cover="all"):
        self.TCA_poly = TCA_poly
        self.TCA_subpolys = TCA_poly._sub_polys
        self.LDC_polys = TCA_poly._poly_LDCs
        
        if num_to_cover == None:
            num_to_cover = 0
        
        if (str(num_to_cover).lower() == "all") or (num_to_cover > len(self.LDC_polys)):
            num_to_cover = len(self.LDC_polys)
        self.num_to_cover = num_to_cover
        
        self._get_LDC_centroids()
        self._get_centroid_distances()
        self._sort_LDC_by_distance()
        self._shift_LDC()
        
        
        
    def _get_LDC_centroids(self):
        """
        The initial LDC centroids are located at the centroids of their
        corresponding TCAs.
        """
        self.initial_centroids = [(poly.centroid.x, poly.centroid.y) 
                                      for poly in self.LDC_polys]


    def _get_centroid_distances(self):
        """ 
        Gets the distance from each LDC centroid to the overall TCA centroid 
        """
        # closest = 0
        self.centroid_distances = []
        p2x, p2y = self.TCA_poly.centroid.x, self.TCA_poly.centroid.y
        for ldc in self.LDC_polys:
            p1x, p1y = ldc.centroid.x, ldc.centroid.y 
           
            # Distance from initial LDC centroid to TCA centroid
            dist = np.sqrt( (p2x-p1x)**2 + (p2y-p1y)**2 )
            self.centroid_distances.append((ldc, dist))
    
    
    def _sort_LDC_by_distance(self):
        """ 
        sorts the list of tuples [(LDC, dist), (LDC, dist), ...] 
        by increasing dist
        """
        self.LDC_dists = sort_by_tup_element(self.centroid_distances, 1)
    
    
    def _shift_LDC(self):
        """
        Once the LDCs are sorted by increasing distance from their centroid
        to the TCA centroid...
        
        The LDCs are shifted toward the TCA centroid
        until the number of LDCs covering the center == num_to_cover.
        
        """
        self.centroid_lines = [] # lines from LDC centers to TCA center
        self.shifted_centroids = []
        contained = []
        margin = []
        new_ldc = []
        # print()
        
        num_covering = 0    # initializing num LDCs covering center
        p2x, p2y = self.TCA_poly.centroid.x, self.TCA_poly.centroid.y
        for i, (ldc, dist) in enumerate(self.LDC_dists):    
            # print("\tLDC {}".format(i+1))    ## For debug ##
            
            # Line from LDC centroid to TCA centroid
            p1x, p1y = ldc.centroid.x, ldc.centroid.y
            shiftx, shifty = p1x, p1y
            linestr = LineString([(p1x,p1y), (p2x, p2y)])
            try:
                slope = (p2y-p1y)/(p2x-p1x)
            except:   # accounts for divide by 0 errors (vertical lines)
                slope = 0
            xdiff = p2x-p1x
            ydiff = p2y-p1y
            
            # The xstep for LDC shifting; each shift is 10% closer (arbitrary)
            # in x to the TCA centroid
            pct_shift = 10
            xstep = xdiff/pct_shift   
            yint = p2y - slope*p2x      # y-intercept of line
            # dist = np.sqrt((p2x-p1x)**2 + (p2y-p1y)**2)
            
            
            # Initial check if each LDC is covering the TCA centroid
            center_covered = self._is_covering_center(ldc)
            if center_covered:
                num_covering += 1
                
            # print("\t - Num covering center: ", num_covering) ## For debug ##
            # print("\t - Dist to TCA Centroid: ", round(dist)) ## For debug ##
            shifts = 0
            while num_covering < self.num_to_cover and not center_covered:
                if slope:           # non-vertical lines
                    shiftx += xstep     
                    shifty = slope*shiftx + yint
                else:               # vertical lines
                    ystep = ydiff/pct_shift
                    shifty += ystep
                # print("\t - Shifting LDC {}".format(i+1))
                
                # Shifting LDC polygon by resetting its center location
                # set_center() is method in PolygonBuilder
                shifted_ldc = ldc.set_center( (shiftx, shifty) )            
                linestr = LineString([(p1x, p1y), (shiftx, shifty)])
                
                ldc = shifted_ldc
                center_covered = self._is_covering_center(shifted_ldc)
                if center_covered:
                    num_covering += 1 
                
                # all shifted centroids
                self.shifted_centroids.append((shiftx, shifty))  
                shifts += 1
                
            # print("\t - Shifted LDC {} ({} times)".format(i+1, shifts)) \
                # if shifts>0 else None
                
            self.centroid_lines.append(linestr)
            
            # last shifted centroids (the final LDC centroid location)
            # self.shifted_centroids.append((shiftx, shifty))   
            
            # Keeping track of the Contained, Margin, and LDC polygons
            cont_poly = self.TCA_poly.intersection(ldc)
            marg_poly = ldc.difference(cont_poly)
            if self._not_empty(cont_poly):
                contained.append(cont_poly)
            if self._not_empty(marg_poly):
                margin.append(marg_poly)
            if self._not_empty(ldc):
                new_ldc.append(ldc)
        
        # Combining the shifted LDC polygons into single polygon
        self.LDC_poly = unary_union(new_ldc)
        self.LDC_polys = new_ldc
        
        # Getting the CONTAINMENT polygons
        self.contained_poly = unary_union(contained)
        self.uncontained_poly = self.TCA_poly.difference(self.LDC_poly) 
        self.margin_poly = unary_union(margin)
        
        # Wrapping up data for Plotting (Might should do this differently)
        self.shots = {}
        self.shots["TCA"] = {"individual": self.TCA_subpolys,
                             "combined": self.TCA_poly}
        self.shots["LDC"] = {"individual": new_ldc,
                             "combined": self.LDC_poly,
                             "shifted_centroids": self.shifted_centroids,
                             "initial_centroids": self.initial_centroids,
                             "num_covering_center": num_covering}
        
        self.shots["Contained"] = {"individual": contained,
                                   "combined": self.contained_poly}
        self.shots["Uncontained"] = {"combined": self.uncontained_poly}
        self.shots["Margin"] = {"individual": margin,
                                "combined": self.margin_poly}
        # print()
        
        
    def _not_empty(self, poly):
        return not poly.is_empty
        
    def _is_covering_center(self, ldc):
        """ Returns a bool of whether the LDC is covering the TCA centroid """
        return ldc.contains(self.TCA_poly.centroid)    



    
    

