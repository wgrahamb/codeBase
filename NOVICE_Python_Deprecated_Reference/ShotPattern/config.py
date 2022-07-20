# -*- coding: utf-8 -*-
"""
Created on Wed Jun  9 09:10:46 2021

@author: msommers
"""

# from polygon_builder import PolygonBuilder
# from kmeans_clustering import KmeansClustering
from ShotPattern.voronoi_patterning import VoronoiPatterning
from ShotPattern.grid_patterning import GridPatterning
from ShotPattern.cover_center import CoverCenter


#=============================================================================
# TCA Patterning Algorithm
#=============================================================================
TCA_algorithm =  GridPatterning   #GridPatterning   # VoronoiPatterning



#=============================================================================
# LDC Patterning Algorithm
#=============================================================================
LDC_algorithm = CoverCenter
