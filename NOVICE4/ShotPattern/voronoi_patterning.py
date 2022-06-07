# -*- coding: utf-8 -*-
"""
Created on Thu May 13 08:48:21 2021

@author: msommers
"""


import numpy as np
import pandas as pd
import os, sys
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

import shapely.geometry as sgeom
from shapely.geometry import LineString, MultiLineString, MultiPolygon, Point
from shapely.ops import linemerge, polygonize, snap
from shapely.affinity import translate, scale

from descartes import PolygonPatch
sys.path.insert(0, os.path.dirname(os.getcwd()))

from scipy.spatial import Voronoi, voronoi_plot_2d #ConvexHull

from ShotPattern.polygon_builder import PolygonBuilder
from ShotPattern.kmeans_clustering import KmeansClustering
import ShotPattern.misc_functions



class VoronoiPatterning(object):
    def __init__(self, poly, cluster_centers, repattern_threshold=0.6):
        """
        Takes a polygon and uses n points inside it to partition the polygon 
        into n polygons, where n >= 3.
        
        Parameters
        ----------
        polygon: shapely.geometry.polygon.Polygon instance or PolygonBuilder 
        instance
            Shape to partition.
        cluster_centers: numpy.ndarray of shape Mx2, where M >= 3.
            Points inside polygon used to partition it into multiple polygons.
        
        Returns
        ----------
        VoronoiPatterning object
        """
        
        self.repattern_threshold = repattern_threshold
        self._initialize(poly, cluster_centers)
        
    
    def _initialize(self, poly, cluster_centers):
        self.recreated = False          
        self.poly = poly
        self.poly_area = poly.area      
        self.poly_length = poly.length
        self.cluster_centers = cluster_centers
        self.num_clusters = len(cluster_centers)
        self.poly_scale = 1.0
        
        if self.num_clusters < 3:
            raise Exception("{} requires at least 3 clusters".format(
                __class__.__name__))
        
        self.vor = Voronoi(self.cluster_centers)
        self._create_polys()
    
        
        
    def _create_patch(self, **kwargs):
        """ 
        Creates a patch of the polygon for plotting
        """
        patch = PolygonPatch(self.poly, **kwargs)
        return patch
        
    
    
    def _adjust_bounds(self):
        margin = 0.1 * self.vor.points.ptp(axis=0)
        xy_min = self.vor.points.min(axis=0) - margin
        xy_max = self.vor.points.max(axis=0) + margin
        self.ax.set_xlim(xy_min[0], xy_max[0])
        self.ax.set_ylim(xy_min[1], xy_max[1])
    
    
    
    def update(self, poly):
        """
        Used for updating the algorithm when the polygon is changed:
            
        Gets the original polygon's interior voronoi lines
        Scales and translates these lines to fit the new polygon
        Uses these lines to split new polygon into sub-polygons
        
        If the re-pattern threshold is exceeded during slicing, 
        then it will re-initialize the voronoi algorithm and slice the new 
        polygon into sub-polygons
        """
        self._update_poly_scale(poly)
        self.poly = poly
        self._slice_into_polys(shift=True)  
        
    
    def _get_centroid_xy(self, poly):
        x, y = poly.centroid.x, poly.centroid.y
        return x, y
    
    
    def _update_poly_scale(self, new_poly):
        """ 
        Updating the factor that is used to down-scale the size of the 
        polygon lines. This factor is a perimeter change ratio.
        """
        self.poly_scale = (new_poly.length/self.poly_length)
    
        
    
    def _rescale(self):
        if hasattr(self, "_xlim"):
            self.ax.set_xlim(self._xlim)
            self.ax.set_ylim(self._ylim)
        
        
        
    def _create_polys(self):
        """ 
        Creates the voronoi lines and uses them to slice the polygon
        into sub-polygons
        """
        self._create_vor_lines()
        self._slice_into_polys()


        
    def _create_vor_lines(self, **kw):
        # print("_create_vor_lines()")
        """
        Creates the voronoi lines to be used for polygon splitting
        
        Completes two checks to determine if the correct number of polygons
        will be produced when performing polygon splitting:
            (1) intersects - ensures at least one voronoi line intersects the
                polygon
            (2) crosses - ensures that the end point of that same voronoi line
                is located outside of the polygon
        
        If the two checks are not successful, it will iteratively increase the
        length of the lines until they are successful.
        
        """
        if self.vor.points.shape[1] != 2:
            raise ValueError("Voronoi diagram is not 2-D")
    
        center = self.vor.points.mean(axis=0)
        ptp_bound = self.vor.points.ptp(axis=0)
        
        multiplier = 10     # = 1 for proof of concept
        num_intersections = 0
        while num_intersections < 1: #required_min_intersections:
            num_intersections = 0
            self.all_segments = []
            self.finite_segments = []
            self.infinite_segments = []
            for pointidx, simplex in zip(self.vor.ridge_points, 
                                         self.vor.ridge_vertices):
                simplex = np.asarray(simplex)
                if np.all(simplex >= 0):
                    fin_seg = self.vor.vertices[simplex]
                    l = LineString(fin_seg)
                    self.finite_segments.append(l)
                else:
                    i = simplex[simplex >= 0][0]  # finite end Voronoi vertex
                    
                    # tangent
                    t = self.vor.points[pointidx[1]] - \
                        self.vor.points[pointidx[0]]  
                    t /= np.linalg.norm(t)
                     
                    # normal
                    n = np.array([-t[1], t[0]]) 
        
                    midpoint = self.vor.points[pointidx].mean(axis=0)
                    direction = np.sign(np.dot(midpoint - center, n)) * n
                    if (self.vor.furthest_site):
                        direction = -direction

                    far_point = self.vor.vertices[i] + direction \
                                * ptp_bound.max() * multiplier
                    
                    inf_seg = [self.vor.vertices[i], far_point]
                    l = LineString(inf_seg)
                
                    # If num_clusters = 3, it takes a minimum of 2 lines to 
                    #split the polygon into 3 polygons
                    intersects_bool = l.intersects(self.poly)
                    
                    # Making sure the far point is not inside the polygon
                    fp = Point(far_point)
                    if not self.poly.contains(fp):
                        crosses_bool = True
                    else:
                        crosses_bool = False
                        
                    if intersects_bool and crosses_bool:
                        num_intersections += 1
                    # print("crosses: {}, intersects: {}".format(crosses_bool,
                    #                                        intersects_bool))
                    
                    self.infinite_segments.append(l)
            
            # print("multiplier", multiplier)
            # print()
            multiplier += 10    # += 1 for proof of concept
            
        self.all_segments = self.finite_segments + self.infinite_segments
        self.long_segments = linemerge(self.all_segments)  
        self.trimmed_segments = self.poly.intersection(self.long_segments)
        self.poly_scale = 1.0
        
        
        
    def _slice_into_polys(self, shift=False):
        """ 
        Slicing poly in to sub-polygons with the voronoi lines. 
        Pass shift = True for slicing the new polygon with the original 
        voronoi line pattern.
        """
        # print("\t_slice_into_polys()\tshift={}".format(shift))
       
        default_scale = 1.0  # scale factor for the lines
        self.line_scale = default_scale
        self.ls_used = self.line_scale
        self.recreated = False
        self.sub_polys = []  # initializing
        self.ls_inc = 0.02
        while len(self.sub_polys) != len(self.cluster_centers):
            self._slice(shift)
            # print("\t- Lines scale:", round(self.line_scale,3))
            self.ls_used = self.line_scale
            self.line_scale += self.ls_inc
            
            # If it has to scale the lines more than X%, 
            # re-cluster and re-voronoi
            if self.ls_used >= (1.0 + self.repattern_threshold):
                print("\tRECREATING VORONOI")
                new_kmeans = KmeansClustering(self.poly, 
                                              num_clusters=self.num_clusters)
                new_cluster_centers = new_kmeans.cluster_centers
                self._initialize(self.poly, new_cluster_centers)
                self.recreated = True
                return
        
        # print("\t- TCA scale:", round(self.poly_scale, 3))
        # print("\t- ls:", round(self.ls_used, 3))
        self.line_scale = default_scale
        
        
        
    def _slice(self, shift):
        """
        Slicing the polygon with the voronoi lines.        
        """

        # Shift = True when you want to move the lines' centroid to the 
        # poly centroid
        global unioned, vor_lines
        if shift:
            vor_lines = self._shift_and_scale() #long_segments
        else:
            vor_lines = self.long_segments #self._shift_and_scale()
        
        # Combining the Polygon with the lines for splitting
        unioned = self.poly.boundary.union(vor_lines)
        self.poly_interior_lines = self.poly.intersection(vor_lines)
        
        # Splitting the Polygon into Sub-polygons
        self.sub_polys = [poly for poly in polygonize(unioned)  
                      if poly.representative_point().within(self.poly)]
        
        # Creating a new MultiPolygon
        self.multipoly = MultiPolygon(self.sub_polys)
        
    
    
    def _shift_and_scale(self):
        """ 
        Up-scaling the original polygon's interior voronoi lines by the 
            line_scale.
        Down-scaling those lines by the poly_scale.
        Translating those lines to be centered at the new polygon's centroid.
        """
        # print("\t_shift_and_scale()")
        # The intiial infinite voronoi segments
        long = self.long_segments
        
        # The initial finite voronoi segments
        trimmed = self.trimmed_segments
        
        # Accounts for case with failure to trim long_segments using poly
        if trimmed.is_empty:    
            trimmed = long
        
        # Scale up the trimmed lines by a small factor
        # so that they extend past the poly boundary
        trimmed_s = scale(trimmed, xfact=self.line_scale, 
                          yfact=self.line_scale)
        
        # Scale down the new lines so that they are proportional to the new
        # poly size (self.poly_scale is iteratively increased if the 
        # right number of sub_polys is not produced)
        trimmed_ss = scale(trimmed_s, xfact=self.poly_scale, 
                           yfact=self.poly_scale)
        
        # Translates the new lines so that their centroid is located at the 
        # poly centroid
        xdiff = self.poly.centroid.x - trimmed_ss.centroid.x
        ydiff = self.poly.centroid.y - trimmed_ss.centroid.y
        trimmed_sst = translate(trimmed_ss, xoff=xdiff, yoff=ydiff)
        
        self.trimmed = trimmed_sst   # for debug
        return trimmed_sst    
    
 
    
    def _get_kwargs(self, default_args, **kwargs):
        """ 
        Gets the arguments to use for plotting.
        Arguments in kwargs supercede the default_args.
        """
        plot_args = default_args
        for k,v in kwargs.items():
            plot_args[k] = v
        return plot_args
        

    
    def _get_fig_ax(self, ax=None, **kwargs):
        """ 
        Returns figure, axis on which to plot.
        """
        if ax:
            fig = ax.get_figure()
        elif "ax" in kwargs:
            ax = kwargs["ax"]
            fig = ax.get_figure()
        else:
            figsize = (8,8)
            fig, ax = plt.subplots(figsize=figsize)
        return fig, ax
    

    
    def _rescale_axis(self, ax):
       ax.axis('equal')
       ax.relim()
       ax.autoscale(True)
       return ax
    
    
    
    def plot(self, ax=None, **kwargs):
        """ 
        Plots the polygon and the sub-polygons produced from splitting it
        with the voronoi lines.
        """
        fig, ax = self._get_fig_ax(ax, **kwargs)
        self.plot_poly(ax)
        self.plot_voronoi_polys(ax, **kwargs)
        self.fig = fig
        self.ax = ax
        return ax
    
    
    def plot_centroids(self, ax=None, **kwargs):
        """ 
        Plots the sub-polygon centroids.
        """
        fig, ax = self._get_fig_ax(ax, **kwargs)
        default_args = dict(marker="X", color="black", linestyle="")
        plot_args = self._get_kwargs(default_args, **kwargs)
        for (x,y) in self.cluster_centers:
            ax.plot(x,y, **plot_args)
        return ax
        
    
    
    def plot_poly(self, ax=None, **kwargs):
        """ 
        Plots the original polygon. 
        """
        fig, ax = self._get_fig_ax(ax, **kwargs)
        default_args = dict(fill=False, linewidth=3, edgecolor="black")
        plot_args = self._get_kwargs(default_args, **kwargs)
        patch = PolygonPatch(self.poly, **plot_args)
        ax.add_patch(patch)
        self._rescale_axis(ax)
        return ax
        
    
    
    def plot_voronoi_polys(self, ax=None, **kwargs):
        """ 
        Plots the sub-polygons created from splitting the original polygon 
        with the voronoi lines.
        """
        fig, ax = self._get_fig_ax(ax, **kwargs)
        default_args = dict(fill=True, alpha=0.8)
        plot_args = self._get_kwargs(default_args, **kwargs)
        colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
        while len(colors) < len(self.sub_polys):
            colors += colors
            
        if self.recreated:
            plot_args["edgecolor"] = "white"
            plot_args["linewidth"] = 3
        else:
            plot_args["edgecolor"] = "black"
    
        for i, poly in enumerate(self.sub_polys):
            patch = PolygonPatch(poly, facecolor=colors[i], 
                                 **plot_args)
            poly._patch = patch
            ax.add_patch(patch)
        self._rescale_axis(ax)
        return ax
    
    
    def vor_plot_2d(self, **kwargs):
        """ 
        Calls voronoi_plot_2d() from scipy.spatial._plotutils. 
        """
        voronoi_plot_2d(self.vor, **kwargs)
        fig = plt.gcf()
        ax = fig.gca()
        ax.set_title("Voronoi")
        self._rescale_axis(ax)
        return ax
    
    
    def plot_voronoi_lines(self, ax=None, **kwargs):
        """ 
        Plots the extended voronoi lines, used for slicing the original 
        polygon.
        """
        fig, ax = self._get_fig_ax(ax, **kwargs)
        
        finite_default_args = dict(colors="black", lw=1, alpha=1.0, 
                                   linestyle="solid")
        infinite_default_args = dict(colors="black", lw=1, alpha=1.0,
                                     linestyle='dashed')
        
        finite_plot_args = self._get_kwargs(finite_default_args, **kwargs)
        infinite_plot_args = self._get_kwargs(infinite_default_args, **kwargs)
        
        ax.add_collection(LineCollection(self.finite_segments, 
                                         **finite_plot_args))
                                         
        ax.add_collection(LineCollection(self.infinite_segments,
                                         **infinite_plot_args))
        self._rescale_axis(ax)
        return ax
        
        
    def plot_interior_voronoi_lines(self, ax=None, **kwargs):
        """ 
        Plots the voronoi lines inside the polygon.
        """
        fig, ax = self._get_fig_ax(ax, **kwargs)
        default_args = dict(color="navy", linewidth=3)
        plot_args = self._get_kwargs(default_args, **kwargs)
        for line in self.poly_interior_lines.geoms:
            ax.plot(*line.xy, **plot_args)  
        self._rescale_axis(ax)
        return ax
    
    
    def plot_voronoi_poly_numbers(self, ax=None, **kwargs):
        """ 
        Plots the number labels for the voronoi sub-polygons.
        """
        fig, ax = self._get_fig_ax(ax, **kwargs)
        default_args = dict(color="black", ha="center", va="center")
        plot_args = self._get_kwargs(default_args, **kwargs) 
        for i, poly in enumerate(self.sub_polys, start=1):
            cx, cy = self._get_centroid_xy(poly)
            text = ax.text(cx, cy, i, **plot_args)
        self._rescale_axis(ax)
        return ax
    
    
    def debug_plot(self, ax=None, **kwargs):
        """ 
        Plots the polygon, the extended voronoi lines, 
        the voronoi lines inside the polygon, and the sub-polygon centroids.
        """
        fig, ax = self._get_fig_ax(ax, **kwargs)
        self.plot_poly(ax, fill=True, **kwargs)
        # self.vor_plot_2d(ax=ax, line_colors="orange", zorder=10,
                           # line_width=2.5)
        self.plot_interior_voronoi_lines(ax)
        self.plot_centroids(ax)
        self.plot_voronoi_lines(ax, **kwargs)
        self._rescale_axis(ax)
        return ax    
        
        
        
   
#=============================================================================
# Example
#============================================================================= 

def voronoi_example(num_runs=1, num_clusters=3):
    # poly = PolygonBuilder.create_circle(5)
    poly = PolygonBuilder.create_ellipse(5,10, rotate_cw=30) 
    # poly = PolygonBuilder.create_random_polygon(4, multiplier=100) 
    # poly = PolygonBuilder.create_regular_polygon(6, radius=5, rotate_cw=5)
    # points = [(0,0), (0,10), (10,10), (15,7), (10,0)]
    # poly = PolygonBuilder.create_from_points(points)
    # poly = PolygonBuilder.create_rectangle(10,5, center=(1,1))

    for n in range(num_runs):
        fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, figsize=(8,8))
    
        poly.plot(ax=ax1, linewidth=2, centroid=True)
        poly.ax.set_title("Polygon Builder")
        
        kmeans = KmeansClustering(poly, num_clusters=num_clusters,
                                  init="k-means++")
        kmeans.plot(ax=ax2)
        kmeans.ax.set_title("Kmeans Clustering Algorithm")
        
        vor = VoronoiPatterning(poly, kmeans.cluster_centers)
        vor.debug_plot(ax=ax3)
        ax3.set_title("Voronoi Patterning Algorithm")
        
        vor.plot(ax=ax4)
        vor.ax.set_title("New Polygons")
        
        subpolys = vor.sub_polys
        fig.tight_layout()
    return vor




if __name__ == "__main__":
    
    plt.close("all")
    
    # Example Use
    vor = voronoi_example(num_runs=1, num_clusters=5)
    
    # Standard Plot
    ax = vor.plot()
    ax.set_title("vor.plot()")
    
    # Debug Plot
    ax = vor.debug_plot()
    ax.set_title("vor.debug_plot()")
    
    # Custom Plot
    ax = vor.plot_poly()
    vor.plot_voronoi_polys(ax)
    vor.plot_voronoi_lines(ax, color="navy", linewidth=4)
    vor.plot_interior_voronoi_lines(ax, color="lime", linewidth=4)
    vor.plot_voronoi_poly_numbers(ax, fontsize=15, zorder=2)
    vor.plot_centroids(ax, marker="+", markersize=25,
                       color="cyan", zorder=1)
    ax.set_title("Custom Plot")
  

    