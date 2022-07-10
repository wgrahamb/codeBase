# -*- coding: utf-8 -*-
"""
Created on Wed May 12 11:17:27 2021

@author: msommers
"""

import os, sys
import numpy as np
import pandas as pd
import random

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, RegularPolygon

import shapely
import shapely.affinity
import shapely.geometry as sgeom
from descartes import PolygonPatch
sys.path.insert(0, os.path.dirname(os.getcwd()))





class PolygonBuilder(sgeom.Polygon):
    def __init__(self, poly):
        super(PolygonBuilder, self).__init__(poly)
        """
        Custom subclass of shapely.geometry.polygon.Polygon class.
        
        Contains class methods to allow for various ways of creating 
        PolygonBuilder objects.
        
        Also contains additional methods for things like plotting, and
        translation.

        Parameters
        ----------
        poly : shapely.geometry.polygon.Polygon instance
            
        Returns
        -------
        PolygonBuilder object
        """
        
        # Not sure what to do here currently
        self.poly = poly
        # self = poly   
        
        
    @classmethod
    def create_circle(cls, radius=1, center=(0,0)):
        circle = sgeom.Point(center).buffer(1)
        circle = shapely.affinity.scale(circle, radius, radius) 
        return cls(circle)
    
    @classmethod 
    def create_ellipse(cls, width, height, center=(0,0), rotate_cw=0):
        circle = sgeom.Point(center).buffer(1)
        ellipse = shapely.affinity.scale(circle, width, height) 
        ellipse = shapely.affinity.rotate(ellipse, -rotate_cw)
        return cls(ellipse)
     
    @classmethod
    def create_rectangle(cls, width, height, center=(0,0), rotate_cw=0):
        rectangle = Rectangle(center, width, height, angle=rotate_cw)
        verts = rectangle.get_verts()
        rectangle = sgeom.Polygon(verts)
        return cls(rectangle)
               
    @classmethod
    def create_random_polygon(cls, num_sides, center=(0,0), multiplier=10):
        arr  = cls._create_rand_poly_array(num_sides, center, multiplier)
        poly = sgeom.Polygon(arr)
        return cls(poly)
    
    @classmethod
    def _create_rand_poly_array(cls, num_sides, center=(0,0), 
                                multiplier=10):
        angle_inc_deg = 360/num_sides
        angles_rad = [a*np.pi/180 for a in np.arange(0, 360, angle_inc_deg)]
        poly_points = []
        for a in angles_rad:
            # r = np.random.random()*multiplier
            r = random.randrange(5,10)*multiplier
            x = r*np.cos(a)
            y = r*np.sin(a)
            poly_points.append((x,y))
        poly_arr = np.array(poly_points)
        return poly_arr

    @classmethod 
    def create_regular_polygon(cls, num_sides, radius, 
                               center=(0,0), rotate_cw=0):
        """ Creates regular polygon based on number of sides and radius,
        which is the distance from the center to each vertex. Can also 
        rotate the polygon as needed (in degrees)."""
        poly = RegularPolygon(center, num_sides, radius=radius)
        verts = poly.get_verts()
        reg_poly = sgeom.Polygon(verts)
        reg_poly = shapely.affinity.rotate(reg_poly, -rotate_cw)
        return cls(reg_poly)

    @classmethod
    def create_from_points(cls, points_list):
        poly = sgeom.Polygon(points_list)
        return cls(poly)
    
    @classmethod
    def create_from_df(cls, df, xcol, ycol):   
        points = []
        for x,y in zip(df[xcol], df[ycol]):
            points.append((x,y))
        return cls.create_from_points(points)
       
        
    def create_df_from_poly(self, poly, xcol, ycol):
        x, y = poly.boundary.coords.xy
        df = pd.DataFrame({xcol: x, ycol: y})
        return df
      
    
    def _create_patch(self, **kwargs):
        patch = PolygonPatch(self, **kwargs)
        return patch
    
    
    
    def _get_kwargs(self, default_args, **kwargs):
        """ arguments in kwargs supercede the default_args """
        plot_args = default_args
        for k,v in kwargs.items():
            plot_args[k] = v
        return plot_args
        

    
    def _get_fig_ax(self, ax=None, **kwargs):
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



    def plot(self, ax=None, centroid=False, **kwargs):
        """ Plots the polygon. """
        fig, ax = self._get_fig_ax(ax, **kwargs)
        patch = self._create_patch(**kwargs)
        ax.add_patch(patch)
        if centroid:
            self.plot_centroid(ax)
        self._rescale_axis(ax)
        self.fig = fig
        self.ax = ax
        

        
    def plot_centroid(self, ax=None, **kwargs):
        fig, ax = self._get_fig_ax(ax, **kwargs)
        default_args = dict(marker="x", markersize=7, color="black")
        plot_args = self._get_kwargs(default_args, **kwargs)
        ax.plot(self.centroid.x, self.centroid.y, **plot_args)
        
        

    def set_center(self, new_center):
        """ Moves the polygon centroid to the specified location. """
        old_center = self.centroid
        xdiff = new_center[0] - old_center.x
        ydiff = new_center[1] - old_center.y
        new_points = []
        for x,y in zip(*self.exterior.xy):
            newx, newy = x + xdiff, y + ydiff
            new_points.append((newx, newy))        
        return self.create_from_points(new_points)




#=============================================================================
# Examples
#=============================================================================

def single_plot_example():
    regpoly = PolygonBuilder.create_regular_polygon(7, radius=5, rotate_cw=5)
    regpoly.plot(linewidth=4, centroid=True, label="Regular polygon")
    regpoly.ax.legend()
    
    
def subplots_example():
    # Creating Polygons
    circle = PolygonBuilder.create_circle(5)
    ellipse = PolygonBuilder.create_ellipse(5,10, rotate_cw=30)
    randpoly = PolygonBuilder.create_random_polygon(10, multiplier=2)
    regpoly = PolygonBuilder.create_regular_polygon(7, radius=5, rotate_cw=5)
    pts = [(0,0), (0,10), (10,10), (15,7), (10,0)]
    poly_from_points = PolygonBuilder.create_from_points(pts)
    rectangle = PolygonBuilder.create_rectangle(10,5, center=(1,1))
    square = PolygonBuilder.create_regular_polygon(
                                        4, radius=1*np.sqrt(2), rotate_cw=45)
    # Plotting Polygons
    fig, axes = plt.subplots(nrows=2, ncols=3,
                             figsize=(10,10))
    
    poly_from_points.plot(linewidth=3,
                          facecolor="firebrick",
                          alpha=1.0,
                          ax=axes[0,0],
                          label="Created from points")
    
    regpoly.plot(linestyle="-",
                 linewidth=3,
                 facecolor="green",
                 alpha=0.6,
                 ax=axes[0,1],
                 label="Regular polygon")

    circle.plot(linestyle="--",
                linewidth=1,
                ax=axes[0,2],
                label="Circle")
    
    randpoly.plot(linestyle="dotted",
                  linewidth=3,
                  facecolor="orange",
                  edgecolor="navy",
                  alpha=0.8,
                  ax=axes[1,0],
                  label="Random polygon")
    
    ellipse.plot(linestyle="dashed",
                 linewidth=2,
                 facecolor="cyan",
                 ax=axes[1,1],
                 centroid=True,
                 label="Ellipse")
    
    rectangle.plot(linestyle="-",
                   linewidth=3,
                   facecolor="purple",
                   alpha=1.0,
                   ax=axes[1,2],
                   label="Rectangle")
    
    rectangle.plot_centroid(ax=axes[1,2], 
                            linestyle="",
                            marker="X",
                            markersize=10,
                            color="orange",
                            label="Centroid")
    
    for shape in [poly_from_points, regpoly, circle, 
                  randpoly, ellipse, rectangle]:
        shape.ax.legend()   
    fig.tight_layout()

    
    
def shifting_polygon_example():
    # Shifting the center of Polygon after creation
    circle = PolygonBuilder.create_circle(5)
    circle.plot(fill=False, 
                linewidth=5,
                edgecolor="orange",
                label="Circle")
    circle.plot_centroid(ax=circle.ax, 
                         color="orange",
                         markersize=10)

    new_circle = circle.set_center((2,2))
    new_circle.plot(fill=False,
                    linewidth=5,
                    edgecolor="navy",
                    label="Shifted circle",
                    ax=circle.ax)
    new_circle.plot_centroid(ax=circle.ax, 
                             color="navy",
                             markersize=10)
    circle.ax.grid()
    circle.ax.legend()


def create_poly_from_data_example():
    pts = [(0,0), (0,10), (10,10), (15,7), (10,0)]
    points_df = pd.DataFrame({"x":[p[0] for p in pts],
                              "y":[p[1] for p in pts]})
    poly_from_df = PolygonBuilder.create_from_df(points_df, "x", "y")
    poly_from_points = PolygonBuilder.create_from_points(pts)
    fig, ax = plt.subplots()
    
    poly_from_points.plot(ax=ax,
                          fill=False,
                          color="green", 
                          linewidth=10,
                          zorder=0,
                          label="Created from points")
    
    poly_from_df.plot(ax=ax, 
                      fill=False, 
                      color="blue", 
                      linewidth=5, 
                      zorder=1,
                      label="Created from df")
    ax.legend()  
  
    
  
def create_df_from_poly_example():
    points = [(0,0), (1,0), (1,1), (0,1)]
    pb = PolygonBuilder.create_from_points(points)
    df = pb.create_df_from_poly(pb.poly, "x", "y")
    
    # Verifying the df produces the same result
    fig, (ax1, ax2) = plt.subplots(2)
    pb.plot(ax=ax1)
    pb2 = PolygonBuilder.create_from_df(df, "x", "y")
    pb2.plot(ax=ax2)
    return df
  
    
    
if __name__ == "__main__":
    plt.close('all')
    
    # Example Uses
    subplots_example()
    # single_plot_example()
    # shifting_polygon_example()
    # create_poly_from_data_example()
    # create_df_from_poly_example()
    
    plt.show()

    
    
    