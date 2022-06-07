# -*- coding: utf-8 -*-
"""
Created on Thu May 13 16:46:45 2021

@author: msommers
"""

import os
import numpy as np
from shapely.geometry import MultiLineString, MultiPolygon
from shapely.ops import polygonize
from descartes import PolygonPatch
import matplotlib.pyplot as plt
import shapely
import math

import ShotPattern.misc_functions
from ShotPattern.polygon_builder import PolygonBuilder




class GridPatterning(object):
    def __init__(self, poly, rows=2, cols=2):
        """
        Splits a polygon into multiple polygons using a grid.
        
        Creates a grid with the specified number of rows (r) and columns (c). 
        Based on r and c, it determines whether to use the Minimum Rotated 
        Rectangle (MRR) grid type, or the Centroid grid type.
        
        MRR Grid:
        The MRR is the rectangle generated around the polygon using the shapely
        module. The outer boundary of the grid created is the MRR.
        ** Can be used for any MxN grid.
        
        Centroid Grid:
        The Centroid grid takes the centroid of the polygon and centers the 
        grid at that point. The grid has the same rotated angle as the MRR.
        ** Should only be used for 1x2, 2x1, or 2x2 grids (currently).
        
        Parameters
        ----------
        poly: shapely.Polygon instance, or PolygonBuilder instance
            Shape to split
        rows: int
            Number of rows in grid
        cols: int
            Number of columns in grid
        
        Returns
        ----------
        GridPatterning object
        
        """
    
        self.poly = poly
        self.rows = rows
        self.cols = cols
        
        self.mrr = self.poly.minimum_rotated_rectangle
        self.mrr_angle = self._get_mrr_azimuth()
        self._determine_grid_type()
        self._create_grid()
        self._grid_split_poly()
    
    
    # temp
    def _create_patch(self, **kwargs):
        self.poly.patch = PolygonPatch(self.poly, **kwargs)
    # temp
    def debug_plot(self):
        pass
    
    def update(self, poly):
        self.poly = poly
        self.mrr = self.poly.minimum_rotated_rectangle
        self._create_grid()
        self._grid_split_poly()
        
    
    
    def _determine_grid_type(self):
        """ 
        For 1x1, 1x2, 2x1, or 2x2 grids, use the Centroid grid type
        
        For the following grid dimensions, use the MRR grid type.
        1x3, 1x4, 1x5, ...  (or the reverse dimensions)
        2x3, 2x4, 2x5, ...  (or the reverse dimensions)
        
        """
        num = self.rows * self.cols
        if num == 2:    # if 1x2 or 2x1, use centroid grid
            self.centroid_grid = True
        elif num == 4:
            if self.rows == 2 and self.cols == 2:  # if 2x2, use centroid grid
                self.centroid_grid = True
            else:
                self.centroid_grid = False   # if 4x1 or 1x4, use mrr grid
        else:
            self.centroid_grid = False # if not 1x2, 2x1, or 2x2, use mrr grid
            
        self.mrr_grid = not self.centroid_grid
        # print("MRR Grid") if self.mrr_grid else print("Centroid Grid")
        
    
    def _create_grid(self):
        """
        If the grid type is MRR:
            The grid is created from the Mininmum Rotated Rectangle (MRR) 
            of the polygon. The grid will be THE SAME dimensions and 
            orientation as the MRR, with the appropriate number or rows and 
            columns. Can be used for any MxN grid.
            
        If the grid type is Centroid:
            The grid will be LARGER than the MRR, but will have the same 
            orientation. 
            Should only be used for 1x1, 1x2, 2x1, or 2x2 grids (Currently!)
        """

        rect = shapely.affinity.rotate(self.mrr, self.mrr_angle)
        xmin, ymin, xmax, ymax = rect.bounds
        xbuffer = (xmax-xmin)/2
        ybuffer = (ymax-ymin) /2
        
        if self.centroid_grid:
            x = np.linspace(xmin-xbuffer, xmax+xbuffer, num=self.rows+1)
            y = np.linspace(ymin-ybuffer, ymax+ybuffer, num=self.cols+1)
            hlines = [((x1, yi), (x2, yi)) for x1, x2 in 
                      zip(x[:-1], x[1:]) for yi in y]
            vlines = [((xi, y1), (xi, y2)) for y1, y2 in 
                      zip(y[:-1], y[1:]) for xi in x]
            lines = MultiLineString(hlines + vlines)
            lines = shapely.affinity.rotate(lines, -self.mrr_angle)
            
            grids = list(polygonize(lines))
            grid_mp = MultiPolygon(grids)
            xoff = self.poly.centroid.x - grid_mp.centroid.x 
            yoff = self.poly.centroid.y - grid_mp.centroid.y
            lines = shapely.affinity.translate(lines, xoff, yoff)
                
        if self.mrr_grid:
            x = np.linspace(xmin, xmax, num=self.rows+1)
            y = np.linspace(ymin, ymax, num=self.cols+1)
        
            hlines = [((x1, yi), (x2, yi)) for x1, x2 in 
                      zip(x[:-1], x[1:]) for yi in y]
            vlines = [((xi, y1), (xi, y2)) for y1, y2 
                      in zip(y[:-1], y[1:]) for xi in x]
            lines = MultiLineString(hlines + vlines)
            lines = shapely.affinity.rotate(lines, -self.mrr_angle)
            
            # Makes a list of polygons for each square in the grid
            grids = list(polygonize(lines))
            
            grid_mp = MultiPolygon(grids)
            xoff = self.poly.centroid.x - grid_mp.centroid.x 
            yoff = self.poly.centroid.y - grid_mp.centroid.y
    
        grid_polys = list(polygonize(lines))
        self.grid_lines = lines
        self.grid_polys = grid_polys
        
        
    def _grid_split_poly(self):
        """ Splits the polygon into sub polygons using the Grid """
        self.sub_polys = []
        for gp in self.grid_polys:
             poly = self.poly.intersection(gp)
             if not poly.is_empty:
                 self.sub_polys.append(poly)
        self.multipoly = MultiPolygon(self.sub_polys)
    
    
    
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
    
    
    
    def plot(self, ax=None, show_grid=False, **kwargs):
        """ Plots the sub polygons created from splitting with the grid. """
        fig, ax = self._get_fig_ax(ax, **kwargs)
        if show_grid:
            self._plot_grid(ax)
            self._plot_MRR(ax)
        self.plot_grid_polys(ax, **kwargs)
        self._rescale_axis(ax)
        self.fig = fig
        self.ax = ax
        
        
        
    def _plot_grid(self, ax=None, **kwargs):
        """ Plots the Grid Lines """
        fig, ax = self._get_fig_ax(ax, **kwargs)
        default_args = dict(fill=False, color="black")
        plot_args = self._get_kwargs(default_args, **kwargs)
        for gp in self.grid_polys:
            patch = PolygonPatch(gp, fill=False, color='k')
            ax.add_patch(patch)
        
        
        
    def _plot_MRR(self, ax=None, **kwargs):
        """ Plots the Minimum Rotated Rectangle of the polygon """
        fig, ax = self._get_fig_ax(ax, **kwargs)
        default_args = dict(fill=False, edgecolor="orange")
        plot_args = self._get_kwargs(default_args, **kwargs)
        patch = PolygonPatch(self.mrr, **plot_args)
        ax.add_patch(patch)
        
        
        
    def plot_grid_polys(self, ax=None, **kwargs):
        """ Plots the sub polygons """
        fig, ax = self._get_fig_ax(ax, **kwargs)
        default_args = dict(fill=True, edgecolor="black", alpha=0.8)
        plot_args = self._get_kwargs(default_args, **kwargs)
        colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
        while len(self.sub_polys) > len(colors):
            colors += colors
        cidx = 0
        for sp in self.sub_polys:
            patch = PolygonPatch(sp, facecolor=colors[cidx], **plot_args)
            ax.add_patch(patch)
            cidx += 1
        
        
        
    def _get_mrr_azimuth(self):
        """ Gets the azimuth of the minimum rotated rectangle"""
        bbox = list(self.mrr.exterior.coords)
        axis1 = self._get_dist_between_points(bbox[0], bbox[3])
        axis2 = self._get_dist_between_points(bbox[0], bbox[1])
    
        if axis1 <= axis2:
            az = self._get_azimuth_between_points(bbox[0], bbox[1])
        else:
            az = self._get_azimuth_between_points(bbox[0], bbox[3])
        return az
        
    
    def _get_dist_between_points(self, a, b):
        """ Gets the distance between points"""
        return math.hypot(b[0] - a[0], b[1] - a[1])
        

    def _get_azimuth_between_points(self, p1, p2):
        """ Gets the Azimuth between 2 points"""
        angle = np.arctan2(p2[0] - p1[0], p2[1] - p1[1])
        return np.degrees(angle)
    
    

def example():
    poly = PolygonBuilder.create_random_polygon(5, multiplier=15)
    
    points = [(0,0), (0,10), (10,10), (15,7), (10,0)]
    poly = shapely.geometry.Polygon(points)
    poly = shapely.affinity.rotate(poly, 45)
    
    fig, (ax1,ax2) = plt.subplots(2)
    gp = GridPatterning(poly, 2,2)
    gp.plot(ax=ax1,
                  show_grid=True)
    ax1.set_title("Centroid Grid")
    
    gp = GridPatterning(poly, 2,3)
    gp.plot(ax=ax2,
                  show_grid=True)
    ax2.set_title("MRR Grid")
    
    fig.tight_layout()
    plt.show()
    return gp


if __name__ == "__main__":
    
    # grid = example()
    
    
    # Inputs
    rows = 2
    cols = 2
    save_fig = False
    save_pickle = False
    stacked = True
    pickle_file = r"C:\Users\msommers\Desktop\Output\threat_model\threat_model_data3.pickle"
    
    # Output directories
    outpath = r"C:\Users\msommers\Desktop\Output\grid_patterning"
    pickle_outpath = os.path.join(outpath, "pickle")
    png_outpath = os.path.join(outpath, "png")
    jpg_outpath = os.path.join(outpath, "jpg")
    
    # Reading and reformat data
    raw_data = misc_functions.read_pickle_data(pickle_file)
    data_dict = misc_functions.data_reformatter(raw_data)
    
    # Creating polygon at t0
    keys = sorted([float(x) for x in data_dict.keys()])
    t0  = keys[0]
    poly_data_t0 = data_dict[str(t0)]["TCA_points"]
    poly = PolygonBuilder.create_from_df(poly_data_t0, "x", "y")
     
    # Creating set of desired times at which to plot
    num_keys = len(keys)
    step = int(num_keys/15)   # arbitrary value
    interval = np.arange(0, num_keys, step)
    some_times = []
    for i, k in enumerate(keys):
        if i in interval:
            some_times.append(k)
    
    # Initializing Grid algorithm
    poly_data = data_dict[str(t0)]["TCA_points"]
    poly = PolygonBuilder.create_from_df(poly_data, "x", "y")
    gp = GridPatterning(poly, rows=rows, cols=cols)

    # Creating Grid plot
    plt.close("all")
    fig, ax = plt.subplots(figsize=(10,10))
    gp.plot(ax=ax, linewidth=10)
    
    # Updating Voronoi plot for the desired time values
    for i, t in enumerate(some_times):
        print("\n[Time {}]".format(t))
        data = data_dict[str(t)]
        poly_data = data["TCA_points"]
        new_poly = PolygonBuilder.create_from_df(poly_data, "x", "y")
        
        # If stacked, each update is plotted on top of the previous figure
        if not stacked:
            fig, ax = plt.subplots(figsize=(10,10))
        
        # Updating Voronoi
        gp.update(new_poly)
        gp.plot(ax=ax)
        fig, ax = gp.fig, gp.ax
        
        # Saving Images
        if save_fig:
            if stacked:    
                filepath = os.path.join(png_outpath, "GridStacked{}x{}_{}.png"
                                        .format(rows, cols, t)) 
            else:
                filepath = os.path.join(png_outpath, "Grid{}x{}_{}.png"
                                        .format(rows, cols, t)) 
            fig.savefig(filepath)
            
        # Saving Pickled Figures
        if save_pickle:
            if stacked:
                filename = "GridStacked{}x{}_{}.pickle".format(rows, cols, t)
            else:
                filename = "Grid{}x{}_{}.pickle".format(rows, cols, t)
            misc_functions.pickle_figure(fig, pickle_outpath, filename)
        
        plt.show()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
