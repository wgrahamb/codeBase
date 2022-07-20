# -*- coding: utf-8 -*-
"""
Created on Wed May 12 16:03:50 2021

@author: msommers
"""

import numpy as np
import pandas as pd
import os, sys
import random

import matplotlib.pyplot as plt

import shapely.geometry as sgeom
from shapely.geometry import MultiPoint

from descartes import PolygonPatch
sys.path.insert(0, os.path.dirname(os.getcwd()))

from sklearn.cluster import KMeans




class KmeansClustering(object):
    def __init__(self, poly, num_clusters=3, num_points=3000, 
                 init="k-means++"):
        """
        Uniformly fills a polygon with points, assigns each point to a
        specific cluster, and creates a convex hull from each cluster's set of 
        points.
        
        Parameters
        ----------
        poly : shapely.geometry.polygon.Polygon instance or PolygonBuilder
        instance
            Polygon to be used for clustering. 
        num_clusters : int, optional
            Number of clusters to create inside the polygon. The default is 3.
        num_points : int, optional
            Number of points to uniformly distribute inside the 
            polygon. The default is 3000.
        init : str, optional
            The clustering algorithm. The default is "k-means++".
            "random" is another option.

        Returns
        -------
        KmeansClustering object
        """
        
        self.poly = poly
        self.poly_patch = PolygonPatch(self.poly, fill=False, linewidth=3)
        self.num_clusters = num_clusters
        self.num_points = num_points
        self.init = init
        self._create_patch(fill=False, linewidth=3,
                          edgecolor="black")    
        self._fill_poly_with_points()
        self._create_clusters()
        self._create_cluster_polys()
        # self._create_cluster_patches()
        
    
    
    def _create_patch(self, **kwargs):
        self.poly.patch = PolygonPatch(self.poly, **kwargs)
    
    
  
    def _fill_poly_with_points(self):
        self.points_in_poly = []
        for i in range(self.num_points):
            rp = self._get_random_point_in_polygon(self.poly)
            x,y = rp.xy
            self.points_in_poly.append((x[0], y[0]))
        self.points_in_poly = np.array(self.points_in_poly)
        
    
        
    def _get_random_point_in_polygon(self, poly):
        minx, miny, maxx, maxy = poly.bounds
        while True:
            p = sgeom.Point(random.uniform(minx, maxx), 
                            random.uniform(miny, maxy))
            if poly.contains(p):
                return p
    
    
        
    def _create_clusters(self):
        # init vals:"k-means++" or "random"
        self.kmeans = KMeans(n_clusters=self.num_clusters, random_state=0, 
                             init=self.init) 
        
        self.fit = self.kmeans.fit(self.points_in_poly)
        self.cluster_centers = self.fit.cluster_centers_
        self.pred = self.kmeans.predict(self.points_in_poly)
        
        
    def _create_cluster_polys(self):
        self.cluster_polys, self.cluster_multipoints = [],[]
        for val in set(self.pred):
            data_idx = np.where(self.pred==val, True, False)
            data = pd.DataFrame(self.points_in_poly, 
                                columns=["x","y"]).loc[data_idx]
            cluster_mp = MultiPoint(
                    [(x,y) for x,y in zip(data["x"], data["y"])])
            shape = cluster_mp.convex_hull
            self.cluster_multipoints.append(cluster_mp)
            self.cluster_polys.append(shape)
 
        
 
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
    
    
    
    def plot_points_distribution(self, ax=None, **kwargs):
        """ Plots the uniform distribuion of points inside the polygon. """
        fig, ax = self._get_fig_ax(ax, **kwargs)
        default_args = dict(linestyle="", marker=".")
        plot_args = self._get_kwargs(default_args, **kwargs)
        for mp in self.cluster_multipoints:
            xs = [p.x for p in mp]
            ys = [p.y for p in mp]
            ax.plot(xs, ys, **plot_args)
        self._rescale_axis(ax)
        return ax


    def plot(self, ax=None, **kwargs):
        """ Plots the polygon with the clusters and their centroids. """
        fig, ax = self._get_fig_ax(ax, **kwargs)
        self.plot_poly(ax=ax)
        self.plot_clusters(ax=ax)
        self.plot_centroids(ax=ax)
        self._rescale_axis(ax)
        self.fig = fig
        self.ax = ax
        plt.show()
        return ax
    
        
    def plot_clusters(self, ax=None, **kwargs):
        fig, ax = self._get_fig_ax(ax, **kwargs)
        default_args = dict(fill=True, edgecolor="black")
        plot_args = self._get_kwargs(default_args, **kwargs)
        for poly in self.cluster_polys:
            patch = PolygonPatch(poly, **plot_args)
            ax.add_patch(patch)
        self._rescale_axis(ax)
        return ax
        
        
    def plot_poly(self, ax=None, **kwargs):
        fig, ax = self._get_fig_ax(ax, **kwargs)
        default_args = dict(fill=False, linewidth=3, edgecolor="black")
        plot_args = self._get_kwargs(default_args, **kwargs)
        patch = PolygonPatch(self.poly, **plot_args)
        ax.add_patch(patch)
        self._rescale_axis(ax)
        return ax


    def plot_centroids(self, ax=None, **kwargs):
        fig, ax = self._get_fig_ax(ax, **kwargs)
        default_args = dict(marker="X", color="black", linestyle="")
        plot_args = self._get_kwargs(default_args, **kwargs)
        for (x,y) in self.cluster_centers:
            ax.plot(x,y, **plot_args)
        self._rescale_axis(ax)
        return ax
        
        
        
    def _rescale_axis(self, ax):
        ax.axis('equal')
        ax.relim()
        ax.autoscale(True)
        return ax


if __name__ == "__main__":
    plt.close("all")
    points = [(0,0), (0,10), (10,10), (15,7), (10,0)]
    poly = sgeom.Polygon(points)
    kmeans = KmeansClustering(poly, num_clusters=5, num_points=2500)
    
    # Standard Plot
    kmeans.plot(markersize=10)
    
    # Custom Plot
    fig, ax = plt.subplots(figsize=(8,8))
    kmeans.plot_poly(ax, fill=False, linewidth=10, facecolor="green")
    kmeans.plot_clusters(ax, fill=False, linewidth=4, edgecolor="black")
    kmeans.plot_points_distribution(ax)
    kmeans.plot_centroids(ax, markersize=10,
                          markeredgecolor="black", markerfacecolor="cyan")
    
    
    plt.show()
    