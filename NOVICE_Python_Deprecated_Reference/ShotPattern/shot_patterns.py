# -*- coding: utf-8 -*-
"""
Created on Thu May 13 08:51:59 2021

@author: msommers
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import math

from descartes import PolygonPatch

from shapely.geometry import LineString, MultiLineString, MultiPolygon, \
    MultiPoint, Point
from shapely.ops import unary_union

import misc_functions
from polygon_builder import PolygonBuilder
from kmeans_clustering import KmeansClustering
from voronoi_patterning import VoronoiPatterning
from grid_patterning import GridPatterning
from cover_center import CoverCenter
import config


import pickle




class PatternShots(object):
    def __init__(self, poly_TCA, num_shots=None, num_to_cover="all"):
        """
        This class implements algorithms for breaking up a TCA into multiple
        regions and patterning shots to cover these regions.
        
        Parameters
        ----------
        poly_TCA : PolygonBuilder instance
            Polygon representing the Threat CONTAINMENT Area (TCA)
        
        num_shots : int, optional
            Number of shots to pattern. 
            
            If this parameter is not passed, num_shots is determined by 
            computing the ratio of TCA area to LDC area.
            
            Note: For the GridPatterning TCA Algorithm,
            PatternShots may not produce this exact number of shots.
            
            The default is None.

        Returns
        -------
        PatternShots object

        """
        # poly_TCA probably be an input in the future
        self.poly_TCA = poly_TCA        
        # num_shots might be temporary (for when you want to test)
        self.num_shots = num_shots
        #  num_to_cover is temporary (only applies to CoverCenter)
        self.num_to_cover = num_to_cover    
        
        self._init_config()
        self._create_initial_LDC_poly()
        self._determine_num_shots()
        self._apply_TCA_alg()
        self._create_LDC_polys() 
        self._apply_LDC_alg()
        self._calculate_containment()
        
        self.initial_TCA_area = self.poly_TCA.area
    
    
    def update(self, new_tca_poly):
        # Calculate the change area from initial TCA to current TCA
        # Will be used for line scaling during TCA splitting process
        self.poly_TCA = new_tca_poly
        self._update_TCA_alg()
        self._create_LDC_polys() 
        self._apply_LDC_alg()
        self._calculate_containment()
        
        
    def _update_TCA_alg(self):
        print('TCA Algorithm: VoronoiPatterning')
        self.TCA_alg.update(self.poly_TCA)
        self.poly_TCAs = self.TCA_alg.sub_polys
        self.mpoly_TCA = MultiPolygon(self.poly_TCAs)
         
  
    def _init_config(self):
        self.TCA_algorithm = config.TCA_algorithm
        self.LDC_algorithm = config.LDC_algorithm


    def _determine_num_shots(self):
        tca_area = self.poly_TCA.area
        ldc_area = self.poly_initial_LDC.area
        ratio = tca_area/ldc_area
        
        # Overwriting the calculated number of shots
        if not self.num_shots:
            self.num_shots = math.ceil(ratio)
            
        self.num_clusters = self.num_shots
        print("Num shots: {}".format(self.num_shots))
        

    # LDC creation is currently hardcoded - points will be inputs later on
    def _create_initial_LDC_poly(self):
        self.temp_radius = 910
        self.temp_num_sides = 8
        self.temp_rotate = 45
        self.poly_initial_LDC = PolygonBuilder.create_regular_polygon(
                                        self.temp_num_sides, 
                                        radius=self.temp_radius, 
                                        rotate_cw=self.temp_rotate)
    
        
    def _apply_TCA_alg(self):
        # Voronoi requires 3 or more clusters; if num < 3, use GridPatterning
        if issubclass(self.TCA_algorithm, VoronoiPatterning) and \
            self.num_clusters < 3:
                self.TCA_algorithm = GridPatterning
        
        if issubclass(self.TCA_algorithm, VoronoiPatterning):
            print('TCA Algorithm: VoronoiPatterning')
            kmeans = KmeansClustering(self.poly_TCA, 
                                      num_clusters=self.num_clusters,
                                      init="k-means++")
            self.TCA_alg = self.TCA_algorithm(self.poly_TCA, 
                                                kmeans.cluster_centers)
            
        elif issubclass(self.TCA_algorithm, GridPatterning):
            print('TCA Algorithm: GridPatterning')
            rows, cols = self._determine_grid_rows_cols()
            # print(rows, cols)
            self.TCA_alg = self.TCA_algorithm(self.poly_TCA,
                                                rows=rows, cols=cols)
        self.poly_TCAs = self.TCA_alg.sub_polys
        self.mpoly_TCA = MultiPolygon(self.poly_TCAs)
        
        
    def _determine_grid_rows_cols(self):
        # Might need better method of doing this
        r = 1
        c = 1
        num = r*c
        ns = self.num_shots
        while num <= ns:
            num = r*c
            if num >= ns:
                break
            r += 1
            num = r*c
            if num >= ns:
                break
            c += 1
            if num >= ns:
                break
        rows = r
        cols = c
        return rows, cols
        
     
    def _setup_fig(self, figsize=None):
        self.fig, self.ax = plt.subplots(figsize=figsize)
        self.TCA_alg.plot(ax=self.ax,
                            show_grid=False)


    # LDC creation is currently hardcoded - points will be inputs later on
    def _create_LDC_polys(self):
        self.poly_LDCs = []
        self.temp_radius -= 50
        for poly in self.poly_TCAs:
            ### added for debug ###
            # square_length = np.sqrt(poly.area)
            # radius = square_length/2
            radius = self.temp_radius
            ldc = self._create_LDC(poly, radius)
            self.poly_LDCs.append(ldc)
        self.poly_LDC = unary_union(self.poly_LDCs)

    # LDC creation is currently hardcoded - points will be inputs later on
    def _create_LDC(self, poly, radius):
        x,y = poly.centroid.x, poly.centroid.y
        return PolygonBuilder.create_regular_polygon(self.temp_num_sides, 
                                                radius, center=(x,y), 
                                                rotate_cw=self.temp_rotate)

    def _apply_LDC_alg(self):
        if issubclass(self.LDC_algorithm, CoverCenter):
            print("LDC Algorithm: CoverCenter")
            self.LDC_alg = self.LDC_algorithm(self.poly_LDCs,
                                         self.poly_TCA, 
                                         self.poly_TCAs,
                                         num_to_cover=self.num_to_cover)
        else:
            raise Exception("No valid LDC Algorithm!")
            
        self.shots = self.LDC_alg.shots
        self.poly_contained = self.shots["Contained"]["combined"]
        self.poly_uncontained = self.shots["Uncontained"]["combined"]
        self.poly_margin = self.shots["Margin"]["combined"]
        
        # MultiPolygons (for potential use later on)
        self.mpoly_LDC = MultiPolygon(self.poly_LDCs)
        self.mpoly_contained = \
            MultiPolygon(self.shots["Contained"]["individual"])
        self.mpoly_margin = MultiPolygon(self.shots["Margin"]["individual"])
    
    
    def _calculate_containment(self):
        contained = self.shots["Contained"]["combined"]
        uncontained = self.shots["Uncontained"]["combined"]
        self.containment_ratio = contained.area / \
            (contained.area + uncontained.area)
        
        
    def plot_shots(self, ax=None):
        """ 
        Creates a plot of the TCA and the LDCs. 
        Shows the Contained, Uncontained, and Margin areas.
        Shows the shifting of the LDCs to cover the TCA center.
        """
        
        # Sets up Figure
        if not ax:
            self._setup_fig(figsize=(8,8))
        else:
            self.fig = ax.figure
            self.ax = ax
            
        ldcs = self.shots["LDC"]
        contained = self.shots["Contained"]
        margin = self.shots["Margin"]
        tcas = self.shots["TCA"]
 
              
        # Plot individual LDC, Contained, and Margin patches
        self._plot_individual_patches(ldcs,
                                      fill=False,
                                      linewidth=1,
                                      edgecolor="black",
                                      label=None)
        
        self._plot_individual_patches(contained,
                                      fill=False,
                                      linewidth=0,
                                      hatch="**",
                                      label="Contained")
        
        self._plot_individual_patches(margin,
                                      fill=True,
                                      facecolor="blue",
                                      edgecolor="black",
                                      label="Margin")
        
        # Plot LDC centroids
        self._plot_centroids(ldcs, tcas, 
                             initial=False, shifted=True, lines=False)
        
        # Plot TCA centroid
        self.ax.plot(tcas["combined"].centroid.x, tcas["combined"].centroid.y,
                      linestyle="", marker="*", markersize=25, 
                      markeredgecolor="black", color="cyan", 
                      label="TCA centroid")
        
        # Plots Combined LDC patch
        self._plot_patch(ldcs["combined"], 
                         fill=False, 
                         linewidth=3,
                         edgecolor="cyan")
        
        # Axis Formatting
        title_string = \
        """
        TCA Area: {} km^2
        # LDCs Covering Center: {} (Required: {}+)
        CONTAINMENT Ratio: {}
        """.format(
            round(tcas["combined"].area/(1000**2),2), 
            ldcs["num_covering_center"], self.LDC_alg.num_to_cover,
            round(self.containment_ratio,2))
    
        self.ax.set_title(title_string)
        self.ax.legend()
        self.ax.relim()
        self.ax.autoscale(True)
        self.ax.set_facecolor("lightgray")
        
        
        
    def _plot_individual_patches(self, polys, label=None, **kwargs):
        patches = 0
        for poly in polys["individual"]:
            patch = self._plot_patch(poly, **kwargs)
            if patch:
                patches += 1
        if patches > 0:
            patch.set_label(label)
    
        
    def _plot_patch(self, poly, **kwargs):
        if self._not_empty(poly):
            patch = PolygonPatch(poly, **kwargs)
            self.ax.add_patch(patch)
            return patch
        else:
            return None

    
    def _not_empty(self, poly):
        return not poly.is_empty 
    
    
    def _plot_centroids(self, ldcs, tcas, 
                        initial=True, shifted=True, lines=True):
        """
        For each LDC, has the option to plot..
            (1) the initial LDC centroid, 
            (2) the shifted LDC centroid(s)
            (3) the line from the initial LDC centroid to the TCA centroid
        """
        tcax = tcas["combined"].centroid.x
        tcay = tcas["combined"].centroid.y
        
        if initial or lines:
            i_cent = None
            for ldcx, ldcy in ldcs["initial_centroids"]:
                if initial:
                    i_cent, = self.ax.plot(ldcx, ldcy, 
                                 linestyle="", marker="o", color="cyan", 
                                 markersize=15, markeredgecolor="black",
                                 zorder=10)
                if lines:
                    line, = self.ax.plot([ldcx, tcax], [ldcy, tcay],
                                         linestyle="--", color="cyan")
            if i_cent:
                i_cent.set_label("Initial LDC centroid")
                        
        if shifted:
            s_cent = None
            for ldcx, ldcy in ldcs["shifted_centroids"]:
                s_cent, = self.ax.plot(ldcx, ldcy, 
                             linestyle="", marker="X", color="cyan", 
                             markersize=15, markeredgecolor="black",
                             zorder=10)
            if s_cent:
                s_cent.set_label("Shifted LDC centroid")
        
            
 



if __name__ == "__main__":
    plt.close("all")
    
    # Output Directories
    outpath = r"C:\Users\msommers\Desktop\Output\shot_patterns"
    pickle_outpath = os.path.join(outpath, "pickle")
    png_outpath = os.path.join(outpath, "png")
    
    # Inputs
    num_shots = 3
    num_to_cover = "all"
    save_fig = False
    save_pickle = False
    TESTING = 1  # 1, 2
    
    
    #=========================================================================
    # For Testing Sim Data 
    #=========================================================================
    if TESTING == 1:
        
        # Reading and reformatting data
        pickle_file = r"C:\Users\msommers\Desktop\Output\threat_model\threat_model_data3.pickle"
        raw_data = misc_functions.read_pickle_data(pickle_file)
        data_dict = misc_functions.data_reformatter(raw_data)
        
        # Creating TCA polygon at t0
        keys = sorted([float(x) for x in data_dict.keys()])
        t0  = keys[0]
        poly_data_t0 = data_dict[str(t0)]["TCA_points"]
        poly = PolygonBuilder.create_from_df(poly_data_t0, "x", "y")
        
        # Initializing ShotPattern
        sp = PatternShots(poly, num_shots=num_shots, num_to_cover=num_to_cover)
        sp.plot_shots()
        fig, ax = sp.fig, sp.ax
        
        # Creating set of desired times at which to plot
        num_keys = len(keys)
        step = int(num_keys/15)
        interval = np.arange(0, num_keys, step)
        some_times = []
        for i, k in enumerate(keys):
            if i in interval:
                some_times.append(k)
        
        # Updating Shot Pattern plot for the desired time values
        for t in some_times:
            time_str = "[Time {}]".format(t) 
            print("\n{}".format(time_str))
            data = data_dict[str(t)]
            poly_data = data["TCA_points"]
            poly = PolygonBuilder.create_from_df(poly_data, "x", "y")
            
            # Updating the TCA/LDC patterning algorithms for new TCA
            sp.update(poly)
            sp.plot_shots()  #ax=ax
            
            fig, ax = sp.fig, sp.ax
            title = ax.get_title()
            ax.set_title( title + time_str)
            
            # Saving Images
            if save_fig:
                filepath = os.path.join(png_outpath, "ShotPattern{}_{}.png"
                                       .format(num_shots, t))
                fig.savefig(filepath)
       
            # Save Pickled Figures
            if save_pickle:
                filename = "ShotPattern{}_{}".format(num_shots, t)
                misc_functions.pickle_figure(fig, pickle_outpath, filename)
       
       
    #=========================================================================
    # For Testing Arbitrary Polygons #
    #=========================================================================
    if TESTING == 2:
        radius = 2500   # dist from center to each vertex
        patterns = []
        linestrings = []
        tca_poly = PolygonBuilder.create_regular_polygon(4, 
                                                         radius=radius, 
                                                         rotate_cw=30)
        sp = PatternShots(tca_poly, num_to_cover=num_to_cover, 
                          num_shots=num_shots)  
        sp.plot_shots()
        rotate = 30
        x, y = 0, 0
        xlim = sp.ax.get_xlim()
        ylim = sp.ax.get_ylim()
        for i in range(5):  # arbitrary
            # Creating a new TCA polygon
            center = (x,y)
            new_tca_poly = PolygonBuilder.create_regular_polygon(4, 
                                                          radius=radius, 
                                                      rotate_cw=rotate)
            new_tca_poly = new_tca_poly.set_center(center)
            sp.update(new_tca_poly)
            sp.plot_shots()
            fig, ax = sp.fig, sp.ax
            ax.set_xlim(xlim)
            ax.set_ylim(ylim)
            
            # Adjusting the TCA size through each iteration
            radius -= 0 #200
            rotate += 0
            
            # Adjusting the TCA center through each iteration
            x += 50
            y += 0  #50
            
            # Saving Images
            if save_fig:
                filepath = os.path.join(png_outpath, "ShotPattern_Test{}.png"
                                        .format(i+1))
                fig.savefig(filepath)
            
            # Saving Pickled Figures
            if save_pickle:
                filename = "ShotPattern_Test{}.pickle".format(i)
                misc_functions.pickle_figure(fig, pickle_outpath, filename)
            
    plt.show()
    
