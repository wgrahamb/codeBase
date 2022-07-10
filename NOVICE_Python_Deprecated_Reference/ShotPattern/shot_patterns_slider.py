# -*- coding: utf-8 -*-
"""
Created on Thu May 13 08:51:59 2021

@author: msommers
"""

import os
import numpy as np
import pandas as pd
import math
import pickle

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

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




class PatternShots(object):
    def __init__(self, data_dict, num_shots=None, num_to_cover="all", 
                 keep_ax_lims=True, xcol="x", ycol="y",
                 TCA_algorithm=VoronoiPatterning, LDC_algorithm=CoverCenter):
        """
        This class implements algorithms for breaking up a TCA into multiple
        regions and patterning shots to cover these regions.
        
        Time slider allows user to view these Shot Patterns over time.
        Use left and right arrow keys or drag slider to increment time values.
        
        Parameters
        ----------
        data_dict: dict
            
            Required structure:
                data_dict = { time_0: {"TCA_points": points_df,
                                    "LDC_points": [points_df1, points_df2],
                                     other_keys: other_values
                                    },
                              time_1: {"TCA_points": points_df,
                                    "LDC_points": [points_df1, points_df2],
                                     other_keys: other_values
                                    },
                              ...
                              time_n: {...}
                             }
            
        num_shots: int, optional
            Number of shots to pattern. 
            
            If this parameter is not passed, num_shots is determined by 
            computing the ratio of TCA area to LDC area.
            
            Note: For the GridPatterning TCA Algorithm,
            PatternShots may not produce this exact number of shots.
            
            The default is None.

        num_to_cover: int, or "all"
            Number of LDCs required to cover the TCA center.
            
        keep_ax_lims: bool
            If True, the plot will retain the initial axis limits and ticks
            over time.
            If False, the plot will re-scale the axis as needed.
         
        xcol: str
            column name for xdata in TCA and LDC dataframes
        ycol: str
            column name for ydata in TCA and LDC dataframes
            
        Returns
        -------
        PatternShots object

        """
        
        self.data = data_dict
        # num_shots might be temporary (for when you want to test)
        self.num_shots = num_shots
        #  num_to_cover is temporary (only applies to CoverCenter)
        self.num_to_cover = num_to_cover    
        self.keep_ax_lims = keep_ax_lims
        self.xcol = xcol
        self.ycol = ycol
        self.TCA_algorithm = TCA_algorithm
        self.LDC_algorithm = LDC_algorithm
        
        self._create_figure()
        self._get_times()
        self._create_TCA_poly()
        self._add_time_slider()
        
        # self._init_config()
        self._create_initial_LDC_poly()     # temp
        self._determine_num_shots()
        print("[Time {}]\n".format(self.init_time))
        self._apply_TCA_alg()
        
        # self._update(self.init_time)
        ### Could replace these by calling self._update ###
        self._get_current_data()
        self._create_LDC_polys() 
        self._apply_LDC_alg()
        self._collect_shots()
        self._calculate_containment()
        self.TCA_alg.plot(ax=self.ax)
                          
        self.plot_shots()
        ### Could replace these by calling self._update ###
        
        self._get_ax_lims_ticks()
        self._update(self.init_time)
        self._connections()

    

    def _update(self, i):
        self._remove_artists()
        self._get_current_data()
        print("="*25 + "\n")
        print("[Time {}]".format(self.current_time))
        self._create_TCA_poly()
        
        # Might remove if statement, and just call _update_TCA_alg()
        if hasattr(self, "TCA_alg"):
            self._update_TCA_alg()
        else:
            self._apply_TCA_alg()
            
        self._create_LDC_polys() 
        self._apply_LDC_alg()
        self._collect_shots()
        self._calculate_containment()
        self.TCA_alg.plot(ax=self.ax)
        self.plot_shots()
        self._set_ax_lims_ticks()
    
    
                
    def _create_figure(self):
        # plt.close('all')
        self.fig, self.ax = plt.subplots(figsize=(12,12))
        self.ax.set_facecolor("lightgray")
        plt.subplots_adjust(bottom=0.15, left=0.1, right=0.95, top=0.9) #top=0.95)
        self.ax.axis("equal")
        grid_args = {"color":"darkgray","linestyle":"--","linewidth":0.5}
        self.ax.grid(**grid_args)



    def _create_TCA_poly(self):
        # Creating TCA polygon at t0
        keys = sorted([float(x) for x in self.data.keys()])
        t0, tf = keys[0], keys[-1]
        df = self.current_data["TCA_points"]
        # print(df)
        # self.poly_TCA = PolygonBuilder.create_from_df(df, "x", "y")
        # polys = []
        # for df in dfs:
            # tca_poly = PolygonBuilder.create_from_df(df, self.xcol, self.ycol)
            # polys.append(tca_poly)
        # self.poly_TCA = MultiPolygon(polys) #unary_union(polys)
        self.poly_TCA = PolygonBuilder.create_from_df(df, self.xcol, self.ycol)
    
        
    def _get_times(self):
        global times
        for k in self.data.copy().keys():   # ensures keys are floats, not str
            self.data[float(k)] = self.data.pop(k)
        self.times = sorted(list(self.data.keys()))
        self.min_time, self.max_time = min(self.times), max(self.times)
        self.init_time = self.min_time
        self.current_time = self.min_time
        self.current_data = self.data[self.current_time]
        times = self.times
        
        # Determine time_step
        tdiff = pd.Series(self.times).diff()
        min_tdiff = round(np.nanmin(tdiff),3)
        self.time_step = min_tdiff
        
    
    def _add_time_slider(self):
        # [left, bottom, width, height]
        self.ax_time = plt.axes([0.15, 0.05, 0.75, 0.03], 
                           facecolor="lightgoldenrodyellow")
        self.time_slider = Slider(ax=self.ax_time,
                                  label="Time",
                                  valmin=self.min_time,
                                  valmax=self.max_time,
                                  valinit=self.init_time,
                                  valstep=self.time_step,
                                  valfmt = "%1.1f")
        # self._get_polygon_change_times()
        # self._add_slider_ticks()
        self.time_slider.on_changed(self._update)
        
        
    
    def _init_config(self):
        self.TCA_algorithm = config.TCA_algorithm
        self.LDC_algorithm = config.LDC_algorithm

    

    # LDC creation is currently hardcoded - points will be inputs later on
    def _create_initial_LDC_poly(self):
        self.temp_radius = 900
        self.temp_num_sides = 8
        self.temp_rotate = 45
        self.poly_initial_LDC = PolygonBuilder.create_regular_polygon(
                                        self.temp_num_sides, 
                                        radius=self.temp_radius, 
                                        rotate_cw=self.temp_rotate)


    
    def _determine_num_shots(self):
        tca_area = self.poly_TCA.area
        ldc_area = self.poly_initial_LDC.area
        ratio = tca_area/ldc_area
        
        # Overwriting the calculated number of shots
        if not self.num_shots:
            self.num_shots = math.ceil(ratio)
            
        self.num_clusters = self.num_shots
        # print("Num shots: {}".format(self.num_shots))
        


    def _apply_TCA_alg(self):
        # Voronoi requires 3 or more clusters; if num < 3, use GridPatterning
        if issubclass(self.TCA_algorithm, VoronoiPatterning) and \
            self.num_clusters < 3:
                self.TCA_algorithm = GridPatterning
        
        if issubclass(self.TCA_algorithm, VoronoiPatterning):
            print('Initializing TCA Algorithm: VoronoiPatterning')
            kmeans = KmeansClustering(self.poly_TCA, 
                                      num_clusters=self.num_clusters,
                                      init="k-means++")
            self.TCA_alg = self.TCA_algorithm(self.poly_TCA, 
                                                kmeans.cluster_centers)
            
        elif issubclass(self.TCA_algorithm, GridPatterning):
            print('Initializing TCA Algorithm: GridPatterning')
            rows, cols = self._determine_grid_rows_cols()
            # print(rows, cols)
            self.TCA_alg = self.TCA_algorithm(self.poly_TCA,
                                                rows=rows, cols=cols)
        self.poly_TCAs = self.TCA_alg.sub_polys
        self.mpoly_TCA = MultiPolygon(self.poly_TCAs)
        
    
    
    # LDC creation is currently hardcoded - points will be inputs later on
    def _create_LDC_polys(self):
        self.poly_LDCs = []
        
        # Using a desired area of a regular polygon to calculate its desired
        # radius (center to vertext)
        # Polygon area A = n * 0.5bh
        #   n = num_sides 
        #   0.5bh is area of triangle
        
        # theta_deg = angle formed between each vertex and the center
        theta_deg = 360/self.temp_num_sides
        
        # desired area would allow the sum of LDC areas to equal the total TCA area
        desired_ldc_area = self.poly_TCA.area/self.num_shots
        
        # b = side length
        b = np.sqrt( (4*desired_ldc_area*np.tan(np.radians(theta_deg/2))) / 
                    self.temp_num_sides )
        
        # a = length from center to each vertex
        a = (0.5*b)/np.sin(np.radians(theta_deg/2))
        self.temp_radius = a
        # print("\t\tdesired ldc area", desired_ldc_area)
        for poly in self.poly_TCAs:
            radius = self.temp_radius
            ldc = self._create_LDC(poly, radius)
            # print("\t\tldc area", ldc.area)
            
            self.poly_LDCs.append(ldc)
        
        # should I use unary_union or MultiPolygon?
        self.poly_LDC = unary_union(self.poly_LDCs)
        
        
    
    
    def _apply_LDC_alg(self):
        if issubclass(self.LDC_algorithm, CoverCenter):
            print("\nLDC Algorithm: CoverCenter")
            self.LDC_alg = self.LDC_algorithm(self.poly_LDCs,
                                         self.poly_TCA, 
                                         self.poly_TCAs,
                                         num_to_cover=self.num_to_cover)
        else:
            raise Exception("No valid LDC Algorithm!")
         
            
            
    def _collect_shots(self):
        self.shots = self.LDC_alg.shots
        self.poly_contained = self.shots["Contained"]["combined"]
        self.poly_uncontained = self.shots["Uncontained"]["combined"]
        self.poly_margin = self.shots["Margin"]["combined"]
        
        # MultiPolygons (for potential use later on)
        self.mpoly_LDC = MultiPolygon(self.poly_LDCs)
        self.mpoly_contained = \
            MultiPolygon(self.shots["Contained"]["individual"])
        # self.mpoly_margin = MultiPolygon(self.shots["Margin"]["individual"])
    
    
    
    def _calculate_containment(self):
        contained = self.shots["Contained"]["combined"]
        uncontained = self.shots["Uncontained"]["combined"]
        self.containment_ratio = contained.area / \
            (contained.area + uncontained.area)
        
    
    
    def _on_key_press(self, event):
        """
        Can use left or right arrow keys to increment time slider.
        """
        key = event.key
        if key == "n":   # "n"
            next_idx = self.current_idx + 1
            if next_idx <= len(self.times)-1:
                self.time_slider.set_val(self.times[next_idx])
        if key == "b":    # "b"
            previous_idx = self.current_idx -1
            if previous_idx >= 0:
                self.time_slider.set_val(self.times[previous_idx])
    
    
        
    def _get_current_data(self):
        self.current_time = float(self.time_slider.valtext._text)
        if self.current_time in self.times:
            self.current_idx = self.times.index(self.current_time)
        else:
            print('Skipping Missing Time: ', self.current_time)
            self.current_idx = self._get_adjacent_index()
            self.current_time = self.times[self.current_idx]
            self.time_slider.valtext.set_text(self.current_time)
        self.current_data = self.data[self.current_time]
        
        
        
    def _get_adjacent_index(self):
        copy = self.times.copy()
        copy.append(self.current_time)
        sorted_copy = sorted(copy)
        val_idx = sorted_copy.index(self.current_time)-1
        next_idx = val_idx + 1
        if next_idx <= len(self.times) - 1:
            return next_idx    # next index
        else:
            return val_idx -1  # previous index
        
        
        
    def _update_TCA_alg(self):
        print('\nUpdating TCA Algorithm: {}\n'.format(
            self.TCA_algorithm.__name__))
        self.TCA_alg.update(self.poly_TCA)
        self.poly_TCAs = self.TCA_alg.sub_polys
        self.mpoly_TCA = MultiPolygon(self.poly_TCAs)
         
  

    def _get_ax_lims_ticks(self):
        self._xlim, self._ylim = self.ax.get_xlim(), self.ax.get_ylim()
        self._xticks, self._yticks = self.ax.get_xticks(), self.ax.get_yticks()
    
    
    
    def _set_ax_lims_ticks(self):
        if self.keep_ax_lims:
            if hasattr(self, "_xlim"):
                self.ax.set_xlim(self._xlim)
                self.ax.set_ylim(self._ylim)
                self.ax.set_xticks(self._xticks)
                self.ax.set_yticks(self._yticks)
    
    
    
    def _connections(self):
        self.fig.canvas.mpl_connect('key_press_event', self._on_key_press)
        
      
        
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
 
    
 
    # LDC creation is currently hardcoded - points will be inputs later on
    def _create_LDC(self, poly, radius):
        x,y = poly.centroid.x, poly.centroid.y
        return PolygonBuilder.create_regular_polygon(self.temp_num_sides, 
                                                radius, center=(x,y), 
                                                rotate_cw=self.temp_rotate)

    
    
    def plot_shots(self, ax=None):
        """ 
        Creates a plot of the TCA and the LDCs. 
        Shows the Contained, Uncontained, and Margin areas.
        Shows the shifting of the LDCs to cover the TCA center.
        """
       
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
                             initial=False, shifted=False, lines=False)
        
        # Plot TCA centroid
        # tca_cent, = self.ax.plot(tcas["combined"].centroid.x, 
        #                          tcas["combined"].centroid.y,
        #               linestyle="", marker="*", markersize=25, 
        #               markeredgecolor="black", color="cyan", 
        #               label="TCA centroid")
        
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

    
    
    def _plot_centroids(self, ldcs, tcas, 
                        initial=True, shifted=True, lines=True):
        """
        For each LDC, has the option to plot..
            (1) the initial LDC centroid, 
            (2) the shifted LDC centroid()
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
        
            
        
    def _not_empty(self, poly):
        return not poly.is_empty 
    
    
    
    def _remove_artists(self):
        for a in self.ax.lines + self.ax.collections + self.ax.patches:
            a.remove()
    
    


if __name__ == "__main__":
    plt.close("all")
    
    # Output Directories
    outpath = r"C:\Users\msommers\Desktop\Output\shot_patterns"
    pickle_outpath = os.path.join(outpath, "pickle")
    png_outpath = os.path.join(outpath, "png")
    
    # Inputs
    num_shots = 7
    num_to_cover = 0  # "all"  
    save_fig = False
    save_pickle = False

    # Reading and reformatting data
    pickle_file = r"C:\Users\msommers\Desktop\Output\threat_model\threat_model_data3.pickle"
    raw_data = misc_functions.read_pickle_data(pickle_file)
    data_dict = misc_functions.data_reformatter(raw_data)
    
    # Initializing ShotPattern
    sp = PatternShots(data_dict, 
                      num_shots=num_shots, 
                      num_to_cover=num_to_cover,
                      keep_ax_lims=True,
                      )
                      # TCA_algorithm=GridPatterning)
    plt.show()
    
    # ========================================================================
    # NSim Testing
    # ========================================================================
    # nsim_dir = r"C:\Users\msommers\Desktop\NSim\NSimCBFCDelivery6-1-21\NSim"
    
    # num_shots = 7
    # num_to_cover = 1 #"all"
    
    # nsim_dir = r"C:\Users\msommers\Desktop\NSim\Full20210607CBFC\Full20210607\NSim"
    # bin_dir = os.path.join(nsim_dir, "bin") #, "temp")
    
    # tca_file = os.path.join(bin_dir, "TCAPatch.txt")
    # ldc_file = os.path.join(bin_dir, "LDCPatch.txt")
    
    # header = "Time,ID,Type,ENUx,ENUy,ENUz,Engx,Engy,Zero".split(",")
    # tca_df = pd.read_csv(tca_file, delimiter=" ") 
    # ldc_df = pd.read_csv(ldc_file, delimiter=" ")
    
    # ldc_df.columns = header
    # tca_df.columns = header

    # plt.close("all")
    # data_dict = {}
        
    # for t, ldf in ldc_df.groupby("Time"):
    #     data_dict[t] = {}
    #     data_dict[t]["LDC_points"] = []
    #     data_dict[t]["TCA_points"] = []
        
    #     # Filtering TCA data by ID
    #     time_filt = tca_df["Time"] == t
        
    #     for num, (id_, ldf_) in enumerate(ldf.groupby("ID")):
    #         data_dict[t]["LDC_points"].append(ldf_[["Engx","Engy"]])
            
    #         # Filtering TCA data (by time and ID)
    #         id_filt = tca_df["ID"] == id_
    #         filts = time_filt & id_filt
    #         x = tca_df[filts]["Engx"]
    #         y = tca_df[filts]["Engy"]
        
    #     data_dict[t]["TCA_points"].append(tca_df[filts][["Engx", "Engy"]])
    
    # sp = PatternShots(data_dict, 
    #                   num_shots=num_shots, 
    #                   num_to_cover=num_to_cover,
    #                   keep_ax_lims=True,
    #                   xcol="Engx", ycol="Engy")   
    # plt.show()
    # ========================================================================
    # NSim Testing
    # ========================================================================
    
    
    
    
   
    
    
