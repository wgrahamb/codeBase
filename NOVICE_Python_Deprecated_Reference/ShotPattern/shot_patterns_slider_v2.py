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



from shapely.geometry import LineString, MultiLineString, MultiPolygon, \
    MultiPoint, Point
from shapely.ops import unary_union

import misc_functions
import nsim_readers
import config

from misc_functions import get_plane_equation, get_dist_from_plane,  \
    is_coplanar, calc_rows_cols
from polygon_builder import PolygonBuilder
from kmeans_clustering import KmeansClustering
from voronoi_patterning import VoronoiPatterning
from grid_patterning import GridPatterning
from cover_center_v2 import CoverCenter


import threading
import time

from descartes import PolygonPatch

#=============================================================================
# To Do:
    # 1. Add functionality for multiple separate TCAs - Currently it can only
    # split a single TCA into sub-TCAs. 
    
    # 2. Add other LDC patterning algorithms - such as using a probability 
    # density function
#=============================================================================


class PatternShots(object):
    def __init__(self, data_dict, num_shots=None, num_to_cover=0,
                 TCA_algorithm=VoronoiPatterning, LDC_algorithm=CoverCenter,
                 keep_ax_lims=True, xcol="EPx", ycol="EPy",
                 ):
        """
        This class implements algorithms for breaking up a TCA into multiple
        regions and patterning shots to cover these regions.
        
        Time slider allows user to view these Shot Patterns over time.
        Use left and right arrow keys or drag slider to increment time values.
        
        Parameters
        ----------
        data_dict: dict
            
            Required structure:
                data_dict = { time_0: {"TCA": df, "LDC": df},
                              time_1: {"TCA": df, "LDC": df},
                              ...
                              time_n: {"TCA": df, "LDC": df},
                             }
                TCA df required columns: EPx, EPy, ID
                LDC df required columns: EPx, EPy, ID, TCA_ID
                
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
        self.num_shots = num_shots          # for testing
        self.num_to_cover = num_to_cover    # only applies to CoverCenter
        
        self.keep_ax_lims = keep_ax_lims
        self.xcol = xcol
        self.ycol = ycol
        self.TCA_algorithm = TCA_algorithm
        self.LDC_algorithm = LDC_algorithm
        
        # self.artists = []
        # self.TCA_Alg_IDs = []
        self.TCA_algs = {}
        
        self.thread_algorithms = False     # False seems to be fastest
        self.thread_plots = False          # False seems to be fastest
        print("Threaded Algs: {}".format(self.thread_algorithms))
        print("Threaded Plots: {}".format(self.thread_plots))
        
        self._get_times()
        self._get_ax_extremes()
        self._assign_axes()
        
        self._create_fig()
        # self._get_ax_lims_ticks()
        self._add_time_slider()
        self._get_current_data()
        self._create_polys()
        self._apply_algorithms()
        
        
        self._collect_shots()
        self._calculate_containment()
        self._plot_shot_patterns()
        # self._set_ax_lims_ticks()
        self._update_legend()
        # self._share_common_axes(True)
        self._connections()
        
        """
        self._pair_planes_and_reformat_data()
        self._assign_TCAs_to_axes()
        """
    

    def _get_centroid_xy(self, poly):
        x, y = poly.centroid.x, poly.centroid.y
        return x, y
    

    def _duplicate_LDCs(self, tca_id=None):
        if tca_id is not None:
            tca = self.poly_TCAs[tca_id]
            tcas = [tca]
        else:
            tcas = self.poly_TCAs.values()
        for tca in tcas:
            tca_id = tca._ID
            ldcs = tca._poly_LDCs
            self.poly_TCAs[tca_id]._poly_LDCs = []
            for sp in tca._sub_polys:
                cx, cy = self._get_centroid_xy(sp)
                for ldc in ldcs:
                    new_ldc = ldc.set_center((cx, cy))
                    new_ldc._poly_TCA = tca
                    new_ldc._ax = tca._ax
                    self.poly_TCAs[tca_id]._poly_LDCs.append(new_ldc)
            

    def _assign_TCAs_to_axes(self):
        self.ax_dict = {}
        for idx, tca_id in enumerate(self.TCA_IDs):
            self.ax_dict[tca_id] = idx
        

    def _assign_axes(self):
        tca_ids = []
        ldc_ids = []
        for t in self.times:
            tca_df = self.data[t]["TCA"]
            tca_ids.extend(list(tca_df["ID"].unique()))
            ldc_df = self.data[t]["LDC"]
            ldc_ids.extend(list(ldc_df["ID"].unique()))
        
        self.TCA_IDs = sorted(list(set(tca_ids)))
        self.LDC_IDs = sorted(list(set(ldc_ids)))
        self.ax_dict = {}
        for idx, tca_id in enumerate(self.TCA_IDs):
            self.ax_dict[tca_id] = idx
        
        

    def _pair_planes_and_reformat_data(self):
        """
        Pairing LDCs to TCAs and reformatting the data
            1. Iterates through TCAs.
            2. Creates plane from 3 points of TCA.
            3. Iterates through LDCs.
            4. Gets a point from the LDC and calculates the distance from that
                point to the TCA plane.
            5. If the point is within some arbitrary distance (currently 10.0),
                it pairs the LDC and TCA together.
        """
        global new_data_dict
        new_data_dict = {}
        self.max_num_TCAs = 0
        self.TCA_IDs = []
        for t in self.times:
            print("[Time {}]".format(round(t,2)))
            new_data_dict[t] = {}
            current_data = self.raw_data[t]
            ldc_enu = current_data["LDC_ENU_points"]
            ldc_ep  = current_data["LDC_EP_points"]
            tca_enu = current_data["TCA_ENU_points"]
            tca_ep  = current_data["TCA_EP_points"]
            
            # Verifying that it's pairing correctly by shuffling the order
            # np.random.shuffle(ldc_enu)
            # np.random.shuffle(tca_enu)
            
            pairs, paired_ldcs = [],[]
            for i, (df_enu, df_ep) in enumerate(zip(tca_enu, tca_ep), start=1):
                # Getting indexes roughly evenly spaced from df_enu
                # if len(df_enu) == 21, gets element indexes 5, 10, 15
                step = math.floor(int(len(df_enu)/4))
                num_tcas = len(tca_enu)
                
                tca_id = int(df_ep.iloc[0]["ID"])
                if tca_id not in self.TCA_IDs:
                    self.TCA_IDs.append(tca_id)
                
                if num_tcas > self.max_num_TCAs:
                    # used to determine subplot configuration (rows and cols)
                    self.max_num_TCAs = num_tcas    
                    
                # p1, p2, p3 form a plane
                cols = ["ENUx","ENUy","ENUz"]
                p1 = np.array(df_enu.iloc[step][cols])
                p2 = np.array(df_enu.iloc[2*step][cols])
                p3 = np.array(df_enu.iloc[3*step][cols])

                plane_dict = {"LDC_ENU_points": [], "LDC_EP_points": []}
                plane_dict["TCA_ENU_points"] = df_enu
                plane_dict["TCA_EP_points"] = df_ep
                for i2, (df2_enu, df2_ep) in enumerate(zip(ldc_enu, ldc_ep),
                                                       start=1):
                    # if i2 in paired_ldcs:
                    #     continue    
                    p4 = np.array(df2_enu.iloc[0][cols])
                    
                    # If p4 is within certain distance of plane, it's close enough
                    coplanar, dist = is_coplanar(p1,p2,p3,p4)
                    if coplanar:
                        pairs.append((i, i2))
                        paired_ldcs.append(i2)
                        print("Pairing Planes: TCA_{} --> LDC_{} (dist={})" \
                              .format(i, i2, round(dist,2)))
                        # print("paired ldcs", paired_ldcs)
                        plane_dict["LDC_ENU_points"].append(df2_enu)
                        plane_dict["LDC_EP_points"].append(df2_ep)
                    else:
                        pass
                        # print("No Match for TCA_{} --> LDC_{}".format(i,i2))

                num_keys = len(new_data_dict[t].keys())
                new_data_dict[t]["Plane_{}".format(num_keys+1)] = plane_dict
            print()
        print()
        self.TCA_IDs = sorted(self.TCA_IDs)
        self.data = new_data_dict
            
    
    def get_num_TCAs(self):
        return len(self.current_data["TCA"]["ID"].unique())
    
    def get_num_LDCs(self):
        return len(self.current_data["LDC"]["ID"].unique())
    
    

    def _update(self, i):
        start = time.time()
        self._remove_artists()
        self._get_current_data()
        print("\n[Time {}]".format(self.current_time))
        self._create_polys()
        
        self._apply_algorithms()
            # self._apply_TCA_algorithms()
            # self._duplicate_LDCs()
            # self._apply_LDC_algorithms()
    
        self._collect_shots()
        self._calculate_containment()
        self._plot_shot_patterns()
        # self._set_ax_lims_ticks()
        self._update_legend()
        print("Elapsed: {} s".format(round(time.time()-start, 3)))

    
    
    def _share_common_axes(self, bool_):
        if not bool_:
            self.remove_common_axes()
        else:
            common_ax_dict = {}
            for ax in self.axes:
                label = ax.get_xlabel()
                if label in common_ax_dict.keys():
                    common_ax_dict[label].append(ax)
                else:
                    common_ax_dict[label] = [ax]
                  
            for lab, ax_list in common_ax_dict.items():
                if len(ax_list) > 1:
                    ax0 = ax_list[0]
                    for ax in ax_list[1:]:
                        ax0._shared_x_axes.join(ax0, ax)
                        # ax0._shared_y_axes.join(ax0, ax)
            
            for ax in self.axes:
                ax.autoscale()



    def remove_common_axes(self):
        for ax in self.axes:
            other_axes = [axis for axis in self.axes if axis != ax]
            for oa in other_axes:
                ax._shared_x_axes.remove(oa)
                # ax._shared_yaxes.remove(oa)

    
    
    def _plot_threaded(self):
        threads = []
        for tca in self.poly_TCAs.values():
            pthread = threading.Thread(target=self._init_plot_thread, 
                                      args=(tca,))   
            pthread.start()
            threads.append(pthread)
        [t.join() for t in threads]
        


    def _init_plot_thread(self, tca):
        ax = tca._ax
        tca._TCA_alg.plot(ax=ax)
        self._plot_TCA(tca)
        self._plot_LDCs(tca)
        return

        
    
    def _plot_shot_patterns(self):
        start = time.time()
        if self.thread_plots:
            self._plot_threaded()
        else:
            self._plot_TCA()
            self._plot_LDCs()
        
        print("\tplot elapsed {} s".format(round(time.time()-start, 3)))
        plt.show()
        
        
        
    def _plot_LDC(self):
        patch = None
        for tca in self.poly_TCAs.values():
            ldcs = tca._poly_LDCs
            for ldc in ldcs:
                ax = ldc._ax
                patch = PolygonPatch(ldc,
                                     edgecolor="black",
                                     facecolor="orange",
                                     alpha=0.5,
                                     zorder=10)
                art = ax.add_patch(patch)
                cent, = ax.plot(*ldc.centroid.xy,
                                linestyle="",
                                marker="o",
                                markerfacecolor="orange",
                                markeredgecolor="black",
                                zorder=10)
        if patch:
            art.set_label("LDCs ({})".format(self.get_num_LDCs()))    
            cent.set_label("LDC centroids")
            
        
        
    def _plot_TCA(self, tca=None):
        if tca:
            tcas = [tca]
        else:
            tcas = self.poly_TCAs.values()
            
        patch = None
        for tca in tcas:
            ax = tca._ax
        
            tca._TCA_alg.plot(ax=ax)
            patch = PolygonPatch(tca,
                                 edgecolor="black",
                                 alpha=0.5,
                                 zorder=1,
                                 fill=False)
            art = ax.add_patch(patch)
        if patch:
            art.set_label("TCAs ({})".format(self.get_num_TCAs()))
            
     
    
    def _plot_LDCs(self, tca=None):
        """ 
        Creates a plot of the TCA and the LDCs. 
        Shows the Contained, Uncontained, and Margin areas.
        Shows the shifting of the LDCs to cover the TCA center.
        """
        if tca:
            tcas = [tca]
        else:
            tcas = self.poly_TCAs.values()
            
        for tca in tcas:
            ax = tca._ax
            ldcs = tca._shots["LDC"]
            contained = tca._shots["Contained"]
            margin = tca._shots["Margin"]
            tcas = tca._shots["TCA"]
     
            # Plot individual LDC, Contained, and Margin patches
            self._plot_individual_patches(ax,
                                          ldcs,
                                          fill=False,
                                          linewidth=1,
                                          edgecolor="black",
                                          label=None)
            
            self._plot_individual_patches(ax,
                                          contained,
                                          fill=False,
                                          linewidth=0,
                                          hatch="**",
                                          label="Contained")
            
            self._plot_individual_patches(ax,
                                          margin,
                                          fill=True,
                                          facecolor="blue",
                                          edgecolor="black",
                                          label="Margin")
            
            # Plot LDC centroids
            self._plot_centroids(ax, ldcs, tcas, 
                                 initial=False, shifted=False, lines=False)
            
            # Plot TCA centroid
            # tca_cent, = self.ax.plot(tcas["combined"].centroid.x, 
            #                          tcas["combined"].centroid.y,
            #               linestyle="", marker="*", markersize=25, 
            #               markeredgecolor="black", color="cyan", 
            #               label="TCA centroid")
            
            # Plots Combined LDC patch
            self._plot_patch(ax,
                             ldcs["combined"], 
                             fill=False, 
                             linewidth=3,
                             edgecolor="cyan")
            
            # Axis Formatting
            title_string = \
            """
            Cont. Ratio: {}
            """.format(round(tca._containment_ratio,2))
            # """
            # TCA Area: {} km^2
            # # LDCs Covering Center: {} (Required: {}+)
            # CONTAINMENT Ratio: {}
            # """.format(
            #     round(tcas["combined"].area/(1000**2),2), 
            #     ldcs["num_covering_center"], tca._LDC_alg.num_to_cover,
            #     round(tca._containment_ratio,2))
            ax.set_title(title_string)
            
        
    
    def _plot_individual_patches(self, ax, polys, label=None, **kwargs):
        patches = 0
        for poly in polys["individual"]:
            patch = self._plot_patch(ax, poly, **kwargs)
            if patch:
                patches += 1
        if patches > 0:
            patch.set_label(label)
    
        
    
    def _plot_patch(self, ax, poly, **kwargs):
        if self._not_empty(poly):
            patch = PolygonPatch(poly, **kwargs)
            ax.add_patch(patch)
            return patch
        else:
            return None

    
    
    def _plot_centroids(self, ax, ldcs, tcas, 
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
                    i_cent, = ax.plot(ldcx, ldcy, 
                                      linestyle="", marker="o", color="cyan", 
                                      markersize=15, markeredgecolor="black",
                                      zorder=10)
                if lines:
                    line, = ax.plot([ldcx, tcax], [ldcy, tcay],
                                    linestyle="--", color="cyan")
            if i_cent:
                i_cent.set_label("Initial LDC centroid")
                        
        if shifted:
            s_cent = None
            for ldcx, ldcy in ldcs["shifted_centroids"]:
                s_cent, = ax.plot(ldcx, ldcy, 
                                  linestyle="", marker="X", color="cyan", 
                                  markersize=15, markeredgecolor="black",
                                  zorder=10)
            if s_cent:
                s_cent.set_label("Shifted LDC centroid")
        
            
        
    def _not_empty(self, poly):
        return not poly.is_empty 
    

        
    def  _update_legend(self):
        all_artists = []
        for ax in self.axes:
            ax_artists = ax.lines + ax.collections + ax.patches
            all_artists.extend(ax_artists)
            
        all_artists = [a for a in all_artists if a.get_label() is not None]
        
        artists_with_labels = [a for a in all_artists
                               if "_line" not in a.get_label()
                               and a.get_label() != ""
                               and a.get_label() is not None]
        
        labels = [a.get_label() for a in artists_with_labels]
        self.ax_time.legend(handles=artists_with_labels)
        by_label = dict(zip(labels, artists_with_labels))
        self.ax_time.legend(by_label.values(), by_label.keys(),
                            bbox_to_anchor=(0.5, 1.05), 
                            loc='lower center', ncol=len(by_label))
        
        

    def _create_fig(self):
        self.fig = plt.figure(figsize=(14,8))
        plt.subplots_adjust(bottom=0.2, left=0.1, right=0.95, top=0.9)
        rows, cols = calc_rows_cols(len(self.TCA_IDs))
        self.gs = self.fig.add_gridspec(nrows=rows, ncols=cols)
        self.axes = []
        for r in range(rows):
            for c in range(cols):
                ax = self.fig.add_subplot(self.gs[r,c])
                ax.axis("equal")
                ax.set_facecolor("lightgray")
                self.axes.append(ax)
        
        
    
    def _get_ax_extremes(self):
        to_concat = []
        for t in self.times:
            time_data = self.data[t]
            ldc_df = time_data["LDC"]
            tca_df = time_data["TCA"]
            to_concat.extend([ldc_df, tca_df])
            
        combined = pd.concat(to_concat)
        epx, epy = combined["EPx"], combined["EPy"]
        minx, maxx = epx.min(), epx.max()
        miny, maxy = epy.min(), epy.max()
        buffer = 0.1    # 10% buffer on each side
        self._xlims = (minx*(1-buffer), maxx*(1+buffer))
        self._ylims = (miny*(1-buffer), maxy*(1+buffer))
        
        
        
    def _create_polys(self):
        # These are created from EP_points
        self.poly_TCAs = {}
        self.poly_LDCs = {}
    
        # The TCAs and LDCs have already been coorelated into plane_dict
        # for i, (key, plane_dict) in enumerate(self.current_data.items()):
        for tca_id, tca_df in self.current_data["TCA"].groupby("ID"):
            tca_poly = PolygonBuilder.create_from_df(tca_df, 
                                                     self.xcol, 
                                                     self.ycol)
            # Assigning attributes to the TCA
            tca_poly._ID = tca_id
            tca_poly._ax = self.axes[self.ax_dict[tca_id]]
            tca_poly._poly_LDCs = []
            
            ldc_area = 0
            for ldc_id, ldc_df in self.current_data["LDC"].groupby("ID"):
                
                ldc_tca_id = ldc_df["TCA_ID"].iloc[0]
                if ldc_tca_id == tca_id:
                    
                    ldc_poly = PolygonBuilder.create_from_df(ldc_df, 
                                                             self.xcol, 
                                                             self.ycol)
                    
                    # Assigning attributes to the LDC
                    ldc_poly._ID = ldc_id
                    ldc_poly._ax = tca_poly._ax
                    ldc_poly._poly_TCA = tca_poly
                    
                    # Adding to the LDC dictionary
                    self.poly_LDCs[ldc_poly._ID] = ldc_poly
                    tca_poly._poly_LDCs.append(ldc_poly)
                
                    # Cumulative area of LDCs for each TCA
                    ldc_area += ldc_poly.area
            
            if self.num_shots:
                # Specifying the number of shots for each TCA
                tca_poly._num_clusters = self.num_shots
                tca_poly._num_shots = self.num_shots
            else:
                # Calculating the number of shots required for each TCA
                ratio = tca_poly.area/ldc_area
                tca_poly._num_clusters = math.ceil(ratio)
                tca_poly._num_shots = math.ceil(ratio)
                
            # Assigning TCA algorithm for TCA (Not applying it!)
            tca_poly._TCA_algorithm = self.TCA_algorithm
            
            # Adding to the TCA dictionary
            self.poly_TCAs[tca_id] = tca_poly
            
        self.mpoly_TCAs = MultiPolygon([p for p in self.poly_TCAs.values()]) 
        self.upoly_TCAs = unary_union([p for p in self.poly_TCAs.values()])  
        self.mpoly_LDCs = MultiPolygon([p for p in self.poly_LDCs.values()]) 
        self.upoly_LDCs = unary_union([p for p in self.poly_LDCs.values()])
    
        
        
    def _get_times(self):
        global times
        # Ensures keys are floats, not str
        for k in self.data.copy().keys():   
            self.data[float(k)] = self.data.pop(k)
            
        self.times = sorted(list(self.data.keys()))
        self.min_time, self.max_time = min(self.times), max(self.times)
        self.init_time = self.min_time
        self.current_time = self.min_time
        times = self.times
        
        # Determine time_step
        tdiff = pd.Series(self.times).diff()
        min_tdiff = round(np.nanmin(tdiff),3)
        self.time_step = min_tdiff
    
        
    
    def _add_time_slider(self):
        # [left, bottom, width, height]
        self.ax_time = plt.axes([0.15, 0.05, 0.75, 0.03], 
                           facecolor="lightgoldenrodyellow")
        if self.time_step > 0.1:
            valfmt = "%1.1f"
        if self.time_step < 0.1:
            valfmt = "%1.2f"
        if self.time_step < 0.01:
            valfmt = "%1.3f"
        self.time_slider = Slider(ax=self.ax_time,
                                  label="Time",
                                  valmin=self.min_time,
                                  valmax=self.max_time,
                                  valinit=self.init_time,
                                  valstep=self.time_step, #0.1,
                                  valfmt = valfmt)
        self._add_slider_ticks()
        self.time_slider.on_changed(self._update)
        
    
    def _add_slider_ticks(self):
        # Currently breaks if the number of LDCs changes over time
        for t in self.times:
            self.ax_time.axvline(t, 0, 1, color="blue", lw=1, ls="dotted")        
        self.ax_time.set_xticks(self.times)
        self.ax_time.set_xticklabels([])
        # num = 0
        # tick_labs = []
        # for i, t in enumerate(self.times):
        #     odd = i % 2
        #     if odd:
        #         tick_labs.append(t)
        #     else:
        #         tick_labs.append("")
        # self.ax_time.set_xticklabels(tick_labs)
        return self.ax_time
        
    
    
    def _init_config(self):
        self.TCA_algorithm = config.TCA_algorithm
        self.LDC_algorithm = config.LDC_algorithm


    
    def _calc_num_shots(self):
        min_num_shots = self.get_num_TCAs()
        total_shots = min_num_shots
        for tca in self.poly_TCAs.values():
            ldc_area = 0
            ldcs = tca._poly_LDCs
        
        

    def _apply_TCA_algorithms(self):
        # print("_apply_TCA_algorithms() (threaded = {})".format(
        #                                             self.thread_algorithms))
        start = time.time()
        if self.thread_algorithms:
            active_ids = self._get_active_TCAs()
            algorithm_threads = []
            for tca_id in active_ids:
                alg_thread = threading.Thread(target=self._update_TCA, 
                                              args=(tca_id,))
                alg_thread.start()
                algorithm_threads.append(alg_thread)
            [t.join() for t in algorithm_threads]
            
        else:
            active_ids = self._get_active_TCAs()
            for tca_id in active_ids:
                self._update_TCA(tca_id)
        print("\talgorithms elapsed: {} s".format(round(time.time()-start,3)))
            
        
    def _apply_LDC_algorithms(self):
        active_ids = self._get_active_TCAs()
        for tca_id in active_ids:
            self._update_LDCs(tca_id)    
    
        
    
    def _apply_algorithms(self):
        start = time.time()
        active_ids = self._get_active_TCAs()
        if self.thread_algorithms:
            threads = []
            for tca_id in active_ids:
                alg_thread = threading.Thread(target=self._apply, 
                                      args=(tca_id,))
                alg_thread.start()
                threads.append(alg_thread)
            [t.join() for t in threads]
        else:
            for tca_id in active_ids:
                self._apply(tca_id)
        print("\talgorithms elapsed: {} s".format(round(time.time()-start,3)))
            
        
        
    def _apply(self, tca_id):
        self._update_TCA(tca_id)
        self._duplicate_LDCs(tca_id)
        self._update_LDCs(tca_id)
    
    
    
    def _update_TCA(self, tca_id):
        tca = self.poly_TCAs[tca_id]
        if issubclass(self.TCA_algorithm, VoronoiPatterning) and \
            tca._num_clusters < 3:
            tca._TCA_algorithm = GridPatterning
        
        if tca_id not in self.TCA_algs.keys():
            # self.TCA_Alg_IDs.append(tca_id)
                    
            if issubclass(tca._TCA_algorithm, VoronoiPatterning):
                # print('\tInitializing TCA Algorithm [ID{}]: VoronoiPatterning' \
                      # .format(tca_id))
                kmeans = KmeansClustering(tca, 
                                          num_clusters=tca._num_clusters,
                                          init="k-means++")
                tca._TCA_alg = tca._TCA_algorithm(tca,
                                                  kmeans.cluster_centers,
                                                  repattern_threshold=0.5)
                                                   # repattern_threshold=0.25)
                
            elif issubclass(tca._TCA_algorithm, GridPatterning):
                # print('\tInitializing TCA Algorithm [ID{}]: GridPatterning' \
                      # .format(tca_id))
                rows, cols = self._determine_grid_rows_cols(tca)
                tca._TCA_alg = tca._TCA_algorithm(tca,
                                                  rows=rows, cols=cols)
            self.TCA_algs[tca_id] = tca._TCA_alg
            
        else:
            # print('\tUpdating TCA Algorithm [ID{}]: {}'.format(tca_id,
                                            # tca._TCA_algorithm.__name__))
            tca._TCA_alg = self.TCA_algs[tca_id]
            tca._TCA_alg.update(tca)
            
        tca._sub_polys = tca._TCA_alg.sub_polys
        tca._mpoly = MultiPolygon(tca._sub_polys)
        self.poly_TCAs[tca_id] = tca
        
    
        
    def _update_LDCs(self, tca_id):
        tca = self.poly_TCAs[tca_id]
        tca._LDC_alg = self.LDC_algorithm(tca, num_to_cover=self.num_to_cover)
            
    
    
    def _get_active_TCAs(self):
        tca_df = self.current_data["TCA"]
        tca_ids = sorted(list(tca_df["ID"].unique()))
        return tca_ids
       
 
        
    def _collect_shots(self, tca_id=None):
        if tca_id:
            tca = self.poly_TCAs[tca_id]
            tcas = [tca]
        else:
            tcas = self.poly_TCAs.values()
            
        for tca in tcas:
            tca._shots = tca._LDC_alg.shots
            tca._poly_contained = tca._shots["Contained"]["combined"]
            tca._poly_uncontained = tca._shots["Uncontained"]["combined"]
            tca._poly_margin = tca._shots["Margin"]["combined"]
            
            # MultiPolygons (for potential use later on)
            tca._mpoly_LDC = MultiPolygon(tca._poly_LDCs)
            tca._mpoly_contained = \
                MultiPolygon(tca._shots["Contained"]["individual"])
            # tca._mpoly_margin = MultiPolygon(tca._shots["Margin"]["individual"])
    

        
    def _calculate_containment(self):
        for tca_id, tca in self.poly_TCAs.items():
            contained = tca._shots["Contained"]["combined"]
            uncontained = tca._shots["Uncontained"]["combined"]
            self.poly_TCAs[tca_id]._containment_ratio = contained.area / \
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
        
        
   
    def _get_ax_lims_ticks(self):
        for ax in self.axes:
            self._xlim, self._ylim = ax.get_xlim(), ax.get_ylim()
            self._xticks, self._yticks = ax.get_xticks(), ax.get_yticks()
    
    
    
    def _set_ax_lims_ticks(self):
        if self.keep_ax_lims:
            if hasattr(self, "_xlim"):
                for ax in self.axes:
                    ax.set_xlim(self._xlim)
                    ax.set_ylim(self._ylim)
                    ax.set_xticks(self._xticks)
                    ax.set_yticks(self._yticks)
        self.fig.canvas.draw()
        
    
    def _connections(self):
        self.fig.canvas.mpl_connect('key_press_event', self._on_key_press)
        
      
        
    def _determine_grid_rows_cols(self, tca):
        # Might need better method of doing this
        r = 1
        c = 1
        num = r*c
        ns = tca._num_shots #self.num_shots
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
 
   

    def _remove_artists(self):
        for ax in self.axes:
            for a in ax.lines + ax.collections + ax.patches:
                a.remove()
    
    


if __name__ == "__main__":
    plt.close("all")
    
    # ========================================================================
    # Threat Model Testing
    # ========================================================================
        
    # # Inputs
    # num_shots = 6
    # num_to_cover =  0  #"all"  
    
    # pickle_file = r"C:\Users\msommers\Desktop\Output\threat_model\1A_5I_1T.pickle"
    # pickle_file = r"C:\Users\msommers\Desktop\Output\threat_model\2A_25I_2T.pickle"
    # raw_data = misc_functions.read_pickle_data(pickle_file)
    # data_dict = misc_functions.data_reformatter_v4(raw_data)

    # sp = PatternShots(data_dict, 
    #                   num_shots=num_shots, 
    #                   num_to_cover=num_to_cover,
    #                   keep_ax_lims=False,
    #                   )
                      # TCA_algorithm=GridPatterning)

    # ========================================================================
    # Threat Model Testing
    # ========================================================================




    
    
    
    # ========================================================================
    # NSim Testing
    # ========================================================================
    
    
    # Inputs
    nsim_dir = r"C:\Users\msommers\Desktop\NSim\Full20210607CBFC\Full20210607\NSim"
    bin_dir = os.path.join(nsim_dir, "bin")
    
    scenario_name = "CB_Raid"    # CB_L1, CB_L2, CB_L3, CB_L4x2, CB_Raid
    scenario_dir = os.path.join(bin_dir, scenario_name)
    data_dir = scenario_dir   #bin_dir
    
    tca_df, ldc_df = nsim_readers.read_nsim_TCA_LDC(data_dir)
    eng_plans = nsim_readers.read_nsim_groups_engaged(data_dir)
    data_dict = nsim_readers.create_nsim_data_dict(tca_df, ldc_df, eng_plans)
    ldc_df = data_dict["LDC"]
    # nsim_readers.plot_nsim_groups(ldc_df)
    
    new_dict = {}
    for key, df in data_dict.items():
        for t, df_ in df.groupby("Time"):
            if t in new_dict:
                new_dict[t].update({key: df_})
            else:
                new_dict[t] = {key: df_}
             
    sp = PatternShots(new_dict, num_shots=None)
    
    
    # ========================================================================
    # NSim Testing
    # ========================================================================
    
    
    
    
    
    
    
    
    

        
    # ========================================================================
    # NSim Testing OLD
    # ========================================================================
    # nsim_dir = r"C:\Users\msommers\Desktop\NSim\NSimCBFCDelivery6-1-21\NSim"
    
    # num_shots = 4 #4
    # num_to_cover = 0 #"all"
    
    # nsim_dir = r"C:\Users\msommers\Desktop\NSim\Full20210607CBFC\Full20210607\NSim"
    # bin_dir = os.path.join(nsim_dir, "bin") 
    
    # scenario_name = "CB_Raid"    # CB_L1, CB_L2, CB_L3, CB_L4x2, CB_Raid, CB_L1_Testing   
    # scenario_dir = os.path.join(bin_dir, scenario_name)
    # data_dir = scenario_dir     # directory to read output from (bin_dir)
    
    # tca_file = os.path.join(data_dir, "TCAPatch.txt")
    # ldc_file = os.path.join(data_dir, "LDCPatch.txt")
    
    # header = "Time,ID,Type,ENUx,ENUy,ENUz,EPx,EPy,Zero".split(",")
    # tca_df = pd.read_csv(tca_file, delimiter=" ") 
    # ldc_df = pd.read_csv(ldc_file, delimiter=" ")
    
    # ldc_df.columns = header
    # tca_df.columns = header

    # # plt.close("all")
    # data_dict = {}
    # for t, ldf in ldc_df.groupby("Time"):
    #     data_dict[t] = {}
    #     data_dict[t]["LDC_EP_points"] = []
    #     data_dict[t]["TCA_EP_points"] = []
        
    #     data_dict[t]["LDC_ENU_points"] = []
    #     data_dict[t]["TCA_ENU_points"] = []
        
    #     # Filtering TCA data by ID
    #     time_filt = tca_df["Time"] == t
        
    #     ep_cols = ["Time", "ID", "EPx", "EPy"]
    #     enu_cols = ["Time", "ID", "ENUx", "ENUy", "ENUz"]
    #     for num, (id_, ldf_) in enumerate(ldf.groupby("ID")):
            
    #         # Filtering TCA data (by time and ID)
    #         id_filt = tca_df["ID"] == id_
    #         filts = time_filt & id_filt
            
    #         # Actual LDCs and TCAs
    #         # data_dict[t]["TCA_EP_points"].append(tca_df[filts][ep_cols])
    #         # data_dict[t]["TCA_ENU_points"].append(tca_df[filts][enu_cols])             
            
    #         # Switching the LDCs and TCAs for testing purposes only
    #         data_dict[t]["TCA_EP_points"].append(ldf_[ep_cols])
    #         data_dict[t]["TCA_ENU_points"].append(ldf_[enu_cols]) 

    #         # Actual LDCs and TCAs
    #         # data_dict[t]["LDC_EP_points"].append(ldf_[ep_cols]) 
    #         # data_dict[t]["LDC_ENU_points"].append(ldf_[enu_cols]) 

    #         # Switching the LDCs and TCAs for testing purposes only
    #         data_dict[t]["LDC_EP_points"].append(tca_df[filts][ep_cols]) 
    #         data_dict[t]["LDC_ENU_points"].append(tca_df[filts][enu_cols]) 
    
    # sp = PatternShots(data_dict, 
    #                   num_shots=num_shots, 
    #                   num_to_cover=num_to_cover,
    #                   keep_ax_lims=False,
    #                   xcol="EPx", ycol="EPy",
    #                   )
    #                   # TCA_algorithm=GridPatterning)
    # plt.show()
    # ========================================================================
    # NSim Testing OLD
    # ========================================================================
        
        
        
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
