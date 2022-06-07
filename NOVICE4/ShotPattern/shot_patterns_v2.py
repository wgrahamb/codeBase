# -*- coding: utf-8 -*-
"""
Created on Fri Aug 13 13:43:28 2021

@author: msommers
"""



# -*- coding: utf-8 -*-
"""
Created on Thu May 13 08:51:59 2021

@author: msommers
"""

import os
import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt
import time

from shapely.geometry import MultiPolygon
from shapely.affinity import scale, translate
from shapely.ops import unary_union
from descartes import PolygonPatch

import ShotPattern.misc_functions
from ShotPattern.polygon_builder import PolygonBuilder
from ShotPattern.kmeans_clustering import KmeansClustering
from ShotPattern.voronoi_patterning import VoronoiPatterning
from ShotPattern.grid_patterning import GridPatterning
from ShotPattern.cover_center_v2 import CoverCenter
import ShotPattern.config

# import nsim_readers


class PatternShots(object):
    def __init__(self, df_TCA, df_LDC,
                 num_shots=None, num_to_cover=0,
                 TCA_algorithm=VoronoiPatterning, 
                 LDC_algorithm=CoverCenter,
                 xcol="EPx", ycol="EPy",
                 ):
        """
        TCA = Threat CONTAINMENT Area
        LDC = Lethal Divert Capability 
        
        This class implements algorithms for breaking up a TCA into multiple
        regions and patterning shots to cover these regions.
        
        TCA_algorithm is the class containing the algorithm used for 
        partitioning the TCA into sub-TCAs
        
        LDC_algorithm is the class containing the algorithm used for placing
        the LDCs on the sub-TCAs
        
        Parameters
        ----------
        
        df_TCA and df_LDC: pandas DataFrame
            DataFrames of the points in the Engagement Plane
            df_TCA required columns: EPx, EPy, ID
            df_LDC required columns: EPx, EPy, ID, TCA_ID
                
        num_shots: int or None, optional
            Number of shots to pattern. 
            
            If this parameter is not passed, num_shots is determined by 
            computing the ratio of TCA area to LDC area.
            
            Note: For the GridPatterning TCA Algorithm,
            PatternShots may not produce this exact number of shots.
            
            The default is None.

        num_to_cover: int, None, or "all"
            Number of LDCs required to cover the TCA center.
            
        xcol and ycol: str
            Column name for xdata in TCA and LDC dataframes
            
        Returns
        -------
        PatternShots object

        """
        
        self.df_TCA = df_TCA
        self.df_LDC = df_LDC
        self.num_shots = num_shots          
        self.num_to_cover = num_to_cover
        self.xcol = xcol
        self.ycol = ycol
        self.TCA_algorithm = TCA_algorithm
        self.LDC_algorithm = LDC_algorithm
        
        self.TCA_ID = self.df_TCA["ID"].iloc[0]
        self.TCA_alg = None
        self._create_polys()
        self._calc_num_shots()
        self._apply_algorithms()
        self._collect_shots()
        self._calculate_containment()
        


    def _get_centroid_xy(self, poly):
        """
        Extracts the centroid x,y coordinates of the shapely polygon.
        """
        x, y = poly.centroid.x, poly.centroid.y
        return x, y



    def _duplicate_LDCs(self):
        """
        Takes the original LDC and duplicates it for each shot. Places the 
        centroid of each new LDC at  sub-TCA centroid.
        """
        self.poly_TCA._poly_LDCs = []
        self.poly_LDCs = []
        for poly in self.TCA_alg.sub_polys:
            cx, cy = self._get_centroid_xy(poly)
            new_ldc = self.poly_LDC.set_center((cx, cy))
            new_ldc._poly_TCA = self.poly_TCA
            self.poly_TCA._poly_LDCs.append(new_ldc)
            self.poly_LDCs.append(new_ldc)
            


    def update(self, tca_df, ldc_df):
        """
        Uses the input TCA and LDC dataframes to update the existing 
        algorithms.
        """
        self._create_polys(tca_df, ldc_df)
        self._apply_algorithms()
        self._collect_shots()
        self._calculate_containment()
        print("Elapsed: {} s".format(round(time.time()-start, 3)))


   
    def create_df_from_poly(self, poly):
        """
        Uses shapely polygon to create dataframe of boundary coordinates.
        """
        x, y = poly.boundary.coords.xy
        df = pd.DataFrame({self.xcol: x, self.ycol: y})
        return df
    
    

    def _create_polys(self, tca_df=pd.DataFrame(), ldc_df=pd.DataFrame()):
        """
        Creates TCA and LDC polygons from their dataframes. Assigns proper IDs.
        """
        if not tca_df.empty:
            self.df_TCA = tca_df
        if not ldc_df.empty:
            self.df_LDC = ldc_df
            
        tca_poly = PolygonBuilder.create_from_df(self.df_TCA, self.xcol, 
                                                 self.ycol)
        tca_poly._ID = self.TCA_ID
        
        ldc_poly = PolygonBuilder.create_from_df(self.df_LDC, self.xcol, 
                                                 self.ycol)
        ldc_poly._ID = self.df_LDC["ID"].iloc[0]
        ldc_poly._poly_TCA = tca_poly
        self.poly_TCA = tca_poly
        self.poly_LDC = ldc_poly
        
        
        
    def _calc_num_shots(self):
        """
        If num_shots is not passed in at initialization,
        calculates the number of shots required to cover the TCA.
        """
        if self.num_shots:
            self.num_clusters = self.num_shots
        else:
            ratio = self.poly_TCA.area/self.poly_LDC.area
            self.num_clusters = math.ceil(ratio)
            self.num_shots = math.ceil(ratio)
        
      
    
    def _init_config(self):
        """
        Currently unused. Allows for importing of algorithms from config file.
        """
        self.TCA_algorithm = config.TCA_algorithm
        self.LDC_algorithm = config.LDC_algorithm
        
 
    
    def _apply_algorithms(self):
        """
        Updates the TCA algorithms, duplicates and places the LDC polygon for
        each shot, and updates the LDC algorithm.
        """
        start = time.time()
        print("\talgorithms elapsed: {} s".format(round(time.time()-start,3)))
        self._update_TCA()
        self._duplicate_LDCs()
        self._update_LDCs()
        
        
    
    def _update_TCA(self):
        """
        Initializes TCA algorithm if there isn't one.
        After that, it updates the existing TCA algorithm with the newly 
        created TCA and LDC polygons.
        """
        
        # Updating existing TCA algorithm
        if self.TCA_alg:
            print('\tUpdating TCA Algorithm [ID{}]: {}'\
                   .format(self.TCA_ID, self.TCA_algorithm.__name__))
            self.TCA_alg.update(self.poly_TCA)
            self.poly_TCA._sub_polys = self.TCA_alg.sub_polys
        
        # Initializing new TCA algorithm
        else:
            if issubclass(self.TCA_algorithm, VoronoiPatterning) and \
                self.num_clusters < 3:
                self.TCA_algorithm = GridPatterning
            
            if issubclass(self.TCA_algorithm, VoronoiPatterning):
                print('\tInitializing TCA Algorithm [ID{}]: VoronoiPatterning'\
                       .format(self.TCA_ID))
                kmeans = KmeansClustering(self.poly_TCA, 
                                          num_clusters=self.num_clusters,
                                          init="k-means++",
                                          num_points=3000)
                self.TCA_alg = self.TCA_algorithm(self.poly_TCA,
                                                  kmeans.cluster_centers,
                                                  repattern_threshold=0.25)
                self.poly_TCA._sub_polys = self.TCA_alg.sub_polys
                
            elif issubclass(self.TCA_algorithm, GridPatterning):
                print('\tInitializing TCA Algorithm [ID{}]: GridPatterning' \
                       .format(self.TCA_ID))
                rows, cols = self._determine_grid_rows_cols()
                self.TCA_alg = self.TCA_algorithm(self.poly_TCA,
                                                  rows=rows, cols=cols)
                self.poly_TCA._sub_polys = self.TCA_alg.sub_polys
                
            else:
                print("\tFailed to recognize {}".format(
                                            self.TCA_algorithm.__name__))
        
    
        
    def _update_LDCs(self):
        """
        Implements the LDC algorithm. Re-initialized at each time step.
        """
        self.LDC_alg = self.LDC_algorithm(self.poly_TCA, 
                                          num_to_cover=self.num_to_cover)
        
        
        
    def _collect_shots(self):
        """
        Extracts the necessary info from the completed LDC algorithm. Creates
        dataframes for the TCA and LDC polygons.
        """
        self.shots = self.LDC_alg.shots
        self.poly_contained = self.shots["Contained"]["combined"]
        self.poly_uncontained = self.shots["Uncontained"]["combined"]
        self.poly_margin = self.shots["Margin"]["combined"]
        self.poly_LDCs = self.shots["LDC"]["individual"]
        self.poly_TCA._poly_LDCs = self.shots["LDC"]["individual"]
        
        self.df_LDCs = [self.create_df_from_poly(p) 
                        for p in self.poly_LDCs]
        self.df_TCAs = [self.create_df_from_poly(p)
                        for p in self.poly_TCA._sub_polys]
        
        self.mpoly_LDCs = MultiPolygon(self.poly_TCA._poly_LDCs)
        self.mpoly_contained = \
            MultiPolygon(self.shots["Contained"]["individual"])
        self.mpoly_TCAs = MultiPolygon([p for p in self.poly_TCA._sub_polys])
        
        self.upoly_LDCs = unary_union(self.mpoly_LDCs)
        
        
        
    def _calculate_containment(self):
        """
        Calculates the containment ratio of the patterned shot.
        """
        contained = self.shots["Contained"]["combined"]
        uncontained = self.shots["Uncontained"]["combined"]
        self.containment_ratio = contained.area / \
            (contained.area + uncontained.area)
    

    
    def _determine_grid_rows_cols(self):
        """
        Method of determining the number of rows and columns to use.
        For GridPatterning LDC algorithm only.
        """
        # Might need better method of doing this
        r = 1
        c = 1
        num = r*c
        while num <= self.num_shots:
            num = r*c
            if num >= self.num_shots:
                break
            r += 1
            num = r*c
            if num >= self.num_shots:
                break
            c += 1
            if num >= self.num_shots:
                break
        rows = r
        cols = c
        return rows, cols
 
  
    
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
        """
        Sets the aspect of the axis to equal, re-limits and re-scales the axis.
        """
        ax.axis('equal')
        ax.relim()
        ax.autoscale(True)
        return ax


    
    def plot(self, ax=None, **kwargs):
        """ 
        Plots the TCAs, LDCs, contained & margin regions.
        """
        fig, ax = self._get_fig_ax(ax, **kwargs)
        self.TCA_alg.plot(ax=ax, zorder=0)
        self.plot_contained(ax)
        self.plot_margin(ax)
        self.plot_LDCs(ax)
        self.fig = fig
        self.ax = ax
        return ax
         
        
    def plot_contained(self, ax=None, **kwargs):
        """
        Plots the regions inside the TCA that the LDCs have contained.
        """
        fig, ax = self._get_fig_ax(ax, **kwargs)
        default_args = dict(fill=False, hatch="**", zorder=2)
        plot_args = self._get_kwargs(default_args, **kwargs)
        for contained in self.mpoly_contained:
            patch = PolygonPatch(contained, **plot_args)
            ax.add_patch(patch)
        self._rescale_axis(ax)
        return ax
    
    
    def plot_margin(self, ax=None, **kwargs):
        """
        Plots the regions outside of the TCA that the LDCs have contained.
        """
        fig, ax = self._get_fig_ax(ax, **kwargs)
        default_args = dict(alpha=0.5, color="blue", zorder=1)
        plot_args = self._get_kwargs(default_args, **kwargs)
        for margin in self.shots["Margin"]["individual"]:
            patch = PolygonPatch(margin, **plot_args)
            ax.add_patch(patch)
        self._rescale_axis(ax)
        return ax
    
    
    def plot_LDCs(self, ax=None, **kwargs):
        """
        Plots the outline of the LDCs combined into a single polygon.
        """
        fig, ax = self._get_fig_ax(ax, **kwargs)
        default_args = dict(fill=False, lw=3, edgecolor="cyan", zorder=3)
        plot_args = self._get_kwargs(default_args, **kwargs)
        patch = PolygonPatch(self.upoly_LDCs, **plot_args)
        ax.add_patch(patch)
        self._rescale_axis(ax)
        return ax




if __name__ == "__main__":
    
    # ========================================================================
    # Threat Model Testing
    # ========================================================================
    # plt.close("all")
    # start = time.time()
    
    # # Inputs
    # num_shots = 10         # None
    # num_to_cover = None    # "all"  
    # plot = True           # For quick viewing of geometries
    # xfact = 0.5            # X and Y scaling of polygon on update()
    # yfact = 0.5
    
    
    # # Loading Data
    # pickle_file = r"C:\Users\msommers\Desktop\Output\threat_model\2A_25I_2T.pickle"
    # pickle_file = r"C:\Users\msommers\Desktop\Output\threat_model\1A_5I_1T.pickle"
    # raw_data = misc_functions.read_pickle_data(pickle_file)
    
    
    # # Reformatting Data
    # data_dict = misc_functions.data_reformatter_v4(raw_data)


    # # Using data at t0 to illustrate algorithms
    # times = sorted(list(data_dict.keys()))
    # t0 = times[0]
    # tca_id = 0
    # tcas = data_dict[t0]["TCA"]
    # id_filt = tcas["ID"] == tca_id
    # tca = tcas[id_filt]
        
    # ldcs = data_dict[t0]["LDC"]
    # id_filt = ldcs["TCA_ID"] == tca_id
    # ldc = ldcs[id_filt]
    # ldc_id = ldc["ID"].iloc[0]
    # print("\tloading data elapsed: {}".format(round(time.time() - start, 2 )))
    
    
    # # Initializing Shot Pattern Algorithms
    # sp = PatternShots(tca, ldc, 
    #                   num_shots=num_shots, 
    #                   num_to_cover=num_to_cover,
    #                   xcol="EPx", ycol="EPy",
    #                   )
    #                   #TCA_algorithm=GridPatterning)    
    
    # # Creating new polygons by scaling originals (to illustrate update())
    # new_tca = scale(sp.poly_TCA.poly, xfact=xfact, yfact=yfact)
    # new_tca = translate(new_tca, xoff=2000, yoff=0)
    # x, y = new_tca.boundary.coords.xy
    # tca_df = pd.DataFrame({"EPx":x, "EPy":y, "ID":tca_id})
    
    # new_ldc = scale(sp.poly_LDC.poly, xfact=xfact, yfact=yfact)
    # x, y = new_ldc.boundary.coords.xy
    # ldc_df = pd.DataFrame({"EPx":x, "EPy":y, "ID":ldc_id})
    
    # # Plot the original TCA and LDCs
    # if plot:
    #     # plt.close("all")
    #     fig, ax = plt.subplots(figsize=(13.3, 7.5))
    #     sp.plot(ax=ax)
    
    # # Update at each time step
    # sp.update(tca_df, ldc_df)
    
    # # Plot the updated TCA and LDCs
    # if plot:
    #     sp.plot(ax=ax)
    #     fig.tight_layout()
    #     plt.show()

    # # shiftx = 2200
    # # some_times = np.arange(0.05, 5.0, 0.5)
    # # # some_times = times[:]
    # # for t in some_times:
    # #     t = str(t)
    # #     tcas = data_dict[t]["TCA"]
    # #     id_filt = tcas["ID"] == tca_id
    # #     tca = tcas[id_filt]
        
    # #     # Shifting Polygon along X axis for viewing purposes only
    # #     poly = PolygonBuilder.create_from_df(tca, "EPx", "EPy").poly
    # #     shifted_poly = translate(poly, xoff=shiftx)
    # #     tca = sp.create_df_from_poly(shifted_poly)
        
    # #     ldcs = data_dict[t]["LDC"]
    # #     id_filt = ldcs["TCA_ID"] == tca_id
    # #     ldc = ldcs[id_filt]
    # #     ldc_id = ldc["ID"].iloc[0]
    
    # #     sp.update(tca, ldc)
    # #     sp.plot(ax=ax)
        
    # #     shiftx += 2000

    
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
    
    new_dict = {}
    for key, df in data_dict.items():
        for t, df_ in df.groupby("Time"):
            if t in new_dict:
                new_dict[t].update({key: df_})
            else:
                new_dict[t] = {key: df_}
                
    times = list(new_dict.keys())
    t0 = times[0]
    tcas = new_dict[t0]["TCA"]
    ldcs = new_dict[t0]["LDC"]
    
    tca_list = []
    ldc_list = []
    for tca_id, tca_df in tcas.groupby("ID"):
        tca_list.append(tca_df)
    for ldc_id, ldc_df in ldcs.groupby("ID"):
        ldc_list.append(ldc_df)    
    
    tca_df = tca_list[0]
    ldc_df = ldc_list[0]
    sp = PatternShots(tca_df, ldc_df, num_shots=None)
    sp.plot()
    
    # ========================================================================
    # NSim Testing
    # ========================================================================
    
    
    
    
    
    
    
    
    
    
    
    
    
    
