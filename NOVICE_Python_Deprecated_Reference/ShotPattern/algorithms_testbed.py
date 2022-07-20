# -*- coding: utf-8 -*-
"""
Created on Mon Aug  2 14:19:17 2021

@author: msommers
"""


import pandas as pd
import numpy as np
import os, sys
import matplotlib.pyplot as plt
# from matplotlib.collections import LineCollection

import shapely.geometry as sgeom
from shapely.geometry import LineString, MultiLineString, MultiPolygon, Point, Polygon
# from shapely.ops import linemerge, polygonize, snap
from shapely.affinity import translate, scale, rotate

from descartes import PolygonPatch
# sys.path.insert(0, os.path.dirname(os.getcwd()))

# from scipy.spatial import Voronoi, voronoi_plot_2d #ConvexHull

from voronoi_patterning import VoronoiPatterning
from polygon_builder import PolygonBuilder
from kmeans_clustering import KmeansClustering
import misc_functions

import statistics




#=============================================================================
# Creating Shapes
#=============================================================================

def polygon_builder_test():
    pb = PolygonBuilder.create_circle(5)
    # pb = PolygonBuilder.create_ellipse(5,10, rotate_cw=30)
    # pb = PolygonBuilder.create_random_polygon(8, multiplier=1) 
    # pb = PolygonBuilder.create_regular_polygon(6, radius=5, rotate_cw=0)
    # points = [(0,0), (0,10), (10,10), (15,7), (10,0)]
    # pb = PolygonBuilder.create_from_points(points)
    return pb.poly



#=============================================================================
# Creating Algorithms
#=============================================================================

def kmeans_test(poly, num_clusters=3, num_points=3000):
    kmeans = KmeansClustering(poly, num_clusters, num_points)
    return kmeans

def voronoi_test(poly, cluster_centers):
    vor = VoronoiPatterning(poly, cluster_centers)
    return vor



#=============================================================================
# Plotting Algorithm Initialization Processes
#=============================================================================

def plot_kmeans_process(poly, num_clusters=3, num_points=3000):
    kmeans = KmeansClustering(poly, num_clusters, num_points)
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, figsize=(10,10))
    cmap = plt.get_cmap("tab10")
    
    kmeans.plot_poly(ax1)
    kmeans.plot_points_distribution(ax1, color=cmap(0))
    ax1.set_title("Step 1: Fill polygon with uniformly distributed points")
    
    kmeans.plot_poly(ax2)
    kmeans.plot_points_distribution(ax2)
    ax2.set_title("Step 2: Use Kmeans algorithm to assign points to clusters")
    
    kmeans.plot_poly(ax3)
    kmeans.plot_clusters(ax3)
    ax3.set_title("Step 3: Create convex hull from each cluster")
    
    kmeans.plot_poly(ax4)
    kmeans.plot_clusters(ax4)
    kmeans.plot_centroids(ax4, markersize=10)
    ax4.set_title("Step 4: Get the centroid of each convex hull")
    
    fig.tight_layout()
    return kmeans



def plot_voronoi_process(poly, kmeans):
    fig, (ax1, ax2, ax3) = plt.subplots(3, figsize=(10,10))
    cmap = plt.get_cmap("tab10")
    
    cluster_centers = kmeans.cluster_centers
    vor = VoronoiPatterning(poly, cluster_centers)
    # poly = poly.poly
    
    kmeans.plot_poly(ax1)
    kmeans.plot_clusters(ax1)
    kmeans.plot_centroids(ax1, markersize=10)
    ax1.set_title("Step 1: Get the centroids")
    
    vor.plot_poly(ax2)
    vor.plot_centroids(ax2, markersize=10)
    vor.plot_voronoi_lines(ax2)
    ax2.set_title("Step 2: Use Voronoi algorithm to create lines using centroids")
    
    vor.plot_poly(ax2)
    
    vor.plot(ax=ax3)
    ax3.set_title("Step 6: Slice the polygon with the Voronoi Lines")    
    fig.tight_layout()




def plot_kmeans_voronoi_process(poly, num_clusters=3, num_points=3000):
    kmeans = KmeansClustering(poly, num_clusters, num_points)
    fig, ((ax1, ax2, ax3), (ax4, ax5, ax6)) = plt.subplots(nrows=2, ncols=3, 
                                                           figsize=(13.3,7.5))
    cmap = plt.get_cmap("tab10")
    
    kmeans.plot_poly(ax1)
    kmeans.plot_points_distribution(ax1, color=cmap(0))
    ax1.set_title("Step 1: Fill polygon with\nuniformly distributed points")
    
    kmeans.plot_poly(ax2)
    kmeans.plot_points_distribution(ax2)
    ax2.set_title("Step 2: Use Kmeans algorithm\nto assign points to clusters")
    
    kmeans.plot_poly(ax3)
    kmeans.plot_clusters(ax3)
    ax3.set_title("Step 3: Create convex hull\nfrom each cluster")
    
    kmeans.plot_poly(ax4)
    kmeans.plot_clusters(ax4)
    kmeans.plot_centroids(ax4, markersize=10)
    ax4.set_title("Step 4: Get the centroid\nof each convex hull")
    
    vor = VoronoiPatterning(poly, kmeans.cluster_centers)
    vor.plot_poly(ax5)
    vor.plot_centroids(ax5, markersize=10)
    vor.plot_voronoi_lines(ax5)
    ax5.set_title("Step 5: Use Voronoi algorithm\nto create lines using centroids")
    
    vor.plot(ax=ax6)
    ax6.set_title("Step 6: Slice the polygon\nwith the Voronoi lines")
    
    fig.tight_layout()
    return kmeans, vor
    



#=============================================================================
# Plotting Algorithm Update Processes
#=============================================================================    
        
def plot_voronoi_update(poly, kmeans):
    """ 
    Correct update of consistent shape, decreasing in size and translating 
    """
    fig, (ax1, ax2, ax3) = plt.subplots(nrows=1, ncols=3, figsize=(13.3,7.5))
    cmap = plt.get_cmap("tab10")
    
    cluster_centers = kmeans.cluster_centers
    vor = VoronoiPatterning(poly, cluster_centers)
    vor.plot(ax=ax1)
    vor.plot_interior_voronoi_lines(ax1)
    vor.plot_voronoi_lines(ax1)
    xlim, ylim = ax1.get_xlim(), ax1.get_ylim()
    
    smaller_poly = scale(poly, xfact=0.75, yfact=0.75)
    smaller_shifted_poly = translate(smaller_poly, xoff=1, yoff=1)
    vor.update(smaller_shifted_poly)
    vor.plot(ax=ax2)
    vor.plot_voronoi_lines(ax2)
    vor.plot_interior_voronoi_lines(ax=ax2)
    
    smallest_poly = scale(poly, xfact=0.5, yfact=0.5)
    smallest_shifted_poly = translate(smallest_poly, xoff=2, yoff=2)
    vor.update(smallest_shifted_poly)
    vor.plot(ax=ax3)
    vor.plot_voronoi_lines(ax3)
    vor.plot_interior_voronoi_lines(ax=ax3)
    
    time = 0
    for ax in [ax1, ax2, ax3]:
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
        ax.set_title("Time: {}".format(time))
        time += 1
    return vor
              
        
        
def plot_voronoi_incorrect_update(poly, kmeans):
    """ 
    Incorrect update of consistent shape, decreasing in size and translating
    Re-initialzation occurs at every time step - BAD
    """
    
    fig, (ax1, ax2, ax3) = plt.subplots(nrows=1, ncols=3, figsize=(13.3,7.5))
    cmap = plt.get_cmap("tab10")
    
    cluster_centers = kmeans.cluster_centers
    num_clusters = kmeans.num_clusters
    vor = VoronoiPatterning(poly, cluster_centers)
    vor.plot(ax=ax1)
    vor.plot_interior_voronoi_lines(ax1)
    vor.plot_voronoi_lines(ax1)
    xlim, ylim = ax1.get_xlim(), ax1.get_ylim()
    
    smaller_poly = scale(poly, xfact=0.75, yfact=0.75)
    smaller_shifted_poly = translate(smaller_poly, xoff=1, yoff=1)
    kmeans2 = KmeansClustering(smaller_shifted_poly, 
                               num_clusters=num_clusters)
    
    vor2 = VoronoiPatterning(smaller_shifted_poly, kmeans2.cluster_centers)
    vor2.plot(ax=ax2)
    vor2.plot_voronoi_lines(ax2)
    vor2.plot_interior_voronoi_lines(ax=ax2)
    
    smallest_poly = scale(poly, xfact=0.5, yfact=0.5)
    smallest_shifted_poly = translate(smallest_poly, xoff=2, yoff=2)
    kmeans3 = KmeansClustering(smallest_shifted_poly, 
                               num_clusters=num_clusters)
    
    vor3 = VoronoiPatterning(smallest_shifted_poly, kmeans3.cluster_centers)
    vor3.plot(ax=ax3)
    vor3.plot_voronoi_lines(ax3)
    vor3.plot_interior_voronoi_lines(ax=ax3)
    
    time = 0
    for ax in [ax1, ax2, ax3]:
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
        ax.set_title("Time: {}".format(time))
        time += 1
    return vor    
  
    
  
  
def plot_voronoi_update_reinit(poly, kmeans, repattern_threshold=0.15):
    """ 
    Correct update of changing shape, decreasing in size and translating.
    Example of re-initialization.
    
    A low repattern_threshold increases the probability of Voronoi 
    re-initialization.
    
    """
    fig, (ax1, ax2, ax3) = plt.subplots(nrows=1, ncols=3, figsize=(13.3,7.5))
    cmap = plt.get_cmap("tab10")
    
    cluster_centers = kmeans.cluster_centers
    vor = VoronoiPatterning(poly, cluster_centers, 
                            repattern_threshold=repattern_threshold)
    vor.plot(ax=ax1)
    vor.plot_interior_voronoi_lines(ax1)
    vor.plot_voronoi_lines(ax1)
    xlim, ylim = ax1.get_xlim(), ax1.get_ylim()
    
    smaller_poly = scale(poly, xfact=0.7, yfact=0.4)
    smaller_poly = rotate(smaller_poly, 30)
    smaller_shifted_poly = translate(smaller_poly, xoff=1, yoff=1)
    vor.update(smaller_shifted_poly)
    vor.plot(ax=ax2)
    vor.plot_voronoi_lines(ax2)
    vor.plot_interior_voronoi_lines(ax=ax2)
    
    smallest_poly = scale(smaller_poly, xfact=0.5, yfact=0.5)
    smallest_shifted_poly = translate(smallest_poly, xoff=2, yoff=2)
    vor.update(smallest_shifted_poly)
    vor.plot(ax=ax3)
    vor.plot_voronoi_lines(ax3)
    vor.plot_interior_voronoi_lines(ax=ax3)
    
    time = 0
    for ax in [ax1, ax2, ax3]:
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
        ax.set_title("Time: {}".format(time))
        time += 1
    return vor   
  
    
  

#=============================================================================
# Plotting Voronoi Area Statistics
#=============================================================================
  
def plot_voronoi_areas(poly, kmeans, repattern_threshold=0.8, 
                       xfact=0.5, yfact=0.75, 
                       use_median=False, density=True,
                       sigma_threshold=1.5, sigma_values=[1, 1.5, 2]):
    """
    For a polygon and a scaled polygon:
    Plots the voronoi sub-polygons, a bar chart of each sub-polygon's area,
    and a histogram of the sub-polygon areas
    
    repattern_threshold, int
        If the polygon's line scale exceeds this value, the voronoi algorithm
        will re-pattern (or re-initialize)
    xfact and yfact, int
        x and y scale factors used to create the new polygon from the original
    use_median, bool
        If True, plots the median line instead of the mean line
    density, bool
        If True, plots the probability density on the yaxis of the histogram
        If False, pltos the number of occurences on the yaxis of the histogram
    sigma_threshold, int or float
        For each sub-polygon, if its area is not within the range of 
        mean +- sigma_threshold*sigma, then the polygon shape will be outlined
        in a different color.
    sigma_values, list of ints or floats
        List of sigma values to plot
        Ex: [1, 1.5] would plot a line 4 lines total on each applicable plot:
            (1) mean + 1*sigma,  (2) mean - 1*sigma
            (3) mean + 1.5*sigma (4) mean - 1.5*sigma
    """
    fig, ((ax1, ax2), (ax3, ax4), (ax5, ax6)) = plt.subplots(nrows=3, ncols=2, 
                                                        figsize=(13.3,7.5))
    cmap = plt.get_cmap("tab10")
    cluster_centers = kmeans.cluster_centers
    vor = VoronoiPatterning(poly, cluster_centers, 
                            repattern_threshold=repattern_threshold)

    if use_median:
        mu_label = "Median"
    else:
        mu_label = "Mean"
    
    vor.plot(ax=ax1)
    vor.plot_voronoi_poly_numbers(ax=ax1)
    areas, sigma, mu = get_areas_sigma_mu(vor, use_median)
    plot_areas_barchart(ax3, areas, mu, sigma, value=sigma_threshold)
    plot_areas_hist(ax5, areas, mu, sigma, density=density)
    
    plot_args = dict(lw=2, ls="--")
    plot_sigmas(ax3, sigma_values, mu, sigma, vline=False, **plot_args)
    plot_axhline(ax3, mu, label=mu_label, color="cyan", lw=3)
    plot_sigmas(ax5, sigma_values, mu, sigma, vline=True, **plot_args)
    plot_axvline(ax5, mu, label=mu_label, color="cyan", lw=3)
    plot_polys_outside_sigma_range(ax1, vor, mu, sigma, sigma_threshold)
    
    ax3.legend(title="Sigma = {}".format(round(sigma, 2)), loc="upper right")
    ax5.legend(title="Sigma = {}".format(round(sigma, 2)), loc="upper right")
    
    new_poly = scale(poly, xfact=xfact, yfact=yfact)
    vor.update(new_poly)
    
    vor.plot(ax=ax2)
    vor.plot_voronoi_poly_numbers(ax=ax2)
    
    ##### Testing #####
    # Keeping the same standard deviation from the original polygon
    areas, unused_sigma_, mu = get_areas_sigma_mu(vor, use_median)
    
    # Using the new standard deviation of the new polygon
    # areas, sigma, mu = get_areas_sigma_mu(vor, use_median)
    
    # global perimeter_ratio
    # perimeter_ratio = new_poly.length/poly.length
    # sigma *= perimeter_ratio
    ##### Testing #####
    
    plot_areas_barchart(ax4, areas, mu, sigma, value=sigma_threshold)
    plot_areas_hist(ax6, areas, mu, sigma, density=density)
    plot_sigmas(ax4, sigma_values, mu, sigma, vline=False, **plot_args)
    plot_axhline(ax4, mu, label=mu_label, color="cyan", lw=3)
    plot_sigmas(ax6, sigma_values, mu, sigma, vline=True, **plot_args)
    plot_axvline(ax6, mu, label=mu_label, color="cyan", lw=3)
    plot_polys_outside_sigma_range(ax2, vor, mu, sigma, sigma_threshold)
     
    ax1.set_title("Original Polygon")
    ax3.set_title("Original Polygon: Areas")
    ax5.set_title("Original Polygon: Areas Histogram")
    
    ax2.set_title("New Polygon")
    ax4.set_title("New Polygon: Areas")
    ax6.set_title("New Polygon: Areas Histogram")
    
    ax4.legend(title="Sigma = {}".format(round(sigma, 2)), loc="upper right")
    ax6.legend(title="Sigma = {}".format(round(sigma, 2)), loc="upper right")
    
    fig.tight_layout()
    return vor


def plot_polys_outside_sigma_range(ax, vor, mu, sigma, sigma_threshold,
                                   color="cyan"):
    patch = None
    for i, poly in enumerate(vor.sub_polys, start=1):
        area = poly.area
        within_xsigma = area > mu-sigma_threshold*sigma and \
                        area < mu+sigma_threshold*sigma
        if not within_xsigma:
            patch = PolygonPatch(poly, edgecolor=color, lw=2, fill=False)
            ax.add_patch(patch)
    if patch:
        patch.set_label("Outside {} Sigma".format(sigma_threshold))
        ax.legend(title="Sigma = {}".format(round(sigma,2)),
                  loc="upper right")
    


def plot_sigmas(ax, values, mu, sigma, vline=True, **plot_args):
    colors = ["lightgreen", "firebrick", "violet", "purple"]
    hline = not vline
    for i, val in enumerate(values):
        if vline:
            x1 = mu-val*sigma
            x2 = mu+val*sigma
            left = plot_axvline(ax, x1,
                                color=colors[i], **plot_args)
            right = plot_axvline(ax, x2, 
                                 label="{} Sigma".format(val), 
                                 color=colors[i], **plot_args)
            ax.axvspan(x1, x2, alpha=0.5, color=left.get_color())
        if hline:
            y1 = mu-sigma*val
            y2 = mu+sigma*val
            lower = plot_axhline(ax, y1,
                                 color=colors[i], **plot_args)
            upper = plot_axhline(ax, y2, label="{} Sigma".format(val), 
                                 color=colors[i], **plot_args)
            ax.axhspan(y1, y2, alpha=0.5, color=lower.get_color()) 
            

def get_areas_sigma_mu(vor, use_median=False):
    areas = [poly.area for poly in vor.sub_polys]
    sigma = statistics.pstdev(areas)
    if use_median:
        mu = np.median(areas)
    else:
        mu = np.mean(areas)
        # Testing: total poly area divided by num sub-polys
        # mu = vor.poly.area/len(vor.sub_polys)  
    return areas, sigma, mu


def plot_areas_barchart(ax, areas, mu, sigma, value=1.5):
    xpos = 0
    patch = None
    for i, area in enumerate(areas, start=1):
        ax.bar(xpos, area, width=1.0, alpha=0.8, align="center",
                edgecolor="black")
        ax.text(xpos, mu/10, i, ha="center", va="center")    
        xpos += 1    
    if patch:
        patch.set_label("Outside {} Sigma".format(value))
    ax.set_ylabel("Area")
    ax.set_xticks([])
    ax.set_xticklabels([])


def plot_areas_hist(ax, areas, mu, sigma, density=True):
    n, bins, patches = ax.hist(areas, density=density)
    y = ((1 / (np.sqrt(2 * np.pi) * sigma)) *
          np.exp(-0.5 * (1 / sigma * (bins - mu))**2))
    ax.plot(bins, y, '--', lw=2)
    if density:
        ax.set_ylabel("Probability Density")
    ax.set_xlabel("Area")
        
def plot_axhline(ax, val, **kwargs):
    hline = ax.axhline(val, **kwargs)
    return hline
    
def plot_axvline(ax, val, **kwargs):
    vline = ax.axvline(val, **kwargs)
    return vline







#=============================================================================
# Plotting Voronoi Threat Model Example
#=============================================================================

def voronoi_threat_model_example():
    # Inputs
    num_shots = 4
    save_fig = True
    save_pickle = False
    stacked = True
    pickle_file = r"C:\Users\msommers\Desktop\Output\threat_model\threat_model_data3.pickle"
    
    # Output directories
    outpath = r"C:\Users\msommers\Desktop\Output\voronoi_patterning"
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
    step = int(num_keys/10)   # arbitrary value
    interval = np.arange(0, num_keys, step)
    some_times = []
    for i, k in enumerate(keys):
        if i in interval:
            some_times.append(k)
    
    # Initializing KMeans and Voronoi algorithms
    poly_data = data_dict[str(t0)]["TCA_points"]
    poly = PolygonBuilder.create_from_df(poly_data, "x", "y")
    kmeans = KmeansClustering(poly, num_clusters=num_shots)
    cluster_centers = kmeans.cluster_centers
    print("Original\n[Time {}]".format(t0))
    vor = VoronoiPatterning(poly, cluster_centers)

    # Creating Voronoi plot
    fig, ax = plt.subplots(figsize=(10,10))
    
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
        vor.update(new_poly)
        vor.plot(ax=ax)
        vor.ax.set_title(
            "Time: {}\n(TCA scale factor: {}, Lines Scale Factor: {})".format(
            t, round(vor.poly_scale,3), round(vor.ls_used,3)))
        
        # Saving Images
        if save_fig:
            if stacked:    
                filepath = os.path.join(png_outpath, "VoronoiStacked{}_{}.png"
                                        .format(num_shots, t)) 
            else:
                filepath = os.path.join(png_outpath, "Voronoi{}_{}.png"
                                        .format(num_shots, t)) 
            vor.fig.savefig(filepath)
            
        # Saving Pickled Figures
        if save_pickle:
            if stacked:
                filename = "VoronoiStacked{}_{}.pickle".format(num_shots, t)
            else:
                filename = "Voronoi{}_{}.pickle".format(num_shots, t)
            misc_functions.pickle_figure(fig, pickle_outpath, filename)
        
        plt.show()
    return vor




#=============================================================================
# Basic Voronoi Example
#=============================================================================

def voronoi_basic_example():
    plt.close('all')
    # points = [(0,0), (0,10), (10,10), (15,7), (10,0)]
    points = [(0,0), (0,10), (20,10), (20,0)]
    poly = sgeom.Polygon(points)
    num_clusters = 40
    kmeans = KmeansClustering(poly, num_clusters=num_clusters, 
                              init="k-means++")
    vor = VoronoiPatterning(poly, kmeans.cluster_centers)
    
    fig, ax = plt.subplots()
    vor.debug_plot(ax=ax)
    
    global all_intersections
    all_intersections = []
    for vs in vor.all_segments:
        segment_intersections = list(poly.intersection(vs).coords)
        all_intersections.extend(segment_intersections)

    for ai in all_intersections:
        ax.plot(ai[0], ai[1], marker="x", markersize=10, color="cyan")
    
    vor.plot()
    return vor




#=============================================================================
# Voronoi Nsim Example
#=============================================================================

def voronoi_nsim_example():

    def create_fig(rows, cols):
        fig = plt.figure(figsize=(13.3, 7.5))
        gs = fig.add_gridspec(nrows=rows, ncols=cols)
        axes = []
        for r in range(rows):
            for c in range(cols):
                ax = fig.add_subplot(gs[r,c])
                ax.axis("equal")
                axes.append(ax)
        fig.tight_layout()
        return fig
    
    def assign_TCAs_to_axes(tca_ids):
        ax_dict = {}
        for idx, tca_id in enumerate(tca_ids):
            ax_dict[tca_id] = idx
        return ax_dict

    # Voronoi Settings
    save_fig = False
    save_pickle = False
    num_shots = 3
    num_to_cover = 1   
    repattern_threshold = 0.5
    
    # Input Directories
    nsim_dir = r"C:\Users\msommers\Desktop\NSim\Full20210607CBFC\Full20210607\NSim"
    bin_dir = os.path.join(nsim_dir, "bin")
    scenario_name = "CB_L1"    # CB_L1, CB_L2, CB_L3, CB_L4x2, CB_Raid    
    scenario_dir = os.path.join(bin_dir, scenario_name)
    data_dir = scenario_dir      # directory to read output from (bin_dir)
    
    # Output Directories
    outpath = r"C:\Users\msommers\Desktop\Output\nsim"
    pickle_outpath = os.path.join(outpath, scenario_name, "pickle")
    png_outpath = os.path.join(outpath, scenario_name, "png")
    jpg_outpath = os.path.join(outpath, scenario_name, "jpg")
    
    # Read TCA & LDC patch files
    tca_file = os.path.join(data_dir, "TCAPatch.txt")
    ldc_file = os.path.join(data_dir, "LDCPatch.txt")
    header = "Time,ID,Type,ENUx,ENUy,ENUz,EPx,EPy,Zero".split(",")
    tca_df = pd.read_csv(tca_file, delimiter=" ") 
    ldc_df = pd.read_csv(ldc_file, delimiter=" ")
    ldc_df.columns = header
    tca_df.columns = header
    
    # Create Figure
    tca_ids = sorted(list(tca_df["ID"].unique()))
    num_tcas = len(tca_ids)
    rows, cols = misc_functions.calc_rows_cols(num_tcas)
    plt.close("all")
    ax_dict = assign_TCAs_to_axes(tca_ids)
    fig = create_fig(rows, cols)
    
    # Create Data Dictionary
    data_dict = {}
    for t, ldf in ldc_df.groupby("Time"):
        data_dict[t] = {}
        data_dict[t]["LDC_EP_points"] = []
        data_dict[t]["LDC_ENU_points"] = []
        data_dict[t]["TCA_EP_points"] = []
        data_dict[t]["TCA_ENU_points"] = []
        
        # Filtering TCA data by ID
        time_filt = tca_df["Time"] == t
        
        ep_cols = ["Time", "ID", "EPx", "EPy"]
        enu_cols = ["Time", "ID", "ENUx", "ENUy", "ENUz"]
        for num, (id_, ldf_) in enumerate(ldf.groupby("ID")):
            
            # Switching the LDCs and TCAs for testing purposes only
            data_dict[t]["TCA_EP_points"].append(ldf_[ep_cols])
            data_dict[t]["TCA_ENU_points"].append(ldf_[enu_cols]) 
            
            # Filtering TCA data (by time and ID)
            id_filt = tca_df["ID"] == id_
            filts = time_filt & id_filt
    
            # Switching the LDCs and TCAs for testing purposes only
            data_dict[t]["LDC_EP_points"].append(tca_df[filts][ep_cols]) 
            data_dict[t]["LDC_ENU_points"].append(tca_df[filts][enu_cols]) 
    
    # Extracting list of times
    times = sorted([float(x) for x in data_dict.keys()])
    t0  = times[0]
    
    # Creating set of desired times at which to plot
    num_times = len(times)
    step = int(num_times/10)   # arbitrary value
    interval = np.arange(0, num_times, step)
    some_times = []
    for i, k in enumerate(times):
        if i in interval:
            some_times.append(k)
    
    poly_data = data_dict[t0]["TCA_EP_points"]
    voronoi_dict = {}
    for t in some_times[:15]:
        poly_data = data_dict[t]["TCA_EP_points"]
        time_str = "[Time {}]".format(t)
        print("\n{}".format(time_str))
        
        for df in poly_data:    
            poly = PolygonBuilder.create_from_df(df, "EPx", "EPy")
            id_ = df["ID"].iloc[0]
            ax = fig.axes[ax_dict[id_]]
            print("\t[ID {}]".format(id_))
            
            if id_ in voronoi_dict:
                # Updating KMeans and Voronoi algorithms
                vor = voronoi_dict[id_]
                vor.update(poly)
                vor.plot(ax=ax)
                fig = vor.fig
            else:
                # Initializing KMeans and Voronoi algorithms
                kmeans = KmeansClustering(poly, num_clusters=num_shots)
                cluster_centers = kmeans.cluster_centers
                vor = VoronoiPatterning(poly, cluster_centers, 
                                        repattern_threshold=repattern_threshold)
                vor.plot(ax=ax)
                voronoi_dict[id_] = vor
            
            title = \
                """
                Time {}, {} Shots, 
                TCA Scale {}, Line Scale {},
                Re-Pattern Threshhold: {}%
                """.format(t, num_shots,
                            round(vor.poly_scale,3), round(vor.ls_used,3),
                            round(vor.repattern_threshold*100,1)
                            )
            ax.set_title(title)
            fig.tight_layout()
            print()
            
        # Saving Images
        if save_fig:
            filepath = os.path.join(png_outpath, "{}_Voronoi_{}shots_{}.png"
                                    .format(scenario_name, num_shots, t)) 
            vor.fig.savefig(filepath)
            
        # Saving Pickled Figures
        if save_pickle:
            filename = "{}_Voronoi{}_{}shots_{}.pickle".format(scenario_name,
                                                                num_shots, t)
            misc_functions.pickle_figure(fig, pickle_outpath, filename)







        
if __name__ == "__main__":
    plt.close("all")
    
    poly = polygon_builder_test()
    num_clusters = 5
    kmeans = KmeansClustering(poly, num_clusters=num_clusters)
    vor = VoronoiPatterning(poly, kmeans.cluster_centers, 
                            repattern_threshold=0.6)
    
    #=========================================================================
    # Plotting Algorithm Initialization Processes
    #=========================================================================
    # kmeans = plot_kmeans_process(poly, num_clusters=num_clusters)
    # vor = plot_voronoi_process(poly, kmeans)
    # kmeans, vor = plot_kmeans_voronoi_process(poly, num_clusters=num_clusters)
    
    
    
    #=========================================================================
    # Plotting Algorithm Update Processes
    #=========================================================================
    # vor = plot_voronoi_incorrect_update(poly, kmeans)
    # vor = plot_voronoi_update(poly, kmeans)
    # vor = plot_voronoi_update_reinit(poly, kmeans, repattern_threshold=0.15)
    
    
    
    #=========================================================================
    # Plotting Voronoi Area Statistics
    #=========================================================================
    # poly = PolygonBuilder.create_regular_polygon(8, 10).poly
    # num_clusters = 10
    # kmeans = KmeansClustering(poly, num_clusters=num_clusters)
    # vor = plot_voronoi_areas(poly, kmeans, repattern_threshold=0.8,
    #                           xfact=0.85, yfact=0.5,
    #                           sigma_threshold=1.5, sigma_values=[1.5],
    #                           use_median=True, density=True)
    
   
    #=========================================================================
    # Basic Voronoi Example
    #=========================================================================
    # vor = voronoi_basic_example()
    
    
    
    #=========================================================================
    # Voronoi Threat Model Example
    #=========================================================================
    # vor = voronoi_threat_model_example()
    
    
    
    #=========================================================================
    # Voronoi Nsim Example
    #=========================================================================
    # vor = voronoi_nsim_example()
    
   
    plt.show()
    