Shot Plane Tools using Python

1. polygon_builder.py
	- contains PolygonBuilder class, used to create different types of shapes for Shot Patterning Algorithms.
	
2. kmeans_clustering.py
	- contains KmeansClustering class, used to break polygon into specified number of clusters
		- uniformly fills polygon with points, assigns each point to a specific cluster
		- creates convex hull from each set of points in a cluster
		
3. voronoi_patterning.py 
	- contains VoronoiPatterning class, used to partition polygon by a set of 3 or more points

4. grid_patterning.py
	- contains GridPatterning class, used to partition a polygon by overlaying a grid on top of it
		- currently has two types of grids: Minimum Rotated Rectangle (MRR) Grid, and Centroid Grid

5. misc_functions.py
	- contains miscellaneous functions for easy use
	
6. cover_center.py
	- contains CoverCenter class, used to place LDCs on TCAs and shift them as needed to cover the TCA center.

7. cover_center_v2.py
	- modified version of CoverCenter class, which supports multiple TCAs instead of just one
	
8. shot_patterns.py
	- contains PatternShots class, used to test out Shot Patterning Algorithms.

9. shot_patterns_slider.py
	- contains modified PatternShots class, which contains a slider used for viewing Shot Patterns over time.

10. shot_patterns_slider_v2.py
	- similar to shot_patterns_slider.py, except it is being restructured to support multiple TCAs.
  
11. config.py
	- configuration file, currently only contains the algorithms to be used for different aspects of shot patterning.

12. algorithms_testbed.py
	- contains functions illustrating features/uses of different algorithms (kmeans_clustering, voronoi_patterning, etc.). 
	  Used for prototyping, testing, and producing documentation plots.
