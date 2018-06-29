Description of programs

====Main Scripts =========================================================== Description =================================
wpts_vff_gvf_PLOTS.m						Compre the cost of waypoint, virtual force field (VFF), and optimized GVF and display a UAVs route

==== Functions/helper Scripts =============================================== Description =================================
calcLatDist.m							Determine lateral distance to closet point of a pre-calculated path
cndr.m								GVF class for generating vectors converging and following a circular path
costANDerror.m							Calculate path deviation cost, calculate error from optimal path					
genOptPath.m							Generate the geometrically optimal path
gvfLine.m							GVF class for generating vectors converging and following a line path			
UAV.m 								Dubins UAV class
vectorField.m 							Class for managing summed GVF - creation, summation, guidance vectors, weighting, and plotting functionality
wpt.m								Waypoint generation class
waypontPlanner.m						Generates diversion waypoints around a circular obstacle
vff.m								virtual force field guidance class. Similar to GVF in structure and usage