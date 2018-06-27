Description of programs

====Main Scripts =========================================================== Description =================================
optimizedGVF.m						Determines optimized parameters for GVF for a single scenario
optimizedGVFLookupTable.m				Constructs circulation and decay lookup tables for various avoidance scenarios
optimizedGVFLookupTableInterpolatedAnalysis.m		Compare interpolated lookup table solution to the optimized solution
searchForOptimizedGVF.m					Evaluate cost for an array of solutions (H,k) for a single avoidance scenario (Brute force)
findSingularities.m 					An initially attempt at locating singularities in summed GVF numerically	

==== Functions/helper Scripts =============================================== Description =================================
calcLatDist.m							Determine lateral distance to closet point of a pre-calculated path
cndr.m								GVF class for generating vectors converging and following a circular path
costANDerror.m							Calculate path deviation cost, calculate error from optimal path					
genOptPath.m							Generate the geometrically optimal path
gvfLine.m							GVF class for generating vectors converging and following a line path
icPoints.m							Generates an array of initial conditions used for locating singularities in summed GVF
locateSingularities.m 						Search for singularities in a GVF numerically with fsolve with initial conditions from icPoints.m						
UAV.m 								Dubins UAV class
vectorField.m 							Class for managing summed GVF - creation, summation, guidance vectors, weighting, and plotting functionality



