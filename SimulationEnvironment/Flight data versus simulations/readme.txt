Description of programs

====Main Scripts =========================================================== Description =================================
compareSimulationAndFlightData.m				Compare flight path of simulated Dubin's UAV and indoor quadrotor experiments. Four scenarios were evaluated, flight data found in 'Scenario_i_final.txt' files where 'i' represents the scenario number
optimizedGVF.m							A compy of optimizationGVF.m found elsewhere (used for lookup table generation). Takes UAV parameters, obstacle config, and determines optimized solution


==== Functions/helper Scripts =============================================== Description =================================
calcLatDist.m							Determine lateral distance to closet point of a pre-calculated path
cndr.m								GVF class for generating vectors converging and following a circular path
costANDerror.m							Calculate path deviation cost, calculate error from optimal path					
genOptPath.m							Generate the geometrically optimal path
gvfLine.m							GVF class for generating vectors converging and following a line path
icPoints.m							Generates an array of initial conditions used for locating singularities in summed GVF
locateSingularities.m 						Search for singularities in a GVF numerically with fsolve with initial conditions from icPoints.m	
importFile.m							Imports experimental flight data to be parsed					
UAV.m 								Dubins UAV class
vectorField.m 							Class for managing summed GVF - creation, summation, guidance vectors, weighting, and plotting functionality





