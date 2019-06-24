# Measurement msg, spherical coordinates
std_msgs/Header header
string robot_measured
float64 range		# m
float64 azimuth	   	# deg 0 to 360
float64 elevation  	# deg -90 to 90
float64 fit_error  	# dimensionless 0 to 2.0