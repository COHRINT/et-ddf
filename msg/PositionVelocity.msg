# The regular Odometry msg interestingly does not support specifying correlations between
#  position and velocity states, which is critical for covariance intersection between
#  strapdown and tracking filter
geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
float64[] covariance # 6x6