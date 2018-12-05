% range and bearing from fixed sensor to target

function y_ = range_bearing(x_,R)

x = x_(1); y = x_(2); theta = x_(3);

y_ = [atan2(y,x); sqrt(x^2 + y^2)] + mvnrnd(zeros(1,size(R,1)),R)';


end