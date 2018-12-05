% range and bearing from fixed sensor to target

function y_ = rel_range_bearing(x1_,x2_,R)

x1 = x1_(1); y1 = x1_(2); theta1 = x1_(3);
x2 = x2_(1); y2 = x2_(2); theta2 = x2_(3);

y_ = [atan2(y2-y1,x2-x1)-theta2; sqrt((x2-x1)^2 + (y2-y1)^2)] + mvnrnd(zeros(1,size(R,1)),R)';


end