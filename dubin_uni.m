% Function for ode45 specifying dynamics of Dubin's unicycle

function dydt = dubin_uni(t,y,noise)
% y - full state plus input [x y theta v omega]

x = y(1); y_ = y(2); theta = y(3); v = y(4); omega = y(5); 

dydt(1) = v*cos(theta);
dydt(2) = v*sin(theta);
dydt(3) = omega;
dydt(4) = 0;
dydt(5) = 0;
dydt = dydt';

if (size(noise,1) == size(dydt,1)) && (size(noise,2) == size(dydt,1))
    w = mvnrnd(zeros(1,size(dydt,1)),noise)';
else
    w = zeros(size(dydt));
end
dydt = dydt + w;

end