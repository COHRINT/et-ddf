function A = Atilde(x,u,t)

A = [0 0 -u(1)*sin(x(3));
    0 0 u(1)*cos(x(3));
    0 0 0];

end