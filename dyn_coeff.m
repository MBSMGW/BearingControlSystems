function [K, B] = dyn_coeff(mesh_parameters,geometry_parameters, operational_parameters,omega, X0, Y0)
dX=0.1*X0;
dY=0.1*Y0;
dXdot=omega*dX;
dYdot=omega*dY;

delta=[dX 0 0 0; -dX 0 0 0; 0 dXdot 0 0; 0 -dXdot 0 0; 0 0 dY 0; 0 0 -dY 0; 0 0 0 dYdot; 0 0 0 -dYdot];

tic
for i=1:8
    i
    initial_parameters=[X0+delta(i,1) delta(i,2) Y0+delta(i,3) delta(i,4)];
    [FX,FY]= CalculateLoadCapacity(mesh_parameters,geometry_parameters, operational_parameters,initial_parameters)
    Fx(i)=FX;
    Fy(i)=FY;
end
toc

Kxx=-(Fx(1)-Fx(2))/(2*dX);
Kxy=-(Fx(5)-Fx(6))/(2*dY);
Kyx=-(Fy(1)-Fy(2))/(2*dX);
Kyy=-(Fy(5)-Fy(6))/(2*dY);

Bxx=-(Fx(3)-Fx(4))/(2*dXdot);
Bxy=-(Fx(7)-Fx(8))/(2*dYdot);
Byx=-(Fy(3)-Fy(4))/(2*dXdot);
Byy=-(Fy(7)-Fy(8))/(2*dYdot);

K=[Kxx Kxy; Kyx Kyy];
B=[Bxx Bxy; Byx Byy];
end

