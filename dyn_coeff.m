function [K, B] = dyn_coeff(mesh_parameters,geometry_parameters, operational_parameters,omega, X0, Y0, FX0, FY0)
dX=0.5e-6;
dY=0.5e-6;
dXdot=omega*dX;
dYdot=omega*dY;

delta=[dX 0 0 0; 0 dXdot 0 0; 0 0 dY 0; 0 0 0 dYdot;];

tic
for i=1:4
    i
    initial_parameters=[X0+delta(i,1) delta(i,2) Y0+delta(i,3) delta(i,4)];
    [FX,FY]= CalculateLoadCapacity(mesh_parameters,geometry_parameters, operational_parameters,initial_parameters)
    Fx(i)=FX;
    Fy(i)=FY;
end
toc
% 
% Kxx=abs(Fx(1)-Fx(2))/(2*dX);
% Kxy=abs(Fx(5)-Fx(6))/(2*dY);
% Kyx=abs(Fy(1)-Fy(2))/(2*dX);
% Kyy=abs(Fy(5)-Fy(6))/(2*dY);
% 
% Bxx=abs(Fx(3)-Fx(4))/(2*dXdot);
% Bxy=abs(Fx(7)-Fx(8))/(2*dYdot);
% Byx=abs(Fy(3)-Fy(4))/(2*dXdot);
% Byy=abs(Fy(7)-Fy(8))/(2*dYdot);
% 
Kxx=-(FX0-Fx(1))/dX;
Kxy=-(FX0-Fx(1))/dY;
Kyx=-(FY0-Fy(3))/dX;
Kyy=-(FY0-Fy(3))/dY;

Bxx=-(FX0-Fx(2))/dXdot;
Bxy=-(FX0-Fx(2))/dYdot;
Byx=-(FY0-Fy(4))/dXdot;
Byy=-(FY0-Fy(4))/dYdot;

K=[Kxx Kxy; Kyx Kyy];
B=[Bxx Bxy; Byx Byy];
end

