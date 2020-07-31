clc; clear;
clear all;

g=9.81;

%% mesh parameters
m = 21;         %along x and i
n = 21;         %along z and j

mesh_parameters=[m n];
%% geometry - diameter, length, radial clearence
D = 0.04;   %shaft diameter;
R = D/2;
L = 0.04;      %bearing length
h0 = 50e-6;     %radial clearence

geometry_parameters=[L R h0];

%% operational parameters - RPM, viscosity, load
n_rpm = 1400;    %rotation speed
omega = n_rpm*2*pi/60;
mu = 1e-3;     %dynamic viscosity
PresCond=0;     %ambient pressure condition

operational_parameters=[n_rpm, mu, PresCond];
%% force parameters
mass=3.793;         %rotor full mass, kg 3.793

Jp=0.00075; %polar moment of inertia, kg m^2  3693 0.00075
Jd=0.047331604; % diametral moment of inertia, kg m^2 0.047331604;

%% geometry of two bearings system
a = 87.548e-3; % distance from centre of mass to bearing#1 87.548e-3;
b = 132.452e-3; % distance from centre of mass to bearing#2 THIS ONE IS ACTIVE 132.452e-3;
c = a + 130e-3;  % distance from centre of mass to the coupling (b#1 direction);
d = 22.452e-3; %distance from centre of mass to the point where fd is applied to 22.452e-3

s = 30e-3; %distance from bearing centre to displacement sensors

F2=a*mass*g/(a+b); % static force on bearing#2
F1=mass*g-F2;      % static force on bearing#1
delta=25e-6;       % unbalance 4e-6
force_parameters=[mass];

%% dynamics calculation parameters
Tcalc=0.15; % seconds
% equilibrium position explicit determination
FdX=0;FdY=0;
[t, x] = ode45 (@(t, x) EOM(t, x,FdX,FdY,F2, mesh_parameters, geometry_parameters, operational_parameters), [0 Tcalc], [0 0 0 0]);
xeq=x(end,1);
yeq=x(end,3);

%% dynamics training set calculation
% [XC1, FC1] = dynamics(F1, mesh_parameters, geometry_parameters, operational_parameters,T_calc);

%% parameters of CS elements
% proximity transducer properties
Vmax=10;  % max and min
Vmin=0;  % output voltage
Hmax=6e-3; % max and min
Hmin=0e-3; % measured distance in the linear range
Kps=(Vmax-Vmin)/(Hmax-Hmin); %proximity sensor voltage2gap coefficient
ps=1e-6; % accuracy
taus=0.00125; %time constant

% band limited white noise
n_pow=1e-10; %n_pow=1e-8;

% ADC resolution and sampling freq
fs=1000; % sampling freq in Hz
Ts=1/fs; % sampling time in sec
adc_acc=0.0063; % datasheet absolute accuracy in V

% Servovalve characteristics
Vmax_sv=10; % max and min
Vmin_sv=0; % input voltage
Ksv = 30; % gain coefficient
tau = 0.12; % time constant
svs = 0.1*Vmax_sv; %voltage sensitivity
%% linearised model
% load('b2full.mat');
% Q=eye(size(linsys1.A)); Q(1,1)=2500; Q(2,2)= 500; Q(3,3)=1000; Q(4,4)= 50;
% R=[0.05 0; 0 0.1]; 
% K=lqr(linsys1.A,linsys1.B,Q,R);
% linsyslqr=ss(linsys1.A-linsys1.B*K,linsys1.B,linsys1.C,linsys1.D);
[FX0,FY0]= CalculateLoadCapacity(mesh_parameters,geometry_parameters, operational_parameters,[xeq 0 yeq 0]);
% [K, B] = dyn_coeff(mesh_parameters,geometry_parameters, operational_parameters,omega, xeq, yeq, FX0, FY0)
% 
mm=F2/g;
% Kxx=K(1,1); Kxy=K(1,2); Kyx=K(2,1); Kyy=K(2,2); 
% Bxx=B(1,1); Bxy=B(1,2); Byx=B(2,1); Byy=B(2,2); 
delta=1e-6;
deltav=1e-5;

[FXX1,FYY1]= CalculateLoadCapacity(mesh_parameters,geometry_parameters, operational_parameters,[xeq+delta 0 yeq 0]);
[FXX2,FYY2]= CalculateLoadCapacity(mesh_parameters,geometry_parameters, operational_parameters,[xeq-delta 0 yeq 0]);

Kxx=(FXX1-FXX2)/(2*delta);
Kyx=(FYY1-FYY2)/(2*delta);

[FXX3,FYY3]= CalculateLoadCapacity(mesh_parameters,geometry_parameters, operational_parameters,[xeq 0 yeq+delta 0]);
[FXX4,FYY4]= CalculateLoadCapacity(mesh_parameters,geometry_parameters, operational_parameters,[xeq 0 yeq-delta 0]);

Kxy=(FXX3-FXX4)/(2*delta);
Kyy=(FYY3-FYY4)/(2*delta);

[FXX1,FYY1]= CalculateLoadCapacity(mesh_parameters,geometry_parameters, operational_parameters,[xeq 0+deltav yeq 0]);
[FXX2,FYY2]= CalculateLoadCapacity(mesh_parameters,geometry_parameters, operational_parameters,[xeq 0-deltav yeq 0]);

Bxx=(FXX1-FXX2)/(2*deltav);
Byx=(FYY1-FYY2)/(2*deltav);

[FXX3,FYY3]= CalculateLoadCapacity(mesh_parameters,geometry_parameters, operational_parameters,[xeq 0 yeq 0+deltav]);
[FXX4,FYY4]= CalculateLoadCapacity(mesh_parameters,geometry_parameters, operational_parameters,[xeq 0 yeq 0-deltav]);

Bxy=(FXX3-FXX4)/(2*deltav);
Byy=(FYY3-FYY4)/(2*deltav);

A=[0 1 0 0; Kxx/mm Bxx/mm Kxy/mm Bxy/mm; 0 0 0 1; Kyx/mm Byx/mm Kyy/mm Byy/mm];

B=[0 0; Ksv/mm 0; 0 0; 0 Ksv/mm];

C=[1 0 0 0; 0 0 0 0; 0 0 1 0; 0 0 0 0];

D=0;

b2lin=ss(A,B,C,D);
%b2lind=c2d(b2lin,Ts);

Q=[50 0 0 0; 0 5 0 0; 0 0 15 0; 0 0 0 5];
R=[1 0; 0 1];
Klqr=lqr(A,B,Q,R);
b2lin_lqr=ss(A-B*Klqr,B,C,D);

% ic=[-25e-6 0 -25e-6 0];
% [y1,t,x] = initial(b2lin,ic,3);
% [y2,t,x] = initial(b2lin_lqr,ic,3);

% Kalman Filter design 
Vd = 1000*eye(size(A)); % disturbance covariance
Vn = 1*eye(size(A));      % noise covariance

[Kkf,P,E] = lqe(A,Vd,C,Vd,Vn); 

syskf = ss(A-Kkf*C,[B Kkf],eye(size(A)),0); 
syskfd = c2d(syskf,Ts);