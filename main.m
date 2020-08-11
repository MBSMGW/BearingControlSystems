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

nu = 1/h0; % scaling factor

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

%[FX0,FY0]= CalculateLoadCapacity(mesh_parameters,geometry_parameters, operational_parameters,[xeq 0 yeq 0]);

mm=F2/g;

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
%% Kalman Filter design
Vd = [2750 0 0 0; 0 2750 0 0; 0 0 3750 0; 0 0 0 3750]; % disturbance covariance
Vn = [275 0 0 0; 0 275 0 0; 0 0 375 0; 0 0 0 375];     % noise covariance
[Kkf] = lqe(A,Vd,C,Vd,Vn);
syskf = ss(A-Kkf*C,[B Kkf],eye(size(A)),0);
syskfd = c2d(syskf,Ts);

Aaug=[0 1 0 0 0 0; Kxx/mm Bxx/mm Kxy/mm Bxy/mm 0 0; 0 0 0 1 0 0; Kyx/mm Byx/mm Kyy/mm Byy/mm 0 0; 1 0 0 0 0 0; 0 0 1 0 0 0];
Baug=[0 0; Ksv/mm 0; 0 0; 0 Ksv/mm; 0 0; 0 0];
Caug=[1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];
Daug=D;

b2linaug=ss(Aaug,Baug,Caug,Daug);
Qaug=[1500 0 0 0 0 0; 0 10 0 0 0 0; 0 0 1500 0 0 0; 0 0 0 10 0 0; 0 0 0 0 1500 0; 0 0 0 0 0 1500];
Raug=[10 0; 0 10];
Klqgi=lqr(Aaug,Baug,Qaug,Raug);

%% PID design
A=[0 1 0 0; Kxx/mm Bxx/mm Kxy/mm Bxy/mm; 0 0 0 1; Kyx/mm Byx/mm Kyy/mm Byy/mm];

B=[0 0; Ksv/mm 0; 0 0; 0 Ksv/mm];

C=[1*nu 0 0 0; 0 0 0 0; 0 0 1*nu 0; 0 0 0 0];

D=0;

b2scaled=ss(A,B,C,D);

KPX=1;
KIX=7;
KDX=0;

KPY=1;
KIY=7;
KDY=0;

K=[KPX KIX KDX; KPY KIY KDY];

cost=@(K)costFun()

% subplot(2,2,1)
% margin(b2scaled(1,1))
% legend('PID X -> DISP X')
% subplot(2,2,2)
% margin(b2scaled(3,1))
% legend('PID X -> DISP Y')
% subplot(2,2,3)
% margin(b2scaled(1,2))
% legend('PID Y -> DISP X')
% subplot(2,2,4)
% margin(b2scaled(3,2))
% legend('PID Y -> DISP Y')