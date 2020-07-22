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
T_calc=0.15; % seconds
% [X0, Y0] = EquilibriumPosition(F2,mesh_parameters,geometry_parameters, operational_parameters);
%% linearised model
load('b2full.mat');
Q=eye(size(linsys1.A)); Q(1,1)=2500; Q(2,2)= 500; Q(3,3)=1000; Q(4,4)= 50;
R=[0.05 0; 0 0.1]; 
K=lqr(linsys1.A,linsys1.B,Q,R);
linsyslqr=ss(linsys1.A-linsys1.B*K,linsys1.B,linsys1.C,linsys1.D);

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

