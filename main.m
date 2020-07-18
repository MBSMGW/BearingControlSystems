clc; clear;
clear classes;
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
%% initial position and velocity - X,Y,Xdot,Ydot

X0 = 0; Y0 = 0;
Xdot = 0; Ydot = 0;

initial_parameters=[X0, Xdot, Y0, Ydot];

%% linearised model
% Kxx1=K1(1,1); Kyy1=K1(2,2); Kxy1=K1(1,2); Kyx1=K1(2,1);
% Kxx2=K2(1,1); Kyy2=K2(2,2); Kxy2=K2(1,2); Kyx2=K2(2,1);
% 
% Bxx1=B1(1,1); Byy1=B1(2,2); Bxy1=B1(1,2); Byx1=B1(2,1);
% Bxx2=B2(1,1); Byy2=B2(2,2); Bxy2=B2(1,2); Byx2=B2(2,1);
%% dynamics calculation
% timing
Tcalc=0.15; % second of calculated time

% initial conditions
% Fdx=linspace(-100,100,9);
% Fdy=linspace(-100,100,9);
Fdx=0;
Fdy=0;
% N=length(Fdx)*length(Fdy);
% count=0;
% FdC=zeros(2,length(Fdx));
% for i=1:length(Fdx)
%     for j=1:length(Fdy)
%         count=count+1;
%         FdC(1,count)=Fdx(i);
%         FdC(2,count)=Fdy(j);
%     end
% end
%     [t, x] = ode45 (@(t, x) EOM(t, x,Fdx,Fdy,F1, mesh_parameters, geometry_parameters, operational_parameters), [0 Tcalc], [X0 Xdot Y0 Ydot]);
%     X=x(:, 1);
%     Xdot=x(:, 2);
%     Y=x(:, 3);
%     Ydot=x(:, 4);
% %%
% iteration=41;
% 
% tic
% parfor iteration = 1:N
%     message = ['Completing iteration #',num2str(iteration)];
%     disp(message);
%     
%     Xfilename=['C:\Users\Alexander\OneDrive\Documents\XC1', num2str(iteration),'.mat'];
%     
%     X=0; Xdot=0; Y=0; Ydot=0;
%     
%     FdX=FdC(1,iteration); FdY=FdC(2,iteration);
%     
%     tic  
%     [t, x] = ode45 (@(t, x) EOM(t, x,FdX,FdY,F1, mesh_parameters, geometry_parameters, operational_parameters), [0 Tcalc], [X Xdot Y Ydot]);
%     X=x(:, 1);
%     Xdot=x(:, 2);
%     Y=x(:, 3);
%     Ydot=x(:, 4);
%       
%     %% force calculation
%     
%     Fx=zeros(1,length(t));
%     Fy=zeros(1,length(t));
%     xc = zeros(4, length(t));
%     fc= zeros(2, length(t));
%     
%     for i=1:length(t)
%         [FX,FY]= CalculateLoadCapacity(mesh_parameters,geometry_parameters, operational_parameters,[X(i) Xdot(i) Y(i) Ydot(i)]);
%         xc(1,i)=X(i); xc(2,i)=Xdot(i); xc(3,i)=Y(i); xc(4,i)=Ydot(i);
%         fc(1,i)=FX; fc(2,i)=FY;
%     end
%     
%     XC1{iteration}=xc;
%     FC1{iteration}=fc;
% 
%     eltime=toc;
%     message = ['Complete in ',num2str(eltime),' seconds'];
%     disp(message);
% end
% toc
%%
% for i=1:length(T)-1
%     [FX,FY]= CalculateLoadCapacity(mesh_parameters,geometry_parameters, operational_parameters,[X Xdot Y Ydot])
%     
%     [t, x] = ode45 (@(t, x) EOM(t, x,FX,FY,FdX,FdY,F2), [Tstart Tstop], [X Xdot Y Ydot]);
%     
%     X=x(end, 1);
%     Xdot=x(end, 2);
%     Y=x(end, 3);
%     Ydot=x(end, 4);
% 
%     Tstart=Tstart+dt;
%     Tstop=Tstop+dt
% end

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
n_pow=1e-8; %n_pow=1e-8;

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




