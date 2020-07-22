function [XC1, FC1] = dynamics(F1, mesh_parameters, geometry_parameters, operational_parameters,T_calc)
parpool('local',6);

%initial conditions
X0 = 0; Y0 = 0;
Xdot = 0; Ydot = 0;

Fdx=linspace(-100,100,9);
Fdy=linspace(-100,100,9);

N=length(Fdx)*length(Fdy);
count=0;
FdC=zeros(2,length(Fdx));

for i=1:length(Fdx)
    for j=1:length(Fdy)
        count=count+1;
        FdC(1,count)=Fdx(i);
        FdC(2,count)=Fdy(j);
    end
end

%%
tic
parfor iteration = 1:N
    message = ['Completing iteration #',num2str(iteration)];
    disp(message);
    
    Xfilename=['C:\Users\Alexander\OneDrive\Documents\XC1', num2str(iteration),'.mat'];
    
    X=0; Xdot=0; Y=0; Ydot=0;
    
    FdX=FdC(1,iteration); FdY=FdC(2,iteration);
    
    tic  
    [t, x] = ode45 (@(t, x) EOM(t, x,FdX,FdY,F1, mesh_parameters, geometry_parameters, operational_parameters), [0 Tcalc], [X Xdot Y Ydot]);
    X=x(:, 1);
    Xdot=x(:, 2);
    Y=x(:, 3);
    Ydot=x(:, 4);
      
    %% force calculation
    
    Fx=zeros(1,length(t));
    Fy=zeros(1,length(t));
    xc = zeros(4, length(t));
    fc= zeros(2, length(t));
    
    for i=1:length(t)
        [FX,FY]= CalculateLoadCapacity(mesh_parameters,geometry_parameters, operational_parameters,[X(i) Xdot(i) Y(i) Ydot(i)]);
        xc(1,i)=X(i); xc(2,i)=Xdot(i); xc(3,i)=Y(i); xc(4,i)=Ydot(i);
        fc(1,i)=FX; fc(2,i)=FY;
    end
    
    XC1{iteration}=xc;
    FC1{iteration}=fc;

    eltime=toc;
    message = ['Complete in ',num2str(eltime),' seconds'];
    disp(message);
end

toc
delete(gcp);
end

