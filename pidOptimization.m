%% PID design
A=[0 1 0 0; Kxx/mm Bxx/mm Kxy/mm Bxy/mm; 0 0 0 1; Kyx/mm Byx/mm Kyy/mm Byy/mm];

B=[0 0; Ksv/mm 0; 0 0; 0 Ksv/mm];

C=[1*nu 0 0 0; 0 0 0 0; 0 0 1*nu 0; 0 0 0 0];

D=0;

b2scaled=ss(A,B,C,D);

lambdaE=[100000 100000];
lambdaU=[1000 1000];
lambdaDU=[10000 10000];
% K=[20 100 10 200 100 10];
K=[100 50 0.5 100 50 0];
cost=@(K)costFun(K, Ts, b2scaled, F2, g, xeq, yeq, nu, tau, taus, n_pow, Kps, Ksv, lambdaE, lambdaU,lambdaDU);
K0=K;
options = optimoptions(@fmincon,'Algorithm','sqp','MaxIterations',1500,'Display','iter');
Aineq=eye(length(K))*-1; 
Bineq=zeros(length(K),1);

%[K,fval,exitflag]=fmincon(cost,K0,Aineq,Bineq,[],[],[],[],[],options);

%K = [5.3760   14.9909  0  5.8145   15.0898 0]; % integral lambda squared
%error  (lambda=1);
%K = [5.94996683349224, 15.0000015048944,0.530585340464654,5.00003991629341,15.3616360938196,1.49111714773729];
% lambdaE=[100000 100000];
% lambdaU=[100 100];
% lambdaDU=[10000 10000];
%K=[5.63589353178071,14.9999983132525,5.00000013751738,4.99999798634495,15.2194685955133,7.30615490321276];
% ITAE
%%
KPX=DesignVars(5,1).Value;
KIX=DesignVars(3,1).Value;
KDX=DesignVars(1,1).Value;
KPY=DesignVars(6,1).Value;
KIY=DesignVars(4,1).Value;
KDY=DesignVars(2,1).Value;

simOut=sim('PIDdesign','SrcWorkspace','Current');
