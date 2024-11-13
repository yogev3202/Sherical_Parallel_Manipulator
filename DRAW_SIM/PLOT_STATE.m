function  PLOT_STATE(DATA,SOLVER,SIM_TYPE)


Time = DATA.tout;
R2D = 180/pi;
mm2m = 1000;

%% ---------------------------------------
%% SPM Attitude
%% ---------------------------------------

EA = squeeze(DATA.SPM_STATE.signals(1).values);

EA_Err = [DATA.SPM_Attitude_Error.signals(1).values, ...
          DATA.SPM_Attitude_Error.signals(3).values, ...
          DATA.SPM_Attitude_Error.signals(3).values];

F1 = figure('Name','SPM Attitude');
hold on
plot(Time,EA(1,:),'r');
plot(Time,EA(2,:),'g');
plot(Time,EA(3,:),'b');
xlabel('[sec]');
ylabel('[deg]');
legend({'$\varphi_1$','$\varphi_2$','$\varphi_3$'}, ... 
    'Interpreter','latex');
grid minor
Fig_Name1 = 'SPM_Attitude.jpg';


%% ---------------------------------------
%% SPM Motor Angles
%% ---------------------------------------

qm = DATA.SPM_MOTORS.signals(1).values;
F2 = figure('Name','SPM Motors Angles');
hold on
plot(Time,qm(:,1),'r');
plot(Time,qm(:,2),'g');
plot(Time,qm(:,3),'b');
xlabel('[sec]');
ylabel('[deg]');
legend({'$\theta_1$','$\theta_2$','$\theta_3$'}, ... 
    'Interpreter','latex');
grid minor
Fig_Name2 = 'SPM_Motors_Angles.jpg';


%% ---------------------------------------
%% SPM Angular Velocity
%% ---------------------------------------
dEA = squeeze(DATA.SPM_STATE.signals(2).values);
F3 = figure('Name','SPM Angular Velocity');
hold on
plot(Time,dEA(1,:),'r');
plot(Time,dEA(2,:),'g');
plot(Time,dEA(3,:),'b');
xlabel('[sec]');
ylabel('[deg/sec]');
legend({'$\dot{\varphi}_1$','$\dot{\varphi}_2$','$\dot{\varphi}_3$'}, ... 
    'Interpreter','latex');
grid minor
Fig_Name3 = 'SPM_Angular_Vel.jpg';

%% ---------------------------------------
%% SPM Motor Angular Veloceties
%% ---------------------------------------

dqm = DATA.SPM_MOTORS.signals(2).values;
F4 = figure('Name','SPM Motor Angular Veloceties');
hold on
plot(Time,dqm(:,1),'r');
plot(Time,dqm(:,2),'g');
plot(Time,dqm(:,3),'b');
xlabel('[sec]');
ylabel('[deg/sec]');
legend({'$\dot{\theta}_1$','$\dot{\theta}_2$','$\dot{\theta}_3$'}, ... 
    'Interpreter','latex');
grid minor
Fig_Name4 = 'SPM_Motors_Vel.jpg';

%% ---------------------------------------
%% SPM Control Input
%% ---------------------------------------

% SQP_U = SQP.controls_MPC;
% IPOPT_U = IPOPT.controls_MPC;
% QLPV_U = QLPV.controls_MPC;
Control_Input = squeeze(DATA.SPM_MOTORS.signals(3).values);
F5 = figure('Name','SPM Control Input');
hold on
stairs(Time,Control_Input(:,1),'r');
stairs(Time,Control_Input(:,2),'g');
stairs(Time,Control_Input(:,3),'b');
xlabel('[sec]');
ylabel('[Nmm]');
legend({'$\tau_1$','$\tau_2$','$\tau_3$'}, ... 
    'Interpreter','latex');
grid minor
Fig_Name5 = 'SPM_Motor_Torques.jpg';


%% ---------------------------------------
%% SPM Singularity Types
%% ---------------------------------------

D1 = DATA.SPM_Singular.signals(1).values;
D2 = DATA.SPM_Singular.signals(2).values;
D3 = DATA.SPM_Singular.signals(3).values;
CI = DATA.SPM_Singular.signals(4).values;

F6 = figure('Name','SPM Singularity');
hold on
plot(Time,D1,'r');
plot(Time,D2,'g');
plot(Time,D3,'b');
plot(Time,CI,'m');
xlabel('[sec]');
legend({'$\Delta_1$','$\Delta_2$','$\Delta_3$','$CI$'}, ... 
    'Interpreter','latex');
grid minor
Fig_Name6 = 'SPM_Singular.jpg';

%% ---------------------------------------
%% Solver Data : CPT
%% ---------------------------------------

CPT = DATA.MPC_Solver_Info.signals(1).values;
F7 = figure('Name','Solver CPT');
plot(Time,CPT,'b');
xlabel('[sec]');
ylabel('[msec]');
grid minor
Fig_Name7 = 'Solver_CPT.jpg';

%% ---------------------------------------
%% Solver Data : KKT
%% ---------------------------------------
KKT = DATA.MPC_Solver_Info.signals(2).values;
F8 = figure('Name','Solver KKT');
plot(Time,KKT,'b');
xlabel('[sec]');
ylabel('[KKT]');
grid minor
Fig_Name8 = 'Solver_KKT.jpg';

%% ---------------------------------------
%% Solver Data : Cost
%% ---------------------------------------

Cost = DATA.MPC_Solver_Info.signals(3).values;
F9 = figure('Name','Solver Cost');
plot(Time,Cost,'b');
xlabel('[sec]');
ylabel('[Cost]');
grid minor
Fig_Name9 = 'Solver_Cost.jpg';


%% ---------------------------------------
%% SPM Attitude Error
%% ---------------------------------------
psi_Err = DATA.SPM_Attitude_Error.signals(1).values; 
theta_Err = DATA.SPM_Attitude_Error.signals(2).values;
phi_Err = DATA.SPM_Attitude_Error.signals(3).values;
F10 = figure('Name','SPM Attitude Error');
hold on
plot(Time,psi_Err,'r');
plot(Time,theta_Err,'g');
plot(Time,phi_Err,'b');
xlabel('[sec]');
ylabel('[deg]');
legend({'$\tilde{\varphi}_1$','$\tilde{\varphi}_2$','$\tilde{\varphi}_3$'}, ... 
    'Interpreter','latex');
grid minor

% create a new pair of axes inside current figure
axes('position',[.65 .500 .25 .25])
box on % put box around new pair of axes
indexOfInterest = (Time < 1.2) & (Time > 0.8); % range of t near perturbation
hold on
plot(Time(indexOfInterest),psi_Err(indexOfInterest),'r') % plot on new axes
plot(Time(indexOfInterest),theta_Err(indexOfInterest),'g') % plot on new axes
plot(Time(indexOfInterest),phi_Err(indexOfInterest),'b') % plot on new axes
axis tight



Fig_Name10 = 'SPM_Attitude_Err.jpg';

%% ---------------------------------------
%% Save Figures to JPG
%% ---------------------------------------
cd Figures
mkdir(SIM_TYPE)
cd(SIM_TYPE)
mkdir(SOLVER)
cd(SOLVER)
saveas(F1,Fig_Name1);
saveas(F2,Fig_Name2);
saveas(F3,Fig_Name3);
saveas(F4,Fig_Name4);
saveas(F5,Fig_Name5);
saveas(F6,Fig_Name6);
saveas(F7,Fig_Name7);
saveas(F8,Fig_Name8);
saveas(F9,Fig_Name9);
saveas(F10,Fig_Name10);
cd ..
cd ..
cd ..

end