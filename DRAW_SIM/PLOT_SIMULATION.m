function  PLOT_SIMULATION(SQP,IPOPT,QLPV)




Time = SQP.tout;
R2D = 180/pi;
mm2m = 1000;

%% ---------------------------------------
%% SPM Attitude
%% ---------------------------------------

SQP_EA = squeeze(SQP.SPM_STATE.signals(1).values);
IPOPT_EA = squeeze(IPOPT.SPM_STATE.signals(1).values);
QLPV_EA = squeeze(QLPV.SPM_STATE.signals(1).values);

SQP_EA_Err = [SQP.SPM_Attitude_Error.signals(1).values, ...
              SQP.SPM_Attitude_Error.signals(3).values, ...
              SQP.SPM_Attitude_Error.signals(3).values];

IPOPT_EA_Err = [IPOPT.SPM_Attitude_Error.signals(1).values, ...
                IPOPT.SPM_Attitude_Error.signals(3).values, ...
                IPOPT.SPM_Attitude_Error.signals(3).values];

QLPV_EA_Err = [QLPV.SPM_Attitude_Error.signals(1).values, ...
               QLPV.SPM_Attitude_Error.signals(3).values, ...
               QLPV.SPM_Attitude_Error.signals(3).values];


figure('Name','SPM Attitude')

subplot(3,2,1)
hold on
plot(Time,SQP_EA(1,:),'r');
plot(Time,SQP_EA(2,:),'g');
plot(Time,SQP_EA(3,:),'b');
xlabel('[sec]');
ylabel('[deg]');
legend('\psi','\theta','\phi');
% title('SQP','Position',[-0.8 5 0])
grid minor

subplot(3,2,2)
hold on
plot(Time,SQP_EA_Err(:,1),'r');
plot(Time,SQP_EA_Err(:,2),'g');
plot(Time,SQP_EA_Err(:,3),'b');
xlabel('[sec]');
ylabel('[deg]');
legend({'$\tilde{\psi}$','$\tilde{\theta}$','$\tilde{\phi}$'}, ... 
    'Interpreter','latex');
title('SQP ERROR')
grid minor

subplot(3,2,3)
hold on
plot(Time,IPOPT_EA(1,:),'r');
plot(Time,IPOPT_EA(2,:),'g');
plot(Time,IPOPT_EA(3,:),'b');
xlabel('[sec]');
ylabel('[deg]');
legend('\psi','\theta','\phi');
title('IPOPT')
grid minor
subplot(3,2,4)
hold on
plot(Time,IPOPT_EA_Err(:,1),'r');
plot(Time,IPOPT_EA_Err(:,2),'g');
plot(Time,IPOPT_EA_Err(:,3),'b');
xlabel('[sec]');
ylabel('[deg]');
legend({'$\tilde{\psi}$','$\tilde{\theta}$','$\tilde{\phi}$'}, ... 
    'Interpreter','latex');
title('IPOPT ERROR')
grid minor

subplot(3,2,5)
hold on
plot(Time,QLPV_EA(1,:),'r');
plot(Time,QLPV_EA(2,:),'g');
plot(Time,QLPV_EA(3,:),'b');
xlabel('[sec]');
ylabel('[deg]');
legend('\psi','\theta','\phi');
title('QLPV')
grid minor
subplot(3,2,6)
hold on
plot(Time,QLPV_EA_Err(:,1),'r');
plot(Time,QLPV_EA_Err(:,2),'g');
plot(Time,QLPV_EA_Err(:,3),'b');
xlabel('[sec]');
ylabel('[deg]');
legend({'$\tilde{\psi}$','$\tilde{\theta}$','$\tilde{\phi}$'}, ... 
    'Interpreter','latex');
title('QLPV ERROR')
grid minor




%% ---------------------------------------
%% SPM Motor Angles
%% ---------------------------------------

SQP_qm = SQP.SPM_MOTORS.signals(1).values;
IPOPT_qm = IPOPT.SPM_MOTORS.signals(1).values;
QLPV_qm = QLPV.SPM_MOTORS.signals(1).values;


figure('Name','SPM Motors Angles')

subplot(3,1,1)
hold on
plot(Time,SQP_qm(:,1),'r');
plot(Time,SQP_qm(:,2),'g');
plot(Time,SQP_qm(:,3),'b');
xlabel('[sec]');
ylabel('[deg]');
legend('\theta_1','\theta_2','\theta_3');
title('SQP')
grid minor

subplot(3,1,2)
hold on
plot(Time,IPOPT_qm(:,1),'r');
plot(Time,IPOPT_qm(:,2),'g');
plot(Time,IPOPT_qm(:,3),'b');
xlabel('[sec]');
ylabel('[deg]');
legend('\theta_1','\theta_2','\theta_3');
title('IPOPT')
grid minor

subplot(3,1,3)
hold on
plot(Time,QLPV_qm(:,1),'r');
plot(Time,QLPV_qm(:,2),'g');
plot(Time,QLPV_qm(:,3),'b');
xlabel('[sec]');
ylabel('[deg]');
legend('\theta_1','\theta_2','\theta_3');
title('QLPV')
grid minor

%% ---------------------------------------
%% SPM Angular Velocity
%% ---------------------------------------
SQP_dEA = squeeze(SQP.SPM_STATE.signals(2).values);
IPOPT_dEA = squeeze(IPOPT.SPM_STATE.signals(2).values);
QLPV_dEA = squeeze(QLPV.SPM_STATE.signals(2).values);

figure('Name','SPM Angular Velocity')

subplot(3,1,1)
hold on
plot(Time,SQP_dEA(1,:),'r');
plot(Time,SQP_dEA(2,:),'g');
plot(Time,SQP_dEA(3,:),'b');
xlabel('[sec]');
ylabel('[deg/sec]');
legend({'$\dot{\psi}$','$\dot{\theta}$','$\dot{\phi}$'}, ... 
    'Interpreter','latex');

title('SQP')
grid minor

subplot(3,1,2)
hold on
plot(Time,IPOPT_dEA(1,:),'r');
plot(Time,IPOPT_dEA(2,:),'g');
plot(Time,IPOPT_dEA(3,:),'b');
xlabel('[sec]');
ylabel('[deg/sec]');
legend({'$\dot{\psi}$','$\dot{\theta}$','$\dot{\phi}$'}, ... 
    'Interpreter','latex');
title('IPOPT')
grid minor
subplot(3,1,3)
hold on
plot(Time,QLPV_dEA(1,:),'r');
plot(Time,QLPV_dEA(2,:),'g');
plot(Time,QLPV_dEA(3,:),'b');
xlabel('[sec]');
ylabel('[deg/sec]');
legend({'$\dot{\psi}$','$\dot{\theta}$','$\dot{\phi}$'}, ... 
    'Interpreter','latex');
title('QLPV')
grid minor

%% ---------------------------------------
%% SPM Motor Angular Veloceties
%% ---------------------------------------

SQP_dqm = SQP.SPM_MOTORS.signals(2).values;
IPOPT_dqm = IPOPT.SPM_MOTORS.signals(2).values;
QLPV_dqm = QLPV.SPM_MOTORS.signals(2).values;

figure('Name','SPM Motor Angular Veloceties')

subplot(3,1,1)
hold on
plot(Time,SQP_dqm(:,1),'r');
plot(Time,SQP_dqm(:,2),'g');
plot(Time,SQP_dqm(:,3),'b');
xlabel('[sec]');
ylabel('[deg/sec]');
legend({'$\dot{\theta}_1$','$\dot{\theta}_2$','$\dot{\theta}_3$'}, ... 
    'Interpreter','latex');
title('SQP')
grid minor

subplot(3,1,2)
hold on
plot(Time,IPOPT_dqm(:,1),'r');
plot(Time,IPOPT_dqm(:,2),'g');
plot(Time,IPOPT_dqm(:,3),'b');
xlabel('[sec]');
ylabel('[deg/sec]');
legend({'$\dot{\theta}_1$','$\dot{\theta}_2$','$\dot{\theta}_3$'}, ... 
    'Interpreter','latex');
title('IPOPT')
grid minor
subplot(3,1,3)
hold on
plot(Time,QLPV_dqm(:,1),'r');
plot(Time,QLPV_dqm(:,2),'g');
plot(Time,QLPV_dqm(:,3),'b');
xlabel('[sec]');
ylabel('[deg/sec]');
legend({'$\dot{\theta}_1$','$\dot{\theta}_2$','$\dot{\theta}_3$'}, ... 
    'Interpreter','latex');
title('QLPV')
grid minor



%% ---------------------------------------
%% SPM Control Input
%% ---------------------------------------

% SQP_U = SQP.controls_MPC;
% IPOPT_U = IPOPT.controls_MPC;
% QLPV_U = QLPV.controls_MPC;
SQP_U = squeeze(SQP.SPM_MOTORS.signals(3).values);
IPOPT_U = squeeze(IPOPT.SPM_MOTORS.signals(3).values);
QLPV_U = squeeze(QLPV.SPM_MOTORS.signals(3).values);


figure('Name','SPM Control Input')

subplot(3,1,1)
hold on
stairs(Time,SQP_U(:,1),'r');
stairs(Time,SQP_U(:,2),'g');
stairs(Time,SQP_U(:,3),'b');
xlabel('[sec]');
ylabel('[Nmm]');
legend({'$\tau_1$','$\tau_2$','$\tau_3$'}, ... 
    'Interpreter','latex');
title('SQP')
grid minor

subplot(3,1,2)
hold on
stairs(Time,IPOPT_U(:,1),'r');
stairs(Time,IPOPT_U(:,2),'g');
stairs(Time,IPOPT_U(:,3),'b');
xlabel('[sec]');
ylabel('[Nmm]');
legend({'$\tau_1$','$\tau_2$','$\tau_3$'}, ... 
    'Interpreter','latex');
title('IPOPT')
grid minor
subplot(3,1,3)
hold on
stairs(Time,QLPV_U(:,1),'r');
stairs(Time,QLPV_U(:,2),'g');
stairs(Time,QLPV_U(:,3),'b');
xlabel('[sec]');
ylabel('[Nmm]');
legend({'$\tau_1$','$\tau_2$','$\tau_3$'}, ... 
    'Interpreter','latex');
title('QLPV')
grid minor

%% ---------------------------------------
%% SPM Singularity Types
%% ---------------------------------------

D1_SQP = SQP.SPM_Singular.signals(1).values;
D2_SQP = SQP.SPM_Singular.signals(2).values;
D3_SQP = SQP.SPM_Singular.signals(3).values;
CI_SQP = SQP.SPM_Singular.signals(4).values;

D1_IPOPT = IPOPT.SPM_Singular.signals(1).values;
D2_IPOPT = IPOPT.SPM_Singular.signals(2).values;
D3_IPOPT = IPOPT.SPM_Singular.signals(3).values;
CI_IPOPT = IPOPT.SPM_Singular.signals(4).values;

D1_QLPV = QLPV.SPM_Singular.signals(1).values;
D2_QLPV = QLPV.SPM_Singular.signals(2).values;
D3_QLPV = QLPV.SPM_Singular.signals(3).values;
CI_QLPV = QLPV.SPM_Singular.signals(4).values;


figure('Name','SPM Singularity')
subplot(4,1,1)
hold on
stairs(Time,D1_SQP,'r');
stairs(Time,D1_IPOPT,'g');
stairs(Time,D1_QLPV,'b');
xlabel('[sec]');
ylabel('[D1]');
legend({'$SQP$','$IPOPT$','$QLPV$'}, ... 
    'Interpreter','latex');
grid minor

subplot(4,1,2)
hold on
stairs(Time,D2_SQP,'r');
stairs(Time,D2_IPOPT,'g');
stairs(Time,D2_QLPV,'b');
xlabel('[sec]');
ylabel('[D2]');
legend({'$SQP$','$IPOPT$','$QLPV$'}, ... 
    'Interpreter','latex');
grid minor

subplot(4,1,3)
hold on
stairs(Time,D3_SQP,'r');
stairs(Time,D3_IPOPT,'g');
stairs(Time,D3_QLPV,'b');
xlabel('[sec]');
ylabel('[D3]');
legend({'$SQP$','$IPOPT$','$QLPV$'}, ... 
    'Interpreter','latex');
grid minor

subplot(4,1,4)
hold on
stairs(Time,CI_SQP,'r');
stairs(Time,CI_IPOPT,'g');
stairs(Time,CI_QLPV,'b');
xlabel('[sec]');
ylabel('[CI]');
legend({'$SQP$','$IPOPT$','$QLPV$'}, ... 
    'Interpreter','latex');
grid minor

%% ---------------------------------------
%% Solver Data : Computation Time
%% ---------------------------------------
SQP_CPT = SQP.MPC_Solver_Info.signals(1).values;
IPOPT_CPT = IPOPT.MPC_Solver_Info.signals(1).values;
QLPV_CPT = QLPV.MPC_Solver_Info.signals(1).values;

figure('Name','Solver CPT')

subplot(3,1,1)
plot(Time,SQP_CPT,'b');
xlabel('[sec]');
ylabel('[msec]');
title('SQP')
grid minor

subplot(3,1,2)
plot(Time,IPOPT_CPT,'b');
xlabel('[sec]');
ylabel('[msec]');
title('IPOPT')
grid minor
subplot(3,1,3)
plot(Time,QLPV_CPT,'b');
xlabel('[sec]');
ylabel('[msec]');
title('QLPV')
grid minor


end