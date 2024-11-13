% CTR_PATH = 'Dynamic_Simulation_Final/CTR_PARAMETERS';
% CTR_MASK = Simulink.Mask.get(CTR_PATH);

% x0_SPM = CTR_MASK.Parameters(1, 10).Value;
% x0_SPM = str2num(x0_SPM);
EA0 = x0_SPM(1:3);

THETAv = SPMIK_Fcn(EA0 * (pi/180),'---');
thetaZ0 = -1.140243740887195;

q0_1 = (THETAv(1) - thetaZ0) * (180/pi);
q0_2 = (THETAv(2) - thetaZ0) * (180/pi);
q0_3 = (THETAv(3) - thetaZ0) * (180/pi);