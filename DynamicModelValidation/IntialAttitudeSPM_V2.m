CTR_PATH = 'DynamicValidation/Dynamic_Model_PID/SPM MODEL1';
CTR_MASK = Simulink.Mask.get(CTR_PATH);

EA0 = CTR_MASK.Parameters(1, 1).Value;
EA0 = str2num(EA0) * (pi/180);
THETAv = SPMIK_Fcn(EA0,'---');
thetaZ0 = -1.140243740887195;
q0_1 = (-thetaZ0 + THETAv(1)) * (180/pi);
q0_2 = (-thetaZ0 + THETAv(2)) * (180/pi);
q0_3 = (-thetaZ0 + THETAv(3)) * (180/pi);