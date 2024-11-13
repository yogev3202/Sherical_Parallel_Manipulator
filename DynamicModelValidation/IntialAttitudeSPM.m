CTR_PATH = 'DynamicValidation_DynamicControl/SPM MODEL';
CTR_MASK = Simulink.Mask.get(CTR_PATH);

EA0 = CTR_MASK.Parameters(1, 1).Value;
dEA0 = CTR_MASK.Parameters(1, 2).Value;
EA0 = str2num(EA0) * (pi/180);
dEA0 = str2num(dEA0) * (pi/180);
THETAv = SPMIK_Fcn(EA0,'---');
thetaZ0 = -1.140243740887195;
q1 = (-thetaZ0 + THETAv(1)) * (180/pi);
q2 = (-thetaZ0 + THETAv(2)) * (180/pi);
q3 = (-thetaZ0 + THETAv(3)) * (180/pi);

dq = MotorVel([EA0;dEA0]);
dq1 = -dq(1) * (180/pi);
dq2 = -dq(2) * (180/pi);
dq3 = -dq(3) * (180/pi);


