clc;
clear all
close all

%% Variables

import casadi.*
quaternion = SX.sym('quaternion',4,1);



%% GEOMETRIC PARAMETERS
R2D = 180/pi;
D2R = pi/180;
a1 = 65*D2R; a2 = 60*D2R; 
b1 = 0*D2R; b2 = 110*D2R; 
e1 = 0*D2R; e2 = 240*D2R; e3 = 120*D2R; 

ev = quaternion(1:3);
qs = quaternion(4);

Sa1 = sin(a1); Sa2 = sin(a2);
Sb1 = sin(b1); Sb2 = sin(b2);
Se1 = sin(e1); Se2 = sin(e2); Se3 = sin(e3);

Ca1 = cos(a1); Ca2 = cos(a2);
Cb1 = cos(b1); Cb2 = cos(b2);
Ce1 = cos(e1); Ce2 = cos(e2); Ce3 = cos(e3);



%% -----------------------------------------------------------
% Step 0 : Compute Inverse Kinematics
%% -----------------------------------------------------------
x = [0;30;0]*D2R;
THETAv = SPMIK_Fcn(x,'---');
q1 = THETAv(1); q2 = THETAv(2); q3 = THETAv(3);
[v10,w1,V1m,W1m,M1m,N1m] = UnitVectors(e1,q1);
[v20,w2,V2m,W2m,M2m,N2m] = UnitVectors(e2,q2);
[v30,w3,V3m,W3m,M3m,N3m] = UnitVectors(e3,q3);


%% -----------------------------------------------------------
% Step 3 : Compute quaternion
%% -----------------------------------------------------------


z = SX.sym('z',1);
x = SX.sym('x',1);
g0 = sin(x+z);
g1 = cos(x-z);
g2 = cos(x+z);
g = Function('g',{z,x},{g0,g1,g2});
G = rootfinder('G','newton',g);
disp(G)


g0 = quaternion'*M1m*quaternion - 1;
g1 = quaternion'*M2m*quaternion - 1;
g2 = quaternion'*M3m*quaternion - 1;
g3 = quaternion'*quaternion - 1;
Equations = [g0;g1;g2;g3];

Fun = Function('Fun',{quaternion},{g0,g1,g2,g3});
G = rootfinder('G','newton',Fun);

result = solve(Equations,quaternion);
% result = solve(Equations,quaternion,...
%     'ReturnConditions',true,'IgnoreAnalyticConstraints',true);
quat1_res = vpa(result.quat1,5);
quat2_res = vpa(result.quat2,5);
quat3_res = vpa(result.quat3,5);
quat4_res = vpa(result.quat4,5);

quat1_res = double(quat1_res);
quat2_res = double(quat2_res);
quat3_res = double(quat3_res);
quat4_res = double(quat4_res);

Results = [quat1_res,quat2_res,quat3_res,quat4_res];



%% ---------------------------------------------------
% Check Results
%% ---------------------------------------------------

for k = 1:1:length(Results(:,1))
    
    quat_k = Results(k,:);
    quaternion_res(k,:) = quat_k;
    EA(k,:) = q2EA(quat_k) * R2D;
    MOTORS(k,:) = SPMIK_Fcn(q2EA(quat_k),'---')* R2D;

end


