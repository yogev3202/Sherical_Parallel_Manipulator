clc;
clear all
close all

%% Variables

syms quat [1,4] real
syms a1 a2 b1 b2 e1 e2 e3 ei real
syms q1 q2 q3 qi real
syms vi0(e) wi(e,q) Vim(e) Wim(e,q) Mim(e,q) Nim(e,q)




%% GEOMETRIC PARAMETERS
% R2D = 180/pi;
% D2R = pi/180;
% a1 = 65*D2R; sa1s = sin(a1); ca1s = cos(a1);
% a2 = 60*D2R; sa2s = sin(a2); ca2s = cos(a2);
% b1 = 0*D2R; sb1s = sin(b1); cb1s = cos(b1);
% b2 = 110*D2R; sb2s = sin(b2); cb2s = cos(b2);
% e1 = 0*D2R; se1s = sin(e1); ce1s = cos(e1);
% e2 = 240*D2R; se2s = sin(e2); ce2s = cos(e2);
% e3 = 120*D2R; se3s = sin(e3); ce3s = cos(e3);


ev = quat(1:3)';
qs = quat(4);

Sa1 = sin(a1); Sa2 = sin(a2);
Sb1 = sin(b1); Sb2 = sin(b2);
Se1 = sin(e1); Se2 = sin(e2); Se3 = sin(e3); Sei = sin(ei);

Ca1 = cos(a1); Ca2 = cos(a2);
Cb1 = cos(b1); Cb2 = cos(b2);
Ce1 = cos(e1); Ce2 = cos(e2); Ce3 = cos(e3); Cei = cos(ei);


vi0(e) = [Sb2*sin(e);Sb2*cos(e);Cb2];
wi(e,q) = [sin(e)*(Sb1*Ca1+Cb1*Sa1*cos(q)) - cos(e)*Sa1*sin(q);
           cos(e)*(Sb1*Ca1+Cb1*Sa1*cos(q)) + sin(e)*Sa1*sin(q);
           -Cb1*Ca1 + Sb1*Sa1*cos(q)];

Vim(e) = [-crossVM(vi0(e)) , vi0(e); -vi0(e)' , 0];
Wim(e,q) = [crossVM(wi(e,q)) , wi(e,q); -wi(e,q)' , 0];
Mim(e,q) = (1/Ca2)*(Wim(e,q)'*Vim(e));
Nim(e,q) = 0.5*Mim(e,q) + (1 - 0.5*Ca2)*eye(4);

%% -----------------------------------------------------------
% Step 0 : Compute Inverse Kinematics
%% -----------------------------------------------------------
% x = [0;30;0]*D2R;
% THETAv = SPMIK_Fcn(x,'---');
% q1 = THETAv(1); q2 = THETAv(2); q3 = THETAv(3);

%% -----------------------------------------------------------
% Step 1 : Compute viv0 and wiv
%% -----------------------------------------------------------

v1v0 = vpa(vi0(e1),5); v2v0 = vpa(vi0(e2),5); v3v0 = vpa(vi0(e3),5);
w1v = vpa(wi(e1,q1),5); vpa(wi(e2,q2),5); vpa(wi(e3,q3),5);

%% -----------------------------------------------------------
% Step 2 : Compute Vim and Wim
%% -----------------------------------------------------------

V1m = vpa(Vim(e1),5); V2m = vpa(Vim(e2),5); V3m = vpa(Vim(e3),5);
W1m = vpa(Wim(e1,q1),5); W2m = vpa(Wim(e2,q2),5); W3m = vpa(Wim(e3,q3),5);
M1m = vpa(Mim(e1,q1),5); M2m = vpa(Mim(e2,q2),5); M3m = vpa(Mim(e3,q3),5);
N1m = vpa(Nim(e1,q1),5); N2m = vpa(Nim(e2,q2),5); N3m = vpa(Nim(e3,q3),5);


%% -----------------------------------------------------------
% Step 3 : Compute quaternion
%% -----------------------------------------------------------

quaternion = [ev;qs];

Eq1 = quaternion'*M1m*quaternion == 1;
Eq2 = quaternion'*M2m*quaternion == 1;
Eq3 = quaternion'*M3m*quaternion == 1;
Eq4 = quaternion'*quaternion == 1;

% Eq1 = quaternion'*N1m*quaternion == 1;
% Eq2 = quaternion'*N2m*quaternion == 1;
% Eq3 = quaternion'*N3m*quaternion == 1;
% Eq4 = quaternion'*quaternion == 1;

Equations = [Eq1;Eq2;Eq3;Eq4];

TT = vpa(Equations,5);


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


