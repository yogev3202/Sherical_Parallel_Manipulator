
% computes the Rotation Operator according to 321 sequence 
% input -> [psi,theta,phi]
% outpur -> DCM 
% x1=phi, x2=teta, x3=psi
%rotation of vector v star to v in world frame 


function q = EA2q(EA)

psi = EA(1); % psi
theta = EA(2); % theta
phi = EA(3); % phi

c_psi = cos(psi); s_psi = sin(psi);
c_theta = cos(theta); s_theta = sin(theta);
c_phi = cos(phi); s_phi = sin(phi);




d3m = [c_psi, -s_psi ,  0 ; s_psi , c_psi , 0 ; 0 , 0 , 1];
   
d2m = [c_theta  , 0 , s_theta ; 0 , 1 , 0 ; -s_theta , 0 ,  c_theta];   
   
d1m = [1 , 0 , 0 ; 0 , c_phi , -s_phi ; 0 , s_phi , c_phi];

DCM = d3m*d2m*d1m;


D11 = DCM(1,1);D12 = DCM(1,2);D13 = DCM(1,3);
D21 = DCM(2,1);D22 = DCM(2,2);D23 = DCM(2,3);
D31 = DCM(3,1);D32 = DCM(3,2);D33 = DCM(3,3);



qw_0 = sqrt(1+D11+D22+D33);
qx_0 = (D23-D32)/sqrt(1+D11+D22+D33);
qy_0 = (D31-D13)/sqrt(1+D11+D22+D33);
qz_0 = (D12-D21)/sqrt(1+D11+D22+D33);
q0 = 0.5*[qx_0;qy_0;qz_0;qw_0];


qw_1 = (D23-D32)/sqrt(1+D11-D22-D33);
qx_1 = sqrt(1+D11+D22+D33);
qy_1 = (D12+D12)/sqrt(1+D11-D22-D33);
qz_1 = (D31+D13)/sqrt(1+D11-D22-D33);
q1 = 0.5*[qx_1;qy_1;qz_1;qw_1];


qw_2 = (D31-D13)/sqrt(1-D11+D22-D33);
qx_2 = (D12+D21)/sqrt(1-D11+D22-D33);
qy_2 = sqrt(1+D11-D22-D33);
qz_2 = (D12+D32)/sqrt(1-D11+D22-D33);
q2 = 0.5*[qx_2;qy_2;qz_2;qw_2];



qw_3 = (D12-D21)/sqrt(1-D11-D22+D33);
qx_3 = (D31+D13)/sqrt(1-D11-D22+D33);
qy_3 = (D23+D32)/sqrt(1-D11-D22+D33);
qz_3 = sqrt(1-D11-D22+D33);
q3 = 0.5*[qx_3;qy_3;qz_3;qw_3];



if (D22 > -D33 && D11 > -D22 && D11 > -D33)
    q=q0;
elseif (D22 < -D33 && D11 > D22 && D11 > D33)
    q = q1;
elseif (D22 > D33 && D11 < D22 && D11 < -D33)
    q = q2;
elseif (D22 < D33 && D11 < -D22 && D11 < D33)
    q = q3;
end


end

