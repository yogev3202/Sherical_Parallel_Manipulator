function [h_expr,hN_expr] = hqlpv_expr_Fcn_V2(states)


x1 = states(1);x2 = states(2);x3 = states(3);
x4 = states(4);x5 = states(5);x6 = states(6);

X1 = [x1;x2;x3];
X2 = [x4;x5;x6];


psi = X1(1);
teta = X1(2);
phi = X1(3);

dpsi = X2(1);
dteta = X2(2);
dphi = X2(3);

%% UNIT CONVERTION
    dg2rd = pi/180;
    rd2dg = 1/dg2rd;
    mm2m = 1e-3;

%% GEOMETRIC PARAMETERS
    a1s = 65*dg2rd; 
    a2s = 60*dg2rd; 
    b1s = 0*dg2rd; 
    b2s = 110*dg2rd; 
    eta1s = 0*dg2rd; 
    eta2s = 240*dg2rd; 
    eta3s = 120*dg2rd; 



    sa1s = sin(a1s); ca1s = cos(a1s);
    sa2s = sin(a2s); ca2s = cos(a2s);
    sb1s = sin(b1s); cb1s = cos(b1s);
    sb2s = sin(b2s); cb2s = cos(b2s);
    se1s = sin(eta1s); ce1s = cos(eta1s);
    se2s = sin(eta2s); ce2s = cos(eta2s);
    se3s = sin(eta3s); ce3s = cos(eta3s);
    
%% UNIT VECTORS CONSTANTS

    u1v = [sb1s*se1s;sb1s*ce1s;-cb1s];
    u2v = [sb1s*se2s;sb1s*ce2s;-cb1s];
    u3v = [sb1s*se3s;sb1s*ce3s;-cb1s];

    v1v_str = [sb2s*se1s;sb2s*ce1s;cb2s];
    v2v_str = [sb2s*se2s;sb2s*ce2s;cb2s];
    v3v_str = [sb2s*se3s;sb2s*ce3s;cb2s];    
    
%% UNIT VECTORS COMPUTATION
    
%% UPPER VECTOR JOINTS(V) ----------------------------
    Rz=[cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    Ry=[cos(teta) 0 sin(teta); 0 1 0; -sin(teta) 0 cos(teta)];
    Rx= [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    Qm = Rz*Ry*Rx;
    v1v = Qm*v1v_str;
    v2v = Qm*v2v_str;
    v3v = Qm*v3v_str;    
    
%% MIDDLE VECTOR JOINTS(W) ----------------------------    
 
% LEG1 - W1 ------------------------------------------
    A1_1 = sin(a1s-b1s)*(se1s*v1v(1)+ce1s*v1v(2))+cos(a1s-b1s)*v1v(3)+ca2s;
    B1_1 = sa1s*(ce1s*v1v(1)-se1s*v1v(2));
    C1_1 = -sin(a1s+b1s)*(se1s*v1v(1)+ce1s*v1v(2))+cos(a1s+b1s)*(v1v(3))+ca2s;
    D1_1 = B1_1^2-A1_1*C1_1; 
    T12_1 = (-B1_1-sqrt(D1_1))/A1_1;
    theta1 = 2*atan(T12_1);
    
    
    w1v = [se1s*(sb1s*ca1s+cb1s*sa1s*cos(theta1))-ce1s*sa1s*sin(theta1);
       ce1s*(sb1s*ca1s+cb1s*sa1s*cos(theta1))+se1s*sa1s*sin(theta1);
       sa1s*sb1s*cos(theta1)-ca1s*cb1s
      ];
  
% LEG2 - W2 ------------------------------------------
    A2_1 = sin(a1s-b1s)*(se2s*v2v(1)+ce2s*v2v(2))+cos(a1s-b1s)*v2v(3)+ca2s;
    B2_1 = sa1s*(ce2s*v2v(1)-se2s*v2v(2));
    C2_1 = -sin(a1s+b1s)*(se2s*v2v(1)+ce2s*v2v(2))+cos(a1s+b1s)*(v2v(3))+ca2s;
    D2_1 = B2_1^2-A2_1*C2_1;
    T22_1 = (-B2_1-sqrt(D2_1))/A2_1;
    theta2 = 2*atan(T22_1);
    
    w2v = [se2s*(sb1s*ca1s+cb1s*sa1s*cos(theta2))-ce2s*sa1s*sin(theta2);
           ce2s*(sb1s*ca1s+cb1s*sa1s*cos(theta2))+se2s*sa1s*sin(theta2);
           sa1s*sb1s*cos(theta2)-ca1s*cb1s
          ]; 
      
% LEG3 - W3 ------------------------------------------

    A3_1 = sin(a1s-b1s)*(se3s*v3v(1)+ce3s*v3v(2))+cos(a1s-b1s)*v3v(3)+ca2s;
    B3_1 = sa1s*(ce3s*v3v(1)-se3s*v3v(2));
    C3_1 = -sin(a1s+b1s)*(se3s*v3v(1)+ce3s*v3v(2))+cos(a1s+b1s)*(v3v(3))+ca2s;
    D3_1= B3_1^2-A3_1*C3_1;
    T32_1 = (-B3_1-sqrt(D3_1))/A3_1;
    theta3 = 2*atan(T32_1);
    w3v = [se3s*(sb1s*ca1s+cb1s*sa1s*cos(theta3))-ce3s*sa1s*sin(theta3);
           ce3s*(sb1s*ca1s+cb1s*sa1s*cos(theta3))+se3s*sa1s*sin(theta3);
           sa1s*sb1s*cos(theta3)-ca1s*cb1s
          ];    

% h_expr  = [CI;x4;x5;x6];
% hN_expr = [CI;x4;x5;x6];

 %% NONLINEAR SPM DIFFERENTIAL KINEMATICKS RELATIONS   
 
%% JACOBIAN ANALYSIS
    Spsi = sin(psi); Cpsi = cos(psi);
    Steta = sin(teta); Cteta = cos(teta);
    Sphi = sin(phi); Cphi = cos(phi);
%% PLATFORM

    Em = [ 0  , -Spsi , Cpsi*Cteta;
           0  ,  Cpsi , Spsi*Cteta;
           1  ,    0      , -Steta];
%% LEG 1
    P11 = cross(w1v,v1v);
    P21 = cross(u1v,v1v);
    h11 = dot(P11,u1v);
    h21 = dot(P21,w1v);
    Ja1 = h11^(-1)*(P11'*Em);
    Jm1 = h21^(-1)*(P21'*Em);
    
    Jw11 = u1v*Ja1;
    Jw21 = Jw11+w1v*Jm1;    
 
%% LEG 2
    P12 = cross(w2v,v2v);
    P22 = cross(u2v,v2v);
    h12 = dot(P12,u2v);
    h22 = dot(P22,w2v);
    Ja2 = h12^(-1)*(P12'*Em);
    Jm2 = h22^(-1)*(P22'*Em); 
    
    Jw12 = u2v*Ja2;
    Jw22 = Jw12+w2v*Jm2;    
    
%% LEG 3
    P13 = cross(w3v,v3v);
    P23 = cross(u3v,v3v);
    h13 = dot(P13,u3v);
    h23 = dot(P23,w3v);
    Ja3 = h13^(-1)*(P13'*Em);
    Jm3 = h23^(-1)*(P23'*Em);   
    
    Jw13 = u3v*Ja3;
    Jw23 = Jw13+w3v*Jm3;   
    

%% Output -----------------------------
Ja = [Ja1;Ja2;Ja3];

CI = ConditionIndex(X1);
% CI_Jac_x = jacobian(CI,X);

Acons = CI;


h_expr  = Acons;
hN_expr = Acons;

end