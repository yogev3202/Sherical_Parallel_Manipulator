
%% Continuous-time nonlinear kinematic model of a spm

% 6 states (x): 
%   psi angle (x1)
%   theta angle (x2)
%   phi angle (x3)
%   psi angle rate (x4)
%   theta angle rate (5)
%   phi angle rate (x6)


% 3 inputs: (u)
%   revolute joint1 torque (u1)
%   revolute joint2 torque (u2)
%   revolute joint3 torque (u3)

%% -----------------------------------------------
function states_plus = spm_dynamic_ODEC(states,controls)
%% STATES AND INPUTS
    x1 = states(1);x2 = states(2);x3 = states(3);
    x4 = states(4);x5 = states(5);x6 = states(6);
    
    X1 = [x1;x2;x3];
    X2 = [x4;x5;x6];
    
    X = [X1;X2];
    
    u1 = controls(1);
    u2 = controls(2);
    u3 = controls(3);



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
    
%% TIME DERIVATIVE JACOBIAN ANALYSIS    


    
    dq1 = Ja1*[dpsi;dteta;dphi];
    dq2 = Ja2*[dpsi;dteta;dphi];
    dq3 = Ja3*[dpsi;dteta;dphi];
    
    Wp = Em * [dpsi;dteta;dphi];
    
    
    du1v = [0;0;0];
    du2v = [0;0;0];
    du3v = [0;0;0];
    
    dw1v = dq1 * cross(u1v,w1v);
    dw2v = dq2 * cross(u2v,w2v);
    dw3v = dq3 * cross(u3v,w3v);

%     dw1v = dq1 * cross(w1v,u1v);
%     dw2v = dq2 * cross(w2v,u2v);
%     dw3v = dq3 * cross(w3v,u3v);
    
    dv1v = crossVM(Wp)*v1v;
    dv2v = crossVM(Wp)*v2v;
    dv3v = crossVM(Wp)*v3v;
%% PLATFORM

    dEm = [0 , -dpsi*Cpsi , -dpsi*Spsi*Cteta - dteta*Steta*Cpsi;
           0 , -dpsi*Spsi ,  dpsi*Cpsi*Cteta - dteta*Steta*Spsi;
           0 ,      0     ,                 -dteta*Cteta      ];
   
%% LEG 1   

    dP11 = cross(dw1v,v1v) + cross(w1v,dv1v);
    dP21 = cross(u1v,dv1v);
    dh11 = dot(dP11,u1v);
    dh21 = dot(dP21,w1v) + dot(P21,dw1v);
    dJa1 = h11^(-1)*(dP11'*Em + P11'*dEm) - h11^(-2)*(P11'*Em*dh11);
    dJm1 = h21^(-1)*(dP21'*Em + P21'*dEm) - h21^(-2)*(P21'*Em*dh21);
 
    dJw11 = u1v*dJa1;
    dJw21 = u1v*dJa1+dw1v*Jm1+w1v*dJm1;    
    
%% LEG 2

    dP12 = cross(dw2v,v2v) + cross(w2v,dv2v);
    dP22 = cross(u2v,dv2v);
    dh12 = dot(dP12,u2v);
    dh22 = dot(dP22,w2v) + dot(P22,dw2v);
    dJa2 = h12^(-1)*(dP12'*Em + P12'*dEm) - h12^(-2)*(P12'*Em*dh12);
    dJm2 = h22^(-1)*(dP22'*Em + P22'*dEm) - h22^(-2)*(P22'*Em*dh22);
    
    dJw12 = u2v*dJa2;
    dJw22 = u2v*dJa2+dw2v*Jm2+w2v*dJm2;    

%% LEG 3

    dP13 = cross(dw3v,v3v) + cross(w3v,dv3v);
    dP23 = cross(u3v,dv3v);
    dh13 = dot(dP13,u3v);
    dh23 = dot(dP23,w3v) + dot(P23,dw3v);
    dJa3 = h13^(-1)*(dP13'*Em + P13'*dEm) - h13^(-2)*(P13'*Em*dh13);
    dJm3 = h23^(-1)*(dP23'*Em + P23'*dEm) - h23^(-2)*(P23'*Em*dh23);  
    
    dJw13 = u3v*dJa3;
    dJw23 = u3v*dJa3+dw3v*Jm3+w3v*dJm3;
    
    
%% DCM OF THE LINKS FROM LOCAL FRAME TO INERTIAL FRAME    

%% LEG 1 
    DCM11 = PROXIMAL_ROT(u1v,w1v,v1v);
    DCM21 = DISTAL_ROT(u1v,w1v,v1v);
    
%% LEG 2 
    DCM12 = PROXIMAL_ROT(u2v,w2v,v2v);
    DCM22 = DISTAL_ROT(u2v,w2v,v2v);  
    
%% LEG 3 
    DCM13 = PROXIMAL_ROT(u3v,w3v,v3v);
    DCM23 = DISTAL_ROT(u3v,w3v,v3v); 

%% SPM LINKS PROPERTIES    
    LINKS = LINKS_PROP(); 
%% LEG 1 
    PROXIMAL1 = LINKS.PROXIMAL{1, 1};
    DISTAL1 = LINKS.DISTAL{1, 1};
    
    m11 = PROXIMAL1.m;
    R11_Bf = PROXIMAL1.R_Bf*mm2m;
    I11_Bf = PROXIMAL1.I_Bf*mm2m^2;
    
    m21 = DISTAL1.m;
    R21_Bf = DISTAL1.R_Bf*mm2m;
    I21_Bf = DISTAL1.I_Bf*mm2m^2;    
    
    I11_Sf = DCM11*I11_Bf*DCM11'; 
    I21_Sf = DCM21*I21_Bf*DCM21';
    R11_Sf = DCM11*crossVM(R11_Bf)*DCM11';
    R21_Sf = DCM21*crossVM(R21_Bf)*DCM21';
    
%% LEG 2 
    PROXIMAL2 = LINKS.PROXIMAL{1, 2};
    DISTAL2 = LINKS.DISTAL{1, 2};
    
    m12 = PROXIMAL2.m;
    R12_Bf = PROXIMAL2.R_Bf*mm2m;
    I12_Bf = PROXIMAL2.I_Bf*mm2m^2;
    
    m22 = DISTAL2.m;
    R22_Bf = DISTAL2.R_Bf*mm2m;
    I22_Bf = DISTAL2.I_Bf*mm2m^2;  
    
    I12_Sf = DCM12*I12_Bf*DCM12'; 
    I22_Sf = DCM22*I22_Bf*DCM22';
    R12_Sf = DCM12*crossVM(R12_Bf)*DCM12';
    R22_Sf = DCM22*crossVM(R22_Bf)*DCM22';    
    
%% LEG 3 
    PROXIMAL3 = LINKS.PROXIMAL{1, 3};
    DISTAL3 = LINKS.DISTAL{1, 3};
    
    m13 = PROXIMAL3.m;
    R13_Bf = PROXIMAL3.R_Bf*mm2m;
    I13_Bf = PROXIMAL3.I_Bf*mm2m^2;
    
    m23 = DISTAL3.m;
    R23_Bf = DISTAL3.R_Bf*mm2m;
    I23_Bf = DISTAL3.I_Bf*mm2m^2;   
    
    I13_Sf = DCM13*I13_Bf*DCM13'; 
    I23_Sf = DCM23*I23_Bf*DCM23';
    R13_Sf = DCM13*crossVM(R13_Bf)*DCM13';
    R23_Sf = DCM23*crossVM(R23_Bf)*DCM23';    
    
%% PLATFORM   

    PLATFORM = LINKS.PLATFORM;
    mp = PLATFORM.m;
    Rp_Bf = PLATFORM.R_Bf*mm2m;
    Ip_Bf = PLATFORM.I_Bf*mm2m^2;
    
    Ip_Sf = Qm*Ip_Bf*Qm';
    Rp_Sf = Qm*crossVM(Rp_Bf)*Qm';
 
%% Calculation of Mass Matrix (M)

    %% Leg 1
    M11 = Jw11'*I11_Sf*Jw11;
    M21 = Jw21'*I21_Sf*Jw21;
    M1 = M11+M21;
    %% Leg 2
    M12 = Jw12'*I12_Sf*Jw12;
    M22 = Jw22'*I22_Sf*Jw22;
    M2 = M12+M22;
    %% Leg 3
    M13 = Jw13'*I13_Sf*Jw13;
    M23 = Jw23'*I23_Sf*Jw23;
    M3 = M13+M23;
    %% Top Platform
    Mp = Em'*Ip_Sf*Em;
    %MASS MATRIX
    Mm = M1+M2+M3+Mp;  

%% Calculation of CORIOLIS MATRIX (C)

    %% Leg 1
    W11 = Jw11*X2; % Angular Velocity Proximal
    W21 = Jw21*X2; % Angular Velocity Distal    

    C11 = Jw11'*(I11_Sf*dJw11+crossVM(W11)*I11_Sf*Jw11);
    C21 = Jw21'*(I21_Sf*dJw21+crossVM(W21)*I21_Sf*Jw21);
    C1 = C11+C21;


    %% Leg 2
    W12 = Jw12*X2; % Angular Velocity Proximal
    W22 = Jw22*X2; % Angular Velocity Distal  

    C12 = Jw12'*(I12_Sf*dJw12+crossVM(W12)*I12_Sf*Jw12);
    C22 = Jw22'*(I22_Sf*dJw22+crossVM(W22)*I22_Sf*Jw22); 
    C2 = C12+C22;
 

    %% Leg 3
    W13 = Jw13*X2; % Angular Velocity Proximal
    W23 = Jw23*X2; % Angular Velocity Distal  

    C13 = Jw13'*(I13_Sf*dJw13+crossVM(W13)*I13_Sf*Jw13);
    C23 = Jw23'*(I23_Sf*dJw23+crossVM(W23)*I23_Sf*Jw23);
    C3 = C13+C23;
    
    %% Top Platform
    Cp = Em'*Ip_Sf*dEm+Em'*(crossVM(Wp)*Ip_Sf)*Em;
    %CORIOLIS(C) MATRIX
    Cm = C1+C2+C3+Cp;  

%% Calculation of GRAVITY VECTOR (Gv) 
    gravity = [0;0;-9.80665];
%     gravity = [0;0;-9806.65];
    
%% Leg 1
    G11 = -m11*Jw11'*(R11_Sf*gravity);
    G21 = -m21*Jw21'*(R21_Sf*gravity);
    G1 = G11 + G21;

    %% Leg 2
    G12 = -m12*Jw12'*(R12_Sf*gravity);
    G22 = -m22*Jw22'*(R22_Sf*gravity);
    G2 = G12 + G22;

    %% Leg 3
    G13 = -m13*Jw13'*(R13_Sf*gravity);
    G23 = -m23*Jw23'*(R23_Sf*gravity);
    G3 = G13 + G23;

    %% Top Platform
    Gp = -Em'*(mp*Rp_Sf*gravity);
    %GRAVITY VECTOR (Gv)
    Gv = G1+G2+G3+Gp;    

%% Output ---------------------------    
   

    Ja = [Ja1;Ja2;Ja3];
    gx = Mm^(-1)*Ja';
    fx = -Mm^(-1)*(Cm*X2 + Gv);
    states1_dot = X2;
    states2_dot = fx + gx*[u1;u2;u3];

    Xdot = [states1_dot; states2_dot]; 

%% Output ---------------------------    
   

    Ja = [Ja1;Ja2;Ja3];
    Eps = 1e-10;

    % G_1 = Gv(1)*(sinc(x1/pi+epsilon)/sin(x1+epsilon));
    % G_2 = Gv(2)*(sinc(x2/pi+epsilon)/sin(x2+epsilon));
    % G_3 = Gv(3)*(sinc(x3/pi+epsilon)/sin(x3+epsilon));

    % G_1 = Gv(1)*(1/(x1+epsilon));
    % G_2 = Gv(2)*(1/(x2+epsilon));
    % G_3 = Gv(3)*(1/(x3+epsilon));   

    % G_1 = Gv(1)*(sinc(x1+epsilon)/sinc(x1));
    % G_2 = Gv(2)*(sinc(x2+epsilon)/sinc(x1));
    % G_3 = Gv(3)*(sinc(x3+epsilon)/sinc(x1));   

    G_1 = Gv(1)*(SC(x1,Eps)/sin(x1+Eps));
    G_2 = Gv(2)*(SC(x2,Eps)/sin(x2+Eps));
    G_3 = Gv(3)*(SC(x3,Eps)/sin(x3+Eps));



    X1 = [x1;x2;x3];
    X2 = [x4;x5;x6];
    u  = [u1;u2;u3];

    Ac = [zeros(3),eye(3); -Mm^(-1)*diag([G_1,G_2,G_3]),-Mm^(-1)*Cm];
    Bc = [zeros(3);Mm^(-1)*Ja'];

    states_plus = Ac*[X1+Eps*ones(3,1);X2] + Bc*u;
    % states_plus = Ac*[X1;X2] + Bc*u;

    % disp(states_plus - Xdot);




end

