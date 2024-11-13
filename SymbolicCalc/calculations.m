clc;
clear all

%% Vars

syms x1 x2 x3 q1 q2 q3 real

syms Sc(x,alpha) 
Sc(x,alpha) = sinc((x+alpha)/pi);



%% Constants

dg2rd = pi/180;
rd2dg = 1/dg2rd;
mm2m = 1e-3;
gravity = [0;0;-9.80665];

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

u1v = [sb1s*se1s;sb1s*ce1s;-cb1s];
u2v = [sb1s*se2s;sb1s*ce2s;-cb1s];
u3v = [sb1s*se3s;sb1s*ce3s;-cb1s];

v1v_str = [sb2s*se1s;sb2s*ce1s;cb2s];
v2v_str = [sb2s*se2s;sb2s*ce2s;cb2s];
v3v_str = [sb2s*se3s;sb2s*ce3s;cb2s]; 

%% Unit Vector Computation

%% UPPER VECTOR JOINTS(V) ----------------------------
    Rz=[cos(x1) -sin(x1) 0; sin(x1) cos(x1) 0; 0 0 1];
    Ry=[cos(x2) 0 sin(x2); 0 1 0; -sin(x2) 0 cos(x2)];
    Rx= [1 0 0; 0 cos(x3) -sin(x3); 0 sin(x3) cos(x3)];



    alpha = 1e-15;

    Rz2 = [cos(x1) -Sc(x1,alpha) 0; Sc(x1,alpha) cos(x1) 0; 0 0 1];
    Ry2=[cos(x2) 0 Sc(x2,alpha); 0 1 0; -Sc(x2,alpha) 0 cos(x2)];
    Rx2= [1 0 0; 0 cos(x3) -Sc(x3,alpha); 0 Sc(x3,alpha) cos(x3)];    

    Qm = Rz*Ry*Rx;
    Qm2 = Rz2*Ry2*Rx2 ;
    v1v = Qm*v1v_str;
    v2v = Qm*v2v_str;
    v3v = Qm*v3v_str;    

    % v1v = Qm2*v1v_str;
    % v2v = Qm2*v2v_str;
    % v3v = Qm2*v3v_str;   

    v1v = vpa(v1v,3);
    v2v = vpa(v2v,3);
    v3v = vpa(v3v,3);   
    

    % check1 = subs(Rz,x1,5*(pi/180));
    % check2 = subs(Rz2,x1,5*(pi/180));
    % check1 = vpa(check1,2);
    % check2 = vpa(check2,2);
    % 
    % v1v_new = Qm2*v1v_str;
    % check1 = subs(v1v_new,[x1,x2,x3],[0,0,0]*(pi/180));
    % check2 = subs(v1v,[x1,x2,x3],[0,0,0]*(pi/180));
    % check1 = vpa(check1,2);
    % check2 = vpa(check2,2);
    % 
    % 
    % 
    % if (abs(check1(1)-check2(1)) <= 1e-3),disp('true'); end
    % if (abs(check1(2)-check2(2)) <= 1e-3),disp('true'); end
    % if (abs(check1(3)-check2(3)) <= 1e-3),disp('true'); end


    
% MIDDLE VECTOR JOINTS(W) ----------------------------    
 
% LEG1 - W1 ------------------------------------------
     
    w1v = [se1s*(sb1s*ca1s+cb1s*sa1s*cos(q1))-ce1s*sa1s*sin(q1);
       ce1s*(sb1s*ca1s+cb1s*sa1s*cos(q1))+se1s*sa1s*sin(q1);
       sa1s*sb1s*cos(q1)-ca1s*cb1s
      ];
  
% LEG2 - W2 ------------------------------------------
    w2v = [se2s*(sb1s*ca1s+cb1s*sa1s*cos(q2))-ce2s*sa1s*sin(q2);
           ce2s*(sb1s*ca1s+cb1s*sa1s*cos(q2))+se2s*sa1s*sin(q2);
           sa1s*sb1s*cos(q2)-ca1s*cb1s
          ]; 
      
% LEG3 - W3 ------------------------------------------

    w3v = [se3s*(sb1s*ca1s+cb1s*sa1s*cos(q3))-ce3s*sa1s*sin(q3);
           ce3s*(sb1s*ca1s+cb1s*sa1s*cos(q3))+se3s*sa1s*sin(q3);
           sa1s*sb1s*cos(q3)-ca1s*cb1s
          ];    
      

%% JACOBIAN ANALYSIS
    Spsi = sin(x1); Cpsi = cos(x1);
    Steta = sin(x2); Cteta = cos(x2);
    Sphi = sin(x3); Cphi = cos(x3);
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

     P11 = vpa(P11,3);
     P21 = vpa(P21,3);
     h11 = vpa(h11,3);
     h21 = vpa(h21,3);
     Ja1 = vpa(Ja1,3);
     Jm1 = vpa(Jm1,3);
     Jw11 = vpa(Jw11,3);
     Jw21 = vpa(Jw21,3);

    

 
%% LEG 2
    P12 = cross(w2v,v2v);
    P22 = cross(u2v,v2v);
    h12 = dot(P12,u2v);
    h22 = dot(P22,w2v);
    Ja2 = h12^(-1)*(P12'*Em);
    Jm2 = h22^(-1)*(P22'*Em); 
    
    Jw12 = u2v*Ja2;
    Jw22 = Jw12+w2v*Jm2;  

     P12 = vpa(P12,3);
     P22 = vpa(P22,3);
     h12 = vpa(h12,3);
     h22 = vpa(h22,3);
     Ja2 = vpa(Ja2,3);
     Jm2 = vpa(Jm2,3);
     Jw12 = vpa(Jw12,3);
     Jw22 = vpa(Jw22,3);    
    
%% LEG 3
    P13 = cross(w3v,v3v);
    P23 = cross(u3v,v3v);
    h13 = dot(P13,u3v);
    h23 = dot(P23,w3v);
    Ja3 = h13^(-1)*(P13'*Em);
    Jm3 = h23^(-1)*(P23'*Em);   
    
    Jw13 = u3v*Ja3;
    Jw23 = Jw13+w3v*Jm3;  

    
     P13 = vpa(P13,3);
     P23 = vpa(P23,3);
     h13 = vpa(h13,3);
     h23 = vpa(h23,3);
     Ja3 = vpa(Ja3,3);
     Jm3 = vpa(Jm3,3);
     Jw13 = vpa(Jw13,3);
     Jw23 = vpa(Jw23,3); 

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
    m21 = DISTAL1.m;
    R21_Bf = DISTAL1.R_Bf*mm2m;

    R11_Sf = DCM11*crossVM(R11_Bf)*DCM11';
    R21_Sf = DCM21*crossVM(R21_Bf)*DCM21';


    
%% LEG 2 
    PROXIMAL2 = LINKS.PROXIMAL{1, 2};
    DISTAL2 = LINKS.DISTAL{1, 2};
    
    m12 = PROXIMAL2.m;
    R12_Bf = PROXIMAL2.R_Bf*mm2m;
    
    m22 = DISTAL2.m;
    R22_Bf = DISTAL2.R_Bf*mm2m;
    
    R12_Sf = DCM12*crossVM(R12_Bf)*DCM12';
    R22_Sf = DCM22*crossVM(R22_Bf)*DCM22';    
    
%% LEG 3 
    PROXIMAL3 = LINKS.PROXIMAL{1, 3};
    DISTAL3 = LINKS.DISTAL{1, 3};
    
    m13 = PROXIMAL3.m;
    R13_Bf = PROXIMAL3.R_Bf*mm2m;

    
    m23 = DISTAL3.m;
    R23_Bf = DISTAL3.R_Bf*mm2m;

    R13_Sf = DCM13*crossVM(R13_Bf)*DCM13';
    R23_Sf = DCM23*crossVM(R23_Bf)*DCM23';    
    
%% PLATFORM   

    PLATFORM = LINKS.PLATFORM;
    mp = PLATFORM.m;
    Rp_Bf = PLATFORM.R_Bf*mm2m;

    Rp_Sf = Qm*crossVM(Rp_Bf)*Qm';

%% Calculation of GRAVITY VECTOR (Gv) 

    
%% Leg 1
    G11 = -m11*Jw11'*(R11_Sf*gravity);
    G21 = -m21*Jw21'*(R21_Sf*gravity);
    G1 = G11 + G21;
    % GG1 = collect(G1,sin(x1));
    G1 = vpa(G1,2);

    %% Leg 2
    G12 = -m12*Jw12'*(R12_Sf*gravity);
    G22 = -m22*Jw22'*(R22_Sf*gravity);
    G2 = G12 + G22;
    G2 = vpa(G2,2);

    %% Leg 3
    G13 = -m13*Jw13'*(R13_Sf*gravity);
    G23 = -m23*Jw23'*(R23_Sf*gravity);
    G3 = G13 + G23;
    G3 = vpa(G3,2);

    %% Top Platform
    Gp = -Em'*(mp*Rp_Sf*gravity);
    Gp = vpa(Gp,2);
    %GRAVITY VECTOR (Gv)
    Gv = G1+G2+G3+Gp;   

    

    % Gv_new = subs(Gv,[sin(x1),sin(x2),sin(x3)], ...
    %     [Fs(x1,alpha),Fs(x2,alpha),Fs(x3,alpha)]);




