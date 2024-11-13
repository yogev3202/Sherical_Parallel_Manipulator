function [u] = FeedBackLin(Kp,Ki,x,qr,dqr,z)



%% STATES AND INPUTS
    x1 = x(1);
    x2 = x(2);
    x3 = x(3);



%% UNIT CONVERTION
    dg2rd = pi/180;
    rd2dg = 1/dg2rd;

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
    
%% V vectors ----------------------------
    Rz=[cos(x1) -sin(x1) 0; sin(x1) cos(x1) 0; 0 0 1];
    Ry=[cos(x2) 0 sin(x2); 0 1 0; -sin(x2) 0 cos(x2)];
    Rx= [1 0 0; 0 cos(x3) -sin(x3); 0 sin(x3) cos(x3)];
    Qm = Rz*Ry*Rx;
    v1v = Qm*v1v_str;
    v2v = Qm*v2v_str;
    v3v = Qm*v3v_str;    
    
%% W vectors ----------------------------    
 
%% LEG1 - W1
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
  
%% LEG2 - W2  
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
      
%% LEG3 - W3

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
      
      
 %% Nonlinear SPM Differential Kinematic Plant Model   
 
 
    Em = [ 0  , -sin(x1) , cos(x1)*cos(x2);
           0  ,  cos(x1) , sin(x1)*cos(x2);
           1  ,    0      ,    -sin(x2)]; 
%% Leg 1
    P11 = cross(w1v,v1v);
    P21 = cross(u1v,v1v);
    h11 = dot(P11,u1v);
    h21 = dot(P21,w1v);
    Ja1 = h11^(-1)*(P11'*Em);
    Jm1 = h21^(-1)*(P21'*Em); 
 
%% Leg 2
    P12 = cross(w2v,v2v);
    P22 = cross(u2v,v2v);
    h12 = dot(P12,u2v);
    h22 = dot(P22,w2v);
    Ja2 = h12^(-1)*(P12'*Em);
    Jm2 = h22^(-1)*(P22'*Em);    
    
%% Leg 3
    P13 = cross(w3v,v3v);
    P23 = cross(u3v,v3v);
    h13 = dot(P13,u3v);
    h23 = dot(P23,w3v);
    Ja3 = h13^(-1)*(P13'*Em);
    Jm3 = h23^(-1)*(P23'*Em);    
    
    
    Am = [P11';P12';P13'];
    Bm = diag([h11,h12,h13]);
    
    % dxdt = (Am*Em)^(-1)*Bm*[u1;u2;u3];

    J = (Am*Em)^(-1)*Bm;
    Er = qr-x(1:3);
    % Z = x(4:6);

    u = -J^(-1)*(-Kp*Er - Ki*z + dqr);






end