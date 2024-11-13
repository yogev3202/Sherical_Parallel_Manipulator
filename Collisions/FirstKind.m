function [B1,B2,B3,Bm,detB,Flag] = FirstKind(x)


%% UNIT CONVERTION
dg2rd = pi/180;
%% DEFINE PARAMETERS

%% GEOMETRIC PARAMETERS
a1s = 65*dg2rd; sa1s = sin(a1s); ca1s = cos(a1s);
a2s = 60*dg2rd; sa2s = sin(a2s); ca2s = cos(a2s);
b1s = 0*dg2rd; sb1s = sin(b1s); cb1s = cos(b1s);
b2s = 110*dg2rd; sb2s = sin(b2s); cb2s = cos(b2s);
ETA = [0;240;120]*dg2rd;
eta1s = 0*dg2rd; se1s = sin(eta1s); ce1s = cos(eta1s);
eta2s = 240*dg2rd; se2s = sin(eta2s); ce2s = cos(eta2s);
eta3s = 120*dg2rd; se3s = sin(eta3s); ce3s = cos(eta3s);

%% UNIT VECTORS CONSTANTS

u1v = [sb1s*se1s;sb1s*ce1s;-cb1s];
u2v = [sb1s*se2s;sb1s*ce2s;-cb1s];
u3v = [sb1s*se3s;sb1s*ce3s;-cb1s];
v1v_str = [sb2s*se1s;sb2s*ce1s;cb2s];
v2v_str = [sb2s*se2s;sb2s*ce2s;cb2s];
v3v_str = [sb2s*se3s;sb2s*ce3s;cb2s];

x1 = x(1);
x2 = x(2);
x3 = x(3);

%% -------------------------Inverse Kenematic---------------------------------
%% Platform
RX= [1 0 0; 0 cos(x3) -sin(x3); 0 sin(x3) cos(x3);];
RY=[cos(x2) 0 sin(x2); 0 1 0; -sin(x2) 0 cos(x2);];
RZ=[cos(x1) -sin(x1) 0; sin(x1) cos(x1) 0; 0 0 1;];
Qm = RZ*RY*RX;
v1v = Qm*v1v_str;
v2v = Qm*v2v_str;
v3v = Qm*v3v_str;

%% LEG1
A1_1 = sin(a1s-b1s)*(se1s*v1v(1)+ce1s*v1v(2))+cos(a1s-b1s)*v1v(3)+ca2s;
B1_1 = sa1s*(ce1s*v1v(1)-se1s*v1v(2));
C1_1 = -sin(a1s+b1s)*(se1s*v1v(1)+ce1s*v1v(2))+cos(a1s+b1s)*(v1v(3))+ca2s;
D1_1 = B1_1^2-A1_1*C1_1;
T12_1 = (-B1_1-sqrt(D1_1))/A1_1;
theta1_m = 2*atan(T12_1);
theta1 = theta1_m;

%% LEG2
A2_1 = sin(a1s-b1s)*(se2s*v2v(1)+ce2s*v2v(2))+cos(a1s-b1s)*v2v(3)+ca2s;
B2_1 = sa1s*(ce2s*v2v(1)-se2s*v2v(2));
C2_1 = -sin(a1s+b1s)*(se2s*v2v(1)+ce2s*v2v(2))+cos(a1s+b1s)*(v2v(3))+ca2s;
D2_1 = B2_1^2-A2_1*C2_1;
T22_1 = (-B2_1-sqrt(D2_1))/A2_1;
theta2_m = 2*atan(T22_1);
theta2 = theta2_m;


%% LEG3
A3_1 = sin(a1s-b1s)*(se3s*v3v(1)+ce3s*v3v(2))+cos(a1s-b1s)*v3v(3)+ca2s;
B3_1 = sa1s*(ce3s*v3v(1)-se3s*v3v(2));
C3_1 = -sin(a1s+b1s)*(se3s*v3v(1)+ce3s*v3v(2))+cos(a1s+b1s)*(v3v(3))+ca2s;
D3_1= B3_1^2-A3_1*C3_1;
T32_1 = (-B3_1-sqrt(D3_1))/A3_1;
theta3_m = 2*atan(T32_1);
theta3 = theta3_m;

thetav = [theta1;theta2;theta3];

%% ------------------------- Unit Vectors---------------------------------
v1v = Qm*v1v_str;
v2v = Qm*v2v_str;
v3v = Qm*v3v_str;


%% -----W vectors ------
theta1 = thetav(1);
theta2 = thetav(2);
theta3 = thetav(3);
w1v = [se1s*(sb1s*ca1s+cb1s*sa1s*cos(theta1))-ce1s*sa1s*sin(theta1);
       ce1s*(sb1s*ca1s+cb1s*sa1s*cos(theta1))+se1s*sa1s*sin(theta1);
       sa1s*sb1s*cos(theta1)-ca1s*cb1s
      ];
  
w2v = [se2s*(sb1s*ca1s+cb1s*sa1s*cos(theta2))-ce2s*sa1s*sin(theta2);
       ce2s*(sb1s*ca1s+cb1s*sa1s*cos(theta2))+se2s*sa1s*sin(theta2);
       sa1s*sb1s*cos(theta2)-ca1s*cb1s
      ];
  
w3v = [se3s*(sb1s*ca1s+cb1s*sa1s*cos(theta3))-ce3s*sa1s*sin(theta3);
       ce3s*(sb1s*ca1s+cb1s*sa1s*cos(theta3))+se3s*sa1s*sin(theta3);
       sa1s*sb1s*cos(theta3)-ca1s*cb1s
      ];
  
  
%% Differential Jacobian
B1 = dot(cross(w1v,u1v),v1v);
B2 = dot(cross(w2v,u2v),v2v);
B3 = dot(cross(w3v,u3v),v3v);
Bm = diag([B1,B2,B3]);

detB = det(Bm);

Flag = false;

if (detB <=  0.1 || ~isreal(detB))
    Flag = true;
end

% if (detB <=  0)
%     Flag = true;
% end


end

