function x = SPMNFK_Fcn(x)

%% UNIT CONVERTION
dg2rd = pi/180;
%% GEOMETRIC PARAMETERS
a1s = 65*dg2rd; sa1s = sin(a1s); ca1s = cos(a1s);
a2s = 60*dg2rd; sa2s = sin(a2s); ca2s = cos(a2s);

b1s = 0*dg2rd; sb1s = sin(b1s); cb1s = cos(b1s);
b2s = 110*dg2rd; sb2s = sin(b2s); cb2s = cos(b2s);
ETA = [0;240;120]*dg2rd;
eta1s = 0*dg2rd; se1s = sin(eta1s); ce1s = cos(eta1s);
eta2s = 240*dg2rd; se2s = sin(eta2s); ce2s = cos(eta2s);
eta3s = 120*dg2rd; se3s = sin(eta3s); ce3s = cos(eta3s);
gravity = [0;0;-9.80665];
thetaZ0 = -1.140243740887195*ones(3,1);

%% UNIT VECTORS CONSTANTS

u1v = [sb1s*se1s;sb1s*ce1s;-cb1s];
u2v = [sb1s*se2s;sb1s*ce2s;-cb1s];
u3v = [sb1s*se3s;sb1s*ce3s;-cb1s];
v1v_str = [sb2s*se1s;sb2s*ce1s;cb2s];
v2v_str = [sb2s*se2s;sb2s*ce2s;cb2s];
v3v_str = [sb2s*se3s;sb2s*ce3s;cb2s];

%% W Vectork
THETA = x(10:12);
theta1 = THETA(1);
theta2 = THETA(2);
theta3 = THETA(3);


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

%% Numerical Forward Kinemtics

a3s = 2*asin(sin(b2s)*cos(pi/6));
x_old = x(1:9);
X0 = x_old;



%% -------------------


F = @(X)[dot(w1v,X(1:3))-cos(a2s);
         dot(w2v,X(4:6))-cos(a2s);
         dot(w3v,X(7:9))-cos(a2s);
         dot(X(1:3),X(4:6))-cos(a3s);
         dot(X(1:3),X(7:9))-cos(a3s);
         dot(X(4:6),X(7:9))-cos(a3s);
         norm(X(1:3))-1;
         norm(X(4:6))-1;
         norm(X(7:9))-1];

options = optimoptions('fsolve','Display','iter-detailed','FunctionTolerance',1e-20,...
    'MaxIterations',1000,'OptimalityTolerance',1e-20,'Algorithm','trust-region-dogleg',...
    'CheckGradients',true,'FinDiffType','central');
[X,fval] = fsolve(F,X0,options);
x = X;




end

