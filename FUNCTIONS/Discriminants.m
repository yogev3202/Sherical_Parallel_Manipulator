function Dis = Discriminants(x)
%% UNIT CONVERTION
dg2rd = pi/180;
rd2dg = 1/dg2rd;

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


Rz=[cos(x(1)) -sin(x(1)) 0; sin(x(1)) cos(x(1)) 0; 0 0 1];
Ry=[cos(x(2)) 0 sin(x(2)); 0 1 0; -sin(x(2)) 0 cos(x(2))];
Rx= [1 0 0; 0 cos(x(3)) -sin(x(3)); 0 sin(x(3)) cos(x(3))];

Qm = Rz*Ry*Rx;
v1v = Qm*v1v_str;
v2v = Qm*v2v_str;
v3v = Qm*v3v_str;


%% ---LEG1----
A1_1 = sin(a1s-b1s)*(se1s*v1v(1)+ce1s*v1v(2))+cos(a1s-b1s)*v1v(3)+ca2s;
B1_1 = sa1s*(ce1s*v1v(1)-se1s*v1v(2));
C1_1 = -sin(a1s+b1s)*(se1s*v1v(1)+ce1s*v1v(2))+cos(a1s+b1s)*(v1v(3))+ca2s;
D1 = B1_1^2-A1_1*C1_1;


%% ---LEG2----
A2_1 = sin(a1s-b1s)*(se2s*v2v(1)+ce2s*v2v(2))+cos(a1s-b1s)*v2v(3)+ca2s;
B2_1 = sa1s*(ce2s*v2v(1)-se2s*v2v(2));
C2_1 = -sin(a1s+b1s)*(se2s*v2v(1)+ce2s*v2v(2))+cos(a1s+b1s)*(v2v(3))+ca2s;
D2 = B2_1^2-A2_1*C2_1;



%% ---LEG3----
A3_1 = sin(a1s-b1s)*(se3s*v3v(1)+ce3s*v3v(2))+cos(a1s-b1s)*v3v(3)+ca2s;
B3_1 = sa1s*(ce3s*v3v(1)-se3s*v3v(2));
C3_1 = -sin(a1s+b1s)*(se3s*v3v(1)+ce3s*v3v(2))+cos(a1s+b1s)*(v3v(3))+ca2s;
D3= B3_1^2-A3_1*C3_1;

Dis = [D1;D2;D3];
end