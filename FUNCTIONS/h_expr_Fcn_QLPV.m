function [h_expr,hN_expr] = h_expr_Fcn_QLPV(states,controls,params)


x1 = states(1);x2 = states(2);x3 = states(3);
x4 = states(4);x5 = states(5);x6 = states(6);

p1 = params(1);p2 = params(2);p3 = params(3);
p4 = params(4);p5 = params(5);p6 = params(6);
p7 = params(7);p8 = params(8);p9 = params(9);

X1 = [x1;x2;x3];
X2 = [x4;x5;x6];
X = [X1;X2];

qr   = [p1;p2;p3];
dqr  = [p4;p5;p6];
ddqr = [p7;p8;p9];

u1 = controls(1);
u2 = controls(2);
u3 = controls(3);

psi = X1(1);
teta = X1(2);
phi = X1(3);

dpsi = X2(1);
dteta = X2(2);
dphi = X2(3);

Dis = Discriminants(X1);
CI = ConditionIndex(X1);
dqm = MotorVel(X);

h_expr  = [Dis;CI;dqm];
hN_expr = [Dis;CI;dqm];


end