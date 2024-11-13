

% THETAv = -1.140243740887195*ones(3,1);
EA = [0;10;0]*pi/180;
THETAv = SPMIK_Fcn(EA,'---');

Gv_temp = subs(Gv,[q1,q2,q3],THETAv');
Gv_old = subs(Gv_temp,[x1,x2,x3],EA');
disp(vpa(Gv_old,5)*1000)


G_1 = Gv_temp(1,1)*(Sc(x1,alpha)/sin(x1+alpha));
G_2 = Gv_temp(2,1)*(Sc(x2,alpha)/sin(x2+alpha));
G_3 = Gv_temp(3,1)*(Sc(x3,alpha)/sin(x3+alpha));
Gv_new = diag([G_1,G_2,G_3]) * [x1+alpha;x2+alpha;x3+alpha];
% Gv_new = diag([G_1,G_2,G_3]) * [x1;x2;x3];
Gv_new = subs(Gv_new,[x1,x2,x3],EA');
disp(vpa(Gv_new,5)*1000)


