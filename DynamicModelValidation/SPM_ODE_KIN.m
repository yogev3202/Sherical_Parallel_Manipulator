function x_plus = SPM_ODE_KIN(X)

x0 = X(1:3);
u = X(4:6);

x_dot = Kinematic_ODE_CT(x0,u);
[t,y] = ode45(@(t,y) x_dot, [0 1e-3], x0);
xkp1  = y(end,:)';
x_plus = xkp1;



end