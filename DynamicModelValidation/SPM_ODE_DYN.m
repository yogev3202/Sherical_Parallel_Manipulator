function x_plus = SPM_ODE_DYN(X)

    x0 = X(1:6);
    u = X(7:9);
    x_dot = Dynamic_ODE_CT(x0,u);
    [t,y] = ode45(@(t,y) x_dot, [0 1e-3], x0);
    xkp1  = y(end,:)';
    x_plus = xkp1;

end