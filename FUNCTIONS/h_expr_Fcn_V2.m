function [h_expr,hN_expr] = h_expr_Fcn_V2(states,controls)


x1 = states(1);e2 = states(2);e3 = states(3);
x4 = states(4);e5 = states(5);e6 = states(6);

E1 = [x1;e2;e3];
E2 = [x4;e5;e6];


u1 = controls(1);
u2 = controls(2);
u3 = controls(3);


h_expr  = [Discriminants(E1);ConditionIndex(E1)];
hN_expr = [Discriminants(E1);ConditionIndex(E1)];

end