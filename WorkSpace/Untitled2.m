clc;
clear;
close all

syms PHI S1 S2 xk  Xr Uk Ur Q R real



Xk = PHI*xk + S1*Uk + S2;
J = (Xk - Xr)'*Q*(Xk - Xr) + (Uk - Ur)'*R*(Uk - Ur);
J = expand(J);
collect(J,Uk)