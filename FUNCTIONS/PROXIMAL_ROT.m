
% computes the rotation matrix of the proximal link
% input :
% x - actuator angle theta



function DCM = PROXIMAL_ROT(u,w,v)

 a1s = 65*(pi/180);
 x0 = -1.140243740887195;
 
 Z_BB_hat = [0;0;1];
 Y_BB_hat = [0;1;0];
 X_BB_hat = [1;0;0];
 
 Z_BS = [0;0;1];
%  Y_BS = Rz(x0)*Rz(x - x0)*[0;1;0];
 Y_BS = [w(1);w(2);0];
 X_BS = cross(Y_BS,Z_BS);
% X_BS = cross(Z_BS,Y_BS);
 
 Z_BS_hat = Z_BS/norm(Z_BS);
 Y_BS_hat = Y_BS/norm(Y_BS);
 X_BS_hat = X_BS/norm(X_BS);
 RB = [X_BB_hat,Y_BB_hat,Z_BB_hat];
 RS = [X_BS_hat,Y_BS_hat,Z_BS_hat];
 
%  DCM = (RB * RS');
DCM = RS * RB';


end

function RZ = Rz(x)

    RZ=[cos(x) -sin(x) 0; sin(x) cos(x) 0; 0 0 1;];

end



