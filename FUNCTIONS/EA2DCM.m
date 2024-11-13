function Q = EA2DCM(EA)

psi = EA(1); % psi
theta = EA(2); % theta
phi = EA(3); % phi

c_psi = cos(psi); s_psi = sin(psi);
c_theta = cos(theta); s_theta = sin(theta);
c_phi = cos(phi); s_phi = sin(phi);


Rz = [c_psi , 0 , -s_psi; -s_psi , c_psi , 0 ; 0 , 0 , 1];
Ry = [c_theta , 0 , -s_theta ; 0 , 1 , 0 ; s_theta , 0 , c_theta];
Rx = [1 , 0 , 0 ; 0 , c_phi , s_phi ; 0 , -s_phi , c_phi];

DCM = Rx*Ry*Rz;


end

