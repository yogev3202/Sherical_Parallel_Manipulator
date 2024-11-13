function [Um,Wm,Vm] = unit_vectors(EA,thetav)
    %% UNIT CONVERTION
    dg2rd = pi/180;
    rd2dg = 1/dg2rd;
    mm2m = 10^(-3);
    Nm2Nmm = 1e3;

    %% GEOMETRIC PARAMETERS
    a1s = 65*dg2rd; sa1s = sin(a1s); ca1s = cos(a1s);
    a2s = 60*dg2rd; sa2s = sin(a2s); ca2s = cos(a2s);
    b1s = 0*dg2rd; sb1s = sin(b1s); cb1s = cos(b1s);
    b2s = 110*dg2rd; sb2s = sin(b2s); cb2s = cos(b2s);

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


    PSI = EA(1);
    THETA = EA(2);
    PHI = EA(3);
    Qm = Rz(PSI)*Ry(THETA)*Rx(PHI);
    v1v = Qm*v1v_str;
    v2v = Qm*v2v_str;
    v3v = Qm*v3v_str;
    Vm = [v1v,v2v,v3v];


    %% -----W vectors ------
    theta1 = thetav(1);
    theta2 = thetav(2);
    theta3 = thetav(3);
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
    Wm = [w1v,w2v,w3v];
    Um = [u1v,u2v,u3v];
end

function [RX] = Rx(y)

    RX= [1 0 0; 0 cos(y) -sin(y); 0 sin(y) cos(y);];


end

function [RY] = Ry(z)

    RY=[cos(z) 0 sin(z); 0 1 0; -sin(z) 0 cos(z);];

end

function [RZ] = Rz(x)

    RZ=[cos(x) -sin(x) 0; sin(x) cos(x) 0; 0 0 1;];

end

