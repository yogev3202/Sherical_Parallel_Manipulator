
% input : quaternion
% output : Euler Angles 321

function EA = q2EA(q)

q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);


 DCM =[


    q4^2+q1^2-q2^2-q3^2   ,    2*(q1*q2+q3*q4)   ,   2*(q1*q3-q2*q4);
    2*(q1*q2-q3*q4)       ,    q4^2-q1^2+q2^2-q3^2  , 2*(q2*q3+q1*q4);
    2*(q1*q3+q2*q4)       ,    2*(q2*q3-q1*q4)     , q4^2-q1^2-q2^2+q3^2 

 

    ];

D11 = DCM(1,1);D12 = DCM(1,2);D13 = DCM(1,3);
D21 = DCM(2,1);D22 = DCM(2,2);D23 = DCM(2,3);
D31 = DCM(3,1);D32 = DCM(3,2);D33 = DCM(3,3);

psi = atan(D12/D11);
theta = -asin(D13);
phi = atan(D23/D33);
EA = [psi,theta,phi];





end

