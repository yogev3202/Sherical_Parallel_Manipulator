function D = q2DCM(q)

q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);


 D =[


    q4*q4+q1*q1-q2*q2-q3*q3    2*(q1*q2+q4*q3)         2*(q1*q3-q4*q2) 


    2*(q1*q2-q4*q3)         q4*q4-q1*q1+q2*q2-q3*q3    2*(q2*q3+q4*q1)


    2*(q1*q3+q4*q2)            2*(q2*q3-q4*q1)     q4*q4-q1*q1-q2*q2+q3*q3 


    ];




end

