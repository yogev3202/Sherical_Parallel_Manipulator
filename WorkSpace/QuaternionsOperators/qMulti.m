function q = qMulti(q1,q2)

ev1 = q1(1:3);
qs1 = q1(4);

ev2 = q2(1:3);
qs2 = q2(4);

q = [qs1*ev2+qs2*ev1;qs1*qs2-ev1'*ev2];


end

