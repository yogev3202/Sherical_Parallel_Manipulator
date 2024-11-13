function q_inv = qInverse(q)

qx = q(1);
qy = q(2);
qz = q(3);
qs = q(4);

N = qx^2+qy^2+qz^2+qs^2;

q_inv = (1/N)*[-qx;-qy;-qz;qs];



end

