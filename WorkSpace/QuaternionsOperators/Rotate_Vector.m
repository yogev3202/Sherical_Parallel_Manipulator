function x = Rotate_Vector(q,x0)

w1 = q1(4);
x1 = q1(1);
y1 = q1(2);
z1 = q1(3);

qinv = [-ev;qs];


w2 = qinv(4);
x2 = qinv(1);
y2 = qinv(2);
z2 = qinv(3);

xq = [0;x0];

xw = xq(4);
xx = xq(1);
xy = xq(2);
xz = xq(3);


 result1 = [w1*w2 - x1*x2 - y1*y2 - z1*z2;
              w1*x2 + x1*w2 + y1*z2 - z1*y2;
              w1*y2 - x1*z2 + y1*w2 + z1*x2;
              w1*z2 + x1*y2 - y1*x2 + z1*w2];


x = 


end

