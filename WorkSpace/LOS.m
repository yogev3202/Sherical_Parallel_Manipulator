function L = LOS(x)


x1 = x(1);
x2 = x(2);
x3 = x(3);

RX= [1 0 0; 0 cos(x3) -sin(x3); 0 sin(x3) cos(x3);];
RY=[cos(x2) 0 sin(x2); 0 1 0; -sin(x2) 0 cos(x2);];
RZ=[cos(x1) -sin(x1) 0; sin(x1) cos(x1) 0; 0 0 1;];
Qm = RZ*RY*RX;

L = Qm*[0;0;1];

end

