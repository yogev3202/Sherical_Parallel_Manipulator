
% computes the Euler Angles Phi Theta Psi from the Rotation Operator Q
% according to the sequence 321
% y1 --- psi radians  [-pi,pi]
% y2 --- teta   [-pi/2,pi/2]
% y3 --- phi   [-pi,pi]
% Notice: the OUTPUT yv displays the angles in the order Psi Theta Phi

function EA = DCM2EA(DCM)
r11s= DCM(1,1);
r21s= DCM(2,1);
r31s= DCM(3,1);
r32s= DCM(3,2);
r33s= DCM(3,3);
r12s= DCM(1,2);
r22s= DCM(2,2);
dums= r11s^2+r21s^2;

if abs(r31s) ~= 1 
    y2= atan2(-r31s,sqrt(dums));
    y1= atan2(r21s,r11s);
    y3= atan2(r32s,r33s);
elseif r31s == -1
    y2= pi/2;
    y1= 0;
    y3= atan2(r12s,r22s);
elseif r31s == 1
    y2= -pi/2;
    y1= 0;
    y3= -atan2(r12s,r22s);
else
    disp('error in the input of the EA321Q function')
end
EA = [y1,y2,y3];

end

