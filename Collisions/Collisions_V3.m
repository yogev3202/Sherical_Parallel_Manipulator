function Flag = Collisions_V3(x,OP_Dis)


%% UNIT CONVERTION
dg2rd = pi/180;
%% DEFINE PARAMETERS

%% GEOMETRIC PARAMETERS
a1s = 65*dg2rd; sa1s = sin(a1s); ca1s = cos(a1s);
a2s = 60*dg2rd; sa2s = sin(a2s); ca2s = cos(a2s);
b1s = 0*dg2rd; sb1s = sin(b1s); cb1s = cos(b1s);
b2s = 110*dg2rd; sb2s = sin(b2s); cb2s = cos(b2s);
ETA = [0;240;120]*dg2rd;
eta1s = 0*dg2rd; se1s = sin(eta1s); ce1s = cos(eta1s);
eta2s = 240*dg2rd; se2s = sin(eta2s); ce2s = cos(eta2s);
eta3s = 120*dg2rd; se3s = sin(eta3s); ce3s = cos(eta3s);

%% UNIT VECTORS CONSTANTS

u1v = [sb1s*se1s;sb1s*ce1s;-cb1s];
u2v = [sb1s*se2s;sb1s*ce2s;-cb1s];
u3v = [sb1s*se3s;sb1s*ce3s;-cb1s];
Um = [u1v,u2v,u3v];

v1v_str = [sb2s*se1s;sb2s*ce1s;cb2s];
v2v_str = [sb2s*se2s;sb2s*ce2s;cb2s];
v3v_str = [sb2s*se3s;sb2s*ce3s;cb2s];

x1 = x(1);
x2 = x(2);
x3 = x(3);

%% -------------------------Inverse Kenematic---------------------------------
%% Platform
RX= [1 0 0; 0 cos(x3) -sin(x3); 0 sin(x3) cos(x3);];
RY=[cos(x2) 0 sin(x2); 0 1 0; -sin(x2) 0 cos(x2);];
RZ=[cos(x1) -sin(x1) 0; sin(x1) cos(x1) 0; 0 0 1;];
Qm = RZ*RY*RX;
v1v = Qm*v1v_str;
v2v = Qm*v2v_str;
v3v = Qm*v3v_str;
Vm = [v1v,v2v,v3v];

%% LEG1
A1_1 = sin(a1s-b1s)*(se1s*v1v(1)+ce1s*v1v(2))+cos(a1s-b1s)*v1v(3)+ca2s;
B1_1 = sa1s*(ce1s*v1v(1)-se1s*v1v(2));
C1_1 = -sin(a1s+b1s)*(se1s*v1v(1)+ce1s*v1v(2))+cos(a1s+b1s)*(v1v(3))+ca2s;
D1_1 = B1_1^2-A1_1*C1_1;
T12_1 = (-B1_1-sqrt(D1_1))/A1_1;
theta1_m = 2*atan(T12_1);
theta1 = theta1_m;

%% LEG2
A2_1 = sin(a1s-b1s)*(se2s*v2v(1)+ce2s*v2v(2))+cos(a1s-b1s)*v2v(3)+ca2s;
B2_1 = sa1s*(ce2s*v2v(1)-se2s*v2v(2));
C2_1 = -sin(a1s+b1s)*(se2s*v2v(1)+ce2s*v2v(2))+cos(a1s+b1s)*(v2v(3))+ca2s;
D2_1 = B2_1^2-A2_1*C2_1;
T22_1 = (-B2_1-sqrt(D2_1))/A2_1;
theta2_m = 2*atan(T22_1);
theta2 = theta2_m;


%% LEG3
A3_1 = sin(a1s-b1s)*(se3s*v3v(1)+ce3s*v3v(2))+cos(a1s-b1s)*v3v(3)+ca2s;
B3_1 = sa1s*(ce3s*v3v(1)-se3s*v3v(2));
C3_1 = -sin(a1s+b1s)*(se3s*v3v(1)+ce3s*v3v(2))+cos(a1s+b1s)*(v3v(3))+ca2s;
D3_1= B3_1^2-A3_1*C3_1;
T32_1 = (-B3_1-sqrt(D3_1))/A3_1;
theta3_m = 2*atan(T32_1);
theta3 = theta3_m;

thetav = [theta1;theta2;theta3];

%% ------------------------- Unit Vectors---------------------------------
% v1v = Qm*v1v_str;
% v2v = Qm*v2v_str;
% v3v = Qm*v3v_str;


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

%% Link Interference Detection

OA = 38.4405289169 * 1e-3;
OB = 35.17 * 1e-3;
OC = 29.75 * 1e-3;
OD = 22 * 1e-3;
OE = 24.31 * 1e-3;
OF = 27.19 * 1e-3;
Diameter = 4;
Delta = (0.5*Diameter + 2) * 1e-3;

for i = 1:1:3
     
    uiv = Um(:,i); wiv = Wm(:,i); viv = Vm(:,i);
    
    Ai = uiv*OA; Bi = ((uiv+wiv)/norm((uiv+wiv)))*OB; Ci = wiv*OC;
    Di = wiv*OD; Ei = ((viv+wiv)/norm((viv+wiv)))*OE; Fi = viv*OF;
    Li1 = Bi-Ai; Li2 = Ci-Bi; Li3 = Ei-Di; Li4 = Fi-Ei;
    
    Ap(:,i) = Ai; Bp(:,i) = Bi; Cp(:,i) = Ci;
    Dp(:,i) = Di; Ep(:,i) = Ei; Fp(:,i) = Fi;
    
    L1(:,i) = Li1; L2(:,i) = Li2; 
    L3(:,i) = Li3; L4(:,i) = Li4; 
    

    
    
    
    
end


Ap1 = Ap(:,1); Ap2 = Ap(:,2); Ap3 = Ap(:,3);
Bp1 = Bp(:,1); Bp2 = Bp(:,2); Bp3 = Bp(:,3);
Cp1 = Cp(:,1); Cp2 = Cp(:,2); Cp3 = Cp(:,3);
Dp1 = Dp(:,1); Dp2 = Dp(:,2); Dp3 = Dp(:,3);
Ep1 = Ep(:,1); Ep2 = Ep(:,2); Ep3 = Ep(:,3);
Fp1 = Fp(:,1); Fp2 = Fp(:,2); Fp3 = Fp(:,3);

l_11 = L1(:,1); l_12 = L2(:,1); l_13 = L3(:,1); l_14 = L4(:,1);
l_21 = L1(:,2); l_22 = L2(:,2); l_23 = L3(:,2); l_24 = L4(:,2);
l_31 = L1(:,3); l_32 = L2(:,3); l_33 = L3(:,3); l_34 = L4(:,3);


% Leg 1 -----------------------------------

L11 = struct;
L11.Line = l_11;
% L11.Xa = (Ap1+Bp1)*0.5; L11.Xb = Bp1;
L11.Xa = Ap1; L11.Xb = Bp1;

L12 = struct;
L12.Line = l_12;
L12.Xa = Bp1; L12.Xb = Cp1;

L13 = struct;
L13.Line = l_13;
L13.Xa = Dp1; L13.Xb = Ep1;

L14 = struct;
L14.Line = l_14;
L14.Xa = Ep1; L14.Xb = Fp1;

Leg1 = {L11,L12,L13,L14};

% Leg 2 -----------------------------------

L21 = struct;
L21.Line = l_21;
% L21.Xa = (Ap2+Bp2)*0.5; L21.Xb = Bp2;
L21.Xa = Ap2; L21.Xb = Bp2;

L22 = struct;
L22.Line = l_22;
L22.Xa = Bp2; L22.Xb = Cp2;

L23 = struct;
L23.Line = l_23;
L23.Xa = Dp2; L23.Xb = Ep2;

L24 = struct;
L24.Line = l_24;
L24.Xa = Ep2; L24.Xb = Fp2;

Leg2 = {L21,L22,L23,L24};

% Leg 3 -----------------------------------


L31 = struct;
L31.Line = l_31;
% L31.Xa = (Ap3+Bp3)*0.5; L31.Xb = Bp3;
L31.Xa = Ap3; L31.Xb = Bp3;

L32 = struct;
L32.Line = l_32;
L32.Xa = Bp3; L32.Xb = Cp3;

L33 = struct;
L33.Line = l_33;
L33.Xa = Dp3; L33.Xb = Ep3;

L34 = struct;
L34.Line = l_34;
L34.Xa = Ep3; L34.Xb = Fp3;

Segments_Leg1 = {L11,L12,L13,L14};
Segments_Leg2 = {L21,L22,L23,L24};
Segments_Leg3 = {L31,L32,L33,L34};

%% Ouadratic Optimization Problem


alpha0 = 0.2;
Int = [alpha0;alpha0];
Flag = false;
Epsilon_min = 1e-4;
Epsilon_max = 2 * ((0.5*Diameter + 2) * 1e-3);

for i = 1:1:4
   for j = 1:1:4
      for k = 1:1:4
          Seg_Leg1 = Segments_Leg1{i};
          Seg_Leg2 = Segments_Leg2{j};
          Seg_Leg3 = Segments_Leg3{k};
          S1xa = Seg_Leg1.Xa; S1xb = Seg_Leg1.Xb;
          S2xa = Seg_Leg2.Xa; S2xb = Seg_Leg2.Xb;
          S3xa = Seg_Leg3.Xa; S3xb = Seg_Leg3.Xb;
          
          lbx = [0;0];
          ubx = [1;1];
          PARAMS12 = [S1xa;S1xb;S2xa;S2xb];
          PARAMS13 = [S1xa;S1xb;S3xa;S3xb];
          PARAMS23 = [S2xa;S2xb;S3xa;S3xb];
          % Collision Leg1 Leg2 -----------------
          Optimal_sol = OP_Dis('x0', Int,'lbx' , lbx , 'ubx' , ubx ,'p',PARAMS12);
          A1_sol12 = full(Optimal_sol.x(1));
          A2_sol12 = full(Optimal_sol.x(2));
          Seg1 = A1_sol12*S1xa + (1-A1_sol12)*S1xb;
          Seg2 = A2_sol12*S2xa + (1-A2_sol12)*S2xb;
          Distance12 = norm(Seg1-Seg2);
          
          % Collision Leg1 Leg3 -----------------
          Optimal_sol = OP_Dis('x0', Int,'lbx' , lbx , 'ubx' , ubx ,'p',PARAMS13);
          A1_sol13 = full(Optimal_sol.x(1));
          A2_sol13 = full(Optimal_sol.x(2));
          Seg1 = A1_sol13*S1xa + (1-A1_sol13)*S1xb;
          Seg2 = A2_sol13*S3xa + (1-A2_sol13)*S3xb;
          Distance13 = norm(Seg1-Seg2);
          
          % Collision Leg2 Leg3 -----------------
          Optimal_sol = OP_Dis('x0', Int,'lbx' , lbx , 'ubx' , ubx ,'p',PARAMS23);
          A1_sol23 = full(Optimal_sol.x(1));
          A2_sol23 = full(Optimal_sol.x(2));
          Seg1 = A1_sol23*S2xa + (1-A1_sol23)*S2xb;
          Seg2 = A2_sol23*S3xa + (1-A2_sol23)*S3xb;
          Distance23 = norm(Seg1-Seg2);
          
          % Check Collison
          
          if ( Distance12 <= Epsilon_max && Distance12 > Epsilon_min )
              Flag = true;
              return;
          elseif ( Distance13 <= Epsilon_max && Distance13 > Epsilon_min )
              Flag = true;
              return;
          elseif ( Distance23 <= Epsilon_max && Distance23 > Epsilon_min )
              Flag = true;
              return;
          end
          
      end  
   end

end




end

