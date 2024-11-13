clc;
clear;
close all

%% UNIT CONVERTION
dg2rd = pi/180;
rd2dg = 1/dg2rd;
mm2m = 10^(-3);
I2 = eye(2);
I3 = eye(3);
O3 = zeros(3);
I6 = eye(6);
Nm2Nmm = 1e3;

%% GEOMETRIC PARAMETERS
a1s = 65*dg2rd; sa1s = sin(a1s); ca1s = cos(a1s);
a2s = 60*dg2rd; sa2s = sin(a2s); ca2s = cos(a2s);
b1s = 0*dg2rd; sb1s = sin(b1s); cb1s = cos(b1s);
b2s = 110*dg2rd; sb2s = sin(b2s); cb2s = cos(b2s);
ETA = [0;240;120]*dg2rd;
eta1s = 0*dg2rd; se1s = sin(eta1s); ce1s = cos(eta1s);
eta2s = 240*dg2rd; se2s = sin(eta2s); ce2s = cos(eta2s);
eta3s = 120*dg2rd; se3s = sin(eta3s); ce3s = cos(eta3s);
gravity = [0;0;-9.80665];
% thetaZ0 = -1.140243740887195*ones(3,1);
% thetav = -1.140243740887195*ones(3,1);


% EA = [0;10;25]*dg2rd;
% THETAv = SPMIK_Fcn(EA,'---');
% [Um,Wm,Vm] = unit_vectors(EA,THETAv);
%% UNIT VECTORS CONSTANTS

THETAv = [-54;-78;-62]*(pi/180);


u1v = [sb1s*se1s;sb1s*ce1s;-cb1s];
u2v = [sb1s*se2s;sb1s*ce2s;-cb1s];
u3v = [sb1s*se3s;sb1s*ce3s;-cb1s];
v1v_str = [sb2s*se1s;sb2s*ce1s;cb2s];
v2v_str = [sb2s*se2s;sb2s*ce2s;cb2s];
v3v_str = [sb2s*se3s;sb2s*ce3s;cb2s];
VmSTR = [v1v_str,v2v_str,v3v_str];

VmSTR_IG = {[ 1,1,-1,-1,-1,-1,1,-1,-1 ],...
            [ -1,1,-1,1,-1,-1,1,-1,-1 ],...
            [ 1,1,-1,-1,1,-1,1,-1,-1 ],...
            [ -1,1,-1,-1,-1,1,1,-1,-1 ],...
            [ 1,1,-1,-1,-1,-1,-1,-1,1 ],...
            [ -1,1,-1,-1,-1,-1,1,-1,-1 ],...
            [ 1,1,-1,-1,-1,-1,1,-1,1 ],...
            [ 1,1,11,-1,-1,-1,1,1,-1 ]};
   


%% Plot Numerical FK Configurations

SOL = struct;
SOL.EA = {};
SOL.mode = {};
SOL.count = {};
SOL.THETA = {};

Case = {'+++','++-','+-+','-++','+--','-+-','--+','---'};

for i = 1:1:8
    
    IG = VmSTR_IG{i};
    x_old{i} = [IG';THETAv];
    Sol{i} = FKP(x_old{i});
    v1v = Sol{i}(1:3); v2v = Sol{i}(4:6); v3v = Sol{i}(7:9);
    Vm{i} = [v1v,v2v,v3v];
    Qm{i} = Vm{i}*VmSTR^(-1);
    EA{i} = DCM2EA(Qm{i});
    
    for j = 1:1:8
        
           THETAv_IK = SPMIK_Fcn(EA{i},Case{j})
           if (norm(THETAv_IK-THETAv) <= 1e-5)      
               SOL.EA{i} = EA{i};
               SOL.mode{i} = Case{j};
               SOL.count{i} = j;
               SOL.THETA{i} = THETAv_IK;            
               break;
           end
           
    end
    

end


 INK_CONFIGURATIONS(EA{i},Case{j},j);


% x = FKP(x_old);
% v1v = x(1:3); v2v = x(4:6); v3v = x(7:9);
% Qm = [v1v,v2v,v3v]*[v1v_str,v2v_str,v3v_str]^(-1);







% EA = DCM2EA(Qm);
% 
% % THETAv = [-54;-78;-62];
% Case = {'+++','++-','+-+','-++','+--','-+-','--+','---'};
% 
% for i = 1:1:8
%     
%     THETAv = SPMIK_Fcn(EA,Case{i});
%     [Um,Wm,Vm] = unit_vectors(EA,THETAv);
%     
%     sol = THETAv*(180/pi)
%     
% end




