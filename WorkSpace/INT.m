%% DEFINE GLOBAL PARAMETERS
global u1v u2v u3v v1v_str v2v_str v3v_str 
global a1s a2s b1s b2s ETA eta1s eta2s eta3s 
global se1s ce1s se2s ce2s se3s ce3s sa1s ca1s sa2s ca2s sb1s cb1s sb2s cb2s
global thetaZ0  dEA 

%% UNIT CONVERTION
dg2rd = pi/180;
rd2dg = 1/dg2rd;
epsilon = 0.05;

%% GEOMETRIC PARAMETERS
a1s = 65*dg2rd; sa1s = sin(a1s); ca1s = cos(a1s);
a2s = 60*dg2rd; sa2s = sin(a2s); ca2s = cos(a2s);
b1s = 0*dg2rd; sb1s = sin(b1s); cb1s = cos(b1s);
b2s = 110*dg2rd; sb2s = sin(b2s); cb2s = cos(b2s);
ETA = [0;240;120]*dg2rd;
eta1s = 0*dg2rd; se1s = sin(eta1s); ce1s = cos(eta1s);
eta2s = 120*dg2rd; se2s = sin(eta2s); ce2s = cos(eta2s);
eta3s = 240*dg2rd; se3s = sin(eta3s); ce3s = cos(eta3s);
thetaZ0 = -1.140243740887195*ones(3,1);
dEA = 0.2;

%% UNIT VECTORS CONSTANTS

u1v = [sb1s*se1s;sb1s*ce1s;-cb1s];
u2v = [sb1s*se2s;sb1s*ce2s;-cb1s];
u3v = [sb1s*se3s;sb1s*ce3s;-cb1s];
v1v_str = [sb2s*se1s;sb2s*ce1s;cb2s];
v2v_str = [sb2s*se2s;sb2s*ce2s;cb2s];
v3v_str = [sb2s*se3s;sb2s*ce3s;cb2s];

%% Workspace struct

EA_WS = struct;
EA_WS.EAp = [];
EA_WS.EAn = [];
EA_WS.EA_FK = [];
EA_WS.EA_SK = [];
EA_WS.LOS = [];








