function [r,dr,ddr,ur] = RefGenerator(time,mode)
  
D2R = pi/180;
%% ----------------------------------------------------
%% Fixed Target Frame Scenarios
%% ----------------------------------------------------


r     = [0 ; 0 ; 0] * D2R;
dr    = [0 ; 0 ; 0] * D2R;
ddr   = [0 ; 0 ; 0] * D2R;

% r     = [50 ; 0 ; 30] * D2R;
% dr    = [0 ; 0 ; 0] * D2R;
% ddr   = [0 ; 0 ; 0] * D2R;


% f = 2;
% w = 2*pi*f;
% A = 15;
% r     = [2*A*square(w*time)*0 ; A*square(w*time) ; 0] * D2R ;
% dr    = [0 ; 0 ; 0] * D2R;
% ddr   = [0 ; 0 ; 0] * D2R;

%% ----------------------------------------------------
%% Moivng Target Frame Scenarios
%% ----------------------------------------------------


% r     = [150*time ; 0 ; 20] * D2R;
% dr    = [150 ; 0 ; 0] * D2R;
% ddr   = [0 ; 0 ; 0] * D2R;

% A = 15;
% f = 0.5;
% w = 2*pi*f;
% r     = [0 ; 0 ; A*sin(w*time)] * D2R;
% dr    = [0 ; 0 ; A*w*cos(w*time)] * D2R;
% ddr   = [0 ; 0 ; -A*w^2*sin(w*time)] * D2R;

% Ax = 15;
% fx = 0.5;
% wx = 2*pi*fx;
% 
% Ay = 15;
% fy = 0.5;
% wy = 2*pi*fy;
% 
% Az = 40;
% fz = 0.8;
% wz = 2*pi*fz;
% 
% r     = [Az*sin(wz*time) ; 0 ; Ax*sin(wx*time)] * D2R;
% dr    = [Az*wz*cos(wz*time) ; 0 ; Ax*wx*cos(wx*time)] * D2R;
% ddr   = [-Az*wz^2*sin(wz*time) ; 0 ; -Ax*wx^2*sin(wx*time)] * D2R;


%% ----------------------------------------------------
%% Singular Target Frame Scenarios
%% ----------------------------------------------------

% A = 60;
% f = 0.5;
% w = 2*pi*f;
% r     = [0 ; A*sin(w*time) ; 0] * D2R;
% dr    = [0 ; A*w*cos(w*time) ; 0] * D2R;
% ddr   = [0 ;-A*w^2*sin(w*time) ; 0] * D2R;



%% -----------------------------------

switch mode

    case 'kinematic'
        ur = SPMDIK_Fcn(r,dr); 

    case 'dynamic'
        ur = SPMID_Fcn(r,dr,ddr);

end


 
end

