
clc;
clear all
close all

%% -------------------------------------------
%% DATA COLLECTION
%% -------------------------------------------

EA = [];
EA_BOX = [];
EA_SINGULAR = [];


%% -------------------------------------------
%% Workspace Bounds
%% -------------------------------------------
dg2rd = pi/180;
%% part 1: theta bounds

%theta upper bounds
N = 180/0.1;
Grid  = linspace(0,90,N);

for i = 1:1:N
    
        psi = 0*dg2rd;
        theta = Grid(i)*dg2rd;
        phi = 0*dg2rd;
        x = [psi;theta;phi];
        Dis = Discriminants(x);
        D1 = Dis(1); D2 = Dis(2); D3 = Dis(3);

        if (D1 <= 0 || D2 <= 0 || D3 <= 0)
            THEAT_ub = theta;
            break;
        end  
end

%theta lower bounds
for i = 1:1:N
    
        psi = 0*dg2rd;
        theta = -Grid(i)*dg2rd;
        phi = 0*dg2rd;
        x = [psi;theta;phi];
        Dis = Discriminants(x);
        D1 = Dis(1); D2 = Dis(2); D3 = Dis(3);
        if (D1 <= 0 || D2 <= 0 || D3 <= 0)
            THEAT_lb = theta;
            break;
        end  
end

theta_bound = [THEAT_ub;THEAT_lb];


%% part 2: phi bounds

%phi upper bounds
N = 180/0.1;
Grid  = linspace(0,90,N);

for i = 1:1:N
    
        psi = 0*dg2rd;
        theta = 0*dg2rd;
        phi = Grid(i)*dg2rd;
        x = [psi;theta;phi];
        Dis = Discriminants(x);
        D1 = Dis(1); D2 = Dis(2); D3 = Dis(3);
        if (D1 <= 0 || D2 <= 0 || D3 <= 0)
            PHI_ub = phi;
            break;
        end  
end

%phi lower bounds
N = 180/0.1;
Grid  = linspace(-90,0,N);

for i = N:-1:1
    
        psi = 0*dg2rd;
        theta = 0*dg2rd;
        phi = Grid(i)*dg2rd;
        x = [psi;theta;phi];
        Dis = Discriminants(x);
        D1 = Dis(1); D2 = Dis(2); D3 = Dis(3);
        if (D1 <= 0 || D2 <= 0 || D3 <= 0)
            PHI_lb = phi;
            break;
        end  
end

phi_bound = [PHI_ub;PHI_lb];

%%  part 3: inner workspace
N_theta = (THEAT_ub - THEAT_lb)/(1*dg2rd);
N_phi = (PHI_ub - PHI_lb)/(1*dg2rd);

N_theta = round(N_theta);
N_phi = round(N_phi);

N = min(N_theta,N_phi);

THETA_axis = linspace(THEAT_lb,THEAT_ub,N);
PHI_axis   = linspace(PHI_lb,PHI_ub,N);

Gamma = 0.0;
Epsilon = 0.05;

for  i = 1:1:N
    for j = 1:1:N
        psi = 0;
        theta = THETA_axis(i);
        phi = PHI_axis(j);
        x = [psi;theta;phi];
        Dis = Discriminants(x);
        D1 = Dis(1); D2 = Dis(2); D3 = Dis(3);
        if (D1 >= Gamma && D2 >= Gamma && D3 >= Gamma)
            CI = ConditionIndex(x);
            if (CI >= Epsilon)
                EA = [EA,x];
            else
                EA_BOX = [EA_BOX,x];
            end
        end

        if ((D1 >= 0 && D1 <= Gamma) || (D2 >= 0 && D2 <= Gamma) ...
                || (D3 >= 0 && D3 <= Gamma))
            CI = ConditionIndex(x);
            % if (CI <= 0.05)
            %     EA_BOX = [EA_BOX,x];
            % else
            %     EA = [EA,x];
            % end
            EA_BOX = [EA_BOX,x];
        end

        if (D1 <= 0 || D2 <= 0 || D3 <= 0)
            EA_SINGULAR = [EA_SINGULAR,x];
        end        

       
    end
       
end

%% Save Data 

save('WorkSpace_Data.mat','EA','EA_BOX','EA_SINGULAR');



