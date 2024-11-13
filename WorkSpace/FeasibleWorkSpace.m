
%% Workspace Bounds


%% part 1: theta bounds

%theta upper bounds
N = 180/0.1;
Grid  = linspace(0,90,N);

for i = 1:1:N
    
        psi = 0*dg2rd;
        theta = Grid(i)*dg2rd;
        phi = 0*dg2rd;
        x = [psi;theta;phi];
        [detA,detB,CN] = Singularity(x);
        if (detA == 0 || ~isreal(detA))
            THEAT_ub = theta;
            break;
        elseif (detB == 0 || ~isreal(detB))   
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
        [detA,detB,CN] = Singularity(x);
        if (detA == 0 || ~isreal(detA))
            THEAT_lb = theta;
            break;
        elseif (detB == 0 || ~isreal(detB))   
            THEAT_lb = theta;
            break;
        end  
end

theta_bound = [THEAT_ub;THEAT_lb];


%% part 2: phi bounds

%theta upper bounds
N = 180/0.1;
Grid  = linspace(0,90,N);

for i = 1:1:N
    
        psi = 0*dg2rd;
        theta = 0*dg2rd;
        phi = Grid(i)*dg2rd;
        x = [psi;theta;phi];
        [detA,detB,CN] = Singularity(x);
        if (detA == 0 || ~isreal(detA))
            PHI_ub = phi;
            break;
        elseif (detB == 0 || ~isreal(detB))   
            PHI_ub = phi;
            break;
        end  
end

%theta lower bounds
N = 180/0.1;
Grid  = linspace(-90,0,N);

for i = N:-1:1
    
        psi = 0*dg2rd;
        theta = 0*dg2rd;
        phi = Grid(i)*dg2rd;
        x = [psi;theta;phi];
        [detA,detB,CN] = Singularity(x);
        if (detA == 0 || ~isreal(detA))
            PHI_lb = phi;
            break;
        elseif (detB == 0 || ~isreal(detB))   
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

N = max(N_theta,N_phi);


Grid_theta = linspace(THEAT_lb,THEAT_ub,N);
Grid_phi = linspace(PHI_lb,PHI_ub,N);


[X,Y] = meshgrid(Grid_phi,Grid_theta);



[N1,N2] = size(X);


for  i = 1:1:N
    for j = 1:1:N
        psi = 0;
        theta = Y(i,j);
        phi = X(i,j);
        x = [psi;theta;phi];
        [detA,detB,CN] = Singularity(x);
        
%         if (detA ~= 0 && detB ~= 0 && isreal(detA) && isreal(detB))
%             EA_WS.EAp = [EA_WS.EAp,x];
%         end
        
        if (detA > 0 && detB > 0 && isreal(detA) && isreal(detB))
            EA_WS.EAp = [EA_WS.EAp,x];
        end
        
        if (detA == 0 || ~isreal(detA))
            EA_WS.EA_SK = [EA_WS.EA_SK,x];
        end
        
        if (detB == 0 || ~isreal(detB))
            EA_WS.EA_FK = [EA_WS.EA_FK,x];
        end 
       
    end
       
end

%% Save Data 
% varNames = {'theta','phi','detA','CN'};
% Data = table(WS.theta, WS.phi  , WS.EA_SK , WS.CN , 'VariableNames',varNames);
% save('Data.mat','Data');


