%% part 1: theta bounds
dg2rd = pi/180;
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

%theta upper bounds
% N = 180/0.1;
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

%theta lower bounds
% N = 180/0.1;
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