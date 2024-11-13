%% Initialize Data
function [input, data , Ref] = InitData(settings, Time)

    nx      = settings.nx;       % No. of differential states
    nu      = settings.nu;       % No. of controls
    nz      = settings.nz;       % No. of algebraic states
    ny      = settings.ny;        % No. of outputs (references)    
    nyN     = settings.nyN;       % No. of outputs at terminal stage 
    np      = settings.np;        % No. of parameters (on-line data)
    nc      = settings.nc;        % No. of constraints
    ncN     = settings.ncN;      % No. of constraints at terminal stage
    N       = settings.N;         % No. of shooting points
    nbx     = settings.nbx;      % No. of state bounds
    nbu     = settings.nbu;      % No. of control bounds
    nbu_idx = settings.nbu_idx;  % Index of control bounds
    X0 = settings.x0;

    switch settings.model
                      
        case 'Kinematic_Model'
               
            input.x0 = X0;
            input.u0 = zeros(nu,1); 
            input.z0 = zeros(nz,1);
            input.para0 = 0;  
            
            Qx = [ones(1,3) * 2;ones(1,3) * 600];
            Qu = ones(1,3) * 0.01;
            Q = repmat([Qx Qu]',1,N);
            QN = 3*Qx';

            % upper and lower bounds for states (=nbx)
            lb_x = [];
            ub_x =[];

            % upper and lower bounds for controls (=nbu)           
            lb_u = -500 * (pi/180) * ones(3,1);
            ub_u = 500 * (pi/180) * ones(3,1);
                       
            % upper and lower bounds for general constraints (=nc)
            lb_g = 0.1;
            ub_g = 1e5;            
            lb_gN = 0.1;
            ub_gN = 1e5;


        case 'Dynamic_Model'

            input.x0 = X0;
            input.u0 = zeros(nu,1); 
            input.z0 = zeros(nz,1);
            [r,dr,ddr,ur] = RefGenerator(Time,'dynamic');
            para0 = [r;dr;ddr;ur];  
            input.para0 = para0; 
            % para0 = 0;  
            % input.para0 = para0;             
            
            Qx1 = ones(1,3) * 600;
            Qx2 = ones(1,3) * 0.05;
            Qu = ones(1,3) * 0.1;
            Q = repmat([Qx1 Qx2 Qu]',1,N);
            QN = 3*[Qx1 Qx2]';

            % upper and lower bounds for states (=nbx)
            % lb_x = -100 * (pi/180) * ones(3,1);
            % ub_x =  100 * (pi/180) * ones(3,1);

            lb_x = -[1000;60;60;200;200;200] * (pi/180);
            ub_x =  [1000;60;60;200;200;200] * (pi/180);         

            % upper and lower bounds for controls (=nbu)           
            lb_u = -20*1e-3 * ones(3,1);
            ub_u = 20*1e-3 * ones(3,1);
                       
            % upper and lower bounds for general constraints (=nc)
            lb_g = [0.1*ones(3,1);0.1];
            ub_g = [2*ones(3,1);10000];    
            lb_gN = [0.1*ones(3,1);0.1];
            ub_gN = [2*ones(3,1);10000];  

        case 'Dynamic_Model_V2' 


            input.x0 = X0;
            input.u0 = zeros(nu,1); 
            input.z0 = zeros(nz,1);
            [r,dr,ddr,ur] = RefGenerator(Time,'dynamic');
            para0 = [r;dr;ddr];  
            input.para0 = para0; 
            
            Qx1 = ones(1,3) * 200;
            Qx2 = ones(1,3) * 0.05;
            Qu = ones(1,3) * 0.1;
            % Q = repmat([Qx1 Qx2 50*Qx1 50*Qx2 Qu]',1,N);
            % QN = 0*[Qx1 Qx2 Qx1 Qx2]';
            Q = repmat([Qx1 Qx2 Qu]',1,N);
            QN = 3*[Qx1 Qx2]';            

            % upper and lower bounds for states (=nbx)
            L = -[1000;50;50;300;300;300] * (pi/180);
            U =  [1000;50;50;300;300;300] * (pi/180); 

            lb_x = L;
            ub_x = U ;            

            % upper and lower bounds for controls (=nbu)           
            lb_u = -20*1e-3 * ones(3,1);
            ub_u = 20*1e-3 * ones(3,1);
                       
            % upper and lower bounds for general constraints (=nc)
  
            lb_g = [0.1*ones(3,1);0.1] ;
            ub_g = [2*ones(3,1);10]  ;    
            lb_gN = [0.1*ones(3,1);0.1] ;
            ub_gN = [2*ones(3,1);10] ;   


        case 'Kinematic_Model_V2'
          
            input.x0 = X0;
            input.u0 = zeros(nu,1); 
            input.z0 = zeros(nz,1);
            [r,dr,ddr,ur] = RefGenerator(Time,'kinematic');
            para0 = [r;dr];  
            input.para0 = para0; 
            
            Qx1 = [ones(1,3) * 200];
            Qu = ones(1,3) * 0.01;
            Q = repmat([Qx1 Qu]',1,N);
            QN = 3*Qx1';            

            % upper and lower bounds for states (=nbx)
            L = -[200;60;60] * (pi/180);
            U =  [200;60;60] * (pi/180); 

            lb_x = L;
            ub_x = U ;            

            % upper and lower bounds for controls (=nbu)           
            lb_u = -200 * ones(3,1) * (pi/180);
            ub_u = 200 * ones(3,1) * (pi/180);
                       
            % upper and lower bounds for general constraints (=nc)
  
            lb_g = [0.1*ones(3,1);0.05] ;
            ub_g = [2*ones(3,1);1000]  ;    
            lb_gN = [0.1*ones(3,1);0.05] ;
            ub_gN = [2*ones(3,1);1000] ;   
      
                                
    end

    % Check initial values dimensions
    assert( ...
        size(input.x0, 1) == nx, ...
        'input.x0 is not consistent with nx' ...
    )
    assert( ...
        size(input.u0, 1) == nu, ...
        'input.u0 is not consistent with nu' ...
    )
    assert( ...
        size(input.z0, 1) == nz, ...
        'input.z0 is not consistent with nz' ...
    )

    assert( ...
        size(Q, 1) == ny, ...
        'Q is not consistent with nz' ...
    )
    assert( ...
        size(QN, 1) == nyN, ...
        'QN is not consistent with nz' ...
    )

    % Check bounds dimensions
    assert( ...
        size(lb_g, 1) == size(ub_g, 1), ...
        'Lower and upper bound vectors are not consistent (size(lb_g) != size(ub_g))' ...
    )
    assert( ...
        size(lb_g, 1) == nc, ...
        'lb_g is not consistent with nc' ...
    )
    if np~=0
        assert( ...  
            size(para0, 1) == np, ...
            'The given parameters vector is not of the defined dimension (size(para0) != np)' ...
        )
    end

    % prepare the data
    
    input.lb = repmat(lb_g,N,1);
    input.ub = repmat(ub_g,N,1);
    input.lb = [input.lb;lb_gN];
    input.ub = [input.ub;ub_gN];
            
    lbu = -inf(nu,1);
    ubu = inf(nu,1);
    for i=1:nbu
        lbu(nbu_idx(i)) = lb_u(i);
        ubu(nbu_idx(i)) = ub_u(i);
    end
                
    input.lbu = repmat(lbu,1,N);
    input.ubu = repmat(ubu,1,N);
    
    input.lbx = repmat(lb_x,1,N);
    input.ubx = repmat(ub_x,1,N);
        
    x = repmat(input.x0,1,N+1);  % initialize all shooting points with the same initial state
    u = repmat(input.u0,1,N);    % initialize all controls with the same initial control
    z = repmat(input.z0,1,N);    % initialize all algebraic state with the same initial condition
    para = repmat(para0,1,N+1);  % initialize all parameters with the same initial para
         
    input.x=x;           % (nx by N+1)
    input.u=u;           % (nu by N)
    input.z=z;           % (nz by N)
    input.od=para;       % (np by N+1)
    input.W=Q;           % (ny by N)
    input.WN=QN;         % (nyN by 1)
     
    input.lambda=zeros(nx,N+1);   % langrangian multiplier w.r.t. equality constraints
    input.mu=zeros(N*nc+ncN,1);   % langrangian multipliers w.r.t. general inequality constraints
    input.mu_u = zeros(N*nu,1);   % langrangian multipliers w.r.t. input bounds
    input.mu_x = zeros(N*nbx,1);  % langrangian multipliers w.r.t. state bounds
    
    %% Reference generation

    switch settings.model

        case 'Kinematic_Model'
            [EA,u] = RefGenerator(Time);
            data.REF = [EA',u'];
            Ref.Ref_x = EA';
            Ref.Ref_u = u';

        case 'Dynamic_Model'
            % [r,dr,ddr,ur] = RefGenerator(Time,'dynamic');
            data.REF=[-r',-dr',ur']*0;       
            Ref.Ref_x = [r',dr']*0;
            Ref.Ref_u = ur'; 

        case 'Dynamic_Model_V2' 

            data.REF=[r'*0,dr'*0,ur'];       
            Ref.Ref_x = [r'*0,dr'*0];
            Ref.Ref_u = ur';  

        case 'Kinematic_Model_V2'     
            data.REF=[r'*0,ur'];       
            Ref.Ref_x = [r'*0,dr'*0];
            Ref.Ref_u = ur';              

        case 'Err_Kinematic_Model'

         
            data.REF = [zeros(1,3),ur'];
            Ref.Ref_x = zeros(1,3)';
            Ref.Ref_u = ur';

        case 'Err_Dynamic_Model'

            data.REF=[r'*0,dr'*0,ur'];       
            Ref.Ref_x = [r',dr']*0;
            Ref.Ref_u = ur'; 


        case 'Err_Dynamic_Model_V2'

            data.REF = [zeros(1,9),ur'];
            Ref.Ref_x = zeros(1,9);
            Ref.Ref_u = ur';

       case 'Err_temp'

            % data.REF=[r',dr',r'*0,dr'*0,ur'];       
            % Ref.Ref_x = [r',dr',r'*0,dr'*0];
            % Ref.Ref_u = ur'; 

            data.REF=[r'*0,dr'*0,ur'];       
            Ref.Ref_x = [r'*0,dr'*0];
            Ref.Ref_u = ur';             

      
    end
    
end