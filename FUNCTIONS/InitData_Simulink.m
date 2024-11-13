%% Initialize Data
function [input, data , Ref] = InitData_Simulink(settings, Params)

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
    X0      = settings.x0;

    qr    = Params(1:3);
    dqr   = Params(4:6);
    ddqr  = Params(7:9);
    ur    = Params(10:12);

 
    state0 = X0;
    u0 = zeros(nu,1); 
    z0 = zeros(nz,1);
    para0 = [qr;dqr;ddqr];  
    
    
    Qx1 = ones(1,3) * 1000;
    Qx2 = ones(1,3) * 0.05;
    Qz  = ones(1,3) * 0;
    Qu = ones(1,3) * 20;
    Q = repmat([Qx1 Qx2 Qz Qu]',1,N);
    QN = 3*[Qx1 Qx2 Qz]';
    
    % upper and lower bounds for states (=nbx)
    
    Xlb = -300*(pi/180) * ones(3,1);
    Xub = 300*(pi/180) * ones(3,1);
    Alpha_ub = Xub - dqr;
    Alpha_lb = Xlb - dqr;
    lb_x = Alpha_lb;
    ub_x =  Alpha_ub;
    
    % upper and lower bounds for controls (=nbu)           
    lb_u = -100*1e-3 * ones(3,1);
    ub_u = 100*1e-3 * ones(3,1);
               
    % upper and lower bounds for general constraints (=nc)
    lb_g = [0.1];
    ub_g = [1e5];            
    lb_gN = lb_g;
    ub_gN = ub_g;              

                                


    % Check initial values dimensions
    assert( ...
        size(state0, 1) == nx, ...
        'input.x0 is not consistent with nx' ...
    )
    assert( ...
        size(u0, 1) == nu, ...
        'input.u0 is not consistent with nu' ...
    )
    assert( ...
        size(z0, 1) == nz, ...
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
    
    lbu = -inf(nu,1);
    ubu = inf(nu,1);
    for i=1:nbu
        lbu(nbu_idx(i)) = lb_u(i);
        ubu(nbu_idx(i)) = ub_u(i);
    end
                
    lbu = repmat(lbu,1,N);
    ubu = repmat(ubu,1,N);
    
    lbx = repmat(lb_x,1,N);
    ubx = repmat(ub_x,1,N);
        
    x = repmat(state0,1,N+1);  % initialize all shooting points with the same initial state
    u = repmat(u0,1,N);    % initialize all controls with the same initial control
    z = repmat(z0,1,N);    % initialize all algebraic state with the same initial condition
    para = repmat(para0,1,N+1);  % initialize all parameters with the same initial para
         
    input.x=x;           % (nx by N+1)
    input.u=u;           % (nu by N)
    input.z=z;           % (nz by N)
    input.od=para;       % (np by N+1)
    input.W=Q;           % (ny by N)
    input.WN=QN;         % (nyN by 1)
    input.lbu = repmat(lbu,1,N);
    input.ubu = repmat(ubu,1,N);
    input.lbx = repmat(lb_x,1,N);
    input.ubx = repmat(ub_x,1,N);

    input.lb = repmat(lb_g,N+1,1);
    input.ub = repmat(ub_g,N+1,1);
     
    input.lambda=zeros(nx,N+1);   % langrangian multiplier w.r.t. equality constraints
    input.mu=zeros(N*nc+ncN,1);   % langrangian multipliers w.r.t. general inequality constraints
    input.mu_u = zeros(N*nu,1);   % langrangian multipliers w.r.t. input bounds
    input.mu_x = zeros(N*nbx,1);  % langrangian multipliers w.r.t. state bounds
    
    %% Reference generation



    data.REF = [zeros(1,9),ur'];
    Ref.Ref_x = zeros(1,9);
    Ref.Ref_u = ur';

            

      

    
end