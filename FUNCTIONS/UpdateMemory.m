function [mem] = UpdateMemory(settings, opt, input , mem_old)

    Ts_st = settings.Ts_st;    % Shooting interval
    nx    = settings.nx;       % No. of differential states
    nu    = settings.nu;       % No. of controls
    nz    = settings.nz;       % No. of algebraic states
    ny    = settings.ny;       % No. of outputs (references)    
    nyN   = settings.nyN;      % No. of outputs at terminal stage 
    np    = settings.np;       % No. of parameters (on-line data)
    nbx   = settings.nbx;      % No. of bounds on states
    nbu   = settings.nbu;      % No. of bounds on controls
    nbx_idx = settings.nbx_idx; % indexes of states which are bounded
    nbu_idx = settings.nbu_idx; % indexes of controls which are bounded
    nc    = settings.nc;       % No. of general constraints
    ncN   = settings.ncN;      % No. of general constraints at terminal stage
    N     = settings.N;        % No. of shooting points
    N2    = settings.N2;       % No. of partial condensing blocks
    r     = settings.r;        % No. of input move blocks

    %% initialize memory
    mem = struct;
    mem.warm_start=0;
    mem.hot_start=0;
    
    if strcmp(opt.hotstart, 'yes')
        mem.hot_start=1;
    end

    % if strcmp(opt.warm_start, 'yes')
    %     mem.warm_start=1;
    % end
    
    if strcmp(opt.condensing,'partial_condensing')
        mem.settings2.N = N2;
        mem.settings2.nx = nx;
        mem.settings2.Nc = N/N2;
        mem.settings2.nu = mem.settings2.Nc*nu;
        mem.settings2.nbx = nbx;
        mem.settings2.nbx_idx = nbx_idx;
        
        if nbu>0
            mem.settings2.nbu_idx=[];
            for i=1:mem.settings2.Nc        
               mem.settings2.nbu_idx = [mem.settings2.nbu_idx,nbu_idx+(i-1)*nu];
            end
        else
            mem.settings2.nbu_idx=[];
        end
        
        mem.settings2.nc = nc*mem.settings2.Nc+(mem.settings2.Nc-1)*nbx;
        mem.settings2.ncN = ncN;
        
        % mem.mem2.Q = zeros(nx, (N2+1)*nx);
        % mem.mem2.S = zeros(nx, N2*mem.settings2.nu);
        % mem.mem2.R = zeros(mem.settings2.nu, N2*mem.settings2.nu);
        % mem.mem2.Cgx = zeros(mem.settings2.nc, N2*nx);
        % mem.mem2.Cgu = zeros(mem.settings2.nc, N2*mem.settings2.nu);
        % mem.mem2.gx = zeros(nx,N2+1);
        % mem.mem2.gu = zeros(mem.settings2.nu,N2);
        % mem.mem2.lb_dx = zeros(N2*nbx,1);
        % mem.mem2.ub_dx = zeros(N2*nbx,1);
        % mem.mem2.lc = zeros(mem.settings2.nc*N2+ncN,1);
        % mem.mem2.uc = zeros(mem.settings2.nc*N2+ncN,1);

        mem.mem2.Q = mem_old.Q;
        mem.mem2.S = mem_old.S;
        mem.mem2.R = mem_old.R;
        mem.mem2.Cgx = mem_old.Cgx;
        mem.mem2.Cgu = mem_old.Cgu;
        mem.mem2.gx = mem_old.gx;
        mem.mem2.gu = mem_old.gu;
        mem.mem2.lb_dx = mem_old.lb_dx;
        mem.mem2.ub_dx = mem_old.lb_du;
        mem.mem2.lc = mem_old.lc;
        mem.mem2.uc = mem_old.uc;       
        
    end
    
    % QP solver initialization
    switch opt.qpsolver
        case 'qpoases'   
            mem.qpoases_opt = qpOASES_options('MPC');
%             mem.qpoases_opt = qpOASES_options('default');
        case 'qpoases_mb'   
            mem.qpoases_opt = qpOASES_options('MPC');
%             mem.qpoases_opt = qpOASES_options('default');
        case 'quadprog_dense'
            mem.quadprog_opt.Algorithm = 'interior-point-convex';
            mem.quadprog_opt.Display = 'off';
            mem.quadprog_opt.OptimalityTolerance = 1e-6;
            mem.quadprog_opt.ConstraintTolerance = 1e-6;
            mem.quadprog_opt.StepTolerance = 1e-6;
        case 'hpipm_sparse'
            mem.mu0=1e4;
            mem.max_qp_it = 30;
            mem.pred_corr = 1;
            mem.cond_pred_corr = 1;
            mem.solver_mode = 1;
        case 'hpipm_pcond'
            mem.mu0=1e4;
            mem.max_qp_it = 30;
            mem.pred_corr = 1;
            mem.cond_pred_corr = 1;
            mem.solver_mode = 1;
        case 'ipopt_dense'
            ipopt_opts=ipoptset('constr_viol_tol',1e-3,'acceptable_tol',1e-3,'hessian_constant','yes',...
                        'mehrotra_algorithm','yes','mu_oracle','probing','jac_c_constant','yes',...
                        'jac_d_constant','yes','mu_strategy','adaptive','adaptive_mu_globalization',...
                        'never-monotone-mode','accept_every_trial_step','yes');
    
            mem.ipopt.options.eq=false(N*nc+ncN+N*nbx,1);
            mem.ipopt.options.ineq=true(N*nc+ncN+N*nbx,1);
            mem.ipopt.x0=input.u(:);
            
            mem.ipopt.options.nleq=[];
            mem.ipopt.options.nlineq=[];
            mem.ipopt.options.ipopt=ipopt_opts;
            mem.ipopt.options.ipopt.print_level=0;
            
            
        case 'ipopt_sparse'
            nw = (N+1)*nx+N*nu;
            neq = (N+1)*nx;
            nineq = N*nc+ncN;
            
            ipopt_opts=ipoptset('constr_viol_tol',1e-3,'acceptable_tol',1e-3,...
                        'hessian_constant','yes','jac_c_constant','yes','jac_d_constant','yes',...
                        'mehrotra_algorithm','yes',...
                        'mu_oracle','probing',...
                        'mu_strategy','adaptive',...
                        'adaptive_mu_globalization','never-monotone-mode',...
                        'accept_every_trial_step','yes');
    
            mem.ipopt.options.eq=[false(nineq,1);true(neq,1)];
            mem.ipopt.options.ineq=[true(nineq,1);false(neq,1)];
            mem.ipopt.x0=input.u(:);
            
            mem.ipopt.options.nleq=[];
            mem.ipopt.options.nlineq=[];
            mem.ipopt.options.ipopt=ipopt_opts;
            mem.ipopt.options.ipopt.print_level=0;
           
            mem.ipopt.options.ub = inf(nw,1);
            mem.ipopt.options.lb = -inf(nw,1);

            mem.sparse_H = zeros(nw,nw);
            mem.sparse_g = zeros(nw,1);
            mem.sparse_dG = zeros(neq,nw);  
            mem.sparse_dG(1:nx,1:nx) = eye(nx);
            mem.sparse_minus_eye = -eye(nx);
            mem.sparse_dB = zeros(nineq,nw);
            mem.sparse_dBx = zeros(N*nbx,nw);
            mem.sparse_dBu = zeros(N*nu,nw);
            mem.sparse_G = zeros(neq,1);
            mem.sparse_ub = inf(nineq+N*nbx+N*nu,1);
            mem.sparse_lb = -inf(nineq+N*nbx+N*nu,1);
            
        case 'ipopt_partial_sparse'
                                
            nw = (N2+1)*mem.settings2.nx+N2*mem.settings2.nu;
            neq = (N2+1)*mem.settings2.nx;
            nineq = N2*mem.settings2.nc+mem.settings2.ncN;

            ipopt_opts=ipoptset('constr_viol_tol',1e-3,'acceptable_tol',1e-3,'hessian_constant','yes',...
                                'mehrotra_algorithm','yes','mu_oracle','probing','jac_c_constant','yes',...
                                'jac_d_constant','yes','mu_strategy','adaptive','adaptive_mu_globalization',...
                                'never-monotone-mode','accept_every_trial_step','yes');

            mem.mem2.ipopt.options.eq=[false(nineq,1);true(neq,1)];
            mem.mem2.ipopt.options.ineq=[true(nineq,1);false(neq,1)];
            mem.mem2.ipopt.x0 = input.u(:);

            mem.mem2.ipopt.options.nleq=[];
            mem.mem2.ipopt.options.nlineq=[];
            mem.mem2.ipopt.options.ipopt=ipopt_opts;
            mem.mem2.ipopt.options.ipopt.print_level=0;

            mem.mem2.sparse_H = zeros(nw,nw);
            mem.mem2.sparse_g = zeros(nw,1);
            mem.mem2.sparse_dG = zeros(neq,nw);  
            mem.mem2.sparse_dG(1:nx,1:nx) = eye(nx);
            mem.mem2.sparse_minus_eye = -eye(nx);
            mem.mem2.sparse_dB = zeros(nineq,nw);
            mem.mem2.sparse_dBx = zeros(N2*nbx,nw);
            mem.mem2.sparse_dBu = zeros(N2*mem.settings2.nu,nw);
            mem.mem2.sparse_G = zeros(neq,1);
            mem.mem2.sparse_ub = inf(nineq+N2*nbx+N2*mem.settings2.nu,1);
            mem.mem2.sparse_lb = -inf(nineq+N2*nbx+N2*mem.settings2.nu,1);

            mem.mem2.ipopt.options.ub = input.ub;
            mem.mem2.ipopt.options.lb = input.lb;
                         
        case 'osqp_sparse'
            nw = (N+1)*nx+N*nu;
            neq = (N+1)*nx;
            nineq = N*nc+ncN;
            
            mem.qp_obj = osqp;
            mem.osqp_options = mem.qp_obj.default_settings();
            mem.osqp_options.eps_abs=1e-4;
            mem.osqp_options.eps_rel=1e-4;
            mem.osqp_options.polish = true;
            mem.osqp_options.verbose = false;
%             mem.osqp_options.warm_start = false;
            
            mem.sparse_H = zeros(nw,nw);
            mem.sparse_g = zeros(nw,1);
            mem.sparse_dG = zeros(neq,nw);  
            mem.sparse_dG(1:nx,1:nx) = eye(nx);
            mem.sparse_minus_eye = -eye(nx);
            mem.sparse_dB = zeros(nineq,nw);
            mem.sparse_dBx = zeros(N*nbx,nw);
            mem.sparse_dBu = zeros(N*nu,nw);
            mem.sparse_G = zeros(neq,1);
            mem.sparse_ub = inf(nineq+N*nbx+N*nu,1);
            mem.sparse_lb = -inf(nineq+N*nbx+N*nu,1);
            
            for i=0:N-1
                for j=1:nbx
                    mem.sparse_dBx(i*nbx+1:(i+1)*nbx, (i+1)*nx+nbx_idx(j)) = 1;
                end
            end
            mem.sparse_dBu(:,neq+1:end) = eye(N*nu,N*nu);
            
            osqp_H_pattern = zeros(nw,nw); 
            osqp_dG_pattern = zeros(neq,nw);  osqp_dG_pattern(1:nx,1:nx) = eye(nx);
            osqp_dB_pattern = zeros(nineq,nw);
            
            for i=0:N-1	
                osqp_H_pattern(i*nx+1:(i+1)*nx, i*nx+1:(i+1)*nx) = ones(nx,nx);	
                osqp_H_pattern(i*nx+1:(i+1)*nx, neq+i*nu+1:neq+(i+1)*nu) = ones(nx,nu);	
                osqp_H_pattern(neq+i*nu+1:neq+(i+1)*nu, i*nx+1:(i+1)*nx) = ones(nu,nx);	
                osqp_H_pattern(neq+i*nu+1:neq+(i+1)*nu, neq+i*nu+1:neq+(i+1)*nu) = ones(nu,nu);	
                	
                osqp_dG_pattern((i+1)*nx+1:(i+2)*nx, i*nx+1:(i+2)*nx) = [ones(nx,nx),-eye(nx,nx)];	
                osqp_dG_pattern((i+1)*nx+1:(i+2)*nx, neq+i*nu+1:neq+(i+1)*nu) = ones(nx,nu);	
                	
                osqp_dB_pattern(i*nc+1:(i+1)*nc, i*nx+1:(i+1)*nx) = ones(nc,nx);	
                osqp_dB_pattern(i*nc+1:(i+1)*nc, neq+i*nu+1:neq+(i+1)*nu) = ones(nc,nu);	
            end	
            osqp_H_pattern(N*nx+1:(N+1)*nx, N*nx+1:(N+1)*nx) = ones(nx,nx);	
            osqp_dB_pattern(N*nc+1:N*nc+ncN, N*nx+1:(N+1)*nx) = ones(ncN,nx);
            
            osqp_A_pattern = [osqp_dG_pattern; mem.sparse_dBu; mem.sparse_dBx; osqp_dB_pattern];
            
            mem.sparse_H_idx = triu(osqp_H_pattern)~=0;
            mem.sparse_A_idx = osqp_A_pattern~=0;

            mem.qp_obj.setup(osqp_H_pattern, [], osqp_A_pattern, -inf(size(osqp_A_pattern,1),1), inf(size(osqp_A_pattern,1),1), mem.osqp_options);
            
        case 'osqp_partial_sparse'
            nw = (N2+1)*mem.settings2.nx+N2*mem.settings2.nu;
            neq = (N2+1)*mem.settings2.nx;
            nineq = N2*mem.settings2.nc+mem.settings2.ncN;
            
            mem.mem2.qp_obj = osqp;
            mem.mem2.osqp_options =mem.mem2.qp_obj.default_settings();
            mem.mem2.osqp_options.eps_abs=1e-4;
            mem.mem2.osqp_options.eps_rel=1e-4;
            mem.mem2.osqp_options.max_iter = 1e4;
            mem.mem2.osqp_options.polish = true;
            mem.mem2.osqp_options.verbose = false;
            
            mem.mem2.sparse_H = zeros(nw,nw);
            mem.mem2.sparse_g = zeros(nw,1);
            mem.mem2.sparse_dG = zeros(neq,nw);  
            mem.mem2.sparse_dG(1:nx,1:nx) = eye(nx);
            mem.mem2.sparse_minus_eye = -eye(nx);
            mem.mem2.sparse_dB = zeros(nineq,nw);
            mem.mem2.sparse_dBx = zeros(N2*nbx,nw);
            mem.mem2.sparse_dBu = zeros(N2*mem.settings2.nu,nw);
            mem.mem2.sparse_G = zeros(neq,1);
            mem.mem2.sparse_ub = inf(nineq+N2*nbx+N2*mem.settings2.nu,1);
            mem.mem2.sparse_lb = -inf(nineq+N2*nbx+N2*mem.settings2.nu,1);
            
            for i=0:N2-1
                for j=1:nbx
                    mem.mem2.sparse_dBx(i*nbx+1:(i+1)*nbx, (i+1)*nx+nbx_idx(j)) = 1;
                end
            end
            mem.mem2.sparse_dBu(:,neq+1:end) = eye(N2*mem.settings2.nu,N2*mem.settings2.nu);
            
            osqp_H_pattern = zeros(nw,nw); 
            osqp_dG_pattern = zeros(neq,nw);  osqp_dG_pattern(1:nx,1:nx) = eye(nx);
            osqp_dB_pattern = zeros(nineq,nw);
            
            for i=0:N2-1	
                osqp_H_pattern(i*nx+1:(i+1)*nx, i*nx+1:(i+1)*nx) = ones(nx,nx);	
                osqp_H_pattern(i*nx+1:(i+1)*nx, neq+i*mem.settings2.nu+1:neq+(i+1)*mem.settings2.nu) = ones(nx,mem.settings2.nu);	
                osqp_H_pattern(neq+i*mem.settings2.nu+1:neq+(i+1)*mem.settings2.nu, i*nx+1:(i+1)*nx) = ones(mem.settings2.nu,nx);	
                osqp_H_pattern(neq+i*mem.settings2.nu+1:neq+(i+1)*mem.settings2.nu, neq+i*mem.settings2.nu+1:neq+(i+1)*mem.settings2.nu) = ones(mem.settings2.nu,mem.settings2.nu);	
                	
                osqp_dG_pattern((i+1)*nx+1:(i+2)*nx, i*nx+1:(i+2)*nx) = [ones(nx,nx),-eye(nx,nx)];	
                osqp_dG_pattern((i+1)*nx+1:(i+2)*nx, neq+i*mem.settings2.nu+1:neq+(i+1)*mem.settings2.nu) = ones(nx,mem.settings2.nu);	
                	
                osqp_dB_pattern(i*mem.settings2.nc+1:(i+1)*mem.settings2.nc, i*nx+1:(i+1)*nx) = ones(mem.settings2.nc,nx);	
                osqp_dB_pattern(i*mem.settings2.nc+1:(i+1)*mem.settings2.nc, neq+i*mem.settings2.nu+1:neq+(i+1)*mem.settings2.nu) = ones(mem.settings2.nc,mem.settings2.nu);	
            end	
            osqp_H_pattern(N2*nx+1:(N2+1)*nx, N2*nx+1:(N2+1)*nx) = ones(nx,nx);	
            osqp_dB_pattern(N2*mem.settings2.nc+1:N2*mem.settings2.nc+ncN, N2*nx+1:(N2+1)*nx) = ones(ncN,nx);
            
            osqp_A_pattern = [osqp_dG_pattern; mem.mem2.sparse_dBu; mem.mem2.sparse_dBx; osqp_dB_pattern];
            
            mem.mem2.sparse_H_idx = triu(osqp_H_pattern)~=0;
            mem.mem2.sparse_A_idx = osqp_A_pattern~=0;

            mem.mem2.qp_obj.setup(osqp_H_pattern, [], osqp_A_pattern, -inf(size(osqp_A_pattern,1),1), inf(size(osqp_A_pattern,1),1), mem.mem2.osqp_options);
            
        case 'qpalm_sparse'
            nw = (N+1)*nx+N*nu;
            neq = (N+1)*nx;
            nineq = N*nc+ncN;
                
            mem.sparse_H = zeros(nw,nw);
            mem.sparse_g = zeros(nw,1);
            mem.sparse_dG = zeros(neq,nw);  
            mem.sparse_dG(1:nx,1:nx) = eye(nx);
            mem.sparse_minus_eye = -eye(nx);
            mem.sparse_dB = zeros(nineq,nw);
            mem.sparse_dBx = zeros(N*nbx,nw);
            mem.sparse_dBu = zeros(N*nu,nw);
            mem.sparse_G = zeros(neq,1);
            mem.sparse_ub = inf(nineq+N*nbx+N*nu,1);
            mem.sparse_lb = -inf(nineq+N*nbx+N*nu,1);
            
            for i=0:N-1
                for j=1:nbx
                    mem.sparse_dBx(i*nbx+1:(i+1)*nbx, (i+1)*nx+nbx_idx(j)) = 1;
                end
            end
            mem.sparse_dBu(:,neq+1:end) = eye(N*nu,N*nu);
            
    end
    
    switch opt.hessian
        case 'Gauss_Newton'
            mem.hessian=0;
        case 'Generalized_Gauss_Newton'
            mem.hessian=1;
        otherwise
            error('Please choose a correct Hessian approximation');
    end
            
          
    switch opt.integrator
        case 'ERK4'
            mem.sim_method = 1;
            mem.A_tab=[0, 0, 0, 0;
                       0.5, 0, 0, 0;
                       0, 0.5, 0, 0;
                       0, 0, 1, 0];
            mem.B_tab=[1/6, 1/3, 1/3, 1/6];
            mem.num_steps = 2;
            mem.num_stages = 4;
            mem.h=Ts_st/mem.num_steps;
            mem.nx = nx;
            mem.nu = nu;
            mem.nz = nz;
            mem.Sx = eye(nx);
            mem.Su = zeros(nx,nu);
        case 'IRK3'
            mem.sim_method = 2;
            
            % Gauss-Legendre
            mem.A_tab=[5/36,             2/9-sqrt(15)/15, 5/36-sqrt(15)/30;
                       5/36+sqrt(15)/24, 2/9            , 5/36-sqrt(15)/24;
                       5/36+sqrt(15)/30, 2/9+sqrt(15)/15, 5/36];
            mem.B_tab=[5/18;4/9;5/18];

            % Radau IIA
%             mem.A_tab=[(88-7*sqrt(6))/360,    (296-169*sqrt(6))/1800, (-2+3*sqrt(6))/225;
%                        (296+169*sqrt(6))/1800, (88+7*sqrt(6))/360     (-2-3*sqrt(6))/225;
%                        (16-sqrt(6))/36,       (16+sqrt(6))/36,         1/9];
%             mem.B_tab=[(16-sqrt(6))/36;(16+sqrt(6))/36;1/9];
            mem.num_steps = 2;
            mem.num_stages = 3;
            mem.h= Ts_st/mem.num_steps;
            mem.nx = nx;
            mem.nu = nu;
            mem.nz = nz;
            mem.Sx = eye(nx);
            mem.Su = zeros(nx,nu);
            mem.newton_iter = 5;
            mem.JFK = mem.h*[mem.B_tab(1)*eye(nx,nx), mem.B_tab(2)*eye(nx,nx), mem.B_tab(3)*eye(nx,nx)];
            
        case 'IRK3-DAE'
            mem.sim_method = 3;
            
            % Radau IIA
            mem.A_tab=[(88-7*sqrt(6))/360,    (296-169*sqrt(6))/1800, (-2+3*sqrt(6))/225;
                       (296+169*sqrt(6))/1800, (88+7*sqrt(6))/360     (-2-3*sqrt(6))/225;
                       (16-sqrt(6))/36,       (16+sqrt(6))/36,         1/9];
            mem.B_tab=[(16-sqrt(6))/36;(16+sqrt(6))/36;1/9];
            mem.num_steps = 2;
            mem.num_stages = 3;
            mem.h= Ts_st/mem.num_steps;
            mem.nx = nx;
            mem.nu = nu;
            mem.nz = nz;
            mem.Sx = eye(nx);
            mem.Su = zeros(nx,nu);
            mem.newton_iter = 5;
            mem.JFK = mem.h*[mem.B_tab(1)*eye(nx,nx), zeros(nx,nz), mem.B_tab(2)*eye(nx,nx), zeros(nx,nz), mem.B_tab(3)*eye(nx,nx), zeros(nx,nz)];
        otherwise 
            error('Please choose a correct integrator');       
    end
    
    if nz>0 && ~strcmp(opt.integrator,'IRK3-DAE')
        error('Please choose IRK-DAE for DAE systems');
    end
    
    if nz==0 && strcmp(opt.integrator,'IRK3-DAE')
        error('Please do not choose IRK-DAE for ODE systems');
    end
    
    % globalization
    if strcmp(opt.RTI,'yes')
        mem.sqp_maxit = 2;         % use RTI
    else       
        mem.sqp_maxit = 6;      % maximum number of iterations for each sampling instant
    end
    mem.kkt_lim = 1e-4;          % tolerance on optimality
    mem.mu_merit=0;              % initialize the parameter
    mem.eta=1e-4;                % merit function parameter
    mem.tau=0.8;                 % step length damping factor
    mem.mu_safty=1.1;            % constraint weight update factor (for merit function)
    mem.rho=0.5;                 % merit function parameter
    mem.alpha=1;                 % default step length
    mem.obj=0;
    
    % allocate memory
    mem.A = mem_old.A;
    mem.B = mem_old.B;
    mem.Cx = mem_old.Cx;
    mem.Cgx = mem_old.Cgx;
    mem.Cgu = mem_old.Cgu;
    mem.CgN = mem_old.CgN;
    mem.gx = mem_old.gx;
    mem.gu = mem_old.gu;
    mem.a = mem_old.a;
    mem.ds0 = mem_old.ds0;
    mem.lc = mem_old.lc;
    mem.uc = mem_old.uc;
    mem.lb_du = mem_old.lb_du;
    mem.ub_du = mem_old.ub_du;
    mem.lb_dx = mem_old.lb_dx;
    mem.ub_dx = mem_old.ub_dx;
    
    mem.Hc = mem_old.Hc;
    mem.Ccx = mem_old.Ccx;
    mem.Ccg = mem_old.Ccg;
    mem.gc = mem_old.gc;
    mem.lcc = mem_old.lcc;
    mem.ucc = mem_old.ucc;
    mem.lxc = mem_old.lxc;
    mem.uxc = mem_old.uxc;
    
    if strcmp(opt.qpsolver, 'qpoases_mb')
        mem.r = r;
        mem.index_T = [0, 1, 3, 6, 10, 15, 20, 35, 50, 65, 80];
%         mem.index_T = [0, 1, 10, 50];
%         mem.index_T = [0 1 3 5 7 10 13 16 19 22 25 28 31 35 40 45 50];
        mem.Hc_r = zeros(mem.r*nu,mem.r*nu);
        mem.Ccx_r = zeros(N*nbx,mem.r*nu);
        mem.Ccg_r = zeros(N*nc+ncN,mem.r*nu);
        mem.gc_r = zeros(mem.r*nu,1);
        mem.L = zeros(mem.r,1);
        mem.Gr = zeros(N*nx,mem.r*nu);
        T = zeros(N,mem.r);
        for i=1:mem.r
            T(mem.index_T(i)+1:mem.index_T(i+1),i)=1;
        end
        mem.T = kron(T, eye(nu));
    end
    
    if opt.nonuniform_grid
        mem.r = r;
        mem.index_T = [0, 1, 3, 6, 10, 15, 20, 35, 50, 65, 80];
%         mem.index_T = [0, 1, 10, 50];
%         mem.index_T = [0 1 3 5 7 10 13 16 19 22 25 28 31 35 40 45 50];
        T = zeros(N,mem.r);
        for i=1:mem.r
            T(mem.index_T(i)+1:mem.index_T(i+1),i)=1;
        end
        mem.T = kron(T, eye(nu));
    end
    
    mem.dx = mem_old.dx;
    mem.du = mem_old.du;
    mem.lambda_new = mem_old.lambda_new;
    mem.mu_new = mem_old.mu_new;
    mem.mu_x_new = mem_old.mu_x_new;
    mem.mu_u_new = mem_old.mu_u_new;
    mem.z_out = mem_old.z_out;
    
    mem.q_dual = mem_old.q_dual;
    mem.dmu = mem_old.dmu;
    
    for i=1:nbx
        mem.Cx(i,nbx_idx(i)) = 1.0;
    end
    
    mem.reg = 1e-8;
    
    mem.Q = mem_old.Q;
    mem.S = mem_old.R;
    mem.R = mem_old.R;
                      
    mem.iter=mem_old.iter;
    
     %% for CMON-RTI
     
%     mem.F_old = zeros(nx,N);	
%     mem.CMON_pri = zeros(N,1);	
%     mem.CMON_dual = zeros(N,1);	
%     mem.q_dual = zeros(nx,N+1);	
%     mem.V_pri = zeros(nx,N);
%     mem.V_dual = zeros(nx+nu,N);
%     mem.dmu = zeros(N*nu+N*nbx+N*nc+ncN,1);
%     mem.threshold_pri = 0;	
%     mem.threshold_dual = 0;	
%     mem.tol=0;	
%     mem.perc=100;
%     mem.idxc=zeros(N,1);
%     
%     mem.tol_abs=1e-1;
%     mem.tol_ref=1e-1;  	       
%     mem.alpha_cmon = 1;      
%     mem.beta_cmon = 1;        
%     mem.c1 = 0.1;
%     mem.gamma = 0;	
%     mem.rho_cmon = 0;
%           
%     mem.local = 0;
%     
%     mem.rho_ratio=[];
%     mem.gamma_ratio=[];
%     
%     mem.r_ratio=[];
%     
%     mem.shift_x = zeros(nx,N+1);
%     mem.shift_u = zeros(nu,N);
%     
%     mem.Ns=25;
       
end

