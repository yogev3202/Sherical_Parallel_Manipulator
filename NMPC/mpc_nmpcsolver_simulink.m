function [input,mem,cpt,StopCrit,OBJ,QP_Flag] = mpc_nmpcsolver_simulink(input, settings, mem, opt)

    mem.sqp_it=0;
    mem.alpha =1;
    mem.obj=0;
    StopCrit = 2*mem.kkt_lim;
    % X0 = input.x0;
    % model_type = settings.model;
    tic;
    while(mem.sqp_it < mem.sqp_maxit  &&  StopCrit > mem.kkt_lim && mem.alpha>1E-8 )
        % if (strcmp(model_type,'Dynamic_Model_QLPV_V2') || ...
        %     strcmp(model_type,'Dynamic_Model_QLPV'))
        %     input.od(1:6,:) = [X0(1:6),input.x(1:6,2:end)];
        %     % input.od(1:6,:) = input.x(1:6,:);
        % end              
%% ----------- QP Preparation

        if strcmp(opt.qpsolver, 'qpoases_mb')
            qp_generation_mb(input, settings, mem);
        else
            qp_generation(input, settings, mem);
        end

 

        switch opt.condensing
             case 'default_full'  
                 if ~strcmp(opt.qpsolver, 'qpoases_mb')
                    Condensing(mem, settings);
                 else
                    Condensing_mb(mem, settings);
                 end
             case 'blasfeo_full'               
                 Condensing_Blasfeo(mem, settings);               
             case 'no'

         end

%% ----------  Solving QP

        switch opt.qpsolver
            case 'qpoases_sequence'              
                [~,mem] = mpc_qp_solve_qpoases_sequence(settings,mem);
            case 'qpoases'    
                [~,mem] = mpc_qp_solve_qpoases(settings,mem);
            case 'qpoases_mb'              
                [~,mem] = mpc_qp_solve_qpoases_mb(settings,mem, opt);
            case 'hpipm_sparse'               
                hpipm_sparse(mem,settings);
            case 'hpipm_pcond'                             
                hpipm_pcond(mem,settings);
            case 'ipopt_dense'               
                [~,mem] = mpc_qp_solve_ipopt_dense(settings,mem);
            case 'quadprog_dense'
                [~,mem] = mpc_qp_solve_quadprog(settings,mem);
            case 'ipopt_sparse'
                [~,mem] = mpc_qp_solve_ipopt_sparse(settings,mem);
            case 'ipopt_partial_sparse'
                [~, mem] = mpc_qp_solve_ipopt_partial_sparse(settings,mem.settings2,mem, mem.mem2);
            case 'osqp_sparse'
                [~, mem] = mpc_qp_solve_osqp(settings,mem);
            case 'osqp_partial_sparse'  
                [~, mem] = mpc_qp_solve_osqp_partial(settings,mem.settings2,mem,mem.mem2);
            case 'qpalm_cond'
                [~, mem] = mpc_qp_solve_qpalm_cond(settings,mem);
            case 'qpalm_sparse'
                [~, mem] = mpc_qp_solve_qpalm_sparse(settings,mem);
        end

%% ---------- Line search        

        Line_search(mem, input, settings);
%% ---------- KKT calculation 
        [eq_res, ineq_res, KKT, OBJ] = solution_info(input, settings, mem);

        StopCrit = max([eq_res, ineq_res, KKT]);

        mem.sqp_it=mem.sqp_it+1;
        QP_Flag = mem.QP_Flag;
    
    end
    cpt=toc*1e3;
    
end

