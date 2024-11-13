function [Uopt,settings] = QLPV_CTR(settings)

    lam_g0          = settings.lam_g;
    lam_x0          = settings.lam_x;
    Q               = settings.Q;
    Ts              = settings.Ts;
    x_int           = settings.x_int;
    u_int           = settings.u_int;
    xr              = settings.xr;
    ur              = settings.ur;
    Rho_0           = settings.Rho;
    QPGuess         = settings.QPGuess;
    N               = settings.N;
    nx              = settings.nx;
    nu              = settings.nu;
    Eps             = settings.Eps; 

    Refs = [xr;ur];
    RefsN = xr;

% % % Inputs = {'x0','Refs','Q','Ts','epsilon','Rho','Int_Guess','lam_x_initial'...
% % %     ,'lam_g_initial'};
% % 
% % % Outputs = {'x','f','g','lam_g','lam_x','lam_p'};
  
    k = 0;
    ErrTol = 1;
  
        while (k <= 8 && ErrTol>=1e-3)

            
            % tic;
            % [X,F,G,lamg,lamx,lamp] = GenMPC('MPC_step',QPGuess,Refs,RefsN,Ts,Q,QN,...
            %                              Rho_0,lbw,ubw,lbg,ubg,lam_x0,lam_g0);
            % CPT = toc*1e3;

% % % Inputs = {'x0','Refs','Q','Ts','epsilon','Rho','Int_Guess','lam_x_initial'...
% % %     ,'lam_g_initial'};
% % 
% % % Outputs = {'x','f','g','lam_g','lam_x','lam_p'};


            tic;
            [X,F,G,lamg,lamx,lamp] = GenMPC('MPC_step',x_int,Refs,Q,Ts,Eps,...
                                         Rho_0,QPGuess,lam_x0,lam_g0);
            CPT = toc*1e3;



            lam_x0 = lamx;
            lam_g0 = lamg;


            QPGuess = X(1:nu*N);
            %  x0,Uk,Rho,Ts
            xk = x_int;
            Xpred(1:6) = xk;
            for i = 1:1:N
                    x_range = (i-1)*nx+1:1:i*nx;
                    u_range = (i-1)*nu+1:1:i*nu;
                    uk = QPGuess(u_range);
                    dxdt = Dynamic_ODE_CT(xk,uk);
                    [T,Y] = ode45(@(T,Y) dxdt, [0 Ts], xk);
                    xkp1 = Y(end,:)';
                    xk = xkp1;
                    Xpred(x_range+nx) = xk;
   
            end
        
            St_pred = Xpred';
            Rho = Xpred';
            ErrTol = norm(Rho - Rho_0);
            Rho_0 = Rho;

            k = k+1;

   
        end

     settings.J = F;
     Upred = X(1:nu*N);
     Uopt = Upred(1:nu);






end

