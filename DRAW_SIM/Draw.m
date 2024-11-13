%% Plot results

TimeV = 0:Ts:time(end);


states = {'$\psi$', '$\theta$','$\phi$'};
Erros = {'$\bf{\tilde{\psi}}$', '$\tilde{\theta}$','$\tilde{\phi}$'};
controls = {'$u_1$', '$u_2$','$u_3$'};

figure(1)
    for i=1:length(states)
        subplot(3, 3, (i-1)*3+1);
        plot(TimeV, state_sim(:,i)*R2D); grid on;
        ylabel('[deg]');
        xlabel('[sec]');
        title(states{i},'interpreter','latex');

        subplot(3, 3, (i-1)*3+2);
        plot(TimeV, error_sim(:,i)*R2D); grid on;
        ylabel('[deg]');
        xlabel('[sec]');
        title(Erros{i},'interpreter','latex');

        subplot(3, 3, (i-1)*3+3);
        plot(TimeV, controls_MPC(:,i)*1000); grid on;
        ylabel('[Nmm]');
        xlabel('[sec]');
        title(controls{i},'interpreter','latex');
    end

    figure(2)
    subplot(2, 1, 1);
    plot(TimeV(1:end-1),OBJ,'b','linewidth',2);
    xlabel('[sec]');
    ylabel('Cost Value');

    subplot(2, 1, 2);
    plot(TimeV(1:end-1),CPT(:,1),'b','linewidth',2);
    xlabel('[sec]');
    ylabel('CPT [msec]');


% % start generating pictures
% switch settings.model
% 
% 
%     case 'Kinematic_Model'
% 
% 
%     case 'Dynamic_Model'
% 
% 
%     case 'Dynamic_Model_V2'    
% 
%         states = {'$\psi$', '$\theta$','$\phi$'};
%         Erros = {'$\bf{\tilde{\psi}}$', '$\tilde{\theta}$','$\tilde{\phi}$'};
%         controls = {'$u_1$', '$u_2$','$u_3$'};
% 
%         figure(1)
%         for i=1:length(states)
%             subplot(3, 3, (i-1)*3+1);
%             plot(TimeV, state_sim(:,i)*R2D); grid on;
%             ylabel('[deg]');
%             xlabel('[sec]');
%             title(states{i},'interpreter','latex');
% 
%             subplot(3, 3, (i-1)*3+2);
%             plot(TimeV, error_sim(:,i)*R2D); grid on;
%             ylabel('[deg]');
%             xlabel('[sec]');
%             title(Erros{i},'interpreter','latex');
% 
%             subplot(3, 3, (i-1)*3+3);
%             plot(TimeV, controls_MPC(:,i)*1000); grid on;
%             ylabel('[Nmm]');
%             xlabel('[sec]');
%             title(controls{i},'interpreter','latex');
%         end
% 
%         figure(2)
%         subplot(2, 1, 1);
%         plot(TimeV(1:end-1),OBJ,'b','linewidth',2);
%         xlabel('[sec]');
%         ylabel('Cost Value');
% 
%         subplot(2, 1, 2);
%         plot(TimeV(1:end-1),CPT(:,1),'b','linewidth',2);
%         xlabel('[sec]');
%         ylabel('CPT [msec]');
%     case 'Err_Kinematic_Model'
% 
%         states = {'$\psi$', '$\theta$','$\phi$'};
%         Erros = {'$\bf{\tilde{\psi}}$', '$\tilde{\theta}$','$\tilde{\phi}$'};
%         controls = {'$u_1$', '$u_2$','$u_3$'};
% 
%         figure(1)
%         for i=1:length(states)
%             subplot(3, 3, (i-1)*3+1);
%             plot(TimeV, state_sim(:,i)*R2D); grid on;
%             ylabel('[deg]');
%             xlabel('[sec]');
%             title(states{i},'interpreter','latex');
% 
%             subplot(3, 3, (i-1)*3+2);
%             plot(TimeV, error_sim(:,i)*R2D); grid on;
%             ylabel('[deg]');
%             xlabel('[sec]');
%             title(Erros{i},'interpreter','latex');
% 
%             subplot(3, 3, (i-1)*3+3);
%             plot(TimeV, controls_MPC(:,i)*R2D); grid on;
%             ylabel('[deg/sec]');
%             xlabel('[sec]');
%             title(controls{i},'interpreter','latex');
%         end
% 
%         figure(2)
%         subplot(2, 1, 1);
%         plot(TimeV(1:end-1),OBJ,'b','linewidth',2);
%         xlabel('[sec]');
%         ylabel('Cost Value');
% 
%         subplot(2, 1, 2);
%         plot(TimeV(1:end-1),CPT(:,1),'b','linewidth',2);
%         xlabel('[sec]');
%         ylabel('CPT [msec]');
% 
%     case 'Err_Dynamic_Model'
% 
%         states = {'$\psi$', '$\theta$','$\phi$'};
%         Erros = {'$\bf{\tilde{\psi}}$', '$\tilde{\theta}$','$\tilde{\phi}$'};
%         controls = {'$u_1$', '$u_2$','$u_3$'};
% 
%         figure(1)
%         for i=1:length(states)
%             subplot(3, 3, (i-1)*3+1);
%             plot(TimeV, state_sim(:,i)*R2D); grid on;
%             ylabel('[deg]');
%             xlabel('[sec]');
%             title(states{i},'interpreter','latex');
% 
%             subplot(3, 3, (i-1)*3+2);
%             plot(TimeV, error_sim(:,i)*R2D); grid on;
%             ylabel('[deg]');
%             xlabel('[sec]');
%             title(Erros{i},'interpreter','latex');
% 
%             subplot(3, 3, (i-1)*3+3);
%             plot(TimeV, controls_MPC(:,i)*1000); grid on;
%             ylabel('[Nmm]');
%             xlabel('[sec]');
%             title(controls{i},'interpreter','latex');
%         end
% 
%         figure(2)
%         subplot(2, 1, 1);
%         plot(TimeV(1:end-1),OBJ,'b','linewidth',2);
%         xlabel('[sec]');
%         ylabel('Cost Value');
% 
%         subplot(2, 1, 2);
%         plot(TimeV(1:end-1),CPT(:,1),'b','linewidth',2);
%         xlabel('[sec]');
%         ylabel('CPT [msec]');
% 
%     case 'Err_Dynamic_Model_V2'
% 
%         states = {'$\psi$', '$\theta$','$\phi$'};
%         Erros = {'$\bf{\tilde{\psi}}$', '$\tilde{\theta}$','$\tilde{\phi}$'};
%         controls = {'$u_1$', '$u_2$','$u_3$'};
% 
%         figure(1)
%         for i=1:length(states)
%             subplot(3, 3, (i-1)*3+1);
%             plot(TimeV, state_sim(:,i)*R2D); grid on;
%             ylabel('[deg]');
%             xlabel('[sec]');
%             title(states{i},'interpreter','latex');
% 
%             subplot(3, 3, (i-1)*3+2);
%             plot(TimeV, error_sim(:,i)*R2D); grid on;
%             ylabel('[deg]');
%             xlabel('[sec]');
%             title(Erros{i},'interpreter','latex');
% 
%             subplot(3, 3, (i-1)*3+3);
%             plot(TimeV, controls_MPC(:,i)*1000); grid on;
%             ylabel('[Nmm]');
%             xlabel('[sec]');
%             title(controls{i},'interpreter','latex');
%         end
% 
%         figure(2)
%         subplot(2, 1, 1);
%         plot(TimeV(1:end-1),OBJ,'b','linewidth',2);
%         xlabel('[sec]');
%         ylabel('Cost Value');
% 
%         subplot(2, 1, 2);
%         plot(TimeV(1:end-1),CPT(:,1),'b','linewidth',2);
%         xlabel('[sec]');
%         ylabel('CPT [msec]');
% 
% 
% 
% end