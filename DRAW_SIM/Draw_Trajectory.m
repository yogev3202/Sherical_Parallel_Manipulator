

R2D = 180/pi;

load('WorkSpace_Data.mat');
Min_theta_axis = min(EA(2,:)*R2D);
Max_theta_axis = max(EA(2,:)*R2D);

Min_phi_axis = min(EA(3,:)*R2D);
Max_phi_axis = max(EA(3,:)*R2D);

Ver1 = [Min_theta_axis,Max_phi_axis];
Ver2 = [Max_theta_axis,Max_phi_axis];
Ver3 = [Max_theta_axis,Min_phi_axis];
Ver4 = [Min_theta_axis,Min_phi_axis];


figure('Name','Trajectoy')
hold on

plot(state_sim(:,2)*R2D,state_sim(:,3)*R2D, 'k--','LineWidth',2);


% plot(EA(2,:)*(180/pi),EA(3,:)*(180/pi),'ro', ...
%     'Marker','o','MarkerSize',1);

plot(EA_BOX(2,:)*(180/pi),EA_BOX(3,:)*(180/pi),'b+', ...
    'Marker','+','MarkerSize',1);

plot(EA_SINGULAR(2,:)*(180/pi),EA_SINGULAR(3,:)*(180/pi),'g+', ...
    'Marker','+','MarkerSize',1);

plot([Ver1(1),Ver2(1)],[Ver1(2),Ver2(2)],'k','LineWidth',2);
plot([Ver2(1),Ver3(1)],[Ver2(2),Ver3(2)],'k','LineWidth',2);
plot([Ver3(1),Ver4(1)],[Ver3(2),Ver4(2)],'k','LineWidth',2);
plot([Ver4(1),Ver1(1)],[Ver4(2),Ver1(2)],'k','LineWidth',2);

xlabel('\theta [deg]');
ylabel('\phi [deg]');





