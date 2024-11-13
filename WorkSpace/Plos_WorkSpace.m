


% Min_theta_axis = min(EA_BOX(2,:)*(180/pi));
% Max_theta_axis = max(EA_BOX(2,:)*(180/pi));
% 
% Min_phi_axis = min(EA_BOX(3,:)*(180/pi));
% Max_phi_axis = max(EA_BOX(3,:)*(180/pi));

Min_theta_axis = min(EA(2,:)*(180/pi));
Max_theta_axis = max(EA(2,:)*(180/pi));

Min_phi_axis = min(EA(3,:)*(180/pi));
Max_phi_axis = max(EA(3,:)*(180/pi));

Ver1 = [Min_theta_axis,Max_phi_axis];
Ver2 = [Max_theta_axis,Max_phi_axis];
Ver3 = [Max_theta_axis,Min_phi_axis];
Ver4 = [Min_theta_axis,Min_phi_axis];



Fig1 = figure('Name','SPM Valid WorkSpace');
hold on
Fig1.WindowStyle = 'normal';
% Fig1.Color = 'r';
% Fig1.Marker = 'o';
% Fig1.MarkerSize = 1;

plot(EA(2,:)*(180/pi),EA(3,:)*(180/pi),'ro', ...
    'Marker','o','MarkerSize',1);

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





% figure('Name','Euclidian Woarkspace mapping');
% hold on;
% plot(EA_WS.EAp(2,:)*rd2dg,EA_WS.EAp(3,:)*rd2dg,'b*');
% plot(EA_WS.EA_SK(2,:)*rd2dg,EA_WS.EA_SK(3,:)*rd2dg,'g*');
% plot(EA_WS.EA_FK(2,:)*rd2dg,EA_WS.EA_FK(3,:)*rd2dg,'r*');
% plot(EA_WS.FK_SK(2,:)*rd2dg,EA_WS.FK_SK(3,:)*rd2dg,'m*');
% hold off
% legend('Feasible WS' , 'SK','FK','Location','northeast','NumColumns',2)
% legend('Feasible WS' , 'SK','Location','northeast','NumColumns',2)
% xlabel("\theta [deg]");
% ylabel("\phi [deg]");
% 
% 
% 
% 
% [X,Y,Z] = sphere(20) ;
% Fig = figure('Name','Spherical Woarkspace mapping');
% tiledlayout(1,2);
% tiledlayout("horizontal")
% nexttile;
% hold on;
% box on;
% nexttile;
% hold on;
% box on;
% ax = gca;
% ax.BoxStyle = 'full';
% SphereUnit = surf(X,Y,Z);
% view(3);
% SphereUnit.EdgeColor = 'k';
% SphereUnit.LineStyle = '--';
% SphereUnit.FaceColor = 'w';
% SphereUnit.Marker = 'none';
% SphereUnit.FaceLighting = 'none';
% Points1 = scatter3(EA_WS.LOS(1,:),EA_WS.LOS(2,:),EA_WS.LOS(3,:));
% Points1.MarkerEdgeColor = 'r';
% Points1.MarkerFaceColor = 'r';
% Points1.SizeData = 1;
% 
% nexttile;
% hold on;
% box on;
% ax = gca;
% ax.BoxStyle = 'full';
% SphereUnit = surf(X,Y,Z);
% view(3);
% SphereUnit.EdgeColor = 'k';
% SphereUnit.LineStyle = '--';
% SphereUnit.FaceColor = 'w';
% SphereUnit.Marker = 'none';
% SphereUnit.FaceLighting = 'none';
% Points1 = scatter3(EA_WS.LOS(1,:),EA_WS.LOS(2,:),EA_WS.LOS(3,:));
% Points1.MarkerEdgeColor = 'r';
% Points1.MarkerFaceColor = 'r';
% Points1.SizeData = 1;
% Points2 = scatter3(Sphere.Xvalue_FK,Sphere.Yvalue_FK,Sphere.Zvalue_FK);
% Points2.MarkerEdgeColor = 'b';
% Points2.MarkerFaceColor = 'b';
% Points2.SizeData = 1;
% Points3 = scatter3(Sphere.Xvalue_SK,Sphere.Yvalue_SK,Sphere.Zvalue_SK);
% Points3.MarkerEdgeColor = 'g';
% Points3.MarkerFaceColor = 'g';
% xlabel('x');ylabel('y');zlabel('z');
% 
% 
% figure('Name','Static Torques')
% subplot(3,1,1)
% plot(1:1:L,tau1,'r*');
% xlabel("Orientation");
% ylabel("Nmm");
% title("Leg_1");
% 
% subplot(3,1,2)
% plot(1:1:L,tau2,'g*');
% xlabel("Orientation");
% ylabel("Nmm");
% title("Leg_2");
% 
% subplot(3,1,3)
% plot(1:1:L,tau3,'b*');
% xlabel("Orientation");
% ylabel("Nmm");
% title("Leg_3")
% legend('Leg_1' , 'Leg_2','Leg_3','Location','northeast','NumColumns',2)
% 
% 
% 
% figure('Name','Condition Index')
% CI_Points = scatter3(EA_WS.EA(2,:)*(180/pi),EA_WS.EA(3,:)*(180/pi),EA_WS.CI);
% CI_Points.MarkerEdgeColor = 'b';
% CI_Points.MarkerFaceColor = 'b';
% CI_Points.SizeData = 1;
% 
% xlabel("\theta [deg]");
% ylabel("\phi [deg]");
% 
% 
% 
