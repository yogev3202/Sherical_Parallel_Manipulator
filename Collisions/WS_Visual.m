function  WS_Visual(WorkSpace)


THETA1 = WorkSpace.EA_NoColl(2,:)*(180/pi);
PHI1 = WorkSpace.EA_NoColl(3,:)*(180/pi);

THETA2 = WorkSpace.EA_Coll(2,:)*(180/pi);
PHI2 = WorkSpace.EA_Coll(3,:)*(180/pi);

THETA3 = WorkSpace.EA_Diff(2,:)*(180/pi);
PHI3 = WorkSpace.EA_Diff(3,:)*(180/pi);




figure('Position', [500, 250, 850, 400])
tiledlayout(1,2);

nexttile
hold on
S1 = scatter(THETA1,PHI1,'ro');
S2 = scatter(THETA2,PHI2,'g+');
hold off
ax = gca;
ax.XLabel.String{1} = '\theta [deg]';
ax.XLabel.String{2} = '\bf{(a)}';
ax.YLabel.String = '\phi [deg]';

S1.MarkerFaceAlpha = 1;
S1.MarkerEdgeAlpha = 1;

S2.MarkerFaceAlpha = 1;
S2.MarkerEdgeAlpha = 1;
legend("with collision","without collision",'Location','northwestoutside');
% axis equal


nexttile
plot(THETA3,PHI3,'b+');
ax = gca;
ax.XLabel.String{1} = '\theta [deg]';
ax.XLabel.String{2} = '\bf{(b)}';
ax.YLabel.String = '\phi [deg]';








% figure('Position', [500, 250, 850, 400])
% tiledlayout(1,3);
% 
% nexttile
% plot(THETA1,PHI1,'r+');
% title("without collision");
% ax = gca;
% ax.XLabel.String{1} = '\theta [deg]';
% ax.XLabel.String{2} = '\bf{(a)}';
% ax.YLabel.String = '\phi [deg]';
% 
% nexttile
% plot(THETA2,PHI2,'b+');
% title("with collision");
% ax = gca;
% ax.XLabel.String{1} = '\theta [deg]';
% ax.XLabel.String{2} = '\bf{(b)}';
% ax.YLabel.String = '\phi [deg]';
% 
% nexttile
% hold on
% S1 = scatter(THETA1,PHI1,'ro');
% S2 = scatter(THETA2,PHI2,'g+');
% % S1 = scatter(THETA3,PHI3,'r+');
% hold off
% % legend("with collision","without collision",'Location','northwestoutside');
% title("Difference");
% ax = gca;
% ax.XLabel.String{1} = '\theta [deg]';
% ax.XLabel.String{2} = '\bf{(c)}';
% ax.YLabel.String = '\phi [deg]';
% 
% S1.MarkerFaceAlpha = 1;
% S1.MarkerEdgeAlpha = 1;
% 
% S2.MarkerFaceAlpha = 1;
% S2.MarkerEdgeAlpha = 1;

end

