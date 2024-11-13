function  WS_Visual_V2(WS1,WS2,Case)


THETA1 = WS1.EA(2,:)*(180/pi);
PHI1 = WS1.EA(3,:)*(180/pi);

THETA2 = WS2.EA(2,:)*(180/pi);
PHI2 = WS2.EA(3,:)*(180/pi);


nexttile
hold on
S1 = scatter(THETA1,PHI1,40,'ro','filled');
S2 = scatter(THETA2,PHI2,60,'gs');
% legend("with collision","without collision",'Location','northwestoutside');
ax = gca;
ax.XLabel.String{1} = '\theta [deg]';
ax.XLabel.String{2} = Case;
ax.YLabel.String = '\phi [deg]';

S1.MarkerFaceAlpha = 1;
% S1.MarkerEdgeAlpha = 1;
% S1.MarkerFaceAlpha = 'flat';
% S1.MarkerFaceColor  = 'r';

% 'filled'
% S2.MarkerFaceAlpha = 0.8;
% S2.MarkerFaceAlpha = 0.1;
S2.MarkerEdgeColor = 'g';
S2.MarkerEdgeAlpha = 0.6;
% S2.AlphaData = 0.1;

end

