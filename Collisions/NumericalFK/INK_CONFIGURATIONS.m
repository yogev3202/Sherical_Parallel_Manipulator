function INK_CONFIGURATIONS(EA,mode,count)

{'+++','++-','+-+','-++','+--','-+-','--+','---'};


TITLE = {'(+,+,+)','(+,+,-)','(+,-,+)',...
    '(-,+,+)','(+,-,-)','(-,+,-)','(-,-,+)','(-,-,-)'};
t = tiledlayout(2,4,'TileSpacing','Compact');

for i = 1:1:8
    
    THETA = SPMIK_Fcn(EA,mode);
    [Um,Wm,Vm] = unit_vectors(EA,THETA);

 
    Fig = nexttile;
    hold on
%     set(gca,'XTick',[], 'YTick', [],'ZTick', []);
    SPM_Visual(Um,Wm,Vm,EA);
    title(TITLE{count});
    Fig = gca;
    Fig.XTickLabel = [];
    Fig.YTickLabel = [];
    Fig.ZTickLabel = [];
    
    Fig.XLabel.String = '';
    Fig.YLabel.String = '';
    Fig.ZLabel.String = '';

end

end

