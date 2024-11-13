
clc;
clear;
close all



%% Workspace bounderies
WS_BOUND;

%% Genrate Grid of Euler Alngles

THETA = linspace(THEAT_lb,THEAT_ub,100);
PHI = linspace(PHI_lb,PHI_ub,100);
N = length(PHI);

WorkSpace = struct;
WorkSpace.EA_NoColl = [];
WorkSpace.EA_Coll = [];
WorkSpace.EA_Diff = [];

%% Classification of the Workspace Grid
FlagA = 0;
FlagB = 0;
FlagColl = 0;
Epsilon = 0.15;
Gamma = 0.0;
Diameter = 2;


%% classification with collisions Iner WorkSpace

for i = 1:1:N
    waitbar(i/N);
   for j = 1:1:N
       
        x = [0;THETA(i);PHI(j)];
       
        Dis = Discriminants(x);
        CI = ConditionIndex(x);
        D1 = Dis(1); D2 = Dis(2); D3 = Dis(3);
        FlagColl = Collisions_Mex_Fcn(x,Diameter);
        % FlagColl = 0;
        if (D1 >= Gamma && D2 >= Gamma && D3 >= Gamma && CI >= Epsilon && ~FlagColl)
               WorkSpace.EA_Coll = [WorkSpace.EA_Coll,x]; 
               FlagA = 0;
        else
                FlagA = 1;

        end

        if (D1 >= Gamma && D2 >= Gamma && D3 >= Gamma && CI >= Epsilon)
                WorkSpace.EA_NoColl = [WorkSpace.EA_NoColl,x];
                FlagB = 0;
        else 
                FlagB = 1;
        end    

        if (~FlagB && FlagColl == 1)

            WorkSpace.EA_Diff = [WorkSpace.EA_Diff,x];

        end
   end   
end


WS_Visual(WorkSpace);
% save('WorkSpace_Epsilon_0.15_Diameter_2.mat','WorkSpace');
% %% Plot WorkSpace
% Sim6 = struct;
% Sim6.WS1 = WorkSpace;
% Sim6.WS2 = WS2;
% save('WS_COL6.mat','Sim6');
% WS_Visual(WorkSpace,WS2);
% 
% 
% 
% 
% %% Load Data ---------------------
% 
% clc;
% clear;
% close all
% 
% figure('Position', [300, 150, 1200, 600])
% tiledlayout(1,3);
% % legend("with collision","without collision",'Location','northwestoutside');
% 
% 
% Case = {'\bf{(a)}','\bf{(b)}','\bf{(c)}'};
% load('WS_COL4.mat');
% load('WS_COL5.mat');
% load('WS_COL6.mat');
% 
% 
% 
% Sim1_WS1 = Sim4.WS1;
% Sim1_WS2 = Sim4.WS2;
% 
% Sim2_WS1 = Sim5.WS1;
% Sim2_WS2 = Sim5.WS2;
% 
% Sim3_WS1 = Sim6.WS1;
% Sim3_WS2 = Sim6.WS2;
% 
% Sim1Collect = {Sim1_WS1,Sim2_WS1,Sim3_WS1};
% Sim2Collect = {Sim1_WS2,Sim2_WS2,Sim3_WS2};
% 
% for i = 1:1:3
% 
% 
%     WorkSpace1 = Sim1Collect{i};
%     WorkSpace2 = Sim2Collect{i};
%     WS_Visual_V2(WorkSpace1,WorkSpace2,Case{i});
% 
% 
% end
% 
% % Sim1_WS1 = Sim1.Data1.WS1;
% % Sim1_WS2 = Sim1.Data1.WS2;
% % WS_Visual(Sim1_WS1,Sim1_WS2);
% 
% 
% 
% 
% legend("with collision","without collision",'Location','northeastoutside');





