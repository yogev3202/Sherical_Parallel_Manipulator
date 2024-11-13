
clc;
clear all
close all

%% Load Fixed Targer Frame Data

Sim_Type = 'FF04';

switch Sim_Type

    case 'FF01'

        SQP = importdata('FF01_SQP.mat');
        IPOPT = importdata('FF01_IPOPT.mat');
        QLPV = importdata('FF01_QLPV.mat');

    case 'FF02'

        SQP = importdata('FF02_SQP.mat');
        IPOPT = importdata('FF02_IPOPT.mat');
        QLPV = importdata('FF02_QLPV.mat');

    case 'FF03'

        SQP = importdata('FF03_SQP.mat');
        IPOPT = importdata('FF03_IPOPT.mat');
        QLPV = importdata('FF03_QLPV.mat');


    case 'FF04'

        SQP = importdata('FF04_SQP.mat');
        IPOPT = importdata('FF04_IPOPT.mat');
        QLPV = importdata('FF04_QLPV.mat');

    case 'FF01_Singular'

        SQP = importdata('FF01_SQP_Singular.mat');
        IPOPT = importdata('FF01_IPOPT_Singular.mat');
        QLPV = importdata('FF01_QLPV_Singular.mat');        

    case 'MF01'

        SQP = importdata('MF01_SQP.mat');
        IPOPT = importdata('MF01_IPOPT.mat');
        QLPV = importdata('MF01_QLPV.mat');

    case 'MF02'

        SQP = importdata('MF02_SQP.mat');
        IPOPT = importdata('MF02_IPOPT.mat');
        QLPV = importdata('MF02_QLPV.mat');

    case 'MF03'

        SQP = importdata('MF03_SQP.mat');
        IPOPT = importdata('MF03_IPOPT.mat');
        QLPV = importdata('MF03_QLPV.mat');

    case 'MF04'

        SQP = importdata('MF04_SQP.mat');
        IPOPT = importdata('MF04_IPOPT.mat');
        QLPV = importdata('MF04_QLPV.mat');        

    case 'MF01_Singular'

        SQP = importdata('MF01_SQP_Singular.mat');
        IPOPT = importdata('MF01_IPOPT_Singular.mat');
        QLPV = importdata('MF01_QLPV_Singular.mat');
        

end



% PLOT_SIMULATION(SQP,IPOPT,QLPV)


Solver_Type = {'SQP','IPOPT','QLPV'};
Solver_Data = {SQP,IPOPT,QLPV};
for k = 1:1:3  
    Data = Solver_Data{k};
    Solver = Solver_Type{k};
    PLOT_STATE(Data,Solver,Sim_Type)
    CPT_Mean(k) = sum(Data.MPC_Solver_Info.signals(1).values)/ ...
    length(Data.MPC_Solver_Info.signals(1).values);

    close all
end


XX = ["SQP" "IPOPT" "QLPV"];
YY = CPT_Mean;
bar(XX,YY)
ylabel('[msec]');

close all


% PLOT_STATE(QLPV,'QLPV','MF03');
