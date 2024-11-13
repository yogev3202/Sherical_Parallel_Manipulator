
clc;
clear all
close all

Ts = 1e-3;
T0 = 0;
Tf = 4;

[r,dr,ddr,ur] = RefGenerator(0,'dynamic');

Xmdl_1 = [r;dr];
Xmdl_2 = [r;dr];
Err_Mdl = {Xmdl_2-Xmdl_1};
Mdl1    = {Xmdl_1};
Mdl2    = {Xmdl_2};

for time = T0:Ts:Tf

    [r,dr,ddr,ur] = RefGenerator(time,'dynamic');

    dXmdl_1 = spm_dynamic_ODEC(Xmdl_1,ur);
    dXmdl_2 = Dynamic_ODE_CT(Xmdl_2,ur);
    [~,y1] = ode45(@(t,y1) dXmdl_1, [0 Ts], Xmdl_1);
    [~,y2] = ode45(@(t,y2) dXmdl_2, [0 Ts], Xmdl_2);

    Xmdl_plus_1 = y1(end,:)';
    Xmdl_plus_2 = y2(end,:)';

    Xmdl_1 = Xmdl_plus_1;
    Xmdl_2 = Xmdl_plus_2;

    Err_k = Xmdl_2-Xmdl_1;

    Err_Mdl = {Err_Mdl{:},Err_k};
    Mdl1    = {Mdl1{:},Xmdl_1};
    Mdl2    = {Mdl2{:},Xmdl_2};

end




%% Plot Error Between Models

Err    = cell2mat(Err_Mdl);
Model1 = cell2mat(Mdl1);
Model2 = cell2mat(Mdl2);
Time = 0:Ts:Tf+Ts;

figure(1)
subplot(211)
hold on 
plot(Time,Model1(1,:)*(180/pi),'r');
plot(Time,Model1(2,:)*(180/pi),'g');
plot(Time,Model1(3,:)*(180/pi),'b');

plot(Time,Model2(1,:)*(180/pi),'k--');
plot(Time,Model2(2,:)*(180/pi),'k--');
plot(Time,Model2(3,:)*(180/pi),'k--');

xlabel('[sec]');
ylabel('[deg]');
legend('x1','x2','x3');

subplot(212)
hold on 
plot(Time,Model1(4,:)*(180/pi),'r');
plot(Time,Model1(5,:)*(180/pi),'g');
plot(Time,Model1(6,:)*(180/pi),'b');

plot(Time,Model2(4,:)*(180/pi),'k--');
plot(Time,Model2(5,:)*(180/pi),'k--');
plot(Time,Model2(6,:)*(180/pi),'k--');
xlabel('[sec]');
ylabel('[deg/sec]');
legend('x4','x5','x6');




figure(2)
subplot(211)
hold on 
plot(Time,Err(1,:)*(180/pi),'r');
plot(Time,Err(2,:)*(180/pi),'g');
plot(Time,Err(3,:)*(180/pi),'b');
xlabel('[sec]');
ylabel('[deg]');
legend('x1','x2','x3');

subplot(212)
hold on 
plot(Time,Err(4,:)*(180/pi),'r');
plot(Time,Err(5,:)*(180/pi),'g');
plot(Time,Err(6,:)*(180/pi),'b');
xlabel('[sec]');
ylabel('[deg/sec]');
legend('x4','x5','x6');





