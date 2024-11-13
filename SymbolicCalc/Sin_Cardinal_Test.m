

clc
clear all
close all

Index = (2*pi)/(0.1*pi/180);
Grid = round(Index);
epsilon = 1e-9;

X = linspace(-pi,pi,Grid);
N = length(X);

for k = 1:1:N

    X1 = X(k);
    X2 = X(k);
    % Y1(k) = pi*(X1/pi)*sinc(X1/pi,epsilon);
    Y1(k) = SinCardinal(X1,epsilon);
    Y2(k) = sinc(X2);
    Y3(k) = Y1(k) - Y2(k);

end


figure(1)
subplot(2,1,1)
hold on
plot(X,Y1,'r');
plot(X,Y2,'b');
xlabel('x');
ylabel('y')
legend('F1','F2');

subplot(2,1,2)
hold on
plot(X,Y3,'r');
xlabel('x');
ylabel('error');


