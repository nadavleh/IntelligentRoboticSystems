clc; clear all; close all;
data=dlmread('data.txt');
Time=data(:,1);
z=data(:,2);

% Initializing Parameters
T=10;
dt=0.1;
a=1.5;
sigma_a2=.25;
sigma2=1;
I=eye(2);

x_real(:,1)=[0;0];
x(:,1)=[0;0];     % x0 - initial state
P=zeros(2);       % P0 - initial covariance

A=[1 dt; 0 1];
B=[dt^2/2; dt];
u=a;
C=[1 0];
R=[dt^4/4 dt^3/2; dt^3/2 dt^2]*sigma_a2;
Q=sigma2;

% b)
figure(1);
plot(Time,z,'k',Time,smooth(z),'b'); grid on; hold on;
title('Position VS. Time');
xlabel('t[sec]'); ylabel('x[m]');
legend('Observations','Smoothed Observations','Location','SouthEast');

figure(2);
plot(Time(2:end),diff(z)/dt,'b',Time(2:end),diff(smooth(z))/dt,'c'); grid on; hold on;
title('Velocity VS. Time');
xlabel('t[sec]'); ylabel('$\dot{x}$[m]','Interpreter','latex');
legend('Velocity estimated by numerical derivations','Velocity estimated by numerical derivations on smoothed data','Location','SouthEast');

% c)
% Simulation:

% MC=1000;
% for m=1:MC

k=1;
for t=dt:dt:T
    x_real(:,k+1)=A*x_real(:,k) + B*u + mvnrnd([0;0],R)';
    k=k+1;
end

figure(1);
plot(Time,x_real(1,:),'r'); grid on; hold on;
legend('Observations','Smoothed Observations','Real position','Location','SouthEast');

figure(2);
plot(Time(2:end),x_real(2,2:end),'r'); grid on; hold on;
legend('Velocity estimated by numerical derivations','Velocity estimated by numerical derivations on smoothed data','Real velocity','Location','SouthEast');

% d)
% "z" is given ,else --> z=C*x_real + mvnrnd(0,Q);

k=1;
for t=dt:dt:T
    
% Predition
x(:,k+1)=A*x(:,k) + B*u ;       % state prediction (mean) x(k|k-1)
P = A*P*A' + R;                 % covarinace prediction P(k|k-1)

x_1_0(:,k)=x(:,k+1);% only for Plot
P_1_0(:,:,k)=P;     %   purpose

% Correction/Filtering
S = C*P*C' + Q;                 % innovation covarince
K = P*C' / S;                   % Kalman Gain
y = z(k+1) - C*x(:,k+1);        % innovation measurement
x(:,k+1) = x(:,k+1) + K*(y) ;   % state filtered (mean) x(k|k)
P = (I-K*C)*P*(I-K*C)' + K*Q*K';%P = (I- K*C)*P  % covariance filtered P(k|k)

x_1_1(:,k)=x(:,k+1);% only for Plot
P_1_1(:,:,k)=P;     %   purpose

k=k+1;
end

figure(1);
plot(Time,x(1,:),'g'); grid on; hold on;
legend('Observations','Smoothed Observations','Real position','position estimated by Kalman Filter','Location','SouthEast');

% figure(2);
% plot(Time(2:end),x(2,2:end),'g'); grid on; hold on;
% legend('Velocity estimated by numerical derivations','Velocity estimated by numerical derivations on smoothed data','Real velocity','velocity estimated by Kalman Filter','Location','SouthEast');

%e)
for i=[0 5 10-dt]
Ind=find(Time==i);
prediction=gmdistribution(x_1_0(1,Ind),P_1_0(1,1,Ind));
posterior=gmdistribution(x_1_1(1,Ind),P_1_1(1,1,Ind));
observation=gmdistribution(z(Ind+1),sigma2);

xx=x(1,Ind)-30:.0001:x(1,Ind)+30;
figure;
plot(xx,pdf(prediction,xx'),'g',xx,pdf(observation,xx'),'b',xx,pdf(posterior,xx'),'r',[x(1,Ind+1) x(1,Ind+1)],[0 1],'--k');
grid on; title(['time: ' num2str(i) '[sec]']); xlabel('x[m]'); ylabel('density');
legend('prediction PDF','observation PDF','correction PDF');
end

% sigma2_obs(m)=mean((z'-x_real(1,:)).^2);
% end
% sigma2_obs_MC=mean(sigma2_obs);

x1=load('x1.mat');
x100=load('x100.mat');
x_r=load('x_real.mat');

x1=x1.x;
x100=x100.x;
x_r=x_r.x_real;

figure;
plot(Time(2:end),x_r(2,2:end),'r',Time(2:end),x100(2,2:end),'g',Time(2:end),x1(2,2:end),'b'); grid on; hold on;
legend('Real velocity','velocity \sigma^2=100','velocity \sigma^2=1','Location','SouthEast');
title('Velocity VS. Time');
xlabel('t[sec]'); ylabel('$\dot{x}$[m]','Interpreter','latex');