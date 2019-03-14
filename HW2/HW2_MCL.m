clc; clear all; close all;

%% section a - Experiment with the cRobot class
myworld = cWorld(); % object containing "world size" and "landmarks"
myworld.plot;
myrobot = cRobot();

myrobot.set(40,40,0);
myrobot.plot('b','robot');
myrobot.print;

myrobot.set(60,50,pi/2);
myrobot.plot('r','robot');
myrobot.print;

myrobot.set(30,70,3*pi/4);
myrobot.plot('g','robot');
myrobot.print;

%% section e - robot poses with no action commands noises
myrobot = cRobot();
myrobot.set(10,15,0);
myrobot.set_noise(0,0,0);
u=[0 60; pi/3 30; pi/4 30; pi/4 20; pi/4 40];

x(1)=myrobot.x;
y(1)=myrobot.y;
for i=1:size(u,1)
    myrobot.move(u(i,:));
    x(i+1)=myrobot.x;
    y(i+1)=myrobot.y;
end
myworld.plot; hold on;
plot(x,y,'k:');

%% section f - real robot poses (with action commands noises)
myrobot = cRobot();
myrobot.set(10,15,0);
myrobot.set_noise(.1,5,5);
u=[0 60; pi/3 30; pi/4 30; pi/4 20; pi/4 40];

x(1)=myrobot.x;
y(1)=myrobot.y;
for i=1:size(u,1)
    myrobot.move(u(i,:));
    x(i+1)=myrobot.x;
    y(i+1)=myrobot.y;
    z(:,i)=myrobot.sense();
end
plot(x,y,'k');

%% Tracking Algorithm
Np=1000;
x0=10; y0=15; theta0=0;
for i=1:Np % Initialization
    particle{i} = cRobot();
    particle{i}.set(10,15,0);
    particle{i}.set_noise(5,.1,5);
    W=ones(1,Np)/Np;
end

for t=1:size(u,1)
    for i=1:Np % prediction for each particle
        particle{i}.move(u(t,:));
        W(i)= particle{i}.measurement_probability(z(:,t)); % Likelihood (Importance weights)
        particle{i}.plot('k','particle');
    end
    
    W=W./sum(W);                        % normalized wheights
    
    sampleIND=randsample(Np,Np,true,W); % Bootstrap Indeces
    
    for i=1:Np % % Re-sampling
        particle_temp{i} = particle{sampleIND(i)};
        x_hat(i)=particle_temp{i}.x;
        y_hat(i)=particle_temp{i}.y;
        particle_temp{i}.plot('g','particle');
    end
    particle=particle_temp;
    W=ones(1,Np)/Np; % wheights init after Re-sampling.
    
    X_hat(t)=sum(x_hat)/Np;                 % x-state estimation
    Y_hat(t)=sum(y_hat)/Np;                 % y-state estimation

%     pause(0.3);
%     plot(x(i),0,'r*',X_hat(i),0,'g^'); hold on;
%     plot(X,zeros(length(X),1),'k.'); hold off;
end
    plot(X_hat,Y_hat,'co',x_hat,y_hat,'k.'); hold off;

