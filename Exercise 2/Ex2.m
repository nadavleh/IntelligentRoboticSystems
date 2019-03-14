%% Section a -Experiment with the cRobot class 
myworld=cWorld();
myworld.plot();
hold on;
myrobot1=cRobot();
myrobot1.set(40,40,0);
myrobot1.plot();
myrobot2=cRobot();
myrobot2.set(60,50,pi/2);
myrobot2.plot();
myrobot3=cRobot();
myrobot3.set(30,70,3*pi/4);
myrobot3.plot();
hold off

%% Section e - intended robot path
myworld=cWorld();
myrobot=cRobot();
forward_noise=0; 
turn_noise=0; 
sensor_noise=5;
myrobot.set_noise(forward_noise,turn_noise,sensor_noise);
myrobot.set(10,15,0);
u=[0 60; pi/3 30; pi/4 30; pi/4 20; pi/4 40];

robot_command_pose=zeros(length(u)+1,2);
robot_command_pose(1,:)=[myrobot.x myrobot.y];
for i= 1:length(u)  % move the robot
    myrobot.move(u(i,1),u(i,2));
    robot_command_pose(i+1,:)=[myrobot.x myrobot.y];  
end
   
myworld.plot(); 
hold on;
plot(robot_command_pose(:,1),robot_command_pose(:,2),'k:');
%% Section f
% set the robot simulation position and noise parameters and move the
% robot according to the model. in each step take the measurements to the
% landmarks
forward_noise=5; 
turn_noise=0.1; 
sensor_noise=5;
myrobot.set_noise(forward_noise,turn_noise,sensor_noise);
myrobot.set(10,15,0);
u=[0 60; pi/3 30; pi/4 30; pi/4 20; pi/4 40];

robot_true_pose=zeros(length(u)+1,2); %initialize the robots true pose array
robot_true_pose(1,:)=[myrobot.x myrobot.y]; %the robot's first position
measurments=[];
for i= 1:length(u)  % move the robot
    myrobot.move(u(i,1),u(i,2));
    robot_true_pose(i+1,:)=[myrobot.x myrobot.y];
    measurments(:,i)=myrobot.sense(myworld); %take the measurement each step
end
plot(robot_true_pose(:,1),robot_true_pose(:,2),'k-');

%% section g - particle filtering
N=1000;
u=[0 60; pi/3 30; pi/4 30; pi/4 20; pi/4 40];
T=length(u)+1; %the last time steps occures after the last motor command

forward_noise=5; 
turn_noise=0.1; 
sensor_noise=5;

particle_weight=ones(N,1)*(1/N); % at the beggining each particle weighs the same

%initialize particles
particle=cRobot();
particle.set_noise(forward_noise,turn_noise,sensor_noise);
particle.set(10,15,0);

% it's more efficient memory, and run time-wise to put the particles into an array
particle_array=[particle.x*ones(N,1), particle.y*ones(N,1), zeros(N,1)];   

% initialize the estimated position array
% the initial position is deterministic
robot_estimated_pose(1,:)=[10 15];

myworld.plot(); 
hold on;
% particle filter
for t=2:T % the estimation is from t=2 (pose in t=1 is known), to 
    %advance particles with model and weight them
    for i=1:N
        %set each particle position to the previous position
        particle.set( particle_array(i,1),particle_array(i,2),particle_array(i,3)); 
        %move each particle
        particle.move(u(t-1,1),u(t-1,2));
        particle_array(i,:)=[particle.x particle.y particle.theta];                                 
        particle_weight(i)=particle.measurement_probability(measurments(:,t-1),particle_array(i,1:2),myworld);     
    end
% % uncomment to show projected particles:
    plot(particle_array(:,1),particle_array(:,2),'ko','MarkerSize',1,'MarkerFaceColor','k') 

    % re-sample particles and estimate robot positiom via the expectation
    % of the re-sampled set:
    % Re-Sample
    particle_array=LoVarResampling(particle_array,particle_weight);  
% % uncomment to show resampled particles:    
    plot(particle_array(:,1),particle_array(:,2),'.','color',[0.5 0.5 0.5]);
     pause();
 
    % calc the mean (expectancy of equally weighted particles)
    % of the particles position:
    robot_estimated_pose(t,:)=[mean(particle_array(:,1)) mean(particle_array(:,2))];
    %the Re-Sampled set is equally weighted (this opperation is
    %unnescesary but theoretically right):
    particle_weight=ones(N,1)*(1/N);

end
%ploting:

plot(robot_command_pose(:,1),robot_command_pose(:,2),'k:');
% legend('command')
plot(robot_true_pose(:,1),robot_true_pose(:,2),'k-');
% legend('true position')
plot(robot_estimated_pose(:,1),robot_estimated_pose(:,2),'b-');
legend('landmarks', ... %'command','true position','estimated position');
                    'projeced particles','resampled particles', ...
                    'projeced particles','resampled particles', ...
                    'projeced particles','resampled particles', ...
                    'projeced particles','resampled particles', ...
                    'projeced particles','resampled particles', ...
                    'command','true position','estimated position');













