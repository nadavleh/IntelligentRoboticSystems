%% section b - add a function "move" to the cRobot class

% function   obj = move(obj,u_rotation,u_translation)
%   obj.theta = obj.theta+u_rotation + mvnrnd(0,obj.turn_noise,1);
%   obj.x =  obj.x + (u_translation + mvnrnd(0,obj.forward_noise)) ...
%           *cos(obj.theta);
%   obj.y = obj.y + (u_translation + mvnrnd(0,obj.forward_noise))...
%           *sin(obj.theta);   
%   %cyclic world
%   if obj.x>100
%       obj.x=obj.x-100;
%   elseif obj.x<0
%       obj.x=obj.x+100;
%   end
%   if obj.y>100
%       obj.y=obj.y-100;
%   elseif obj.y<0
%       obj.y=obj.y+100;
%   end 
%   if obj.theta>2*pi%%%%%%%%%%%%%%%%%%%
%       obj.theta=mod(obj.theta,2*pi);
%   elseif obj.theta<0
%       obj.theta=-mod(obj.theta,2*pi)+2*pi;
%   end
% 
%   
%% Section c - add a function "sense" to the cRobot class
% this function simulates the robot measurements, it calculates
% the robot true position to each landmark and adds white noise
% according to the measurement noise parameter. the output is an
% array (column vector) r of distances to an associated  
% landmark with white noise. 

% function   [r]=sense(obj,map_obj)
%    r=[];
%    phi=[];
%    r=( (obj.x*ones(length(map_obj.landmarks),1)-map_obj.landmarks(:,1)).^2 ...
%    +(obj.y*ones(length(map_obj.landmarks),1)-map_obj.landmarks(:,1)).^2  ).^0.5 ...
%    +mvnrnd(zeros(1,4),obj.sense_distance_noise*eye(4),1)';
%     % phi = ( atan2((obj.y*ones(length(map_obj.landmarks),1)-map_obj.landmarks(:,1)), ...
%     % (obj.x*ones(length(map_obj.landmarks),1)-map_obj.landmarks(:,1))));
% end
% forward_noise=5; 
% turn_noise=0.5; 
% sensor_noise=5;
% myrobot1.set_noise(forward_noise,turn_noise,sensor_noise);

%% Section d - add a function "measurement_probability" to the cRobot class
% this function is for particle weithing. r_measured is a column
% vector of the measured features at time t, pose_x is the
% particle position vector, map_obj is the map object which
% contains the features measured (landmarks) 

% function   [p] = measurement_probability(obj, r_measured, pose_x, map_obj)
%    r_at_pose_x=( (pose_x(1)*ones(length(map_obj.landmarks),1)-map_obj.landmarks(:,1)).^2 ...
%    +(pose_x(2)*ones(length(map_obj.landmarks),1)-map_obj.landmarks(:,1)).^2  ).^0.5;
%    % beacuse the measurment of each landmark is an i.i.d random
%    % variable, the joint probability of the measurements is the
%    % product of probabilitys. we evaluate the the weight of the
%    % particle accordind to the difference between the measured
%    % distance to a landmark and the particle's distance to it.
%    p=prod(normpdf(r_at_pose_x-r_measured,0,obj.sense_distance_noise));         
% end 
%% Low Variance Re-Sampling 
% The function used for re-sampling was written according to the algorithm
% presented in Probabilistic Robotics, with the added feature of weights
% normalization.

% function equally_weighted_set=LoVarResampling(particle_set,weights)
% %Example: mu = 0; sigma = 10; pd =makedist('Normal',mu,sigma);
% %x=-100:0.1:100; weights=pdf(pd,x);re_sampled_x=LoVarResampling(x,weights);
% %hist(re_sampled_x);
% 
%     % chi (2D array) is are the particles weighted according to the likelyhood pdf, first
%     % we need to know the number of particles N, and the state vector's
%     % dimensions 'dim'(for example position velocity and temperature). first
%     % let's assume that N>>dim and turn chi to N-by-dim (if not already)
%     [n,m]=size(particle_set);
%     N=max(n,m);        % particle number 
%     if N ~= n          % make 'chi' an N-by-dim array
%          particle_set=particle_set';
%     end
%     clear n m
%     % we need to normalize the weights so that cmd<=1
%     weights=weights/sum(weights);
% 
%     cmd=weights(1);          % comulative mass dist. of weights 
%     i=1;
%     equally_weighted_set=[];
%     r=unifrnd(0,1/N);
%     for m=1:N
%         U = r+(m-1)/N;
%         while U>cmd
%             i=i+1;
%             cmd=cmd+weights(i);
%         end
%         equally_weighted_set(m,:)=particle_set(i,:);
%     end
% end
