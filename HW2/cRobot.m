classdef cRobot < handle
    
  
    
   properties 
      
      % pose
      x;
      y;
      theta;
      
      % noise
      forward_noise = 0;
      turn_noise    = 0;
      sense_distance_noise   = 0; 
     
      
   end
   
   methods
       
      function obj = cRobot % method constructor
      
          % place robot with a random pose
          obj.x = rand*cWorld.world_size;
          obj.y = rand*cWorld.world_size;
          obj.theta = rand*2*pi;
          
          
      end
      
   
       
       % sets the new pose
       function  obj = set(obj,new_x,new_y, new_orientation)
           
           
         if (new_x < 0 | new_x >= cWorld.world_size)
            error('X coordinate out of bound');
         end
         
         if (new_y < 0 | new_y >= cWorld.world_size)
            error('Y coordinate out of bound');
         end
            
         if (new_orientation < 0 | new_orientation >= 2 * pi);
            error('Orientation must be in [0,2pi]');
         end
            
        
         obj.x = new_x;
         obj.y = new_y;
         obj.theta = new_orientation;
        
         
       end
       
       % prints the pose of the robot to the Matlab prompt
       function   print(obj)
          
           display(['[x= ',num2str(obj.x), '  y=', num2str(obj.y), '  heading=', num2str(obj.theta),']']);
           
       end
       
       % plots the pose of the robot in the world
       function plot(obj,mycolor,style)
           
             if(nargin == 1)
                 mycolor = 'b'; % default
                 style = 'robot'
             end
             
             if(nargin == 2)
                 style = 'robot'; % default
             end
           
             hold on;
           
             
             % different plot styles for the robot
             switch style
                 
                 case 'robot'
                     
                     % size of robot
                     phi = linspace(0,2*pi,101);
                     r = 1;
                     
                     
                         % plot robot body
                         plot(obj.x + r*cos(phi),obj.y + r*sin(phi),'Color',mycolor);
                         hold on;
                         % plot heading direction
                         plot([obj.x, obj.x + r*cos(obj.theta)],[obj.y, obj.y + r*sin(obj.theta)],'Color',mycolor);
                         
                         axis equal;
                    
                     
                 case 'particle'
                     
                     
                         % plots robot position but ignores heading
                         plot(obj.x,obj.y,'.','Color',mycolor,'MarkerSize',10);
                         
                     
                     
                 otherwise
                     
                     disp('unknown style');
                     
             end
             
              xlim([0,cWorld.world_size]);
              ylim([0,cWorld.world_size]);
                     
       end
       
       
       % sets new noise parameters
       function obj = set_noise(obj, new_forward_noise, new_turn_noise, new_sensing_distance_noise )
          obj.forward_noise = new_forward_noise;
          obj.turn_noise    = new_turn_noise;
          obj.sense_distance_noise   = new_sensing_distance_noise;
       end
       
       %% Time Propagation (b)
       function move(obj,u)
           
           u1= normrnd(u(1),obj.forward_noise);
           u2= normrnd(u(2),obj.turn_noise);
           
           obj.theta = obj.theta + u1;
           obj.x = obj.x + u2*cos(obj.theta);
           obj.y = obj.y + u2*sin(obj.theta);
       end
       
       %% Observations - distances from landmarks (c)
       function d=sense(obj)
           m=[20 20; 80 80; 20 80; 80 20]; % landmarks
           d=sqrt( (m(:,1)-obj.x).^2 + (m(:,2)-obj.y).^2 ) + mvnrnd(zeros(4,1),obj.sense_distance_noise*eye(4))';
       end
       
       %% Measurement Probability  - distances from landmarks (c)
       function prob=measurement_probability(obj,d)
           m=[20 20; 80 80; 20 80; 80 20]; % landmarks
           d_est=sqrt( (m(:,1)-obj.x).^2 + (m(:,2)-obj.y).^2 );
           prob=mvnpdf(d,d_est,10*eye(4)); % since the covariance
       end
       
   end
   
end
       
       
       
       
      