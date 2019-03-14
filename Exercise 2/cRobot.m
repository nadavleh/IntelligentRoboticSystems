classdef cRobot < handle %cRobot class inherits from handle class (< means a subset of..)
   properties %properties states the variables of the class - it's state
      % pose
      x;
      y;
      theta;      
      % noise
      forward_noise = 0;
      turn_noise    = 0;
      sense_distance_noise   = 0;    
   end
   methods % methods specify the actions that the object preforms - the functions
      function obj = cRobot % method constructor, when def a func. to be tha name of the class this is the constructor      
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
        function   obj = move(obj,u_rotation,u_translation)
          obj.theta = obj.theta+u_rotation + mvnrnd(0,obj.turn_noise^2,1);
          obj.x =  obj.x + (u_translation + mvnrnd(0,obj.forward_noise^2))*cos(obj.theta);
          obj.y = obj.y + (u_translation + mvnrnd(0,obj.forward_noise^2))*sin(obj.theta);   
          %cyclic world
          if obj.x>100
              obj.x=obj.x-100;
          elseif obj.x<0
              obj.x=obj.x+100;
          end
          if obj.y>100
              obj.y=obj.y-100;
          elseif obj.y<0
              obj.y=obj.y+100;
          end   
          if obj.theta>2*pi
              obj.theta=mod(obj.theta,2*pi);
          elseif obj.theta<0
              obj.theta=-mod(obj.theta,2*pi)+2*pi;
          end
        end     
       function   [r]=sense(obj,map_obj)
           % this function simulates the robot measurements, it calculates
           % the robot true position to each landmark and adds white noise
           % according to the measurement noise parameter. the output is an
           % array (column vector) r of distances to an associated  
           % landmark with white noise. 
           r=[];          
           r=( (obj.x-map_obj.landmarks(:,1)).^2 ...
           +(obj.y-map_obj.landmarks(:,2)).^2  ).^0.5 ...
           +mvnrnd(zeros(1,4),(obj.sense_distance_noise^2)*eye(4),1)';
       end
       function   [p] = measurement_probability(obj, r_measured, pose, map_obj)
           % this function is for particle weithing. r_measured is a column
           % vector of the measured features at time t, pose_x is the
           % particle position vector, map_obj is the map object which
           % contains the features measured (landmarks) 
           r_at_pose_x=( (pose(1)-map_obj.landmarks(:,1)).^2 ...
           +(pose(2)-map_obj.landmarks(:,2)).^2  ).^0.5;
           
           % beacuse the measurment of each landmark is an i.i.d random
           % variable, the joint probability of the measurements is the
           % product of probabilitys. we evaluate the the weight of the
           % particle accordind to the difference between the measured
           % distance to a landmark and the particle's distance to it.
           p=prod(normpdf(r_at_pose_x-r_measured,0,obj.sense_distance_noise));
           
       end     
   end
   
end
       
       
       
       
      