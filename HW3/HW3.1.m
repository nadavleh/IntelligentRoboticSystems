%% Ex.3-MDP 

%% section A

%%%%%%%%%%%%%%%%%%%%Dynamics Probability%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p=zeros(12,12,4);

% North P(from state i, to state j, given action=1)
for i=1:12
    % state 5 is an obstacle, states 11 and 10 are terminal states
    if (i==5) || (i==11) || (i==10)
        continue
    elseif (i==4) || (i==6) || (i==7)
         p(i,i,1)=0.8 ; p(i,i+3,1)=0.1; p(i,i-3,1)=0.1;   
    elseif (i==8) || (i==3) 
         p(i,i,1)=0.1 ; p(i,i+3,1)=0.1; p(i,i-1,1)=0.8;   
    end    
end
p(1,1,1)=0.9; p(1,4,1)=0.1;
p(2,2,1)=0.2; p(2,1,1)=0.8;
p(9,12,1)=0.1 ; p(9,8,1)=0.8; p(9,6,1)=0.1; 
p(12,12,1)=0.1 ; p(12,12-3,1)=0.1; p(12,12-1,1)=0.8; 

% East P(from state i, to state j, given action=2)
for i=1:12
    % state 5 is an obstacle, states 11 and 10 are terminal states
    if (i==5) || (i==11) || (i==10)
        continue
    elseif (i==4) || (i==6) 
         p(i,i,2)=0.2 ; p(i,i+3,2)=0.8;
    elseif (i==1) || (i==7) 
         p(i,i,2)=0.1 ; p(i,i+3,2)=0.8; p(i,i+1,2)=0.1;  
    elseif (i==3) || (i==9) 
         p(i,i,2)=0.1 ; p(i,i+3,2)=0.8; p(i,i-1,2)=0.1;
    end    
end
% p(1,1)=0.9; p(1,4)=0.1;
p(2,2,2)=0.8; p(2,1,2)=0.1; p(2,3,2)=0.1;
p(8,7,2)=0.1 ; p(8,9,2)=0.1; p(8,11,2)=0.8; 
p(12,12,2)=0.9 ; p(12,11,2)=0.1;

% South P(from state i, to state j, given action=3)
for i=1:12
    % state 5 is an obstacle, states 11 and 10 are terminal states
    if (i==5) || (i==11) || (i==10)
        continue
    elseif (i==1) || (i==8) || (i==7)
         p(i,i,3)=0.1 ; p(i,i+3,3)=0.1; p(i,i+1,3)=0.8;   
    elseif (i==4) || (i==6) || (i==9) 
         p(i,i,3)=0.8 ; p(i,i+3,3)=0.1; p(i,i-3,3)=0.1;   
    end    
end
p(2,2,3)=0.2; p(2,3,3)=0.8;
p(3,3,3)=0.9 ; p(3,6,3)=0.1;
p(12,12,3)=0.9 ; p(12,3,3)=0.1;

% West P(from state i, to state j, given action=4)
for i=1:12
    % state 5 is an obstacle, states 11 and 10 are terminal states
    if (i==5) || (i==11) || (i==10)
        continue
    elseif (i==4) || (i==6) 
         p(i,i,4)=0.2 ; p(i,i-3,4)=0.8;
    elseif (i==8) || (i==2) 
         p(i,i,4)=0.8 ; p(i,i+1,4)=0.1; p(i,i-1,4)=0.1;  
    elseif (i==12) || (i==9) 
         p(i,i,4)=0.1 ; p(i,i-1,4)=0.1; p(i,i-3,4)=0.8;
    end    
end
p(1,1,4)=0.9; p(1,2,4)=0.1;
p(3,3,4)=0.9 ; p(3,2,4)=0.1; 
p(7,7,4)=0.1 ; p(7,4,4)=0.8; p(7,8,4)=0.1;

%%%%%%%%%%%%%%%%%%%%%%Reward%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Each state guaranties a reward of 0.04 exept for the final goals at
% states 10 adn 11 which delivers a reward of +1 or -1 accordingly
r=-0.04*ones(12,1);
r(5)=0; r(10)=1; r(11)=-1; 


%% section B - best policy by value iteration

% define object of class cWorld
 myworld=cWorld();
 myworld.R=r;
 myworld.Pr=p(:,:,:);
 
 gamma = 1;
 tol=10^-4;
 
%  define itterative value vectors and assign initial values 0, for v_K we
%  assign ones just for the start so it will go inside the loop, then after
%  v_k is assigned zero again
 v_k=ones(12,1);
 v_kp1=zeros(12,1);
 policy=zeros(12,1);
 %  define an array for intermediate step
 intermidiate=zeros(12,1,4);
 
 while max( abs(v_k-v_kp1) ) >= tol
     % assign the last value for convergence check
     v_k=v_kp1;
     % we calculate the Action Value Function (AVF) as it appears in the
     % Bellman optimality equation: the istant reward (r is the state
     % reward but it needs to be calculated as action reward towards the
     % state) + the optimal State Value Function (SVF)*it's dynamic
     % probability. 
     % So, intermidiate is a 12-by-1-by-4 matrix with the AVF according to
     % actions {1,2,3,4} (N,E,S,W)
     for i=1:4
         intermidiate(:,:,i)=r+gamma*p(:,:,i)*v_kp1;
     end
     % As intermidiate is the itterative AVF for action {1,2,3,4}, we need
     % to find the maximum AVF for each sate so according to bellman
     % optimallity V*(s)=max_a(q*(s,a)), we loop through all 12 states and
     % take maximum AVF, and save the its corresponding action as the
     % deterministic policy
     for i=1:length(v_kp1)
        [ v_kp1(i), policy(i) ]= max( intermidiate(i,1,:) );
     end     
 end
% Results 
myworld.plot;
myworld.plot_value(v_kp1)
myworld.plot;
myworld.plot_policy(policy)

%% section C - best policy by policy iteration

% initialize policy for all actions in all states equally,
% policy(action,state)
policy_k=0.25*ones(12,4);
 
% difine state reward
commonReward=-0.04;
r=commonReward*ones(12,1);
% r(:,5)=0; r(:,10)=1; r(:,11)=-1; 

% impossible to complete vector wise?
 v=zeros(12,1);
 v_k=ones(12,1);
 v_kp1=zeros(12,1);
 p_mean=zeros(12,12);
 %  define an array for intermediate step
 intermidiate=zeros(12,1,4);
 
% while max( abs(v_k-v_kp1) ) >= tol
     
     for i=1:12
         for a = 1:4
            p_mean(i,:)=p_mean(i,:)+policy_k(i,a)*p(i,:,a);
         end 
     end
     
     v = ( eye(12) - gamma*p_mean)^(-1)*r
     
     policy_kp1=extract_policy(v);
     
     
     
% end


