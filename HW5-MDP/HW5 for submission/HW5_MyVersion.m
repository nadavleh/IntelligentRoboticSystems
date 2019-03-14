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
commonReward=-0.04;
r=commonReward*ones(12,1);
r(5)=0; r(10)=1; r(11)=-1; 


%% section B - best policy by value iteration

% define object of class cWorld
 myworld=cWorld();
%  % not of any use..
%  myworld.R=r;
%  myworld.Pr=p(:,:,:);
 
 gamma = 1;   % discount factor
 tol=10^-4;   % tollerance for value itterations
 
% difine state reward
commonReward=-0.02;
r=commonReward*ones(12,1);
r(5)=0; r(10)=1; r(11)=-1; 
 
%  define itterative value vectors and assign initial values 0, for v_K we
%  assign ones just for the start so it will go inside the loop, then after
%  v_k is assigned zero again
 v_k=ones(12,1);
 v_kp1=zeros(12,1);
 policy=zeros(12,1);
 %  define an array for intermediate step
 AVF=zeros(12,1,4);
 counter=0;
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
         AVF(:,:,i)=r+gamma*p(:,:,i)*v_k;
     end
     % As intermidiate is the itterative AVF for action {1,2,3,4}, we need
     % to find the maximum AVF for each sate so according to bellman
     % optimallity V*(s)=max_a(q*(s,a)), we loop through all 12 states and
     % take maximum AVF, and save the its corresponding action as the
     % deterministic policy
     for i=1:length(v_kp1)
       [ v_kp1(i), policy(i) ]= max( AVF(i,1,:) );
     end 
     counter=counter+1;
 end
 disp(['number of iterations = ', num2str(counter)])

% Results 
myworld.plot;
myworld.plot_value(v_kp1)
myworld.plot;
myworld.plot_policy(policy)

%% section C - best policy by policy iteration
gamma = 1;   % discount factor
tol=10^-4;   % tollerance for value itterations
 
% difine state reward
commonReward=-0.02;
r=commonReward*ones(12,1);
r(5)=0; r(10)=1; r(11)=-1; 

% initialize policy for all actions in all states equally,
% policy(action,state)
policy_kp1=0.25*ones(12,4);

% % uncomment to try random deterministic policy
% for i=1:12
%     policy_kp1(i,randi(4))=1;
% end
policy_k  = zeros(12,4);

v=zeros(12,1);

p_mean=zeros(12,12);

v_befor=ones(12,1);
v_k=ones(12,1);
v_kp1=zeros(12,1);
counter=0;
% we itterate untill the new policy doest change the SVF vector 
 while max( abs(v_befor-v_kp1) ) >= tol
     
     % calculating vector wise, we nead a mean transition matrix
     p_mean=zeros(12,12);
     % because the policy is state dependent, i.e. policy(state,action) is
     % probabilistic, we cant simply multiply the policy(a) by p(:,:,a), so
     % we need to average the each row which corresponds to a state, to get
     % the mean transitioon matrix
     for i=1:12
         for a = 1:4
            p_mean(i,:)=p_mean(i,:)+policy_kp1(i,a)*p(i,:,a);
         end 
     end
     v_befor=v_kp1;
     
     % now we need to itterate to find the SVF i.e.: policy evaluation
     v_k=ones(12,1);
     v_kp1=zeros(12,1);
     while max( abs(v_k-v_kp1) ) >= tol
         v_k=v_kp1;
         v_kp1=r+gamma*p_mean*v_k;
     end

     %after evaluating the SVF, we will assign a greedy policy that gives 
     %maximum probability to the action that takes us to surrounding states
     %that gives the maximum value

     policy_kp1=extract_policy2(v_kp1);
    
     counter=counter+1;
 end
 
counter

%%%%get determinstic policy%%%%%%%%%
% once the the policy update and correspondind SVF updates converged, we
% need to extract a deterministic policy from our policies (action
% probabilitys)
% at each state, the maximum probability in ech state is the deterministic
% policy we will take, and each maximum value's index in a row, is 
% correspondent to an action N=1, E=2, .. and so on.
det_policy=zeros(12,1);
for i=1:12
    [~,det_policy(i)]=max(policy_kp1(i,:));
end

myworld.plot;
myworld.plot_value(v_kp1)
myworld.plot;
myworld.plot_policy(det_policy)
