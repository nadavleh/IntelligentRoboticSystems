function [ new_policy ] = extract_policy2( v )
    
    new_policy=zeros(12,4);

    
    NESW_value=zeros(12,4);
    for s=1:12
       if s==10 || s==11 || s==5 
           continue
       elseif  s==1
           NESW_value(s,:)=[v(s) ,v(s+3), v(s+1), v(s)];       
       elseif  s==2
           NESW_value(s,:)=[v(s-1) ,v(s), v(s+1), v(s)];
       elseif  s==3
           NESW_value(s,:)=[v(s-1) ,v(s+3), v(s), v(s)];
       elseif  s==4
           NESW_value(s,:)=[v(s) ,v(s+3), v(s), v(s-3)];
       elseif  s==6
           NESW_value(s,:)=[v(s) ,v(s+3), v(s), v(s-3)];
       elseif  s==7
           NESW_value(s,:)=[v(s-1) ,v(s+3), v(s+1), v(s)];
       elseif  s==8
           NESW_value(s,:)=[v(s-1) ,v(s+3), v(s+1), v(s)];
       elseif  s==9
           NESW_value(s,:)=[v(s-1) ,v(s+3), v(s), v(s-3)];
       elseif  s==12 
           NESW_value(s,:)=[v(s-1) ,v(s), v(s), v(s-3)];  
       end
      
    end
    
    for s = 1:12
        % the maximum AVF
        [max_AVF, ~ ] = max(NESW_value(s,:)); % the best AVF, now we need
                                              % to find its index which
                                              % corresponds with an action
                                              % {N,E,S,W}={1,2,3,4}, and
                                              % also find if there are
                                              % actions which gives the
                                              % same AVF
                                              
         % the actions which correspond to this AVF                                     
         best_actions = find( NESW_value(s,:)==max_AVF );
         
         % assign equal probability to best actions
         if length(best_actions) == 1
             p=1;
         elseif length(best_actions) == 2
             p=0.5;
         elseif length(best_actions) == 3
             p=1/3;             
         elseif length(best_actions) == 4
             p=0.25;             
         end
         
         for i=1:4
             if ( sum( ismember(best_actions,i) ) == 0 )
                 new_policy(s,i)=0;
             else
                 new_policy(s,best_actions)=p;
             end
         end

    end
      
    
    
    
    
    


end
