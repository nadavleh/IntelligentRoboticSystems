function [ new_policy ] = extract_policy( v )
    
    new_policy=zeros(12,4);
    v_as_mat=reshape(v,3,4);
    vv=zeros(5,6);
    vv(2:4,2:5)=v_as_mat;
    vv=vv(:);
    
    
    differ=zeros(12,4);
    j=1;
    for s=7:24
       if s==10 || s==11 || s==15 ||s==16 || s==20 ||s==21
           continue
       else
       differ(j,:)=[vv(s-1)-vv(s) ,vv(s+5)-vv(s), vv(s+1)-vv(s), vv(s-5)-vv(s)];        
       end
       j=j+1;
    end
    
    for s = 1:12
        [max_AVF, ind] = max(differ(s,:));
        for a=1:4
            
            
            
        end
    end
      
    
    
    
    
    


end

