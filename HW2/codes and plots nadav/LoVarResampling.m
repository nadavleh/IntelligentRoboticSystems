function equally_weighted_set=LoVarResampling(particle_set,weights)
%Example: mu = 0; sigma = 10; pd =makedist('Normal',mu,sigma);
%x=-100:0.1:100; weights=pdf(pd,x);re_sampled_x=LoVarResampling(x,weights);
%hist(re_sampled_x);

    % chi (2D array) is are the particles weighted according to the likelyhood pdf, first
    % we need to know the number of particles N, and the state vector's
    % dimensions 'dim'(for example position velocity and temperature). first
    % let's assume that N>>dim and turn chi to N-by-dim (if not already)
    [n,m]=size(particle_set);
    N=max(n,m);        % particle number 
    if N ~= n          % make 'chi' an N-by-dim array
         particle_set=particle_set';
    end
    clear n m
    % we need to normalize the weights so that cmd<=1
    weights=weights/sum(weights);

    cmd=weights(1);          % comulative mass dist. of weights 
    i=1;
    equally_weighted_set=[];
    r=unifrnd(0,1/N);
    for m=1:N
        U = r+(m-1)/N;
        while U>cmd
            i=i+1;
            cmd=cmd+weights(i);
        end
        equally_weighted_set(m,:)=particle_set(i,:);
    end
end
 