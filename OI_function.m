function sigma_lower = OI_function(pos_history)
    size_pos_history = size(pos_history);
    mean_horizon = size_pos_history(1);
    N = size_pos_history(2);
    % build O matrix
    O = zeros(10,10);
    for i=1:mean_horizon
        temp = H_function( [squeeze(pos_history(i,:,:)) zeros(N,1)] );
        O = O + temp*temp.';
    end
    O = (1/mean_horizon)*O;

    % extract minimum singular value/eigenvalue
    sigma_lower = sqrt(svds(O,1,"smallestnz"));
end