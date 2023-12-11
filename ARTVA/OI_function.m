function sigma_lower = OI_function(pos_history)
    size_pos_history = size(pos_history);
    mean_horizon = size_pos_history(1);
    N = size_pos_history(2);
    n_coords = size_pos_history(3);
    % build O matrix
    O = zeros(10,10);
%     O = zeros(N,N);
    for i=1:mean_horizon
        if n_coords == 2
            temp = H_function(N, [squeeze(pos_history(i,:,:)) zeros(N,1)] );
        elseif n_coords == 3
            temp = H_function(N, squeeze(pos_history(i,:,:)) );
        end
        O = O + temp*temp.';
%         O = O + temp.'*temp;
    end
    O = (1/mean_horizon)*O;

    sigma_lower = min_sv_O(O,min([N 10]));
end