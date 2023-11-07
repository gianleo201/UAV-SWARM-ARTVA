function c_ineq = quadrotor_IneqConstriant(X,U,e,data,neigh_pos,N_neighbours,non_neigh_pos,N_non_neighbours,d_safe,v_max,quadrotor_params)

    p = data.PredictionHorizon;
    x = X(1:p+1,1:3);
    dx = X(1:p+1,4:6);

    c_ineq = zeros((1+N_neighbours+1)*(p+1)+2,1);

    %% VELOCITY CONSTRAINT

    for k = 1 : p+1
        c_ineq(k) = norm(dx(k,:)) - v_max;
    end

    %% ONLY HARD SAFETY CONSTRAINT (1+N_neighbours+N_non_neighbours)*(p+1)+2
    
    % collision avoidance constraint (by known value)
    for i_th_neighbour = 1:N_neighbours
        i = 1;
        for k = (i_th_neighbour)*(p+1)+1 : (i_th_neighbour+1)*(p+1)
            c_ineq(k) = - norm(x(i,:)-neigh_pos(i_th_neighbour,1:3)) + d_safe;
            i = i + 1;
        end
    end

    % collision avoidance constraint (by sensor value) (only nearest)
    if N_non_neighbours > 0
        nearest_non_neighbour = non_neigh_pos(1,1:3);
        for i = 1:N_non_neighbours
            if norm(x(1,:)-non_neigh_pos(i,1:3)) < norm(x(1,:)-nearest_non_neighbour)
                nearest_non_neighbour = non_neigh_pos(i,1:3);
            end
        end
        if norm(x(1,:)-nearest_non_neighbour) <= (d_safe + 2*v_max * p*0.1)
            i = 1;
            for k = (1+N_neighbours)*(p+1)+1 :(1+N_neighbours+1)*(p+1)
                c_ineq(k) = - norm(x(i,:)-nearest_non_neighbour) + d_safe;
                i = i + 1;
            end
        else
            i = 1;
            for k = (1+N_neighbours)*(p+1)+1 :(1+N_neighbours+1)*(p+1)
                c_ineq(k) = -(2*v_max * p*0.1);
                i = i + 1;
            end
        end
    else
        i = 1;
        for k = (1+N_neighbours)*(p+1)+1 :(1+N_neighbours+1)*(p+1)
            c_ineq(k) = -(2*v_max * p*0.1);
            i = i + 1;
        end
    end

    % convergence to objective constraint (soft)
    c_ineq((1+N_neighbours+1)*(p+1)+1) = norm(x(end,:)-data.References(p,1:3)) - norm(x(1,:)-data.References(p,1:3)) + e;
    c_ineq((1+N_neighbours+1)*(p+1)+2) = e;

end