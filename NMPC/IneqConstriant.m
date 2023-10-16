function c_ineq = IneqConstriant(X,U,e,data,neigh_pos,N_neighbours,non_neigh_pos,N_non_neighbours,d_safe, v_max)
    p = data.PredictionHorizon;
    ref_pt = data.References(1,:);
    x = X(1:p+1,1:2);
    dx = X(1:p+1,3:4);
    c_ineq = zeros((N_neighbours+1)*(p+1)+2,1);
    % velocity constraint
    for k = 1 : p+1
        c_ineq(k) = norm(dx(k,:)) - v_max;
    end
    % collision avoidance constraint
    for i_th_neighbour = 1:N_neighbours
        i = 1;
        for k = i_th_neighbour*(p+1)+1 : (i_th_neighbour+1)*(p+1)
            c_ineq(k) = - norm(x(i,:)-neigh_pos(i_th_neighbour,1:2)) + d_safe;
            i = i + 1;
        end
    end
    % convergence to objective constraint
    c_ineq((N_neighbours+1)*(p+1)+1) = norm(x(end,:)-ref_pt(1:2)) - norm(x(1,:)-ref_pt(1:2)) + e;
    c_ineq((N_neighbours+1)*(p+1)+2) = e;
end