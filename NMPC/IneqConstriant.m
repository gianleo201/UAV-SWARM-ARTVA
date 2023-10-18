function c_ineq = IneqConstriant(X,U,e,data,neigh_pos,N_neighbours,non_neigh_pos,N_non_neighbours,d_safe, v_max)

    p = data.PredictionHorizon;
    ref_pt = data.References(1,:);
    x = X(1:p+1,1:2);
    dx = X(1:p+1,3:4);
    c_ineq = zeros((1+2*N_neighbours+2*N_non_neighbours)*(p+1)+2,1);
%     c_ineq = zeros((1+N_neighbours+N_non_neighbours)*(p+1)+2,1);
%     c_ineq = zeros((1+N_neighbours)*(p+1)+2,1);

    %% VELOCITY CONSTRAINT
    for k = 1 : p+1
        c_ineq(k) = norm(dx(k,:)) - v_max;
    end

    %% HARD AND COVERING SOFT SAFETY CONSTRAINT (1+2*N_neighbours+2*N_non_neighbours)*(p+1)+2

    % collision avoidance constraint (by known values)
    for i_th_neighbour = 1:N_neighbours
        i = 1;
        for k = i_th_neighbour*(p+1)+1 : (i_th_neighbour+1)*(p+1)
            c_ineq(k) = - norm(x(i,:)-neigh_pos(i_th_neighbour,1:2)) + d_safe;
            i = i + 1;
        end
    end
    % collision avoidance constraint (by known values) SOFT
    for i_th_neighbour = 1:N_neighbours
        i = 1;
        for k = (N_neighbours+i_th_neighbour)*(p+1)+1 : (N_neighbours+i_th_neighbour+1)*(p+1)
            c_ineq(k) = - norm(x(i,:)-neigh_pos(i_th_neighbour,1:2)) + 2*d_safe + e;
            i = i + 1;
        end
    end

    % collision avoidance constraint (by sensor value) HARD
    for i_th_non_neighbour = 1:N_non_neighbours
        i = 1;
        for k = (2*N_neighbours+i_th_non_neighbour)*(p+1)+1 :(2*N_neighbours+i_th_non_neighbour+1)*(p+1)
            if norm(x(1,:)-non_neigh_pos(i_th_non_neighbour,1:2)) <= (d_safe + 2*v_max * p*0.1)
                c_ineq(k) = - norm(x(i,:)-non_neigh_pos(i_th_non_neighbour,1:2)) + d_safe;
            else
                c_ineq(k) = - d_safe * 2;
            end
            i = i + 1;
        end
    end
    % collision avoidance constraint (by sensor value) SOFT
    for i_th_non_neighbour = 1:N_non_neighbours
        i = 1;
        for k = (2*N_neighbours+N_non_neighbours+i_th_non_neighbour)*(p+1)+1 :(2*N_neighbours+N_non_neighbours+i_th_non_neighbour+1)*(p+1)
            if norm(x(1,:)-non_neigh_pos(i_th_non_neighbour,1:2)) <= (d_safe + 2*v_max * p*0.1)
                c_ineq(k) = - norm(x(i,:)-non_neigh_pos(i_th_non_neighbour,1:2)) + 2*d_safe+e;
            else
                c_ineq(k) = - d_safe * 2;
            end
            i = i + 1;
        end
    end

    % convergence to objective constraint (soft)
    c_ineq((1+2*N_neighbours+2*N_non_neighbours)*(p+1)+1) = norm(x(end,:)-ref_pt(1:2)) - norm(x(1,:)-ref_pt(1:2)) + e;
    c_ineq((1+2*N_neighbours+2*N_non_neighbours)*(p+1)+2) = e;

    %% ONLY HARD SAFETY CONSTRAINT (1+N_neighbours+N_non_neighbours)*(p+1)+2

%     for i_th_neighbour = 1:N_neighbours
%         i = 1;
%         for k = i_th_neighbour*(p+1)+1 : (i_th_neighbour+1)*(p+1)
%             c_ineq(k) = - norm(x(i,:)-neigh_pos(i_th_neighbour,1:2)) + d_safe;
%             i = i + 1;
%         end
%     end

%     % collision avoidance constraint (by sensor value)
%     for i_th_non_neighbour = 1:N_non_neighbours
%         i = 1;
%         for k = (N_neighbours+i_th_non_neighbour)*(p+1)+1 :(N_neighbours+i_th_non_neighbour+1)*(p+1)
%             if norm(x(1,:)-non_neigh_pos(i_th_non_neighbour,1:2)) <= (d_safe + 2*v_max * p*0.1)
%                 c_ineq(k) = - norm(x(i,:)-non_neigh_pos(i_th_non_neighbour,1:2)) + d_safe;
%             else
%                 c_ineq(k) = - d_safe * 2;
%             end
%             i = i + 1;
%         end
%     end


%     % convergence to objective constraint (soft)
%     c_ineq((1+N_neighbours+N_non_neighbours)*(p+1)+1) = norm(x(end,:)-ref_pt(1:2)) - norm(x(1,:)-ref_pt(1:2)) + e;
%     c_ineq((1+N_neighbours+N_non_neighbours)*(p+1)+2) = e;

end