function c_ineq = IneqConstriant(X,U,e,data,neigh_pos,N_neighbours,non_neigh_pos,N_non_neighbours,d_safe, v_max)

    p = data.PredictionHorizon;
    ref_pt = data.References(1,:);
    x = X(1:p+1,1:2);
    dx = X(1:p+1,3:4);
    u = U(1:p+1,1:2);

%     c_ineq = zeros((3+2*N_neighbours+2*N_non_neighbours)*(p+1)+2,1);
%     c_ineq = zeros((3+N_neighbours+N_non_neighbours)*(p+1)+2,1); % <----
%     c_ineq = zeros((3+N_neighbours+N_non_neighbours)*(p+1),1); % restore up
%     c_ineq = zeros((4+N_neighbours+N_non_neighbours)*(p+1),1); % restore up
%     c_ineq = zeros((3+N_neighbours+1)*(p+1),1);
    c_ineq = zeros((3+N_neighbours+1)*(p+1)+2,1);

    
%     c_ineq = zeros((1+N_neighbours)*(p+1)+2,1); % only velocity
%     constraints

    %% VELOCITY CONSTRAINT

    for k = 1 : p+1
        c_ineq(k) = norm(dx(k,:)) - v_max;
    end

    %% ACTUATION CONSTRAINT
    
    i = 1;
    for k = (p+1)+1 : 2*(p+1)
        c_ineq(k) = u(i,1) - 10;
        i = i + 1;
    end
    i = 1;
    for k = 2*(p+1)+1 : 3*(p+1)
        c_ineq(k) = u(i,2) - 10;
        i = i + 1;
    end

    %% HARD AND COVERING SOFT SAFETY CONSTRAINT (1+2*N_neighbours+2*N_non_neighbours)*(p+1)+2

%     % collision avoidance constraint (by known values)
%     for i_th_neighbour = 1:N_neighbours
%         i = 1;
%         for k = i_th_neighbour*(p+1)+1 : (i_th_neighbour+1)*(p+1)
%             c_ineq(k) = - norm(x(i,:)-neigh_pos(i_th_neighbour,1:2)) + d_safe;
%             i = i + 1;
%         end
%     end
%     % collision avoidance constraint (by known values) SOFT
%     for i_th_neighbour = 1:N_neighbours
%         i = 1;
%         for k = (N_neighbours+i_th_neighbour)*(p+1)+1 : (N_neighbours+i_th_neighbour+1)*(p+1)
%             c_ineq(k) = - norm(x(i,:)-neigh_pos(i_th_neighbour,1:2)) + 1.1*d_safe + e;
%             i = i + 1;
%         end
%     end
% 
%     % collision avoidance constraint (by sensor value) HARD
%     for i_th_non_neighbour = 1:N_non_neighbours
%         i = 1;
%         for k = (2*N_neighbours+i_th_non_neighbour)*(p+1)+1 :(2*N_neighbours+i_th_non_neighbour+1)*(p+1)
%             if norm(x(1,:)-non_neigh_pos(i_th_non_neighbour,1:2)) <= (d_safe + 2*v_max * p*0.1)
%                 c_ineq(k) = - norm(x(i,:)-non_neigh_pos(i_th_non_neighbour,1:2)) + d_safe;
%             else
%                 c_ineq(k) = -(2*v_max * p*0.1);
%             end
%             i = i + 1;
%         end
%     end
%     % collision avoidance constraint (by sensor value) SOFT
%     for i_th_non_neighbour = 1:N_non_neighbours
%         i = 1;
%         for k = (2*N_neighbours+N_non_neighbours+i_th_non_neighbour)*(p+1)+1 :(2*N_neighbours+N_non_neighbours+i_th_non_neighbour+1)*(p+1)
%             if norm(x(1,:)-non_neigh_pos(i_th_non_neighbour,1:2)) <= (d_safe + 2*v_max * p*0.1)
%                 c_ineq(k) = - norm(x(i,:)-non_neigh_pos(i_th_non_neighbour,1:2)) + 1.1*d_safe+e;
%             else
%                 c_ineq(k) = -(2*v_max * p*0.1);
%             end
%             i = i + 1;
%         end
%     end
% 
%     % convergence to objective constraint (soft)
%     c_ineq((1+2*N_neighbours+2*N_non_neighbours)*(p+1)+1) = norm(x(end,:)-ref_pt(1:2)) - norm(x(1,:)-ref_pt(1:2)) + e;
%     c_ineq((1+2*N_neighbours+2*N_non_neighbours)*(p+1)+2) = e;

    %% ONLY HARD SAFETY CONSTRAINT (1+N_neighbours+N_non_neighbours)*(p+1)+2
    
    % collision avoidance constraint (by known value)
    for i_th_neighbour = 1:N_neighbours
        i = 1;
        for k = (2+i_th_neighbour)*(p+1)+1 : (2+i_th_neighbour+1)*(p+1)
            c_ineq(k) = - norm(x(i,:)-neigh_pos(i_th_neighbour,1:2)) + d_safe;
            i = i + 1;
        end
    end

    % collision avoidance constraint (by sensor value) (only nearest)
    if N_non_neighbours > 0
        nearest_non_neighbour = non_neigh_pos(1,1:2);
        for i = 1:N_non_neighbours
            if norm(x(1,:)-non_neigh_pos(i,1:2)) < norm(x(1,:)-nearest_non_neighbour)
                nearest_non_neighbour = non_neigh_pos(i,1:2);
            end
        end
        if norm(x(1,:)-nearest_non_neighbour) <= (d_safe + 2*v_max * p*0.1)
            i = 1;
            for k = (3+N_neighbours)*(p+1)+1 :(3+N_neighbours+1)*(p+1)
                c_ineq(k) = - norm(x(i,:)-nearest_non_neighbour) + d_safe;
                i = i + 1;
            end
        else
            i = 1;
            for k = (3+N_neighbours)*(p+1)+1 :(3+N_neighbours+1)*(p+1)
                c_ineq(k) = -(2*v_max * p*0.1);
                i = i + 1;
            end
        end
    else
        i = 1;
        for k = (3+N_neighbours)*(p+1)+1 :(3+N_neighbours+1)*(p+1)
            c_ineq(k) = -(2*v_max * p*0.1);
            i = i + 1;
        end
    end

    % convergence to objective constraint (soft)
    c_ineq((3+N_neighbours+1)*(p+1)+1) = norm(x(end,:)-data.References(p,1:2)) - norm(x(1,:)-data.References(p,1:2)) + e;
    c_ineq((3+N_neighbours+1)*(p+1)+2) = e;

%     % collision avoidance constraint (by sensor value)
%     for i_th_non_neighbour = 1:N_non_neighbours
%         i = 1;
%         for k = (2+N_neighbours+i_th_non_neighbour)*(p+1)+1 :(2+N_neighbours+i_th_non_neighbour+1)*(p+1)
%             if norm(x(1,:)-non_neigh_pos(i_th_non_neighbour,1:2)) <= (d_safe + 2*v_max * p*0.1)
%                 c_ineq(k) = - norm(x(i,:)-non_neigh_pos(i_th_non_neighbour,1:2)) + d_safe;
%             else
%                 c_ineq(k) = -(2*v_max * p*0.1);
%             end
%             i = i + 1;
%         end
%     end
    
%     % convergence to objective constraint (soft)
%     c_ineq((1+N_neighbours+N_non_neighbours)*(p+1)+1) = norm(x(end,:)-ref_pt(1:2)) - norm(x(1,:)-ref_pt(1:2)) + e;
%     c_ineq((1+N_neighbours+N_non_neighbours)*(p+1)+2) = e;

%     % convergence to objective constraint (soft)
%     i = 1;
%     for k = (2+N_neighbours+N_non_neighbours+1)*(p+1)+1 :(2+N_neighbours+N_non_neighbours+1+1)*(p+1)
% %         c_ineq(k) = (dx(i,:)*(x(i,:)-ref_pt(1:2)).')/norm(x(i,:)-ref_pt(1:2));
%         c_ineq(k) = (dx(i,:)*(x(i,:)-data.References(i,1:2)).')/norm(x(i,:)-data.References(i,1:2));
%         i = i + 1;
%     end

%     if any( c_ineq > 0 )
%         fprintf("Some constraint has been violated\n");
%     end

end