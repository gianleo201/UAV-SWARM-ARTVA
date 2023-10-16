function [Jx, Ju] = UAV_output_jacobian(x,u,neigh_pos,N_neighbours,non_neigh_pos,N_non_neighbours,d_safe, v_max)
    Jx = eye(4);
    Ju = zeros(2,2);
end