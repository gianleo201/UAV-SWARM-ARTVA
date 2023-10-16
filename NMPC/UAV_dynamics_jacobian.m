function [Jx, Ju] = UAV_dynamics_jacobian(x,u,neigh_pos,N_neighbours,non_neigh_pos,N_non_neighbours,d_safe, v_max)
    Jx = [0 0 1 0;
         0 0 0 1;
         0 0 0 0;
         0 0 0 0];
    Ju = [0 0;
         0 0;
         1 0;
         0 1];
end