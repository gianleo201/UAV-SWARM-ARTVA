function dxdt = UAV_dynamics(x,u,neigh_pos,N_neighbours,non_neigh_pos,N_non_neighbours,d_safe, v_max)

    A = [0 0 1 0;
         0 0 0 1;
         0 0 0 0;
         0 0 0 0];
    B = [0 0;
         0 0;
         1 0;
         0 1];
    
    dxdt = A*x+B*u;

end