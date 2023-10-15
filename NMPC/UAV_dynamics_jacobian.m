function [Jx, Ju] = UAV_dynamics_jacobian(x,u,params)
    Jx = [0 0 1 0;
         0 0 0 1;
         0 0 0 0;
         0 0 0 0];
    Ju = [0 0;
         0 0;
         1 0;
         0 1];
end