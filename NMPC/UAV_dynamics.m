function dxdt = UAV_dynamics(x,u,params)

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