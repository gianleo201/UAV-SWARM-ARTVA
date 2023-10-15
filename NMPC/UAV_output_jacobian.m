function [Jx, Ju] = UAV_output_jacobian(x,u,params)
    Jx = eye(4);
    Ju = zeros(2,2);
end