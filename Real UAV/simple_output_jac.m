function [J_y_x,J_y_u] = simple_output_jac(in1,in2,neigh_pos,N_neighbours,non_neigh_pos,N_non_neighbours,d_safe,v_max,in9)
    temp = eye(10,12);
    temp(10,:) = [zeros(1,11) 1];
    J_y_x = temp;
    J_y_u = zeros(10,4);
end