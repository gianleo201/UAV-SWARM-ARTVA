function h_f = CBF_h_f(UAV_pos,obs_pos,d_dist)
    x = UAV_pos(1);
    y = UAV_pos(2);
    z = UAV_pos(3);
    x_obs = obs_pos(1); 
    y_obs = obs_pos(2); 
    z_obs = obs_pos(3);
    h_f = ((x - x_obs)^2 + (y - y_obs)^2 + (z - z_obs)^2)^(1/2) - d_dist;
end