function h_f_dot = CBF_h_f_dot(UAV_pos_vel,obs_pos_vel,d_dist)
    x = UAV_pos_vel(1);
    y = UAV_pos_vel(2);
    z = UAV_pos_vel(3);
    dx = UAV_pos_vel(4);
    dy = UAV_pos_vel(5);
    dz = UAV_pos_vel(6);
    x_obs = obs_pos_vel(1); 
    y_obs = obs_pos_vel(2); 
    z_obs = obs_pos_vel(3);
    x_obs_dot = obs_pos_vel(1);
    y_obs_dot = obs_pos_vel(2);
    z_obs_dot = obs_pos_vel(3);
    h_f_dot = (dx*x - dx*x_obs + dy*y - dy*y_obs + dz*z - dz*z_obs - x*x_obs_dot + x_obs*x_obs_dot - y*y_obs_dot + y_obs*y_obs_dot - z*z_obs_dot + z_obs*z_obs_dot)/((x - x_obs)^2 + (y - y_obs)^2 + (z - z_obs)^2)^(1/2);
end