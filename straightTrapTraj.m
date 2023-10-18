function [x,dx] = straightTrapTraj(x_i,x_f,v_cruise,t_f,Ts)
    trap_num_samples = fix( t_f / Ts ) + 1;

    % extract linear path direction
    path_dir = x_f-x_i;
    L_t = norm(path_dir);
    path_dir = path_dir/L_t;
    
    % compute straight path trapezoidal trajectories
    [q, qd, ~, tSamples, ~] = trapveltraj( ...
        [ 0 L_t ], ...
        trap_num_samples, ...
        "PeakVelocity",v_cruise, ...
        "EndTime",t_f);
    x = repmat(x_i,trap_num_samples,1)+[path_dir(1)*q.' path_dir(2)*q.'];
    dx = [path_dir(1)*qd.' path_dir(2)*qd.'];
end