% build constraints
CNSTR = buildConstraints(TIME_STEP, ...
                         N, ...
                         N_approx_bernstain, ...
                         recievers_pos_ode, ...
                         transmitter_pos_hat(1:2), ...
                         d_t, ...
                         d_safe, ...
                         v_max);

% build initial conditions
NLP_Bns_X0 = zeros(N,2,N_approx_bernstain+1);
NLP_tf_X0 = t_f;
DELTA_angle = 2*pi/N;
for i=1:N

    NLP_Bns_X0(i,:,1) = recievers_pos_ode(i,1:2).'; % initial position = UAV position

    NLP_Bns_X0(i,:,2) = NLP_Bns_X0(i,:,1).' + ...
            (NLP_tf_X0/N_approx_bernstain) * recievers_pos_ode(i,3:4).'; % initial velocity = UAV velocity

    % reach surround the area of the estimated transmitted position
%     i_th_angle = (i-1)*DELTA_angle;
%     i_th_final = transmitter_pos_hat(1:2)+0.9*d_t*[cos(i_th_angle) sin(i_th_angle)];
%     NLP_Bns_X0(i,:,N_approx_bernstain+1) = i_th_final.'; % final position =  transmitter estimate

    i_th_final_direction = transmitter_pos_hat(1:2)-recievers_pos(i,1:2);
    r_safe = sqrt(d_safe/(2*(1-cos(DELTA_angle)))); % cannot enter in a circle area of radius r_safe in order to respect collision avoidance constriant
    i_th_final_direction = i_th_final_direction*(1-(r_safe/norm(i_th_final_direction)));
    for j=1:N
        if j ~= i
            collision_direction = recievers_pos(j,1:2)-recievers_pos(i,1:2);
            i_th_final_direction = i_th_final_direction - (collision_direction/(collision_direction*collision_direction.'));
        end
    end
%     i_th_final = recievers_pos(i,1:2) + (1-(2*d_safe/norm(i_th_final_direction)))*i_th_final_direction;
    i_th_final = i_th_final_direction;
    % check if found final point is within the final area around the
    % estimate
    critical_direction = i_th_final-transmitter_pos_hat(1:2);
    if norm(critical_direction) > d_t
        % if not inside the area move inside it along the radius
        i_th_final = i_th_final-(1-(d_t/norm(critical_direction)))*critical_direction(1:2);
    end
    NLP_Bns_X0(i,:,N_approx_bernstain+1) = i_th_final.';
    
    % assign remaining points to a straight trejectory
    temp_traj = i_th_final-recievers_pos_ode(i,1:2);
    for kth=3:N_approx_bernstain
        NLP_Bns_X0(i,:,kth) = recievers_pos_ode(i,1:2) + ((kth-2)/(N_approx_bernstain-1))*temp_traj;
    end

%     NLP_Bns_X0(i,:,N_approx_bernstain) = NLP_Bns_X0(i,:,N_approx_bernstain+1); % final velocity = 0
%     temp_traj = i_th_final-recievers_pos_ode(i,1:2);
%     for kth=3:N_approx_bernstain-1
%         NLP_Bns_X0(i,:,kth) = recievers_pos_ode(i,1:2) + ((kth-2)/(N_approx_bernstain-2))*temp_traj;
%     end

end

% peek initial trajectories ( before optimizations )
Bns = NLP_Bns_X0;
ComputeTrajs;
plotComputedTrajs;

% set initial conditions
problem.x0 = [NLP_tf_X0 reshape(NLP_Bns_X0,1,[])];
problem.nonlcon = CNSTR;

% solve NLP
[x_opt,f_opt] = fmincon(problem);

% save results
t_f = x_opt(1);
Bns = reshape(x_opt(2:end),N,2,N_approx_bernstain+1);

% explicit computation of the trajecrtory points
ComputeTrajs;
