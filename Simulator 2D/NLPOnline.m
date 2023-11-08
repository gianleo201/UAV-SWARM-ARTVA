%% BUILD CONSTRAINTS

CNSTR = buildConstraints(TIME_STEP, ...
                         N, ...
                         N_approx_bernstain, ...
                         recievers_pos_ode, ...
                         transmitter_pos_hat(1:2), ...
                         d_t, ...
                         d_safe, ...
                         v_max);

%% BUILD INITIAL GUESS

NLP_Bns_X0 = zeros(N,2,N_approx_bernstain+1);
% NLP_tf_X0 = t_f;

% subdivide transmitter estimate position surrounding area in circular
% sectors
DELTA_angle = 2*pi/N;
available_sectors = zeros(1,N);
for i = 1:N
    available_sectors(i) = i;
end

% assign each drone to the nearest sector
distances_matrix = zeros(N,N);
sector_assignments = zeros(1,N);
r_safe = sqrt( (d_safe^2) / ( 2*(1-cos(DELTA_angle)) ) );
max_min_dist = 0;
for i=1:N
    % ith drone
    for j=1:N
        % jth sector
        j_th_angle = (j-1)*DELTA_angle;
        distances_matrix(i,j) = norm(transmitter_pos_hat(1:2)+1.25*r_safe*[cos(j_th_angle) sin(j_th_angle)]-recievers_pos(i,1:2));
    end
    [~, min_dist_i] = min(distances_matrix(i,:));
    sector_assignments(i) = min_dist_i; 
end
for i=1:N
    [min_dist_curr, min_index] = min(reshape(distances_matrix,1,[]));
    i_th_drone = mod(min_index-1,N)+1;
    i_th_sector = ((min_index-i_th_drone)/N)+1;
    distances_matrix(i_th_drone,:) = inf(1,N);
    distances_matrix(:,i_th_sector) = inf(N,1);
    sector_assignments(i_th_drone) = i_th_sector;
    if i == N
        max_min_dist = min_dist_curr;
    end
end

min_feasable_tf = (2*max_min_dist/v_max);
NLP_tf_X0 = min_feasable_tf;

for i=1:N
    
    % initial position = UAV position
    NLP_Bns_X0(i,:,1) = recievers_pos_ode(i,1:2).'; 

    % initial velocity = UAV velocity
    NLP_Bns_X0(i,:,2) = NLP_Bns_X0(i,:,1).' + ...
            (NLP_tf_X0/N_approx_bernstain) * recievers_pos_ode(i,3:4).';

    % reach the area of the estimated transmitted position divided in radial
    % sectors (to each drone the nearest sector)
    i_th_angle = (sector_assignments(i)-1)*DELTA_angle;
    i_th_final = transmitter_pos_hat(1:2)+1.25*r_safe*[cos(i_th_angle) sin(i_th_angle)];
    NLP_Bns_X0(i,:,N_approx_bernstain+1) = i_th_final.'; % final position =  transmitter estimate


%     % reach the area of the estimated transmitted position divided in
%     % sectors (to each drone the nearest sector)
%     r_safe = sqrt( (d_safe^2) / ( 2*(1-cos(DELTA_angle)) ) ); % cannot enter in a circle area of radius r_safe in order to respect collision avoidance constriant
%     % find nearest available sector
%     N_sec = count_available_sectors(available_sectors);
%     distances_to_sectors = zeros(1,N_sec);
%     index_list = zeros(1,N_sec);
%     k_sec = 1;
%     for j=1:length(available_sectors)
%         if available_sectors(j) ~= 0
%             i_th_angle = (j-1)*DELTA_angle;
%             distances_to_sectors(k_sec) = norm(transmitter_pos_hat(1:2)+1.25*r_safe*[cos(i_th_angle) sin(i_th_angle)]-recievers_pos(i,1:2));
%             index_list(k_sec) = j;
%             k_sec = k_sec + 1;
%         end
%     end
%     [~, target_sector] = min(distances_to_sectors);
%     available_sectors(index_list(target_sector)) = 0; % no more available sector
%     i_th_angle = index_list(target_sector)*DELTA_angle;
%     i_th_final = transmitter_pos_hat(1:2)+1.25*r_safe*[cos(i_th_angle) sin(i_th_angle)];
%     NLP_Bns_X0(i,:,N_approx_bernstain+1) = i_th_final.'; % final position =  transmitter estimate

%     % reach the area but stay away from other drones
%     i_th_final_direction = transmitter_pos_hat(1:2)-recievers_pos(i,1:2);
%     r_safe = sqrt( (d_safe^2) / ( 2*(1-cos(DELTA_angle)) ) ); % cannot enter in a circle area of radius r_safe in order to respect collision avoidance constriant
%     i_th_final_direction = i_th_final_direction*(1-(r_safe/norm(i_th_final_direction)));
%     for j=1:N
%         if j ~= i
%             collision_direction = recievers_pos(j,1:2)-recievers_pos(i,1:2);
%             i_th_final_direction = i_th_final_direction - (collision_direction/(collision_direction*collision_direction.'));
%         end
%     end
%     i_th_final = recievers_pos(i,1:2) + i_th_final_direction;
%     % check if found final point is within the final area around the
%     % estimate
%     critical_direction = i_th_final-transmitter_pos_hat(1:2);
%     if norm(critical_direction) > d_t
%         % if not inside the area move inside it along the radius
%         fprintf("A trajectory with a too far ending point has been computed. Rejecting it near the estimate transmitter position ...\n");
%         i_th_final = i_th_final-(1-(d_t/norm(critical_direction)))*critical_direction(1:2);
%     end
%     NLP_Bns_X0(i,:,N_approx_bernstain+1) = i_th_final.';
    
    
% %     if norm(transmitter_pos_hat(1:2)-recievers_pos_ode(i,1:2)) <= d_t
%     if true
% %         assign remaining points to a straight trejectory from start to
% %         i_th_final
%         temp_traj = i_th_final-recievers_pos_ode(i,1:2);
%         for kth=3:N_approx_bernstain
%             NLP_Bns_X0(i,:,kth) = recievers_pos_ode(i,1:2) + ((kth-2)/(N_approx_bernstain-1))*temp_traj;
%         end
%     else
%         % assign remaing points to a trajectory with radial arrival
%         temp_traj = i_th_final-transmitter_pos_hat(1:2);
%         temp_traj_dir = temp_traj/norm(temp_traj);
%         approaching_length = d_t-norm(temp_traj);
%         for kth=3:N_approx_bernstain
%             NLP_Bns_X0(i,:,kth) = transmitter_pos_hat(1:2) + (d_t - ((kth-2)/(N_approx_bernstain-1))*approaching_length)*temp_traj_dir;
%         end
%     end

    % assign remaining points to a straight trejectory from start to
    % i_th_final
    temp_traj = i_th_final-recievers_pos_ode(i,1:2);
    for kth=3:N_approx_bernstain
        NLP_Bns_X0(i,:,kth) = recievers_pos_ode(i,1:2) + ((kth-2)/(N_approx_bernstain-1))*temp_traj;
    end

%     % assign remaining points to a straight trejectory with 0 final
%     % velocity
%     NLP_Bns_X0(i,:,N_approx_bernstain) = NLP_Bns_X0(i,:,N_approx_bernstain+1); % final velocity = 0
%     temp_traj = i_th_final-recievers_pos_ode(i,1:2);
%     for kth=3:N_approx_bernstain-1
%         NLP_Bns_X0(i,:,kth) = recievers_pos_ode(i,1:2) + ((kth-2)/(N_approx_bernstain-2))*temp_traj;
%     end

%     % overwrite all previous mods
%     for kth=3:N_approx_bernstain+1
%         NLP_Bns_X0(i,:,kth) = NLP_Bns_X0(i,:,2);
%     end

end

%% NLP

% set initial guess
problem.x0 = [NLP_tf_X0 reshape(NLP_Bns_X0,1,[])];

% set linear constraints
vec_length = length(reshape(NLP_Bns_X0,1,[]))+1;
num_eq_constr = 4*N;
A_clin = zeros(num_eq_constr,vec_length);
B_clin = zeros(num_eq_constr,1);
p = 1;
for k=1:N
    for i=1:2
        for j = 1:N_approx_bernstain+1
            curr_i = (k-1)+(i-1)*N+(j-1)*2*N+1;
            if j==1
                A_clin(p,curr_i+1) = 1;
                B_clin(p) = recievers_pos_ode(k,i);
                p = p + 1;
            elseif j==2
                A_clin(p,curr_i+1) = 1;
                A_clin(p,1) = -recievers_pos_ode(k,2+i)/N_approx_bernstain;
                relative_curr_pos = (k-1)+(i-1)*N+1;
                A_clin(p,relative_curr_pos+1) = -1;
                p = p + 1;
            end
        end
    end
end
problem.Aeq = A_clin;
problem.beq = B_clin;

% check initial condition
init_cond_feasability_check = (abs(A_clin*problem.x0.'-B_clin) >= 1e-06);
if any(init_cond_feasability_check)
    fprintf("Initial conditions doesn't satisfy constraints\n");
    display(init_cond_feasability_check);
    fprintf("Dummy row\n");
else
    fprintf("Initial conditions satisfies constraints\n");
end

A_clin_ineq = zeros(1,vec_length);
A_clin_ineq(1) = -1;
B_clin_ineq = -min_feasable_tf;
problem.Aineq = A_clin_ineq;
problem.bineq = B_clin_ineq;


% set nonlinear constraints
problem.nonlcon = CNSTR;

% % change weights
% problem.OF = buildObjectiveFunction(ObjectiveWeights .* [1 0 0] ...
%                                     ,N,TIME_STEP,N_approx_bernstain);

tic;
[x_opt,f_opt] = fmincon(problem);
fprintf("Total elpsed time to solve minimization problem: %d\n",toc);

% save results
t_f = x_opt(1);
Bns = reshape(x_opt(2:end),N,2,N_approx_bernstain+1);

% explicit computation of the trajecrtory points
ComputeTrajs;