clear;
EnvINIT; % initialize environment
MagneticFieldSensors; % build Symbolically H matrix

%% UAVS DYNAMICS (symilified to double integrators)

% imageine point robot. Can act with them with 2 cartesian normal forces Fx
% Fy
generic_A_matrix = [0 0 1 0;
                    0 0 0 1;
                    0 0 0 0;
                    0 0 0 0];
generic_B_matrix = [0 0;
                    0 0;
                    1 0;
                    0 1];
temp_A = cell(N,1);
temp_B = cell(N,1);
temp_K = cell(N,1);

% set PD controller
for i=1:N
    temp_A{i} = generic_A_matrix;
    temp_B{i} = generic_B_matrix;
    temp_K{i} = [1 0 10  0;
                 0 1  0 10]; % control gains
end

total_A_matrix = blkdiag(temp_A{:});
total_B_matrix = blkdiag(temp_B{:});
K_uavs_gain = blkdiag(temp_K{:});

%% MAINLOOP SETUP

% robots initial position setup
recievers_pos = reciever_INIT;
recievers_pos_ode = [reciever_INIT(:,1:2) zeros(N,2)];
recievers_pos_ode_history = zeros(END_TIME/TIME_STEP,N,4);
recievers_pos_ode_history(1,:,:) = recievers_pos_ode;


a = 1;
b = 2;
M_real = diag([b^2 a^2 a^2]);


% RLS algorithm setup
beta_ff = 0.95; % forgetly factor
% RLS algorithm initialization
S_RLS = diag(repmat(10,1,10)); % initial covariance matrix
transmitter_pos_hat = [2 -2 0]; % initial guess for the transmitter
last_replanning_transmitter_pos_hat = transmitter_pos_hat; % keep memory at each replanning
M_hat = [1   0      0;
         0     2    0;
         0     0    2];
% m_11 m_12 m_13 m_22 m_23 m_33 p_t_bar rho
X_hat = [M_hat(1,1) M_hat(1,2) M_hat(1,3) M_hat(2,2) M_hat(2,3) M_hat(3,3) ...
        (M_hat*transmitter_pos_hat.').' ...
         transmitter_pos_hat*M_hat*transmitter_pos_hat.'];


% set NLP
t_f = 30; % sec  ( first estimated mission time )
d_t = 5;
if ~exist('rs_dist','var')
    d_safe = 2;
else
    d_safe = rs_dist*0.75;
    fprintf("Security distance among UAVs set to: %d",d_safe);
end
v_max = 5;
ObgectiveWeights = [1 0.1 20];
OF = buildObjectiveFunction(ObgectiveWeights,N,TIME_STEP,N_approx_bernstain);
problem.objective = OF;
problem.solver = 'fmincon';
problem.options = optimoptions('fmincon', ...
                'Display','iter-detailed', ...
                'OptimalityTolerance',0.7);

if NLP_PLANNING
    NLPOnline; % solve NLP
else
    L_t = (((1+sqrt(2))/4)*hl_length);
    et = 1.5*( L_t /v_max);
    trap_num_samples = fix( et / TIME_STEP ) + 1;
    UAV_trajs = zeros(N,trap_num_samples,4);
    for i=1:N
%         reciever_END = reciever_INIT(i,1:2) + hl_length * [0 1];
        reciever_END = reciever_INIT(i,1:2) + ((1+sqrt(2))/4)*hl_length * [cos((i-0.5)*angle_sector) sin((i-0.5)*angle_sector)];
        UAV_path_dir = reciever_END-reciever_INIT(i,1:2);
        UAV_path_dir = UAV_path_dir/norm(UAV_path_dir);
        
        [q, qd, ~, tSamples, ~] = trapveltraj( ...
            [ 0 L_t ], ...
            trap_num_samples, ...
            "PeakVelocity",v_max, ...
            "EndTime",et);
        UAV_trajs(i,:,1:2) = repmat(reciever_INIT(i,1:2),trap_num_samples,1)+[UAV_path_dir(1)*q.' UAV_path_dir(2)*q.'];
        UAV_trajs(i,:,3:4) = [UAV_path_dir(1)*qd.' UAV_path_dir(2)*qd.'];
    end
end


% Event-triggered replanning thresholds
t_replanning = 8; % replanning timer [s]
RHO_THRESHOLD = 2;
SIGMA_TRESHOLD = 0.7;


% compute number of pairs of interdistances
NUM_DIST = 0;
for i = 1:N
    for j = i+1:N
        NUM_DIST = NUM_DIST + 1;
    end
end
INTER_DISTANCES = [zeros(1,NUM_DIST)];
VELOCITIES = [zeros(1,N)];
k = 1;
for i = 1:N
    VELOCITIES(end,i) = norm(recievers_pos_ode(i,3:4));
    for j = i+1:N
        INTER_DISTANCES(end,k) = norm(recievers_pos(i,:)-recievers_pos(j,:));
        k = k + 1;
    end
end



%% MAINLOOP EXECUTION

STEP = 1;
K_STEP = 2;

% start video recording
this_time = string(datetime('now'));
vid_name = "MV-"+this_time;
vid_name = regexprep(vid_name," ",":");
vid_name = regexprep(vid_name,":","-");
writerObj = VideoWriter(vid_name,'MPEG-4');
% set the seconds per image
approx_magnitude = 1;
writerObj.FrameRate = 1/TIME_STEP;
% open the writer
open(writerObj);

% show computed trajs
plotComputedTrajs;
visualize_init;
while t_simulation(STEP) < t_simulation(end)

    % augment simulation-data list
    INTER_DISTANCES = [INTER_DISTANCES;zeros(1,NUM_DIST)];
    VELOCITIES = [VELOCITIES;zeros(1,N)];


%     % if replanning timer is over run another optimization problem
%     if t_replanning <= 0
%         K_STEP = 2; % reset steps
%         NLPOnline; % solve again OCP
%         t_replanning = t_f-TIME_STEP; % reset timer
%         % show computed trajs
%         plotComputedTrajs;
%         % plot new estimated mission endtime
%         set(VIZ_END_MISSION_TIME,"String","Estimated endtime: "+string(t_simulation(STEP)+t_f)+" s");
%     end

    % Event-Triggered replanning
    if  NLP_PLANNING && t_replanning <= 0
        ESTIMATE_ERROR = norm(transmitter_pos_hat-last_replanning_transmitter_pos_hat);
        OPI = OI_function(recievers_pos_ode_history(:,:,1:2));
        if  ESTIMATE_ERROR > RHO_THRESHOLD || ...
            OPI <= SIGMA_TRESHOLD

            K_STEP = 2; % reset steps
            NLPOnline; % solve again NLP
            % show computed trajs
            plotComputedTrajs;
            % plot new estimated mission endtime
            set(VIZ_END_MISSION_TIME,"String","Estimated endtime: "+string(t_simulation(STEP)+t_f)+" s");
        end
            t_replanning = t_f-TIME_STEP; % reset timer
    end

    
    % compute new values
    H_num = H_function(recievers_pos);
    Y_num = Y_function(recievers_pos,transmitter_real_pos.',[a b],[M_real(1,1:3) M_real(2,2:3) M_real(3,3)],0,0);
    
    % RLS algorithm step
    X_hat = ( X_hat.' +  inv(S_RLS)*H_num*(Y_num-H_num.'*X_hat.') ).'; % new estimate of parameters vector;
    S_RLS = beta_ff * S_RLS + H_num*H_num.';

    % extract transmitter position estimate
    M_hat = [X_hat(1) X_hat(2) X_hat(3);X_hat(2) X_hat(4) X_hat(5); X_hat(3) X_hat(5) X_hat(6)];
    transmitter_pos_hat = (inv(M_hat)*X_hat(7:9).').'; % new transmitter estimate

    % set current reference point for UAVS
%     reshape(squeeze(UAV_trajs(:,K_STEP,:)).',1,[]).'
%     repmat([transmitter_pos_hat(1:2) 0 0 0],1,N).'
    temp_dim = size(UAV_trajs);
    if K_STEP > temp_dim(2) % if trajs have ended stay at the end
        curr_traj = squeeze(UAV_trajs(:,end,:));
        curr_traj(:,3:4) = zeros(N,2);
        UAV_references = reshape(curr_traj.',1,[]).';
    else
        UAV_references = reshape(squeeze(UAV_trajs(:,K_STEP,:)).',1,[]).';
    end
    % simulate the UAVS
    step_results = ode45( ...
    UAV_TEAM(UAV_references,K_uavs_gain,total_A_matrix,total_B_matrix), ...
          [0 TIME_STEP], ...
          reshape(recievers_pos_ode.', 1, []));
    recievers_pos_ode = reshape(step_results.y(:,end),4,N).';
    recievers_pos = [ recievers_pos_ode(:,1:2) zeros(N,1)];

    % compute interdistances
    k = 1;
    for i = 1:N
        VELOCITIES(end,i) = norm(recievers_pos_ode(i,3:4));
        for j = i+1:N
            INTER_DISTANCES(end,k) = norm(recievers_pos(i,:)-recievers_pos(j,:));
            k = k + 1;
        end
    end

    % prepare next step
    STEP = STEP + 1;
    K_STEP = K_STEP + 1;
    t_replanning = t_replanning-TIME_STEP;
    
    % update states history
    recievers_pos_ode_history(STEP,:,:) = recievers_pos_ode;

    % visualize step
    pause(TIME_STEP);
    visualize_update;

end

hold off;

% stop and save video recorded
if ~exist('./Movies', 'dir')
       mkdir('./Movies')
end
close(writerObj);
movefile(vid_name+".mp4",'./Movies/.');

%% PLOTS

% plot inter distances
figure(2);
grid on; hold on;
title("UAVs inter distances");
xlabel("time [s]");
ylabel("distance [m]");
for i = 1:NUM_DIST
    plot(t_simulation,INTER_DISTANCES(:,i),"LineWidth",2.0);
end
plot(t_simulation,repmat(d_safe,length(t_simulation),1),"--","Color","Black","LineWidth",1.5);
hold off;

% plot UAVs velocities
figure(3);
grid on; hold on;
title("UAVs velocities");
xlabel("time [s]");
ylabel("velocity [m/s]");
for i = 1:N
    plot(t_simulation,VELOCITIES(:,i),"LineWidth",2.0,"Color",color_list(i));
end
plot(t_simulation,repmat(v_max,length(t_simulation),1),"--","Color","Black","LineWidth",1.5);
hold off;


%% UAV DYNAMICS ODE

function UAV_TEAM_ODE = UAV_TEAM(x_ref,K,p1,p2)
    UAV_TEAM_ODE = @(t,x) p1*x+p2*K*(x_ref-x);
end
