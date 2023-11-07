
close all;

%% EXECUTION FLAGS

NLP_PLANNING = true;
DOUBLE_PHASE = false;
RECORD_VIDEO = false;
COMPUTING_DEVICE_DELAY = false;
USE_NMPC = false;
EXPLORATION_TYPE = "V";  %% select one in the following string list ["V","H","R"]

%% ENVIRONMENT INITIALIZATION

% set up number of reciever agent
N = 4;

% set transimetter initial position
% orientation is not of interest, imagine the transmitter frame parallel to the inertial one
transmitter_real_pos = [6 6 1];

% initial guess for the transmitter
transmitter_pos_hat = [0 0 3];

% color list to indetify each UAV
color_list = ["green","blue"," #FF8000","cyan","magenta"];

% degree of approximation of trajectories ( at least 3rd order )
N_approx_bernstain = 5;

% set up initial conditions for the agents
reciever_INIT = zeros(N,3); %2-dimensional environment. 3d component is always 0.
reciever_INIT(:,3) = 1.5*ones(size(reciever_INIT,1),1);

% generic spawn set
% reciever_INIT(1,1:2) = [10 0];
% reciever_INIT(2,1:2) = [0 10];
% reciever_INIT(3,1:2) = [-10 0];
% reciever_INIT(4,1:2) = [0 -10];
% reciever_INIT(5,1:2) = [7.071 -7.071];

hl_length = 10; % [m]

if EXPLORATION_TYPE == "V"
    % horizontal spawn ( vertical exploration )
    rs_dist = hl_length/N; % [m]
    for i=1:N
        reciever_INIT(i,1:2) = [transmitter_pos_hat(1)-(hl_length/2)+(i-0.5)*rs_dist ...
                                transmitter_pos_hat(2)-(hl_length/2)];
    end
end

if EXPLORATION_TYPE == "H"
    % vertical spawn ( horizontal exploration )
    rs_dist = hl_length/N; % [m]
    for i=1:N
        reciever_INIT(i,1:2) = [transmitter_pos_hat(1)-(hl_length/2) ...
                                transmitter_pos_hat(2)-(hl_length/2)+(i-0.5)*rs_dist];
    end
end

if EXPLORATION_TYPE == "R"
    % radial spawn
    angle_sector = (pi/2)/N; % [rad]
    for i=1:N
        reciever_INIT(i,1:2) = [transmitter_pos_hat(1)-(hl_length/2)+(hl_length/4)*cos((i-0.5)*angle_sector)
                                transmitter_pos_hat(2)-(hl_length/2)+(hl_length/4)*sin((i-0.5)*angle_sector)];
    end
    rs_dist = sqrt( 2*(hl_length/4)^2*(1-cos(angle_sector)) );
end


%% SIMULATION TIME

END_TIME = 50; % secs 2 mins of simulation
TIME_STEP = 0.05; % secs
t_simulation = 0:TIME_STEP:END_TIME;


%% SOME PARAMETERS SETUP

if ~exist('rs_dist','var')
    d_safe = 3;
else
    if rs_dist <= 6 
        d_safe = rs_dist*0.5;
    else
        d_safe = 3;
    end
    fprintf("Security distance among UAVs set to: %.2f\n",d_safe);
end
v_max = 5; % [m/s]

%% NMPC CONTROLLER INIT

NMPC_INIT;
