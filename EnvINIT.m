%% EXECUTION FLAGS

NLP_PLANNING = false;
DOUBLE_PHASE = true;
RECORD_VIDEO = false;

%% ENVIRONMENT INITIALIZATION

% set transimetter initial position
% orientation is not of interest, imagine the transmitter frame parallel to the inertial one
transmitter_real_pos = [-5 5 0];

% initial guess for the transmitter
transmitter_pos_hat = [0 0 0];

% set up number of reciever agent
N = 4;

% build up a color list do indetify each UAV
color_list = ["green","black","yellow","cyan","red"];

% degree of approximation of trajectories ( at least 3rd order )
N_approx_bernstain = 5;

% set up initial conditions for the agents
reciever_INIT = zeros(N,3); %2-dimensional environment. 3d component is always 0.

% generic spawn set
% reciever_INIT(1,1:2) = [10 0];
% reciever_INIT(2,1:2) = [0 10];
% reciever_INIT(3,1:2) = [-10 0];
% reciever_INIT(4,1:2) = [0 -10];
% reciever_INIT(5,1:2) = [7.071 -7.071];

% % horizontal spawn ( vertical exploration )
% hl_length = 20; % [m]
% rs_dist = hl_length/N; % [m]
% for i=1:N
%     reciever_INIT(i,1:2) = [transmitter_pos_hat(1)-(hl_length/2)+(i-0.5)*rs_dist ...
%                             transmitter_pos_hat(2)-(hl_length/2)];
% end

% % vertical spawn ( horizontal exploration )
% hl_length = 15; % [m]
% rs_dist = hl_length/N; % [m]
% for i=1:N
%     reciever_INIT(i,1:2) = [transmitter_pos_hat(1)-(hl_length/2) ...
%                             transmitter_pos_hat(2)-(hl_length/2)+(i-0.5)*rs_dist];
% end

% radial spawn
hl_length = 20; % [m]
angle_sector = (pi/2)/N; % [rad]
for i=1:N
    reciever_INIT(i,1:2) = [transmitter_pos_hat(1)-(hl_length/2)+(hl_length/4)*cos((i-0.5)*angle_sector)
                            transmitter_pos_hat(2)-(hl_length/2)+(hl_length/4)*sin((i-0.5)*angle_sector)];
end
rs_dist = sqrt( 2*(hl_length/4)^2*(1-cos(angle_sector)) );

% simulation time
END_TIME = 30; % secs 2 mins of simulation
TIME_STEP = 0.1; % secs
t_simulation = 0:TIME_STEP:END_TIME;
