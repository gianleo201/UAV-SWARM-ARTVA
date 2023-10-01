EnvINIT;

transmitter_pos_hat = [-1 -1];

OF = buildObjectiveFunction([1 1 10],N,TIME_STEP,N_approx_bernstain);
CNSTR = buildConstraints(N,N_approx_bernstain,[reciever_INIT(:,1:2) zeros(N,2)],transmitter_pos_hat,5,3,5);

problem.objective = OF;
NLP_Bns_X0 = zeros(N,2,N_approx_bernstain+1);
NLP_tf_X0 = 30;
for i=1:N

    NLP_Bns_X0(i,:,1) = reciever_INIT(i,1:2).'; % initial position = UAV position
    NLP_Bns_X0(i,:,N_approx_bernstain+1) = transmitter_pos_hat.'; % final position =  transmitter estimate
    NLP_Bns_X0(i,:,2) = NLP_Bns_X0(i,:,1).' + ...
                    (NLP_tf_X0/N_approx_bernstain) * zeros(2,1); % initial velocity = UAV velocity
    NLP_Bns_X0(i,:,N_approx_bernstain) = NLP_Bns_X0(i,:,N_approx_bernstain+1); % final velocity = 0

    % assign the rest of the midpoints to some equidistant straigth path
    % midpoints
    temp_traj = transmitter_pos_hat-reciever_INIT(i,1:2);
    NLP_Bns_X0(i,:,3) = reciever_INIT(i,1:2) + (1/3)*temp_traj;
    NLP_Bns_X0(i,:,4) = reciever_INIT(i,1:2) + (2/3)*temp_traj;

end

% set initial conditions
problem.x0 = [NLP_tf_X0 reshape(NLP_Bns_X0,1,[])];

% set constraints
problem.nonlcon = CNSTR;

% set solver options
problem.solver = 'fmincon';
problem.options = optimoptions('fmincon');
problem.options.MaxFunctionEvaluations = 200;

% solve NLP
[x_opt,f_opt] = fmincon(problem);

% save results
new_tf = x_opt(1);
new_Bns = reshape(x_opt(2:end),N,2,N_approx_bernstain+1);