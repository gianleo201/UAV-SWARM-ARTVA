NMPC_MODIFIED = false;

% Drone_NMPC = nlmpc(12,4,'MV',[1 2 3 4]);
Drone_NMPC = nlmpc(12,7,'MV',[1 2 3 4]);

% N = 5;
% d_safe = 2;
% v_max = 5;

Drone_NMPC.Ts = 0.05;
Drone_NMPC.PredictionHorizon = 25;
Drone_NMPC.ControlHorizon = 25;

Drone_NMPC.Model.NumberOfParameters = 7;
% Drone_NMPC.Optimization.CustomCostFcn = "quadrotor_ObjFunction";
Drone_NMPC.Model.StateFcn = "quadrotor_model";
Drone_NMPC.Jacobian.StateFcn = "quadrotor_model_jac";
% Drone_NMPC.Model.OutputFcn = "quadrotor_output";
% Drone_NMPC.Jacobian.OutputFcn = "quadrotor_output_jac";
Drone_NMPC.Model.OutputFcn = "simple_output";
Drone_NMPC.Jacobian.OutputFcn = "simple_output_jac";
% Drone_NMPC.Optimization.CustomIneqConFcn = "quadrotor_IneqConstriant";

% constraints on inputs
for i = 1:4
    Drone_NMPC.MV(i).Min = 0;
    Drone_NMPC.MV(i).Max = 10; %N max thrust
end

% constraint on pitch angle ( avoid gimbal problem )
Drone_NMPC.States(10).Min = -0.9*pi/2;
Drone_NMPC.States(10).Max = 0.9*pi/2;
Drone_NMPC.States(11).Min = -0.9*pi/2;
Drone_NMPC.States(11).Max = 0.9*pi/2;

% constraints on height
Drone_NMPC.States(3).Min = 0.05;

% weights on input
Wu = (1/10)^2; 
Drone_NMPC.Weights.ManipulatedVariables = Wu*ones(1,4);

% assign state weights
Wx = (1/0.1)^2;
% Wdx = (1/0.01)^2;
Wdx = 0;
W_phi_theta = (1/(pi/3))^2;
W_psi = (1/(pi/12))^2;
W_pqr = (1/((0.1*pi/2)/Drone_NMPC.Ts))^2;
% Drone_NMPC.Weights.OutputVariables = [Wx*ones(1,3) Wdx*ones(1,3) W_pqr*ones(1,3) W_phi_theta*ones(1,2) W_psi];
Drone_NMPC.Weights.OutputVariables = [Wx*ones(1,3) Wdx*ones(1,3) W_psi];

% initial state
CHECK_X0 = [0 0 1 0 0 0 0 0 0 0 0 0];
% initial guess for control input
CHECK_U0 = [5 5 5 5];
% parameters array
MODEL_PARAMS_INIT;
CHECK_PARAMS = {zeros(3*N,3),0,zeros(3,3),0,d_safe,v_max,model_params};

% validate nlmpc algorithm
Drone_NMPC.validateFcns(CHECK_X0.',CHECK_U0.',[],CHECK_PARAMS,[0 0 2 0 0 0 0]);

[coreData,onlineData] = getCodeGenerationData(Drone_NMPC,CHECK_X0.',CHECK_U0.',CHECK_PARAMS);
onlineData.ref = [0 0 2 0 0 0 0];

if ~NMPC_MODIFIED
    fprintf("NMPC code already generated for %d UAVs\n",N);
    return;
else
    % code generation configuration
    mexConfig = coder.config("mex");
%     mexConfig.IntegrityChecks = true;
%     mexConfig.DynamicMemoryAllocation = 'Off';
    % build c code and executable
    buildMEX(Drone_NMPC,"real_NMPC_UAV",coreData,onlineData,mexConfig);
end
