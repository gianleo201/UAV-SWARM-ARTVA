NMPC_MODIFIED = false;

Drone_NMPC = nlmpc(12,6,4);

N = 5;
d_safe = 2;
v_max = 5;

Drone_NMPC.Ts = 0.05;
Drone_NMPC.PredictionHorizon = 5;
Drone_NMPC.ControlHorizon = 5;

Drone_NMPC.Model.NumberOfParameters = 7;
Drone_NMPC.Optimization.CustomCostFcn = "quadrotor_ObjFunction";
Drone_NMPC.Model.StateFcn = "quadrotor_model";
Drone_NMPC.Jacobian.StateFcn = "quadrotor_model_jac";
Drone_NMPC.Model.OutputFcn = "quadrotor_output";
Drone_NMPC.Jacobian.OutputFcn = "quadrotor_output_jac";
% Drone_NMPC.Optimization.CustomIneqConFcn = "quadrotor_IneqConstriant";
Drone_NMPC.Optimization.SolverOptions.Algorithm = 'sqp';

% constraints on inputs
for i = 1:4
    Drone_NMPC.ManipulatedVariables(i).Min = 0;
    Drone_NMPC.ManipulatedVariables(i).Max = 10; %N max thrust
end
% constraint on pitch angle ( avoid gimball problem )
Drone_NMPC.States(11).Min = -0.9*pi/2;
Drone_NMPC.States(11).Max = 0.9*pi/2;

% (debug) assign output weights
% Drone_NMPC.Weights.ManipulatedVariables = [0.1 0.1 0.1 0.1];
% Drone_NMPC.Weights.OutputVariables = [1 1 100 1 1 1];

% initial state
CHECK_X0 = [0 0 2 0 0 0 0 0 0 0 0 0];
% initial guess for control input
CHECK_U0 = [5 5 5 5];
% parameter cell vector sample
a = 0.1838;
b = 0.1838;
Jxx = 8.000e-06;
Jyy = 0.0009;
Jzz = 0.0009;
m = 0.52;
g = 9.81;
rT = 0.001;
rD = 0.0000001;
rD_vz = 0.00001;
sym_model_params = [a b Jxx Jyy Jzz m g rT rD rD_vz];
CHECK_neigh = {zeros(N,3),0,zeros(N,3),0,d_safe,v_max,sym_model_params};

% validate nlmpc algorithm
% Drone_NMPC.validateFcns(CHECK_X0.',CHECK_U0.',[],CHECK_neigh,[1 1 0 0]);


[coreData,onlineData] = getCodeGenerationData(Drone_NMPC,CHECK_X0.',CHECK_U0.',CHECK_neigh);
onlineData.ref = [1 1 1 0 0 0];



if ~NMPC_MODIFIED
    fprintf("NMPC code already generated for %d UAVs\n",N);
    return;
else
    % code generation configuration
    mexConfig = coder.config("mex");
    mexConfig.IntegrityChecks = true;
    % build c code and executable
    buildMEX(Drone_NMPC,"real_NMPC_UAV",coreData,onlineData,mexConfig);
end

clear CHECK_X0 CHECK_U0 CHECK_neigh;