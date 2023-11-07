NMPC_MODIFIED = false;

Drone_NMPC = nlmpc(4,4,2);

Drone_NMPC.Ts = TIME_STEP;
Drone_NMPC.PredictionHorizon = 5;
Drone_NMPC.ControlHorizon = 5;

Drone_NMPC.Model.NumberOfParameters = 6;
Drone_NMPC.Optimization.CustomCostFcn = "ObjFunction";
Drone_NMPC.Model.StateFcn = "UAV_dynamics";
Drone_NMPC.Jacobian.StateFcn = "UAV_dynamics_jacobian";
Drone_NMPC.Model.OutputFcn = "UAV_output";
Drone_NMPC.Jacobian.OutputFcn = "UAV_output_jacobian";
Drone_NMPC.Optimization.CustomIneqConFcn = "IneqConstriant";
Drone_NMPC.Optimization.SolverOptions.Algorithm = 'sqp';

% initial state
CHECK_X0 = [-5 -5 0 0];
% initial guess for control input
CHECK_U0 = [0 0];
% parameter cell vector sample
CHECK_neigh = {zeros(N,3),N,zeros(N,3),1,d_safe,v_max};

% validate nlmpc algorithm
% Drone_NMPC.validateFcns(CHECK_X0.',CHECK_U0.',[],CHECK_neigh,[1 1 0 0]);


[coreData,onlineData] = getCodeGenerationData(Drone_NMPC,CHECK_X0.',CHECK_U0.',CHECK_neigh);
onlineData.ref = [1 1 0 0];

lst_N_exist = false;
if exist("LAST_N.mat","file")
    LAST_N = load("LAST_N.mat");
    lst_N_exist = true;
end

if ~USE_NMPC
    fprintf("NMPC not used in the simulation. No NMPC code generation\n");
elseif lst_N_exist && LAST_N.N == N && ~NMPC_MODIFIED
    fprintf("NMPC code already generated for %d UAVs\n",N);
    return;
else
    % code generation configuration
    mexConfig = coder.config("mex");
    mexConfig.IntegrityChecks = true;
    % build c code and executable
    buildMEX(Drone_NMPC,"NMPC_UAV",coreData,onlineData,mexConfig);
end

clear CHECK_X0 CHECK_U0 CHECK_neigh onlineData LAST_N;