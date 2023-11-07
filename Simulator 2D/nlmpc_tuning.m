%% UAV NMPC TRIAL CODE

% close all;

CPP_CODE_EXEC = false;

Drone_NMPC = nlmpc(4,4,2);

Drone_NMPC.Ts = 0.1;
Drone_NMPC.PredictionHorizon = 4;
Drone_NMPC.ControlHorizon = 4;

Drone_NMPC.Model.NumberOfParameters = 6;
Drone_NMPC.Optimization.CustomCostFcn = "ObjFunction";
Drone_NMPC.Model.StateFcn = "UAV_dynamics";
Drone_NMPC.Jacobian.StateFcn = "UAV_dynamics_jacobian";
Drone_NMPC.Model.OutputFcn = "UAV_output";
Drone_NMPC.Jacobian.OutputFcn = "UAV_output_jacobian";
Drone_NMPC.Optimization.CustomIneqConFcn = "IneqConstriant";
% Drone_NMPC.Optimization.SolverOptions.Algorithm = 'sqp';

transm_real_pos = [2 2 0];

% initial state
X0 = [-6 -5 0 0];
X = [X0];
% initial guess for control input
U0 = [0 0];
U = [];

% CPU time record
CPU_TIME = [];

neigh = {[ 1 1 0;
          -1 -1 0;
           0 0 0],2,[-2.5 -2.5 0;
                     0 0 0;
                     0 0 0],1,1,5};

% validate nlmpc algorithm
% Drone_NMPC.validateFcns(X0.',U0.',[],neigh,[1 1 0 0]);


[coreData,onlineData] = getCodeGenerationData(Drone_NMPC,X0.',U0.',neigh);
myReference = [transm_real_pos(1:2) 0 0];
onlineData.ref = myReference;

if CPP_CODE_EXEC
    % generate MEX function to speed up simulation
    buildMEX(Drone_NMPC,"NMPC_UAV",coreData,onlineData);
end

TIME_STEP = 0.1;
t_simulation = 0:TIME_STEP:150;

screen_info = get(0,'ScreenSize');
VIZ_H_SIZE = 500;
VIZ_W_SIZE = 600;
fig100 = figure(100); grid on;
set(fig100,'Units','Pixels','Position',[100 200 VIZ_W_SIZE VIZ_H_SIZE]);
hold on;
xlim([-10 10]);
ylim([-10 10]);
VIZ_UAV = plot(X(1:end,1),X(1:end,2),"LineWidth",2.0);
VIZ_point = plot(X(end,1),X(end,2),"x","LineWidth",2.0);
VIZ_OBJECTIVE = plot(myReference(1),myReference(2),'x','LineWidth',2,'MarkerSize',20,'Color','red');
for i = 1:neigh{2}
    i_th_center = neigh{1};
    i_th_center = i_th_center(i,:);
    plotCircle(i_th_center,neigh{5},"--",2,"red",false);
end
for i = 1:neigh{4}
    i_th_center = neigh{3};
    i_th_center = i_th_center(i,:);
    plotCircle(i_th_center,neigh{5},"--",2,"blue",false);
end
hold off;

fig50 = figure(50); grid on;
set(fig50,'Units','Pixels','Position',[screen_info(3)-VIZ_W_SIZE-50 200 VIZ_W_SIZE VIZ_H_SIZE]);
hold on;
ylim([0 TIME_STEP*3]);
hold off;

STEP = 1;
while t_simulation(STEP) < t_simulation(end)

    % compute nlmpc control action
    if STEP == 1

        if CPP_CODE_EXEC
            tic;
            [nmpc_mv, onlineData, info] = NMPC_UAV(X0.',U0.', onlineData);
            elapsed_time = toc;
        else
            opt = nlmpcmoveopt();
            opt.Parameters = neigh;
            tic;
            [nmpc_mv, opt, info] = Drone_NMPC.nlmpcmove(X0.',U0.',myReference,[],opt);
            elapsed_time = toc;
        end

        CPU_TIME = [CPU_TIME elapsed_time];
        figure(50); hold on;
        ax_fig50 = gca;
        VIZ_CPU_TIME = plot(t_simulation(1:STEP),CPU_TIME(1:end),"LineWidth",2.0,"Color","Black");
        VIZ_CPU_MAX_TIME = plot(t_simulation(1:STEP),TIME_STEP*ones(1,STEP),"--","LineWidth",2.0,"Color","red");
        hold off;

    else

        if CPP_CODE_EXEC
            tic;
            [nmpc_mv, onlineData, info] = NMPC_UAV(X(end,:).',U(end,:).',onlineData);
            elapsed_time = toc;
        else
            tic;
            [nmpc_mv, opt, info] = Drone_NMPC.nlmpcmove(X(end,:).',U(end,:).',myReference,[],opt);
            elapsed_time = toc;
        end

        CPU_TIME = [CPU_TIME elapsed_time];
        set(VIZ_CPU_TIME,"XData",t_simulation(1:STEP),"YData",CPU_TIME(1:end));
        set(VIZ_CPU_MAX_TIME,"XData",t_simulation(1:STEP),"YData",TIME_STEP*ones(1,STEP));
        xlim(ax_fig50,[0 STEP*TIME_STEP]);

    end

    % check nlmpc step status
    nmpc_status = info.ExitFlag;
    if nmpc_status < 0
        fprintf("No feasable solution found\n");
    end
    

    U = [U;nmpc_mv.'];

    new_state = simulate_UAV(X(end,:).',U(end,:).',TIME_STEP);
    X = [X;new_state.'];

    % prepare next STEP
    STEP = STEP + 1;
    pause(TIME_STEP);

    % plot step
    set(VIZ_UAV,'XData',X(1:end,1),'YData',X(1:end,2));
    set(VIZ_point,'XData',X(end,1),'YData',X(end,2));
    drawnow;

end
