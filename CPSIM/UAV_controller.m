fprintf('Program started\n');

client = RemoteAPIClient();
sim = client.getObject('sim');
UAV = sim.getObject('/Quadcopter');
UAV_propeller = cell(1,4);
for i=1:4
    UAV_propeller{i} = sim.getObject("/Quadcopter/propeller["+num2str(i-1)+"]/joint");
end
% UAV = cell(N,4);
% for i = 1:N
%     for j = 1:4
%         UAV{i,j} = sim.getObject(['/Quadcopter',num2str(i-1),'/propeller[',num2str(j-1),']/joint']);
%     end
% end


sim.setObjectPosition(UAV,sim.handle_world,{0,0,2});

ref = [1 1 2 0 0 0];
onlineData.ref = ref;

%% START SIM

timevec = 0:0.1:10;
STEP = 1;

% enable the stepping mode on the client:
client.setStepping(true);

U = [0.2943 0.2943 0.2943 0.2943];

sim.startSimulation();

while timevec(STEP) < timevec(end)
    % extract UAV information
    x_k = getState(sim,UAV);

    % do nmpc step
    [nmpc_mv, new_onlineData, info] = NMPC_STEP(Drone_NMPC, ...
                                            x_k.', ...
                                            U.', ...
                                            onlineData, ...
                                            true);
    if info.ExitFlag <= 0
        fprintf("no_feasable_solution_found\n");
    end
    U = nmpc_mv.';
    onlineData = new_onlineData;

    % actuate uav
    control_UAV(sim,UAV_propeller,U);
    client.step();
    pause(0.05);

end

sim.stopSimulation();

fprintf('Program ended\n');


%% functions

function UAV_state = getState(sim,UAV)
    T_temp = sim.getObjectMatrix(UAV,sim.handle_world);
    x = zeros(1,3);
    x(1) = double(T_temp{4});
    x(2) = double(T_temp{8});
    x(3) = double(T_temp{12});
    R = zeros(3,3);
    R(1,1) = double(T_temp{1}); 
    R(1,2) = double(T_temp{2}); 
    R(1,3) = double(T_temp{3});
    R(2,1) = double(T_temp{5});
    R(2,2) = double(T_temp{6});
    R(2,3) = double(T_temp{7});
    R(3,1) = double(T_temp{9});
    R(3,2) = double(T_temp{10});
    R(3,3) = double(T_temp{11});
    rpy = rotm2eul(R);
    [dx,w] = sim.getObjectVelocity(UAV,sim.handle_world);
    dx_mat = zeros(1,3);
    w_mat = zeros(1,3);
    for i = 1:3
        dx_mat(i) = double(dx{i});
        w_mat(i) = double(w{i});
    end
    w_mat = (R.' * w_mat.').';
    UAV_state = [x dx_mat w_mat rpy];
end

function control_UAV(sim,UAV_propeller_list,controls)
    for i = 1:length(UAV_propeller_list)
        sim.setJointTargetVelocity(UAV_propeller_list{i},controls(i));
    end
end
