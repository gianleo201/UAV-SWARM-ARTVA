classdef UAVNetwork < handle

    properties
        NUM_UAVS;
        network_matrix;
        UAVS;
        compute_Y;
        TIME_STEP;
        NMPC_handle;
        THREE_DIMENSIONAL;
        d_safe;
        v_max;
        DELAY_SYNC;
        FOLLOW_TRAJECTORY;
        SENSING_HORIZON;
    end

    methods
        function obj = UAVNetwork(N,TIME_STEP,network_matrix,recievers_pos_ode0,X_hat0,S_RLS0,compute_Y,NMPC_handle,d_safe,v_max,MODE,model_params,UAV_trajs)
            %%
            % flags
            obj.DELAY_SYNC = -1;
            obj.FOLLOW_TRAJECTORY = false;

            obj.THREE_DIMENSIONAL = false;
            if  nargin > 10 &&  MODE == "3D"
                obj.THREE_DIMENSIONAL = true;
            end

            % main variables
            obj.NUM_UAVS = N;
            obj.network_matrix = network_matrix;
            obj.compute_Y = compute_Y;
            obj.UAVS = cell(obj.NUM_UAVS,1);
            obj.TIME_STEP = TIME_STEP;
            obj.NMPC_handle = NMPC_handle;
            obj.d_safe = d_safe;
            obj.v_max = v_max;

            obj.SENSING_HORIZON = 2 * d_safe;


            %% selection matrices and lists
            for i = 1 : obj.NUM_UAVS
                UAV_struct = {};
                UAV_struct.TRANSMITTER_FOUND = false;
                UAV_struct.T_REPLANNING = 4;
                UAV_struct.selection_matrix = [];
                UAV_struct.selection_list = [];
                UAV_struct.complementar_selection_matrix = [];
                UAV_struct.complementar_selection_list = [];
                if nargin > 11
                    UAV_struct.model_params = model_params;
                end
                for j = 1 : obj.NUM_UAVS
                    if network_matrix(i,j) == 1
                        ek = eye(j,obj.NUM_UAVS);
                        ek = ek(j,:);
                        UAV_struct.selection_matrix = [UAV_struct.selection_matrix; ek];
                        UAV_struct.selection_list = [UAV_struct.selection_list j];
                    else
                        ek = eye(j,obj.NUM_UAVS);
                        ek = ek(j,:);
                        UAV_struct.complementar_selection_matrix = [UAV_struct.complementar_selection_matrix; ek];
                        UAV_struct.complementar_selection_list = [UAV_struct.complementar_selection_list j];
                    end
                end
                UAV_struct.dispatcher = Dispatcher(length(UAV_struct.selection_list),TIME_STEP,recievers_pos_ode0(UAV_struct.selection_list,:),X_hat0,S_RLS0);
                % add prev pos and velocity variable to estimate velocity
                % and acceleration
%                 UAV_struct.prev_neigh_pos = zeros(N,3);
%                 UAV_struct.prev_neigh_vel = zeros(N,3);
%                 UAV_struct.prev_non_neigh_values = zeros(N,3);
                UAV_struct.TRACKED_OBS = false;
                if nargin > 12
                    obj.FOLLOW_TRAJECTORY = true;
                    UAV_struct.trajectory_ref = squeeze(UAV_trajs(i,:,:));
                    UAV_struct.trajectory_step = 1;
                end
                obj.UAVS{i} = UAV_struct;
            end

            %% initialize DRLS algorithm
            for i = 1 : obj.NUM_UAVS
                obj.UAVS{i}.X_hat = X_hat0;
                obj.UAVS{i}.S_RLS = S_RLS0;
                % extract transmitter estimate position
                X_hat = obj.UAVS{i}.X_hat;
                M_hat = [X_hat(1) X_hat(2) X_hat(3);X_hat(2) X_hat(4) X_hat(5); X_hat(3) X_hat(5) X_hat(6)];
%                 old_transmitter_pos_hat = transmitter_pos_hat;
                obj.UAVS{i}.transmitter_pos_hat = (inv(M_hat)*X_hat(7:9).').'; % new transmitter estimate
                obj.UAVS{i}.last_transmitter_pos_hat = obj.UAVS{i}.transmitter_pos_hat;
                obj.UAVS{i}.last_trustable_transmitter_pos_hat = obj.UAVS{i}.transmitter_pos_hat;
            end

            %% initialize UAV controller
            for i = 1 : obj.NUM_UAVS

                %% known uavs
                temp_mask = (obj.UAVS{i}.selection_list ~= i);
                num_neigh = length(obj.UAVS{i}.selection_list)-1;
                if ~isempty(obj.UAVS{i}.selection_matrix)
                    if ~obj.THREE_DIMENSIONAL
                        neigh = obj.UAVS{i}.selection_matrix * [recievers_pos_ode0(:,1:2) zeros(size(recievers_pos_ode0,1),1)];
                    else
                        neigh = obj.UAVS{i}.selection_matrix * recievers_pos_ode0(:,1:3);
                    end
                    neigh = neigh(temp_mask,:);
                else
                    neigh = [];
                end
                neigh = [neigh;zeros(obj.NUM_UAVS-num_neigh,3)];
                neigh = [neigh;zeros(2*obj.NUM_UAVS,3)];

                %% sensor
%                 temp_mask1 = (obj.UAVS{i}.complementar_selection_list ~= i);
%                 num_non_neigh = obj.NUM_UAVS-num_neigh-1;
%                 if ~isempty(obj.UAVS{i}.complementar_selection_matrix)
%                     if ~obj.THREE_DIMENSIONAL
%                     non_neigh = obj.UAVS{i}.complementar_selection_matrix * [recievers_pos_ode0(:,1:2) zeros(size(recievers_pos_ode0,1),1)];
%                     else
%                         non_neigh = obj.UAVS{i}.complementar_selection_matrix * recievers_pos_ode0(:,1:3);
%                     end
%                     non_neigh = non_neigh(temp_mask1,:);
%                 else
%                     non_neigh = [];
%                 end
%                 non_neigh = [non_neigh;zeros(obj.NUM_UAVS-num_non_neigh,3)];
                if ~obj.THREE_DIMENSIONAL
                    [nearest_obs,min_dist] = UAVNetwork.compute_nearest_obs([recievers_pos_ode0(i,1:2) 0],[recievers_pos_ode0(1:size(recievers_pos_ode0,1) ~= i,1:2) zeros(size(recievers_pos_ode0,1)-1,1)]);
                    non_neigh = [nearest_obs;zeros(2,3)];
                else
                    [nearest_obs,min_dist] = UAVNetwork.compute_nearest_obs(recievers_pos_ode0(i,1:3),recievers_pos_ode0(1:size(recievers_pos_ode0,1) ~= i,1:3));
                    non_neigh = [nearest_obs;zeros(2,3)];
                end

                num_non_neigh = 3;
             
                if obj.UAVS{i}.TRACKED_OBS && min_dist >= obj.SENSING_HORIZON
                    num_non_neigh = -1;
                    non_neigh = zeros(3,3);
                    obj.UAVS{i}.TRACKED_OBS = false;
                elseif ~obj.UAVS{i}.TRACKED_OBS && min_dist <= obj.SENSING_HORIZON
                    obj.UAVS{i}.TRACKED_OBS = true;
                elseif ~obj.UAVS{i}.TRACKED_OBS && min_dist >=  obj.SENSING_HORIZON
                    num_non_neigh = -1;
                    non_neigh = zeros(3,3);
                elseif obj.UAVS{i}.TRACKED_OBS && min_dist <= obj.SENSING_HORIZON
                    non_neigh(2,:) = (non_neigh(1,:)-prev_non_neigh_matrix(1,:))/obj.NMPC_handle.PredictionHorizon;
                    non_neigh(3,:) = (non_neigh(2,:)-prev_non_neigh_matrix(2,:))/obj.NMPC_handle.PredictionHorizon;
                end
                
                %%
                X0 = recievers_pos_ode0(i,:);
                obj.UAVS{i}.lambda_factor = 0.1;  % promote robust responces
                
                if ~obj.THREE_DIMENSIONAL
                    curr_ref = EMA_const_reference(X0,[obj.UAVS{i}.transmitter_pos_hat(1:2) 0 0],obj.UAVS{i}.lambda_factor,NMPC_handle.PredictionHorizon);
                    U0 = [0 0];
                    NMPC_params = {neigh,num_neigh,non_neigh,num_non_neigh,d_safe,v_max};
                else
                    curr_ref = EMA_const_reference(X0,[obj.UAVS{i}.transmitter_pos_hat zeros(1,size(X0,2)-3)],obj.UAVS{i}.lambda_factor,NMPC_handle.PredictionHorizon);
                    curr_ref = curr_ref(:,[1 2 3 12]);
                    U0 = [0 0 0 0];
                    NMPC_params = {neigh,num_neigh,non_neigh,num_non_neigh,d_safe,v_max,model_params};
                end
                [~,OD] = getCodeGenerationData(NMPC_handle,X0.',U0.',NMPC_params);
                OD.ref = curr_ref;
                if obj.THREE_DIMENSIONAL
                    OD.MVTarget = (model_params(6)*model_params(7)/4)*ones(1,4);
                end
                obj.UAVS{i}.OD = OD;
                obj.UAVS{i}.U = U0;
                obj.UAVS{i}.X = X0;
                obj.UAVS{i}.X_ref = [obj.UAVS{i}.transmitter_pos_hat];
                obj.UAVS{i}.prediction_horizon = NMPC_handle.PredictionHorizon;
                obj.UAVS{i}.CPU_TIME = [];

                %% ACADO initialization
                if obj.THREE_DIMENSIONAL
                obj.UAVS{i}.gravity_compensation = model_params(6)*model_params(7)/4;
                obj.UAVS{i}.ACADO_input = struct;
                obj.UAVS{i}.ACADO_input.u = repmat(obj.UAVS{i}.gravity_compensation*ones(1,4),NMPC_handle.PredictionHorizon,1);
                obj.UAVS{i}.ACADO_input.x = repmat(X0,NMPC_handle.PredictionHorizon+1,1);
                % weights on input
                Wu = (1/0.0008)^2; 
                % state weights
                Wx = (1/0.01)^2;
                Wdx = (1/0.01)^2;
                % Wdx = 0;
                W_phi_theta = (1/(pi/3))^2;
                W_psi = (1/0.01)^2;
                W_pqr = (1/((1e-04*pi/2)/0.05))^2;
%                 W_pqr = 0;
                obj.UAVS{i}.ACADO_input.W = diag([Wx*ones(1,3) Wdx*ones(1,3) W_pqr*ones(1,3) W_psi Wu*ones(1,4)]);
                obj.UAVS{i}.ACADO_input.WN = diag([Wx*ones(1,3) Wdx*ones(1,3) W_pqr*ones(1,3) W_psi]);
                pole_0 = max(10,-CBF_h_f(obj.UAVS{i}.X,non_neigh(1,:),obj.d_safe)/CBF_h_f_dot(obj.UAVS{i}.X(1:6),[non_neigh(1,:) non_neigh(2,:)],obj.d_safe));
                pole_1 = 10;
                if num_non_neigh == 3
                    obj.UAVS{i}.ACADO_input.od = repmat([reshape(non_neigh.',1,[]) obj.d_safe 1 pole_0 pole_1],obj.NMPC_handle.PredictionHorizon+1,1);
                elseif num_non_neigh == -1
                    obj.UAVS{i}.ACADO_input.od = repmat([reshape(non_neigh.',1,[]) obj.d_safe 0 pole_0 pole_1],obj.NMPC_handle.PredictionHorizon+1,1);
                end

%                 obj.UAVS{i}.OD.weights.y = [Wx*ones(1,3) Wdx*ones(1,3) W_pqr*ones(1,3) W_psi];
%                 obj.UAVS{i}.OD.weights.u = zeros(1,4); %Wu*ones(1,4);
%                 obj.UAVS{i}.OD.weights.du = zeros(1,4);
                end

                %% sigma travelled
                H_temp = zeros(10,num_neigh+1);
                if ~obj.THREE_DIMENSIONAL
                    H_temp(:,1) = single_H_function([obj.UAVS{i}.X(1:2) 0]);
                else
                    H_temp(:,1) = single_H_function(obj.UAVS{i}.X(1:3));
                end
                for k = 1:num_neigh
                    if ~obj.THREE_DIMENSIONAL
                        H_temp(:,k) = single_H_function([neigh(k,1:2) 0]);
                    else
                        H_temp(:,k) = single_H_function(neigh(k,1:3));
                    end
                end
                O_sum = H_temp*H_temp.';
                obj.UAVS{i}.O_sum = O_sum;
                obj.UAVS{i}.sigma_travelled = min_sv_O(obj.UAVS{i}.O_sum,num_neigh+1);
                obj.UAVS{i}.k_for_O = 1;
            end

            %% initialize FL controller
            for i = 1:obj.NUM_UAVS
                obj.UAVS{i}.FL_THRUST = sum([1.2753 1.2753 1.2753 1.2753]);
                obj.UAVS{i}.FL_DTHRUST = 0;
            end

        end

        %% DISTRIBUTED LEAST SQUARE ALGORITHM
        function DRLS(obj,beta_ff,Y_num_arg,H_num_arg,ASYNC,TIME)
            
            %% RLS step
            new_S_RLSs = cell(1,obj.NUM_UAVS);
            new_X_HATs = cell(1,obj.NUM_UAVS);
            new_final_S_RLSs = cell(1,obj.NUM_UAVS);
            new_final_X_HATs = cell(1,obj.NUM_UAVS);
            for i = 1 : obj.NUM_UAVS
                Qs = obj.UAVS{i}.selection_matrix;
                if ASYNC
                    last_pos_info_cell = obj.UAVS{i}.dispatcher.pull_latest_info(TIME);
                    last_pos_info = extractPosOde(last_pos_info_cell);
                    if ~obj.THREE_DIMENSIONAL
                        last_pos_info = [last_pos_info(:,1:2) zeros(size(last_pos_info,1),1)];
                    end
                    H_num = H_function(size(last_pos_info,1),last_pos_info);
                    Y_num = obj.compute_Y(size(last_pos_info,1),last_pos_info);
                else
                    H_num = H_num_arg*Qs.';
                    Y_num = Qs*Y_num_arg;
                end
                
                X_hat = obj.UAVS{i}.X_hat;
                S_RLS = obj.UAVS{i}.S_RLS;
                
                % RLS step
                S_RLS = inv(beta_ff * inv(S_RLS) + H_num*H_num.');
                X_hat = ( X_hat.' +  S_RLS*H_num*(Y_num-H_num.'*X_hat.') ).'; % new estimate of parameters vector;
                
                % save step
                new_S_RLSs{i} = S_RLS;
                new_X_HATs{i} = X_hat;
            end

            %% D step

            %% update X_hat and S_RLS
            for i = 1 : obj.NUM_UAVS
                selection_list = obj.UAVS{i}.selection_list;
                X_hat = zeros(1,10);
                S_RLS_temp = zeros(10,10);
                S_RLS_temp1 = zeros(10,10);
                if ASYNC
                    info_cell = obj.UAVS{i}.dispatcher.curr_info_cell;
                    for k = 1:length(selection_list) 
                        if i == selection_list(k)
                            S_RLS_to_be_summed = new_S_RLSs{i};
                            X_hat_to_be_summed = new_X_HATs{i};
                        else
                            S_RLS_to_be_summed = info_cell{k}.S_RLS;
                            X_hat_to_be_summed = info_cell{k}.X_hat;
                        end
                        S_RLS_temp = S_RLS_temp + S_RLS_to_be_summed;
                        X_hat = X_hat + X_hat_to_be_summed;
                    end
                else
                    for k = 1 : length(selection_list)
                        if i == selection_list(k)
                            S_RLS_temp = S_RLS_temp + new_S_RLSs{i};
                            X_hat = X_hat + new_X_HATs{i};
                        else
                            S_RLS_temp = S_RLS_temp + obj.UAVS{selection_list(k)}.S_RLS;
                            X_hat = X_hat + obj.UAVS{selection_list(k)}.X_hat;
                        end
                    end
                end
                X_hat = X_hat/length(obj.UAVS{i}.selection_list);
                S_RLS = S_RLS_temp/length(obj.UAVS{i}.selection_list);
%                 X_hat = X_hat/(-0.5+length(obj.UAVS{i}.selection_list));
%                 S_RLS = S_RLS_temp/(-0.5+length(obj.UAVS{i}.selection_list));
                new_final_X_HATs{i} = X_hat;
                new_final_S_RLSs{i} = S_RLS;
            end

            %% update new S_RLSs and X_HATs
            for i = 1 : obj.NUM_UAVS
                obj.UAVS{i}.S_RLS = new_final_S_RLSs{i};
                obj.UAVS{i}.X_hat = new_final_X_HATs{i};
                X_hat = new_final_X_HATs{i};
                % update transmitter estimate position
                M_hat = [X_hat(1) X_hat(2) X_hat(3);X_hat(2) X_hat(4) X_hat(5); X_hat(3) X_hat(5) X_hat(6)];
                obj.UAVS{i}.transmitter_pos_hat = (inv(M_hat)*X_hat(7:9).').'; % new transmitter estimate

                curr_pos_hat = obj.UAVS{i}.transmitter_pos_hat;
                last_pos_hat = obj.UAVS{i}.last_transmitter_pos_hat;
                if ~obj.THREE_DIMENSIONAL
                    curr_pos_hat = curr_pos_hat(1:2);
                    last_pos_hat = last_pos_hat(1:2);
                    curr_vel = obj.UAVS{i}.X(3:4);
                else
                    curr_vel = obj.UAVS{i}.X(4:6);
                end

                if norm(curr_pos_hat - last_pos_hat) < 1e-04 ...
                    && norm(curr_vel) < 1e-02
                    obj.UAVS{i}.TRANSMITTER_FOUND = true;
                else
                    obj.UAVS{i}.TRANSMITTER_FOUND = false;
                end

%                 obj.UAVS{i}.last_transmitter_pos_hat = obj.UAVS{i}.transmitter_pos_hat;
            end

        end

        function estimates = pull_estimates(obj)
            estimates = zeros(obj.NUM_UAVS,3);
            for i = 1 : obj.NUM_UAVS
                estimates(i,:) = obj.UAVS{i}.transmitter_pos_hat;
            end
        end

        %% DATA TRANSMISSION WITH RANDOM VARIABLE
        function info_transmission(obj,UAV_positions,TIME,medium_velocity,uncertain_time_range)
            for i = 1 : obj.NUM_UAVS
                selection_list = obj.UAVS{i}.selection_list;
                info = cell(1,length(selection_list));
                for j = 1 : length(selection_list)
                    jth_struct = {};
                    jth_struct.pos_ode = UAV_positions(selection_list(j),:);
                    jth_struct.X_hat = obj.UAVS{selection_list(j)}.X_hat;
                    jth_struct.S_RLS = obj.UAVS{selection_list(j)}.S_RLS;
                    if selection_list(j) ~= i
                        %% compute delta time with deterministic + random variable
                        if ~obj.THREE_DIMENSIONAL
                            uav_poss = UAV_positions(:,1:2);
                        else
                            uav_poss = UAV_positions(:,1:3);
                        end
                        d_ij = norm(uav_poss(i,1:2)-uav_poss(selection_list(j),1:2));
                        random_alpha = rand(1,1);
                        delta_spawn_time = ( d_ij/medium_velocity ) + (2*random_alpha-1) * uncertain_time_range;
                        jth_struct.spawn_time = TIME + delta_spawn_time;
                    else
                        jth_struct.spawn_time = TIME;
                    end
                    info{j} = jth_struct;
                end
                obj.UAVS{i}.dispatcher.push_info(info);
            end
        end

        %% FL STEP
        function Us = FL_UAV_TEAM_step(obj,curr_Refs)
            Us = zeros(obj.NUM_UAVS,4);
            for i = 1:obj.NUM_UAVS
                % do FL step
                x_k_aug = [obj.UAVS{i}.X obj.UAVS{i}.FL_THRUST obj.UAVS{i}.FL_DTHRUST].';
                ref_k = [curr_Refs(i,1:2) 2 0;curr_Refs(i,3:4) 0  0;zeros(2,4)];
                T_mod = FL_step(x_k_aug,ref_k,obj.UAVS{i}.model_params);
                % compute real input
                U = input_mapping([obj.UAVS{i}.FL_THRUST;T_mod(2:4,1)],obj.UAVS{i}.model_params).';
                INV_MU = inv_input_mapping_matrix(obj.UAVS{i}.model_params);
                MAX_LIM = INV_MU*10*ones(4,1);
                MIN_LIM = zeros(4,1);
                % apply saturation function
                for j = 1:4; if U(j)< 0; U(j)=0; else; if U(j) > 10; U(j) = 10; end; end; end
                % update controller state
                obj.UAVS{i}.FL_DTHRUST = obj.UAVS{i}.FL_DTHRUST + T_mod(1)*obj.TIME_STEP;
                obj.UAVS{i}.FL_THRUST = obj.UAVS{i}.FL_THRUST + obj.UAVS{i}.FL_DTHRUST*obj.TIME_STEP;
                Us(i,:) = U;
            end
        end
        
        %% NMPC STEP

        function Us = NMPC_UAV_TEAM_step(obj)
            Us = [];
            %% delay in computing control
            if obj.DELAY_SYNC == 0
                obj.DELAY_SYNC = 2;
            elseif obj.DELAY_SYNC > 0
                for i=1:obj.NUM_UAVS
                    Us = [Us;obj.UAVS{i}.U];
                    obj.UAVS{i}.CPU_TIME = [obj.UAVS{i}.CPU_TIME obj.UAVS{i}.CPU_TIME(end)];
                end
                obj.DELAY_SYNC = obj.DELAY_SYNC - 1;
                return;
            end
            for i = 1 : obj.NUM_UAVS
                %% NMPC step
                

                if obj.FOLLOW_TRAJECTORY

                    p = obj.NMPC_handle.PredictionHorizon;
                    k_0 = obj.UAVS{i}.trajectory_step;
                    k_f = obj.UAVS{i}.trajectory_step+p-1;
                    k_suppl = 0;
                    exceeded = k_f-size(obj.UAVS{i}.trajectory_ref,1);
                    if exceeded > 0
                        k_f = k_f - exceeded;
                        k_suppl = exceeded;
                    end
                    if k_0 < size(obj.UAVS{i}.trajectory_ref,1)
                        obj.UAVS{i}.trajectory_step = obj.UAVS{i}.trajectory_step + 1;
                    end
                    temp = obj.UAVS{i}.trajectory_ref(k_0:k_f,:);
                    l = k_f-k_0+1;
                    curr_z = obj.UAVS{i}.X(3);
%                     first_part = [temp(:,1:2) curr_z*ones(l,1) temp(:,3:4) zeros(l,1) zeros(l,6)];
%                     end_part = repmat([temp(end,1:3) zeros(1,9)],k_suppl,1);

%                     first_part = [temp(:,1:2) curr_z*ones(l,1) temp(:,3:4) zeros(l,1) zeros(l,1)];
%                     end_part = repmat([temp(end,1:3) zeros(1,4)],k_suppl,1);

%                     first_part = [temp(:,1:2) curr_z*ones(l,1) temp(:,3:4) zeros(l,1) zeros(l,4)];
%                     end_part = repmat([temp(end,1:2) curr_z zeros(1,7)],k_suppl,1);

                    first_part = [temp(:,1:2) 1.5*ones(l,1) temp(:,3:4) zeros(l,1) zeros(l,4)];
                    end_part = repmat([temp(end,1:2) 1.5 zeros(1,7)],k_suppl,1);
                    obj.UAVS{i}.OD.ref = [first_part;end_part];

                    % ACADO
                    obj.UAVS{i}.ACADO_input.x0 = obj.UAVS{i}.X;
                    obj.UAVS{i}.ACADO_input.y = [[first_part;end_part] repmat(obj.UAVS{i}.gravity_compensation*ones(1,4),obj.NMPC_handle.PredictionHorizon,1)];
                    obj.UAVS{i}.ACADO_input.yN = obj.UAVS{i}.OD.ref(end,:);

                else
                    if ~obj.THREE_DIMENSIONAL
                        curr_ref = [obj.UAVS{i}.X_ref(1:2) 0 0];
                    else
                        curr_ref = [obj.UAVS{i}.X_ref zeros(1,size(obj.UAVS{i}.X,2)-3)];
                    end
    
                    obj.UAVS{i}.OD.ref = EMA_const_reference(obj.UAVS{i}.X, ...
                                    curr_ref, ...
                                    obj.UAVS{i}.lambda_factor, ...
                                    obj.UAVS{i}.prediction_horizon);
                    
%                     obj.UAVS{i}.OD.ref = obj.UAVS{i}.OD.ref(:,1:4);
                    obj.UAVS{i}.OD.ref = obj.UAVS{i}.OD.ref(:,[1 2 3 4 5 6 7 8 9 12]);

                    % ACADO
                    obj.UAVS{i}.ACADO_input.x0 = obj.UAVS{i}.X;
                    obj.UAVS{i}.ACADO_input.y = [[obj.UAVS{i}.X(:,[1 2 3 4 5 6 7 8 9 12]);obj.UAVS{i}.OD.ref] repmat(obj.UAVS{i}.gravity_compensation*ones(1,4),obj.NMPC_handle.PredictionHorizon,1)];
                    obj.UAVS{i}.ACADO_input.yN = obj.UAVS{i}.OD.ref(end,:);
                end

               
%                 tic;
%                 [nmpc_mv, new_OD, info] = NMPC_STEP(obj.NMPC_handle, ...
%                                                     obj.UAVS{i}.X.', ...
%                                                     obj.UAVS{i}.U.', ...
%                                                     obj.UAVS{i}.OD, ...
%                                                     true);
%                 elapsed_time = toc;
% 
%                 % register cpu time
%                 obj.UAVS{i}.CPU_TIME = [obj.UAVS{i}.CPU_TIME elapsed_time];
% 
%                 % check nlmpc step status
%                 nmpc_status = info.ExitFlag;
%                 if nmpc_status < 0
%                     fprintf("No feasable solution found for drone %d\n",i);
%                     fprintf("Exit flag number: %d\n",nmpc_status);
%                     if ~obj.THREE_DIMENSIONAL
%                         obj.UAVS{i}.U = - obj.UAVS{i}.X(3:4); % stop the UAV
%                     else
% %                         obj.UAVS{i}.U = - obj.UAVS{i}.X(4:6); % stop the UAV
%                         obj.UAVS{i}.U = nmpc_mv.';
%                     end
%                 else
%                     obj.UAVS{i}.U = nmpc_mv.';
%                 end
%                     obj.UAVS{i}.OD = new_OD;
                    
                % ACADO
                out = acado_MPCstep(obj.UAVS{i}.ACADO_input);
                % register cpu time
                obj.UAVS{i}.CPU_TIME = [obj.UAVS{i}.CPU_TIME out.info.cpuTime];
                % shift initial guesses
                obj.UAVS{i}.ACADO_input.x = [out.x(2:end,:);out.x(end,:)];
                obj.UAVS{i}.ACADO_input.u = [out.u(2:end,:);out.u(end,:)];
                nmpc_mv = out.u(1,:).';
                obj.UAVS{i}.U = nmpc_mv.';

                %% return input
                if ~obj.THREE_DIMENSIONAL
                    Us = [Us obj.UAVS{i}.U];
                else
                    Us = [Us;obj.UAVS{i}.U];
                end

            end
        end

        %% NMPC NEXT STEP INIT
        function refresh_nmpc_state(obj,recievers_pos_ode,ASYNC)
            for i = 1 : obj.NUM_UAVS
               
                %% uav state
                obj.UAVS{i}.X = recievers_pos_ode(i,:);

                %% known uavs
                temp_mask = (obj.UAVS{i}.selection_list ~= i);
                num_neigh = length(obj.UAVS{i}.selection_list)-1;
                prev_neigh_matrix = obj.UAVS{i}.OD.Parameters{1};
                if num_neigh > 0
                    if ASYNC
                        temp_val = extractPosOde(obj.UAVS{i}.dispatcher.curr_info_cell);
                        if ~obj.THREE_DIMENSIONAL
                            neigh = [temp_val(:,1:2) zeros(size(temp_val,1),1)];
                        else
                            neigh = temp_val(:,1:3);
                        end
                        neigh = neigh(temp_mask,:);
                    else
                        if ~obj.THREE_DIMENSIONAL
                            neigh = obj.UAVS{i}.selection_matrix * [recievers_pos_ode(:,1:2) zeros(size(recievers_pos_ode,1),1)];
                        else
                            neigh = obj.UAVS{i}.selection_matrix * recievers_pos_ode(:,1:3);
                        end
                        neigh = neigh(temp_mask,:);
                    end
                else
                    neigh = [];
                end
                neigh = [neigh;zeros(obj.NUM_UAVS-num_neigh,3)];
                neigh = [neigh;zeros(2*obj.NUM_UAVS,3)];
                neigh(obj.NUM_UAVS+1:2*obj.NUM_UAVS,:) = (neigh(1:obj.NUM_UAVS,:)-prev_neigh_matrix(1:obj.NUM_UAVS,:))/obj.NMPC_handle.PredictionHorizon;
                neigh(2*obj.NUM_UAVS+1:3*obj.NUM_UAVS,:) = (neigh(obj.NUM_UAVS+1:2*obj.NUM_UAVS,:)-prev_neigh_matrix(obj.NUM_UAVS+1:2*obj.NUM_UAVS,:))/obj.NMPC_handle.PredictionHorizon;

                %% sensor
%                 temp_mask1 = (obj.UAVS{i}.complementar_selection_list ~= i);
%                 num_non_neigh = obj.NUM_UAVS-num_neigh-1;
%                 if ~isempty(obj.UAVS{i}.complementar_selection_matrix)
%                     if ~obj.THREE_DIMENSIONAL
%                         non_neigh = obj.UAVS{i}.complementar_selection_matrix * [recievers_pos_ode(:,1:2) zeros(size(recievers_pos_ode,1),1)];
%                     else
%                         non_neigh = obj.UAVS{i}.complementar_selection_matrix * recievers_pos_ode(:,1:3);
%                     end
%                     non_neigh = non_neigh(temp_mask1,:);
%                 else
%                     non_neigh = [];
%                 end
%                 non_neigh = [non_neigh;zeros(obj.NUM_UAVS-num_non_neigh,3)];

                prev_non_neigh_matrix = obj.UAVS{i}.OD.Parameters{3};
                if ~obj.THREE_DIMENSIONAL
                    [nearest_obs,min_dist] = UAVNetwork.compute_nearest_obs([recievers_pos_ode(i,1:2) 0],[recievers_pos_ode(1:size(recievers_pos_ode,1) ~= i,1:2) zeros(size(recievers_pos_ode,1)-1,1)]);
                    non_neigh = [nearest_obs;zeros(2,3)];
                else
                    [nearest_obs,min_dist] = UAVNetwork.compute_nearest_obs(recievers_pos_ode(i,1:3),recievers_pos_ode(1:size(recievers_pos_ode,1) ~= i,1:3));
                    non_neigh = [nearest_obs;zeros(2,3)];
                    
                end

                num_non_neigh = 3;
                d_safe = obj.d_safe;
                v_max = obj.v_max;
             
                if obj.UAVS{i}.TRACKED_OBS && min_dist >= obj.SENSING_HORIZON
                    num_non_neigh = -1;
                    non_neigh = zeros(3,3);
                    obj.UAVS{i}.TRACKED_OBS = false;
                elseif ~obj.UAVS{i}.TRACKED_OBS && min_dist <= obj.SENSING_HORIZON
                    obj.UAVS{i}.TRACKED_OBS = true;
%                     fprintf("UAV %d detected obs\n",i);
                elseif ~obj.UAVS{i}.TRACKED_OBS && min_dist >=  obj.SENSING_HORIZON
                    num_non_neigh = -1;
                    non_neigh = zeros(3,3);
                elseif obj.UAVS{i}.TRACKED_OBS && min_dist <= obj.SENSING_HORIZON
%                     fprintf("UAV %d detected obs\n",i);
                    non_neigh(2,:) = (non_neigh(1,:)-prev_non_neigh_matrix(1,:))/obj.NMPC_handle.PredictionHorizon;
                    non_neigh(3,:) = (non_neigh(2,:)-prev_non_neigh_matrix(2,:))/obj.NMPC_handle.PredictionHorizon;
                end
                
                %%
                obj.UAVS{i}.OD.Parameters{1} = neigh;
                obj.UAVS{i}.OD.Parameters{2} = num_neigh;
                obj.UAVS{i}.OD.Parameters{3} = non_neigh;
                obj.UAVS{i}.OD.Parameters{4} = num_non_neigh;

                %% ACADO ONLINEDATA UPDATE
                % construct obstacle prevision
                ogghie_magnitude = reshape(non_neigh.',1,[]);
                v_hat = ogghie_magnitude(4:6);
                a_hat = ogghie_magnitude(7:9);
                positions = zeros(obj.NMPC_handle.PredictionHorizon+1,3);
                positions(1,:) = ogghie_magnitude(1:3);
                for k = 1:obj.NMPC_handle.PredictionHorizon
                    positions(k+1,:) = positions(1,:) + v_hat*obj.NMPC_handle.Ts*k+0.5*a_hat*(obj.NMPC_handle.Ts*k)^2;

                end

%                 if num_non_neigh == 3
%                     obs_od_data = [positions repmat([v_hat a_hat obj.d_safe 1],obj.NMPC_handle.PredictionHorizon+1,1)];
%                 elseif num_non_neigh == -1
%                     obs_od_data = [positions repmat([v_hat a_hat obj.d_safe 0],obj.NMPC_handle.PredictionHorizon+1,1)];
%                 end
%                 obj.UAVS{i}.ACADO_input.od = obs_od_data;
                pole_0 = max(10,-CBF_h_f(obj.UAVS{i}.X,non_neigh(1,:),obj.d_safe)/CBF_h_f_dot(obj.UAVS{i}.X(1:6),[non_neigh(1,:) non_neigh(2,:)],obj.d_safe));
                pole_1 = 10;
                if num_non_neigh == 3
                    obj.UAVS{i}.ACADO_input.od = repmat([reshape(non_neigh.',1,[]) obj.d_safe 1 pole_0 pole_1],obj.NMPC_handle.PredictionHorizon+1,1);
                elseif num_non_neigh == -1
                    obj.UAVS{i}.ACADO_input.od = repmat([reshape(non_neigh.',1,[]) obj.d_safe 0 pole_0 pole_1],obj.NMPC_handle.PredictionHorizon+1,1);
                end

                

                %% sigma travelled
                H_temp = zeros(10,num_neigh+1);
                if ~obj.THREE_DIMENSIONAL
                    H_temp(:,1) = single_H_function([obj.UAVS{i}.X(1:2) 0]);
                else
                    H_temp(:,1) = single_H_function(obj.UAVS{i}.X(1:3));
                end
                for k = 1:num_neigh
                    if ~obj.THREE_DIMENSIONAL
                        H_temp(:,k) = single_H_function([neigh(k,1:2) 0]);
                    else
                        H_temp(:,k) = single_H_function(neigh(k,1:3));
                    end
                end
                O_k = H_temp*H_temp.';
                obj.UAVS{i}.k_for_O = obj.UAVS{i}.k_for_O + 1; 
                obj.UAVS{i}.O_sum = obj.UAVS{i}.O_sum + (O_k - obj.UAVS{i}.O_sum)/obj.UAVS{i}.k_for_O;
                obj.UAVS{i}.sigma_travelled = min_sv_O(obj.UAVS{i}.O_sum,num_neigh+1);

            end
        end

        %% ONLINEDATA UPDATE & CHECK MISSION END
        function TRANSMITTER_FOUND = refresh_online_data(obj)
            TRANSMITTER_FOUND = true;
            for i = 1 : obj.NUM_UAVS

                if obj.UAVS{i}.TRANSMITTER_FOUND == false
                    TRANSMITTER_FOUND = false;
                end

                cur_pos_hat = obj.UAVS{i}.transmitter_pos_hat;
                last_pos_hat = obj.UAVS{i}.last_transmitter_pos_hat;
                obj.UAVS{i}.last_transmitter_pos_hat = obj.UAVS{i}.transmitter_pos_hat;
                last_trustable_pos_hat = obj.UAVS{i}.last_trustable_transmitter_pos_hat;
                if ~obj.THREE_DIMENSIONAL
                    cur_pos_hat = cur_pos_hat(1:2);
                    last_pos_hat = last_pos_hat(1:2);
                    last_trustable_pos_hat = last_trustable_pos_hat(1:2);
                end
                
                estimate_var = norm(cur_pos_hat - last_pos_hat);
                norm_var = 0.1;
%                 obj.UAVS{i}.T_REPLANNING = obj.UAVS{i}.T_REPLANNING - exp(estimate_var-norm_var) * obj.TIME_STEP;
                obj.UAVS{i}.T_REPLANNING = obj.UAVS{i}.T_REPLANNING - obj.TIME_STEP;

                if  obj.UAVS{i}.T_REPLANNING <= 0 ...
                    && (obj.UAVS{i}.sigma_travelled <= 20 * length(obj.UAVS{i}.selection_list) || norm(cur_pos_hat-last_trustable_pos_hat) > 2)
                    
                    % reset timer
                    obj.UAVS{i}.T_REPLANNING = 4;

                    % reset observability index consumed up to now
                    obj.UAVS{i}.k_for_O = 0;
                    obj.UAVS{i}.O_sum = zeros(10,10);
                    obj.UAVS{i}.sigma_travelled = 0;

                    % change reference point
%                     obj.UAVS{i}.OD.ref = [obj.UAVS{i}.transmitter_pos_hat(1:2) 0 0];
                    if ~obj.THREE_DIMENSIONAL
                        obj.UAVS{i}.X_ref = [obj.UAVS{i}.transmitter_pos_hat(1:2) 0];
                    else
                        obj.UAVS{i}.X_ref = [obj.UAVS{i}.transmitter_pos_hat(1:2) obj.UAVS{i}.transmitter_pos_hat(3)+3];
                    end
                    
                    % update last trustable transmitter position estimate
                    obj.UAVS{i}.last_trustable_transmitter_pos_hat = obj.UAVS{i}.transmitter_pos_hat;
                end

            end
        end
    end

    methods(Static)
        function [x_nearest_obs,min_dist] = compute_nearest_obs(x,x_neighs)
                x_nearest_obs = x_neighs(1,:);
                min_dist = norm(x-x_neighs(1,:));
                for i = 1:size(x_neighs,1)
                    if norm(x-x_neighs(i,:)) < min_dist
                        x_nearest_obs = x_neighs(i,:);
                        min_dist = norm(x-x_neighs(i,:));
                    end
                end
        end
    end

end