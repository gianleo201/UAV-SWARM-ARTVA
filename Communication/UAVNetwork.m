classdef UAVNetwork < handle

    properties
        NUM_UAVS;
        network_matrix;
        UAVS;
        compute_Y;
        TIME_STEP;
        NMPC_handle;
        THREE_DIMENSIONAL;
    end

    methods
        function obj = UAVNetwork(N,TIME_STEP,network_matrix,recievers_pos_ode0,X_hat0,S_RLS0,compute_Y,NMPC_handle,d_safe,v_max,MODE,model_params)
            %%
            obj.NUM_UAVS = N;
            obj.network_matrix = network_matrix;
            obj.compute_Y = compute_Y;
            obj.UAVS = cell(obj.NUM_UAVS,1);
            obj.TIME_STEP = TIME_STEP;
            obj.NMPC_handle = NMPC_handle;
            obj.THREE_DIMENSIONAL = false;
            if  nargin > 10 &&  MODE == "3D"
                obj.THREE_DIMENSIONAL = true;
            end

            %% selection matrices and lists
            for i = 1 : obj.NUM_UAVS
                UAV_struct = {};
                UAV_struct.TRANSMITTER_FOUND = false;
                UAV_struct.T_REPLANNING = 10;
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

                %% sensor
                temp_mask1 = (obj.UAVS{i}.complementar_selection_list ~= i);
                num_non_neigh = obj.NUM_UAVS-num_neigh-1;
                if ~isempty(obj.UAVS{i}.complementar_selection_matrix)
                    if ~obj.THREE_DIMENSIONAL
                    non_neigh = obj.UAVS{i}.complementar_selection_matrix * [recievers_pos_ode0(:,1:2) zeros(size(recievers_pos_ode0,1),1)];
                    else
                        non_neigh = obj.UAVS{i}.complementar_selection_matrix * recievers_pos_ode0(:,1:3);
                    end
                    non_neigh = non_neigh(temp_mask1,:);
                else
                    non_neigh = [];
                end
                non_neigh = [non_neigh;zeros(obj.NUM_UAVS-num_non_neigh,3)];
                
                %%
                X0 = recievers_pos_ode0(i,:);
                obj.UAVS{i}.lambda_factor = 0.1;
                
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
                obj.UAVS{i}.OD = OD;
                obj.UAVS{i}.U = U0;
                obj.UAVS{i}.X = X0;
                obj.UAVS{i}.X_ref = [obj.UAVS{i}.transmitter_pos_hat];
                obj.UAVS{i}.prediction_horizon = NMPC_handle.PredictionHorizon;
                obj.UAVS{i}.CPU_TIME = [];
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
                    recievers_pos = Qs.'*last_pos_info;
                    H_num = H_function(recievers_pos);
                    Y_num = obj.compute_Y(recievers_pos);
                    H_num = H_num*Qs.';
                    Y_num = Qs*Y_num;
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

                obj.UAVS{i}.last_transmitter_pos_hat = obj.UAVS{i}.transmitter_pos_hat;
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
                obj.UAVS{i}.FL_THRUST = obj.UAVS{i}.FL_THRUST + obj.UAVS{i}.FL_DTHRUST*obj.TIME_STEP;
                obj.UAVS{i}.FL_DTHRUST = obj.UAVS{i}.FL_DTHRUST + T_mod(1)*obj.TIME_STEP;
                Us(i,:) = U;
            end
        end
        
        %% NMPC STEP

        function Us = NMPC_UAV_TEAM_step(obj)
            Us = [];
            for i = 1 : obj.NUM_UAVS
                %% NMPC step
                

                if ~obj.THREE_DIMENSIONAL
                    curr_ref = [obj.UAVS{i}.X_ref(1:2) 0 0];
                else
                    curr_ref = [obj.UAVS{i}.X_ref zeros(1,size(obj.UAVS{i}.X,2)-3)];
                end

                obj.UAVS{i}.OD.ref = EMA_const_reference(obj.UAVS{i}.X, ...
                                curr_ref, ...
                                obj.UAVS{i}.lambda_factor, ...
                                obj.UAVS{i}.prediction_horizon);
                
                obj.UAVS{i}.OD.ref = obj.UAVS{i}.OD.ref(:,1:4);
               
                tic;
                [nmpc_mv, new_OD, info] = NMPC_STEP(obj.NMPC_handle, ...
                                                    obj.UAVS{i}.X.', ...
                                                    obj.UAVS{i}.U.', ...
                                                    obj.UAVS{i}.OD, ...
                                                    true);
                elapsed_time = toc;

                %% register cpu time
                obj.UAVS{i}.CPU_TIME = [obj.UAVS{i}.CPU_TIME elapsed_time];

                %% check nlmpc step status
                nmpc_status = info.ExitFlag;
                if nmpc_status < 0
                    fprintf("No feasable solution found for drone %d\n",i);
                    fprintf("Exit flag number: %d\n",nmpc_status);
                    if ~obj.THREE_DIMENSIONAL
                        obj.UAVS{i}.U = - obj.UAVS{i}.X(3:4); % stop the UAV
                    else
                        obj.UAVS{i}.U = - obj.UAVS{i}.X(4:6); % stop the UAV
                    end
%                     obj.UAVS{i}.U = nmpc_mv.';
                    obj.UAVS{i}.OD = new_OD; % do not modify online data
                else
                    obj.UAVS{i}.U = nmpc_mv.';
                    obj.UAVS{i}.OD = new_OD;
                end

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

                %% sensor
                temp_mask1 = (obj.UAVS{i}.complementar_selection_list ~= i);
                num_non_neigh = obj.NUM_UAVS-num_neigh-1;
                if ~isempty(obj.UAVS{i}.complementar_selection_matrix)
                    if ~obj.THREE_DIMENSIONAL
                        non_neigh = obj.UAVS{i}.complementar_selection_matrix * [recievers_pos_ode(:,1:2) zeros(size(recievers_pos_ode,1),1)];
                    else
                        non_neigh = obj.UAVS{i}.complementar_selection_matrix * recievers_pos_ode(:,1:3);
                    end
                    non_neigh = non_neigh(temp_mask1,:);
                else
                    non_neigh = [];
                end
                non_neigh = [non_neigh;zeros(obj.NUM_UAVS-num_non_neigh,3)];
                
                %%
                obj.UAVS{i}.OD.Parameters{1} = neigh;
                obj.UAVS{i}.OD.Parameters{2} = num_neigh;
                obj.UAVS{i}.OD.Parameters{3} = non_neigh;
                obj.UAVS{i}.OD.Parameters{4} = num_non_neigh;
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
                last_pos_hat = obj.UAVS{i}.last_trustable_transmitter_pos_hat;
                if ~obj.THREE_DIMENSIONAL
                    cur_pos_hat = cur_pos_hat(1:2);
                    last_pos_hat = last_pos_hat(1:2);
                end
                
                estimate_var = norm(cur_pos_hat - last_pos_hat);
                norm_var = 0.2;
%                 obj.UAVS{i}.T_REPLANNING = obj.UAVS{i}.T_REPLANNING - exp(estimate_var-norm_var) * obj.TIME_STEP;
                obj.UAVS{i}.T_REPLANNING = obj.UAVS{i}.T_REPLANNING - obj.TIME_STEP;

                if  obj.UAVS{i}.T_REPLANNING <= 0
                    obj.UAVS{i}.T_REPLANNING = 10;
%                     obj.UAVS{i}.OD.ref = [obj.UAVS{i}.transmitter_pos_hat(1:2) 0 0];
                    if ~obj.THREE_DIMENSIONAL
                        obj.UAVS{i}.X_ref = [obj.UAVS{i}.transmitter_pos_hat(1:2) 0];
                    else
                        obj.UAVS{i}.X_ref = [obj.UAVS{i}.transmitter_pos_hat(1:2) obj.UAVS{i}.transmitter_pos_hat(3)+3];
                    end
                    obj.UAVS{i}.last_trustable_transmitter_pos_hat = obj.UAVS{i}.transmitter_pos_hat;
                end

            end
        end
    end
end