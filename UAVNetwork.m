classdef UAVNetwork < handle

    properties
        NUM_UAVS;
        network_matrix;
        UAVS;
        compute_Y;
    end

    methods
        function obj = UAVNetwork(N,TIME_STEP,network_matrix,recievers_pos0,X_hat0,S_RLS0,compute_Y)
            obj.NUM_UAVS = N;
%             obj.pending_info = cell(obj.NUM_UAVS,1);
            obj.network_matrix = network_matrix;
            obj.compute_Y = compute_Y;
            obj.UAVS = cell(obj.NUM_UAVS,1);
            % create selection matrices and lists
            for i = 1 : obj.NUM_UAVS
                UAV_struct = {};
                UAV_struct.selection_matrix = [];
                UAV_struct.selection_list = [];
                for j = 1 : obj.NUM_UAVS
                    if network_matrix(i,j) == 1
                        ek = eye(j,obj.NUM_UAVS);
                        ek = ek(j,:);
                        UAV_struct.selection_matrix = [UAV_struct.selection_matrix; ek];
                        UAV_struct.selection_list = [UAV_struct.selection_list j];
                    end
                UAV_struct.dispatcher = Dispatcher(length(UAV_struct.selection_list),TIME_STEP,recievers_pos0(UAV_struct.selection_list,:));
                obj.UAVS{i} = UAV_struct;
                end
            end
            % initialize algorithm
            for i = 1 : obj.NUM_UAVS
                obj.UAVS{i}.X_hat = X_hat0;
                obj.UAVS{i}.S_RLS = S_RLS0;
                obj.UAVS{i}.S_RLS_temp = zeros(10,10);
                % extract transmitter estimate position
                X_hat = obj.UAVS{i}.X_hat;
                M_hat = [X_hat(1) X_hat(2) X_hat(3);X_hat(2) X_hat(4) X_hat(5); X_hat(3) X_hat(5) X_hat(6)];
%                 old_transmitter_pos_hat = transmitter_pos_hat;
                obj.UAVS{i}.transmitter_pos_hat = (inv(M_hat)*X_hat(7:9).').'; % new transmitter estimate
            end
        end

        function DRLS(obj,beta_ff,Y_num_arg,H_num_arg,ASYNC,TIME)
            
            % RLS algorithm step for each UAV
            for i = 1 : obj.NUM_UAVS
                Qs = obj.UAVS{i}.selection_matrix;
                if ASYNC
                    last_pos_info = obj.UAVS{i}.dispatcher.pull_latest_info(TIME);
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
                
                X_hat = ( X_hat.' +  inv(S_RLS)*H_num*(Y_num-H_num.'*X_hat.') ).'; % new estimate of parameters vector;
                S_RLS = beta_ff * S_RLS + H_num*H_num.';

                obj.UAVS{i}.X_hat = X_hat;
                obj.UAVS{i}.S_RLS = S_RLS;
            end

            % D step for each uav

            % update X_hat
            for i = 1 : obj.NUM_UAVS
                selection_list = obj.UAVS{i}.selection_list;
                X_hat = zeros(1,10);
                S_RLS_temp = zeros(10,10);
                S_RLS_temp1 = zeros(10,10);
                for k = 1 : length(selection_list)
                    S_RLS_temp = S_RLS_temp + obj.UAVS{selection_list(k)}.S_RLS;
%                     S_RLS_temp1 = S_RLS_temp1 + inv(obj.UAVS{selection_list(k)}.S_RLS);
                end
                obj.UAVS{i}.S_RLS_temp = S_RLS_temp;
                for k = 1 : length(selection_list)
%                     X_hat = X_hat + (inv(obj.UAVS{selection_list(k)}.S_RLS)*(obj.UAVS{selection_list(k)}.X_hat).').';
                    X_hat = X_hat + obj.UAVS{selection_list(k)}.X_hat;
                end
%                 X_hat = (inv(S_RLS_temp1)*(X_hat.')).';
                X_hat = X_hat/length(obj.UAVS{i}.selection_list);
                obj.UAVS{i}.X_hat = X_hat;
                % update transmitter estimate position
                M_hat = [X_hat(1) X_hat(2) X_hat(3);X_hat(2) X_hat(4) X_hat(5); X_hat(3) X_hat(5) X_hat(6)];
%                 old_transmitter_pos_hat = transmitter_pos_hat;
                obj.UAVS{i}.transmitter_pos_hat = (inv(M_hat)*X_hat(7:9).').'; % new transmitter estimate
            end
            % update S_RLS
            for i = 1 : obj.NUM_UAVS
                obj.UAVS{i}.S_RLS = obj.UAVS{i}.S_RLS_temp / length(obj.UAVS{i}.selection_list);
            end
            
        end

        function estimates = pull_estimates(obj)
            estimates = zeros(obj.NUM_UAVS,3);
            for i = 1 : obj.NUM_UAVS
                estimates(i,:) = obj.UAVS{i}.transmitter_pos_hat;
            end
        end

        function info_transmission(obj,UAV_positions,TIME,medium_velocity,uncertain_time_range)
            for i = 1 : obj.NUM_UAVS
                selection_list = obj.UAVS{i}.selection_list;
                info = zeros(length(selection_list),4);
                for j = 1 : length(selection_list)
                    info(j,1:3) = UAV_positions(selection_list(j),:);
                    if selection_list(j) ~= i
                        d_ij = norm(UAV_positions(i,1:2)-UAV_positions(selection_list(j),1:2));
                        random_alpha = rand(1,1);
                        delta_spawn_time = ( d_ij/medium_velocity ) + (2*random_alpha-1) * uncertain_time_range;
                        info(j,4) = TIME + delta_spawn_time;
                    else
                        info(j,4) = TIME;
                    end
                end
                obj.UAVS{i}.dispatcher.push_info(info);
            end
        end

    end

end