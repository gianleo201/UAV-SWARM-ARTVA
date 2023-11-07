classdef Dispatcher < handle

    properties

        NUM_UAVS;
        pending_info_cell;
        curr_info_cell;

    end

    methods

        function obj = Dispatcher(N,TIME_STEP,init_pos_ode,X_HAT0,S_RLS0)
            obj.NUM_UAVS = N;
            obj.curr_info_cell = cell(1,obj.NUM_UAVS);
            for i = 1 : obj.NUM_UAVS
                curr_info = {};
                curr_info.pos_ode = init_pos_ode(i,:);
                curr_info.X_hat = X_HAT0;
                curr_info.S_RLS = S_RLS0;
                obj.curr_info_cell{i} = curr_info;
            end
            obj.pending_info_cell = cell(1,obj.NUM_UAVS);
            for i=1:obj.NUM_UAVS
                obj.pending_info_cell{i} = [];
            end
        end

        function push_info(obj, new_info) % up to now info
            for i=1:obj.NUM_UAVS
                curr_uav_pending_info_list = obj.pending_info_cell{i};
                if isempty(curr_uav_pending_info_list)
                    obj.pending_info_cell{i} = new_info{i};
                else
                    for k = 1 : length(curr_uav_pending_info_list)
                        k_elem = curr_uav_pending_info_list(k);
                        if new_info{i}.spawn_time  <= k_elem.spawn_time
                            curr_uav_pending_info_list = [curr_uav_pending_info_list(1:k-1) new_info{i} curr_uav_pending_info_list(k:end) ];
                            obj.pending_info_cell{i} = curr_uav_pending_info_list;
                            break;
                        end
                    end
                end
            end
        end

        function new_curr_info = pull_latest_info(obj, TIME)
            for i=1:obj.NUM_UAVS
                curr_UAV_pending_info_list = obj.pending_info_cell{i};
                if ~isempty(curr_UAV_pending_info_list)
                     % step on to the most up to date info
                     k = 1;
                     while k <= length(curr_UAV_pending_info_list) && curr_UAV_pending_info_list(k).spawn_time <= TIME
                         k = k + 1;
                     end
                     if k > 1
                         obj.curr_info_cell{i} = curr_UAV_pending_info_list(k-1);
                         obj.pending_info_cell{i} = curr_UAV_pending_info_list(k:end);
                     end
                end
            end
            new_curr_info = obj.curr_info_cell;
        end

    end
end