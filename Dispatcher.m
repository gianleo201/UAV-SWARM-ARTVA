classdef Dispatcher < handle

    properties

        NUM_UAVS;
        pending_info;
        curr_info;

    end

    methods

        function obj = Dispatcher(N,TIME_STEP,init_pos_ode)
            obj.NUM_UAVS = N;
            obj.curr_info = init_pos_ode;
            obj.pending_info = cell(1,obj.NUM_UAVS);
            for i=1:obj.NUM_UAVS
                obj.pending_info{i} = [];
            end
        end

        function push_info(obj, ode_info) % up to now uav ode_pos
            for i=1:obj.NUM_UAVS
                new_info.ode_pos = ode_info(i,1:4);
                new_info.spawn_time = ode_info(i,5);
                curr_uav_pending_info = obj.pending_info{i};
                if isempty(curr_uav_pending_info)
                    obj.pending_info{i} = new_info;
                else
                    for k = 1 : length(curr_uav_pending_info)
                        k_elem = curr_uav_pending_info(k);
                        if new_info.spawn_time  <= k_elem.spawn_time
                            curr_uav_pending_info = [new_info curr_uav_pending_info(k:end) ];
                            obj.pending_info{i} = curr_uav_pending_info;
                            break;
                        end
                    end
                end
            end
        end

        function new_curr_info = pull_latest_info(obj, TIME)
            for i=1:obj.NUM_UAVS
                curr_UAV = obj.pending_info{i};
                if ~isempty(curr_UAV)
%                     latest_info = curr_UAV(1);
%                     if latest_info.spawn_time <= TIME
%                         obj.curr_info(i,:) = latest_info.ode_pos;
%                         if length(curr_UAV) == 1
%                             obj.pending_info{i} = [];
%                         else
%                             obj.pending_info{i} = curr_UAV(2:end);
%                         end
%                     end
                     k = 1;
                     while k <= length(curr_UAV) && curr_UAV(k).spawn_time <= TIME
                         k = k + 1;
                     end
                     if k > 1
                         obj.curr_info(i,:) = curr_UAV(k-1).ode_pos;
                         obj.pending_info{i} = curr_UAV(k:end);
                     end
                end
            end
            new_curr_info = obj.curr_info;
        end

    end
end