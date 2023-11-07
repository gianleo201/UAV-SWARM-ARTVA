function pos_ode = extractPosOde(dispatcher_struct)
    
    pos_ode = [];
    for i=1:length(dispatcher_struct)
        pos_ode = [pos_ode;dispatcher_struct{i}.pos_ode];
    end

end