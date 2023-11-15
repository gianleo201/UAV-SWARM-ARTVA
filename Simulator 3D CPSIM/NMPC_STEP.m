function [nmpc_mv, new_onlineData, info] = NMPC_STEP(NMPC_obj,X0,U0,onlineData,CPP_EXEC)
% NMPC WRAPPER to switch c/MATLAB NMPC algorithm execuition
    if CPP_EXEC
        [nmpc_mv, new_onlineData, info] = real_NMPC_UAV(X0,U0, onlineData);
    else
        opt = nlmpcmoveopt();
        opt.Parameters = onlineData.Parameters;
        opt.X0 = onlineData.X0;
        opt.MV0 = onlineData.MV0;
        opt.Slack0 = onlineData.Slack0;
        opt.MVTarget = onlineData.MVTarget;
        if isfield(onlineData,"md")
            md = onlineData.md;
        else
            md = [];
        end
        [nmpc_mv, opt, info] = NMPC_obj.nlmpcmove(X0,U0,onlineData.ref,md,opt);
        new_onlineData = onlineData;
        new_onlineData.Parameters = opt.Parameters;
        new_onlineData.X0 = opt.X0;
        new_onlineData.MV0 = opt.MV0;
        new_onlineData.Slack0 = opt.Slack0;
    end
end