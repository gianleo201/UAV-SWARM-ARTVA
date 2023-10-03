%% BUILD OBJECTIVE FUNCTION

function resFunction = buildObjectiveFunction(W, NUM_AGENTS, TIME_STEP, N_approx_bernstain)

    function out_val = objectiveFunction(x)
        
        % reshape input in tensor form
        t_f = x(1);
%         display(t_f);
        Bns = reshape(x(2:end),[NUM_AGENTS, 2, N_approx_bernstain+1]);

%         % debug
%         ComputeTrajs;
%         DEBUG_plotTraj;

        % minimize mission time
        out_val = W(1) * t_f;
        
        % minimize UAVs acceleration profiles
        Dm1 = BernsteinDifferentiationMatrix(N_approx_bernstain,t_f);
        Dm2 = BernsteinDifferentiationMatrix(N_approx_bernstain-1,t_f);
        w_integral = BeBOT(2*N_approx_bernstain-4,t_f);
        for i=1:NUM_AGENTS
            
            Dtraj_i = squeeze(Bns(i,:,:))*Dm1; % compute first order derivative
            DDtraj_i = Dtraj_i*Dm2; % compute second order derivative
            accsqr_i = BernsteinProduct(DDtraj_i(1,:),DDtraj_i(1,:)) + ...
                BernsteinProduct(DDtraj_i(2,:),DDtraj_i(2,:)); % compute the squared norm of the acceleration
            integral_accsqr_i = accsqr_i*w_integral.'; % compute the final integral value
            out_val = out_val + W(2) *integral_accsqr_i;

        end


        % build O matrix
        O = zeros(10,10);
        mean_horizon = fix(t_f/TIME_STEP);
        temptemp = zeros(NUM_AGENTS,3);
        for i=1:mean_horizon-1
            for j=1:NUM_AGENTS
                temptemp(j,1:2) = BernsteinPoly(squeeze(Bns(j,:,:)),(i-1)*TIME_STEP,0,t_f);
            end
            temp = H_function( temptemp );
            O = O + temp*temp.';
        end
        O = (1/mean_horizon)*O;
%         display(O);
    
        % extract minimum singular value/eigenvalue
        sigma_lower = sqrt(svds(O,1,"smallestnz"));
%         display(sigma_lower);
        
    
        % maximize information gain
        out_val = out_val - W(3) * sigma_lower;

%         display(out_val);

    end

    resFunction = @objectiveFunction;
    
end