%% BUILD CONSTRAINTS

function resFunction = buildConstraints(TIME_STEP, NUM_AGENTS, N_approx_bernstain, p_r0, p_t_hat, d_t, d_safe, v_max)
    function [c, ceq] = constriants(x)
        
        % reshape input in tensor form
        t_f = x(1);
        Bns = reshape(x(2:end),[NUM_AGENTS, 2, N_approx_bernstain+1]);
        
        % nonnegative mission time constraint
        c(1) = -t_f+10;

        keq = 1;
        k = 2;
        for i=1:NUM_AGENTS % each constraint for each agent

            temp0 = Bns(i,:,1);
            Dm = BernsteinDifferentiationMatrix(N_approx_bernstain,t_f);
            temp1 = squeeze(Bns(i,:,:))*Dm; % traj velocity
            temp2 = Bns(i,:,N_approx_bernstain+1);
            
            % initial positon constriant
            ceq(keq) = temp0(1) - p_r0(i,1);
            ceq(keq+1) = temp0(2) - p_r0(i,2);
            keq = keq + 2;

            % initial velocity constraint
            ceq(keq) = temp1(1,1) - p_r0(i,3); 
            ceq(keq+1) = temp1(2,1) - p_r0(i,4);
            keq = keq + 2;

            % final position constraint
            c(k) = norm(temp2-p_t_hat)-d_t;
            k = k + 1;

            % maximum velocity constraint ( computed using algorithm )
            temp4 = BernsteinProduct(temp1(1,:),temp1(1,:)) + ...
                BernsteinProduct(temp1(2,:),temp1(2,:));
            [temp5, ~] = MaximumBernstein(temp4);
            c(k) = temp5-v_max^2;
            k = k + 1;

            % collision avoidance constriants ( computed using algorithm )
            for j = i+1:NUM_AGENTS
                temp6 = squeeze(Bns(i,:,:))-squeeze(Bns(j,:,:));
                temp7 = BernsteinProduct(temp6(1,:),temp6(1,:)) + ...
                    BernsteinProduct(temp6(2,:),temp6(2,:));
                [temp8, ~] = MinimumBernstein(temp7);
                c(k) = -temp8+d_safe^2;
%                 c(k) = -MinDistBernstein2Bernstein(squeeze(Bns(i,:,:)),squeeze(Bns(j,:,:)))+d_safe; % collision aviodance constraint
                k = k + 1;
            end

        end
    end
    
    resFunction = @constriants;
    
end