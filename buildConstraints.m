%% BUILD CONSTRAINTS

function resFunction = buildConstraints(MIN_T0, NUM_AGENTS, N_approx_bernstain, p_r0, p_t_hat, d_t, d_safe, v_max)
    function [c, ceq] = constriants(x)

        c = zeros(1 ...
                 +NUM_AGENTS*(1+1) ...
                 +(NUM_AGENTS*(NUM_AGENTS-1)/2),1);
        
        ceq = zeros(NUM_AGENTS*2*(1+1+1+1),1);

        %% reshape input in tensor form
        t_f = x(1);
        Bns = reshape(x(2:end),[NUM_AGENTS, 2, N_approx_bernstain+1]);

        Dm = BernsteinDifferentiationMatrix(N_approx_bernstain,t_f);
        DDm = BernsteinDifferentiationMatrix(N_approx_bernstain-1,t_f);
        
        %% nonnegative mission time constraint
        c(1) = -t_f+MIN_T0;

        keq = 1;
        k = 2;
        for i=1:NUM_AGENTS % each constraint for each agent

            pos0_BNS = Bns(i,:,1);
            vel_BNS = squeeze(Bns(i,:,:))*Dm; % traj velocity
            posf_BNS = Bns(i,:,N_approx_bernstain+1);
            
            %% initial positon constriant
            ceq(keq) = pos0_BNS(1) - p_r0(i,1);
            ceq(keq+1) = pos0_BNS(2) - p_r0(i,2);
            keq = keq + 2;

            %% initial velocity constraint
            ceq(keq) = vel_BNS(1,1) - p_r0(i,3); 
            ceq(keq+1) = vel_BNS(2,1) - p_r0(i,4);
            keq = keq + 2;

            %% final position constraint
            c(k) = norm(posf_BNS-p_t_hat)-d_t;
            k = k + 1;

            %% maximum velocity constraint ( computed using algorithm )
            DELTA_DEGREE_ELEVATION = 100;
            temp4 = BernsteinProduct(vel_BNS(1,:),vel_BNS(1,:)) + ...
                BernsteinProduct(vel_BNS(2,:),vel_BNS(2,:));
%             [temp5, ~] = MaximumBernstein(temp4);
            ronda = DegElevMatrix(2*(N_approx_bernstain-1),2*(N_approx_bernstain-1)+DELTA_DEGREE_ELEVATION);
            temp4 = temp4 * ronda;
            [temp5, ~] = max(temp4);
            c(k) = temp5-v_max^2;
            k = k + 1;

            %% collision avoidance constriants ( computed using algorithm )
            for j = i+1:NUM_AGENTS
                temp6 = squeeze(Bns(i,:,:))-squeeze(Bns(j,:,:));
                temp7 = BernsteinProduct(temp6(1,:),temp6(1,:)) + ...
                    BernsteinProduct(temp6(2,:),temp6(2,:));
%                 [temp8, ~] = MinimumBernstein(temp7);
                ronda = DegElevMatrix(2*(N_approx_bernstain),2*(N_approx_bernstain)+DELTA_DEGREE_ELEVATION);
                temp7 = temp7 * ronda;
                [temp8,~] = min(temp7);
                c(k) = -temp8+d_safe^2;
%                 c(k) = -MinDistBernstein2Bernstein(squeeze(Bns(i,:,:)),squeeze(Bns(j,:,:)))+d_safe; % collision aviodance constraint
                k = k + 1;
            end

            %% 0 final velocity constraint
            ceq(keq) = vel_BNS(1,end)-vel_BNS(1,end-1);
            ceq(keq+1) = vel_BNS(2,end)-vel_BNS(2,end-1);
            keq = keq + 2;

            %% 0 final velocity acceleration
            acc_BNS = vel_BNS * DDm;
            ceq(keq) = acc_BNS(1,end)-acc_BNS(1,end-1);
            ceq(keq+1) = acc_BNS(2,end)-acc_BNS(2,end-1);
            keq = keq + 2;

        end
    end
    
    resFunction = @constriants;
    
end