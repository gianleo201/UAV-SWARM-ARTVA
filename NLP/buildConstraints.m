%% BUILD CONSTRAINTS

function resFunction = buildConstraints(MIN_T0, NUM_AGENTS, N_approx_bernstain, p_r0, p_t_hat, d_t, d_safe, v_max)
    function [c, ceq] = constriants(x)

%         c = zeros(1 ...
%                  +NUM_AGENTS*(1+1) ...
%                  +(NUM_AGENTS*(NUM_AGENTS-1)/2),1);
        c = zeros(NUM_AGENTS*(1+1) ...
                 +(NUM_AGENTS*(NUM_AGENTS-1)/2),1);
        
%         ceq = zeros(NUM_AGENTS*( 2*(1+1) + 1 ),1);
%         ceq = zeros(NUM_AGENTS*( 2*(1+1)  ),1);
        ceq = 0;
%         ceq = zeros(NUM_AGENTS);

        %% reshape input in tensor form
        t_f = x(1);
        Bns = reshape(x(2:end),[NUM_AGENTS, 2, N_approx_bernstain+1]);

        Dm = BernsteinDifferentiationMatrix(N_approx_bernstain,t_f);
        DDm = BernsteinDifferentiationMatrix(N_approx_bernstain-1,t_f);
        DELTA_DEGREE_ELEVATION = 5;
        
        %% nonnegative mission time constraint
%         c(1) = -t_f+MIN_T0;

        keq = 1;
%         k = 2;
        k = 1;
        for i=1:NUM_AGENTS % each constraint for each agent

            pos_BNS = squeeze(Bns(i,:,:));
            pos0_BNS = pos_BNS(:,1);
            vel_BNS = pos_BNS*Dm; % traj velocity
            posf_BNS = pos_BNS(:,N_approx_bernstain+1);
            
            %% initial positon constriant
%             ceq(keq) = pos0_BNS(1) - p_r0(i,1);
%             ceq(keq+1) = pos0_BNS(2) - p_r0(i,2);
%             keq = keq + 2;

            %% initial velocity constraint
%             ceq(keq) = vel_BNS(1,1) - p_r0(i,3); 
%             ceq(keq+1) = vel_BNS(2,1) - p_r0(i,4);
%             keq = keq + 2;

            %% final position constraint
%             c(k) = norm(posf_BNS-p_t_hat.')-d_t;
            c(k) = (posf_BNS-p_t_hat.').'*(posf_BNS-p_t_hat.')-d_t^2;
            k = k + 1;

            %% maximum velocity constraint ( computed using algorithm )
            temp4 = BernsteinProduct(vel_BNS(1,:),vel_BNS(1,:)) + ...
                BernsteinProduct(vel_BNS(2,:),vel_BNS(2,:));

%             [temp5, tmax] = MaximumBernstein(temp4);

            ronda = DegElevMatrix(2*(N_approx_bernstain-1),2*(N_approx_bernstain-1)+DELTA_DEGREE_ELEVATION);
            temp4 = temp4 * ronda;
            [temp5, imax] = max(temp4);
%             [~,temp5] = deCasteljau(temp4,imax/size(temp4,2));

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
                [temp8,imin] = min(temp7);
%                 [~,temp8] = deCasteljau(temp7,imin/size(temp7,2));

                c(k) = -temp8+d_safe^2;
%                 c(k) = -MinDistBernstein2Bernstein(squeeze(Bns(i,:,:)),squeeze(Bns(j,:,:)))+d_safe; % collision aviodance constraint
                k = k + 1;
            end

%             %% 0 final velocity constraint
%             temp9 = BernsteinProduct(vel_BNS(1,:),vel_BNS(1,:)) + ...
%                 BernsteinProduct(vel_BNS(2,:),vel_BNS(2,:));
% 
%             ceq(keq) = temp9(end);
%             keq = keq + 1;

        end
    end
    
    resFunction = @constriants;
    
end