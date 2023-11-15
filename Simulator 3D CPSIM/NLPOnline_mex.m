[t_f,Bns,CPU_TIME] = NLP_function3D_mex(ObjectiveWeights,TIME_STEP,N,N_approx_bernstain,recievers_pos_ode,transmitter_pos_hat,d_t,d_safe,v_max);
fprintf("CPU time taken to solve optimization problem: %d\n",CPU_TIME);

% explicit computation of the trajecrtory points
ComputeTrajs;


%% GENERATE C CODE

% codegen NLP_function3D -args {zeros(1,3),0.1,1,3,zeros(N,12),zeros(1,3),1.0,5.0,5.0}