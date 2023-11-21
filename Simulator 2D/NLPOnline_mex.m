[t_f,Bns,CPU_TIME] = NLP_function_mex(ObjectiveWeights,TIME_STEP,N,N_approx_bernstain,recievers_pos_ode,transmitter_pos_hat,d_t,d_safe,v_max);
ext_string = sprintf("CPU time taken to solve optimization problem: %d\n",CPU_TIME);
fprintf(ext_string);
NLP_LOG = NLP_LOG+ext_string;


% explicit computation of the trajecrtory points
ComputeTrajs;


%% GENERATE C CODE
% N is the number of UAVs to handle in NLP
% codegen NLP_function -args {zeros(1,3),0.1,1,3,zeros(N,4),zeros(1,3),1.0,5.0,5.0}