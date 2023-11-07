UAV_trajs = zeros(N,fix(t_f/TIME_STEP)+1,4);
time_vec = 0:TIME_STEP:(fix(t_f/TIME_STEP)*TIME_STEP);
OCP_Dm = BernsteinDifferentiationMatrix(N_approx_bernstain,t_f);
for i=1:N
    UAV_trajs(i,:,1:2) = BernsteinPoly(squeeze(Bns(i,:,:)),time_vec,0,fix(t_f/TIME_STEP)*TIME_STEP).';
    DBnsi = squeeze(Bns(i,:,:))*OCP_Dm; % compute velocity referecnes
    UAV_trajs(i,:,3:4) = BernsteinPoly(DBnsi,time_vec,0,fix(t_f/TIME_STEP)*TIME_STEP).';
end