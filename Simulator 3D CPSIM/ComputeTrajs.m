% UAV_trajs = zeros(N,fix(t_f/TIME_STEP)+1,4);
% time_vec = 0:TIME_STEP:(fix(t_f/TIME_STEP)*TIME_STEP);
% OCP_Dm = BernsteinDifferentiationMatrix(N_approx_bernstain,t_f);
% for i=1:N
%     UAV_trajs(i,:,1:2) = BernsteinPoly(squeeze(Bns(i,:,:)),time_vec,0,fix(t_f/TIME_STEP)*TIME_STEP).';
%     DBnsi = squeeze(Bns(i,:,:))*OCP_Dm; % compute velocity referecnes
%     UAV_trajs(i,:,3:4) = BernsteinPoly(DBnsi,time_vec,0,fix(t_f/TIME_STEP)*TIME_STEP).';
% end



UAV_trajs = zeros(N,fix(t_f/TIME_STEP)+1,10);
time_vec = 0:TIME_STEP:(fix(t_f/TIME_STEP)*TIME_STEP);
OCP_Dm = BernsteinDifferentiationMatrix(N_approx_bernstain,t_f);
OCP_DDm = BernsteinDifferentiationMatrix(N_approx_bernstain-1,t_f);
OCP_DDDm = BernsteinDifferentiationMatrix(N_approx_bernstain-2,t_f);
OCP_DDDDm = BernsteinDifferentiationMatrix(N_approx_bernstain-3,t_f);
for i=1:N
    UAV_trajs(i,:,1:2) = BernsteinPoly(squeeze(Bns(i,:,:)),time_vec,0,fix(t_f/TIME_STEP)*TIME_STEP).';
    DBnsi = squeeze(Bns(i,:,:))*OCP_Dm; % compute velocity referecnes
    UAV_trajs(i,:,3:4) = BernsteinPoly(DBnsi,time_vec,0,fix(t_f/TIME_STEP)*TIME_STEP).';
    DDBnsi = DBnsi*OCP_DDm; % compute acceleration referecnes
    UAV_trajs(i,:,5:6) = BernsteinPoly(DDBnsi,time_vec,0,fix(t_f/TIME_STEP)*TIME_STEP).';
    DDDBnsi = DDBnsi*OCP_DDDm; % compute jerk referecnes
    UAV_trajs(i,:,7:8) = BernsteinPoly(DDDBnsi,time_vec,0,fix(t_f/TIME_STEP)*TIME_STEP).';
    DDDDBnsi = DDDBnsi*OCP_DDDDm; % compute jerk derivative referecnes
    UAV_trajs(i,:,9:10) = BernsteinPoly(DDDDBnsi,time_vec,0,fix(t_f/TIME_STEP)*TIME_STEP).';
end

