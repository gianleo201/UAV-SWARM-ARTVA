function quadrotor_model = quadrotor_model(in1,in2,neigh_pos,N_neighbours,non_neigh_pos,N_non_neighbours,d_safe,v_max,in9)
%QUADROTOR_MODEL
%    QUADROTOR_MODEL = QUADROTOR_MODEL(IN1,IN2,NEIGH_POS,N_neighbours,NON_NEIGH_POS,N_non_neighbours,D_SAFE,V_MAX,IN9)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    10-Nov-2023 16:09:32

Jxx = in9(:,3);
Jyy = in9(:,4);
Jzz = in9(:,5);
T1 = in2(1,:);
T2 = in2(2,:);
T3 = in2(3,:);
T4 = in2(4,:);
a = in9(:,1);
b = in9(:,2);
dx = in1(4,:);
dy = in1(5,:);
dz = in1(6,:);
g = in9(:,7);
m = in9(:,6);
p = in1(7,:);
phi = in1(10,:);
psi = in1(12,:);
q = in1(8,:);
r = in1(9,:);
rD = in9(:,9);
rT = in9(:,8);
theta = in1(11,:);
mt1 = [dx;dy;dz;((sin(phi).*sin(psi)+cos(phi).*cos(psi).*sin(theta)).*(T1+T2+T3+T4))./m;-((cos(psi).*sin(phi)-cos(phi).*sin(psi).*sin(theta)).*(T1+T2+T3+T4))./m;-(g.*m-cos(phi).*cos(theta).*(T1+T2+T3+T4))./m;((a.*(T1+T2-T3-T4))./2.0+q.*r.*(Jyy-Jzz))./Jxx;-((b.*(T1-T2-T3+T4))./2.0+p.*r.*(Jxx-Jzz))./Jyy;-((rD.*(T1-T2+T3-T4))./rT-p.*q.*(Jxx-Jyy))./Jzz;(p.*cos(theta)+r.*cos(phi).*sin(theta)+q.*sin(phi).*sin(theta))./cos(theta)];
mt2 = [q.*cos(phi)-r.*sin(phi);(r.*cos(phi)+q.*sin(phi))./cos(theta)];
quadrotor_model = [mt1;mt2];
