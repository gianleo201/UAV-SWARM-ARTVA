ENABLE_FUNCTION_GENERATION = true;
RPY_FIXED_AXIS = true;

%% symbolic dynamic parameters

syms Jxx Jyy Jzz m g real;
J = diag([Jxx Jyy Jzz]);

%% symbolic geometric parameters

syms a b real;

%% symbolic aereodynamics parameters

syms rT rD rD_vz real;

%% symbolic state variables

syms x y z real;
syms dx dy dz real;
syms phi theta psi real;
syms p q r real;
w = [p;q;r];  % angular velocity symbolic vector in mobile rotating frame

%% symbolic control inputs variables

% syms T tau_phi tau_theta tau_psi real;
syms T1 T2 T3 T4 real;

%% symbolic collective vectors

sym_model_state = [x y z dx dy dz p q r phi theta psi].';

% sym_model_inputs = [T tau_phi tau_theta tau_psi].';
sym_model_inputs = [T1 T2 T3 T4].';

sym_model_params = [a b Jxx Jyy Jzz m g rT rD rD_vz];

%% symbolic computations

% 3D roll pitch yaw rotation matrices
tRx= [1         0         0;
      0  cos(phi) -sin(phi);
      0  sin(phi)  cos(phi)];

tRy = [cos(theta) 0 sin(theta);
       0          1          0;
      -sin(theta) 0 cos(theta)];

tRz = [cos(psi) -sin(psi) 0;
       sin(psi)  cos(psi) 0;
       0               0  1];

if  RPY_FIXED_AXIS
    R_fr = simplify(tRz*tRy*tRx); % x y z  (fixed axis!)
else
    R_fr = simplify(tRx*tRy*tRz); % x y' z'' (moving axis!)
end

%% linear equations of motion

gravity_f = [0;0;-m*g];
gravity_r = simplify(R_fr.'*gravity_f);

% T_r = [0;0;T];
T_r = [0;0;T1+T2+T3+T4];


T_f = simplify(R_fr*T_r); % thrust nel fisso

lin_motion = simplify((gravity_f+T_f)/m); % ode velocities in fixed frame

%% rotational equations of motion


% tau_r = [tau_phi;tau_theta;tau_psi];

tau_r = [(a*(T1 + T2 - T3 - T4))/2;
         (b*(-T1 + T2 + T3 - T4))/2;
         -(rD/rT)*(T1 - T2 + T3 - T4)];

wxJw = [q*r*(Jzz-Jyy);
        p*r*(Jxx-Jzz);
        p*q*(Jyy-Jxx)];

rot_motion = simplify(inv(J)*(tau_r-wxJw)); % ode angular velocities

%% kinematic equations (d_(roll,pitch,yaw)_dt <- w)

temp = simplify(tRz*tRy);
om_inv = [temp(:,1) tRz(:,2) [0;0;1]]; % matrice per passare da derivate di angoli di eulero a w 
clear temp;
om = simplify(inv(om_inv)*R_fr*w); % om = [phi_dot;theta_dot;psi_dot] 

%% compute total model

quadrotor_model = [dx;dy;dz;lin_motion;rot_motion;om];

%% compute total model jacobians

quadrotor_model_jac_x = jacobian(quadrotor_model,sym_model_state);
quadrotor_model_jac_u = jacobian(quadrotor_model,sym_model_inputs);

%% output function: [x y z roll pitch yaw]

quadrotor_model_y = [x y z dx dy dz].';
quadrotor_model_y_jac_x = jacobian(quadrotor_model_y,sym_model_state);
quadrotor_model_y_jac_u = jacobian(quadrotor_model_y,sym_model_inputs);


%% generate functions

if ~ENABLE_FUNCTION_GENERATION
    fprintf("Writing functions on file is disabled\n");
    return;
end


syms neigh_pos N_neighbours non_neigh_pos N_non_neighbours d_safe v_max real;
additional_INs = {neigh_pos, N_neighbours, non_neigh_pos, N_non_neighbours, d_safe, v_max};

% matlabFunction(lin_motion,'file',"lin_motion.m",'vars',{sym_model_state,sym_model_inputs,sym_model_params},'optimize',false);
% matlabFunction(rot_motion,'file',"rot_motion.m",'vars',{sym_model_state,sym_model_inputs,sym_model_params},'optimize',false);
% matlabFunction(om,'file','om.m','vars',{sym_model_state,sym_model_inputs,sym_model_params},'optimize',false);
matlabFunction(quadrotor_model,'file','quadrotor_model.m','vars',{sym_model_state,sym_model_inputs,additional_INs{:},sym_model_params},'optimize',false);
matlabFunction(quadrotor_model_jac_x,quadrotor_model_jac_u,'file','quadrotor_model_jac.m','vars',{sym_model_state,sym_model_inputs,additional_INs{:},sym_model_params},'Outputs',{'J_f_x','J_f_u'},'optimize',false);
matlabFunction(quadrotor_model_y,'file','quadrotor_output.m','vars',{sym_model_state,sym_model_inputs,additional_INs{:},sym_model_params},'optimize',false);
matlabFunction(quadrotor_model_y_jac_x, quadrotor_model_y_jac_u,'file','quadrotor_output_jac.m','vars',{sym_model_state,sym_model_inputs,additional_INs{:},sym_model_params},'Outputs',{'J_y_x','J_y_u'},'optimize',false);

% % correct the sintax to be used in simulink
% correctSintax('./lin_motion.m');
% correctSintax('./rot_motion.m');
% correctSintax('./om.m');
% correctSintax('./quadrotor_model.m');
% correctSintax('./quadrotor_model_jac.m');
% correctSintax('./quadrotor_output.m');s
% correctSintax('./model_input_constraints.m');

fprintf("Functions correctly generated and written on file\n");