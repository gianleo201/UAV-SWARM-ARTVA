ENABLE_FUNCTION_GENERATION = true;
RPY_FIXED_AXIS = true;
CHANGE_OF_INPUT = false;

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
w = [p;q;r];  % angular velocity in rotating frame

%% symbolic control inputs variables

if CHANGE_OF_INPUT 
    syms T tau_phi tau_theta tau_psi real;
else
    syms T1 T2 T3 T4 real;
end

%% symbolic collective vectors

sym_model_state = [x y z dx dy dz p q r phi theta psi].';

if CHANGE_OF_INPUT
    sym_model_inputs = [T tau_phi tau_theta tau_psi].';
else
    sym_model_inputs = [T1 T2 T3 T4].';
end

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

if CHANGE_OF_INPUT
    T_r = [0;0;T];
else
    T_r = [0;0;T1+T2+T3+T4];
end

T_f = simplify(R_fr*T_r); % thrust in fixed frame

lin_motion = simplify((gravity_f+T_f)/m); % ode velocities in fixed frame

%% rotational equations of motion

if CHANGE_OF_INPUT
    tau_r = [tau_phi;tau_theta;tau_psi];
else
    tau_r = [(a*(T1 + T2 - T3 - T4))/2;
            (b*(-T1 + T2 + T3 - T4))/2;
           -(rD/rT)*(T1 - T2 + T3 - T4)];
end

wxJw = [q*r*(Jzz-Jyy);
        p*r*(Jxx-Jzz);
        p*q*(Jyy-Jxx)];

rot_motion = simplify(inv(J)*(tau_r-wxJw)); % ode angular velocities

%% kinematic equations (d_(roll,pitch,yaw)_dt <- w)

temp = simplify(tRz*tRy);
om_inv = [temp(:,1) tRz(:,2) [0;0;1]]; % fro PHI_dot to w
clear temp;
om = simplify(inv(om_inv)*R_fr*w); % om = [phi_dot;theta_dot;psi_dot] 

%% compute total model

quadrotor_model = [dx;dy;dz;lin_motion;rot_motion;om];


G = jacobian(quadrotor_model,sym_model_inputs);
F = simplify(quadrotor_model-G*sym_model_inputs);


%% compute total model jacobians

quadrotor_model_jac_x = jacobian(quadrotor_model,sym_model_state);
quadrotor_model_jac_u = jacobian(quadrotor_model,sym_model_inputs);

%% output function: [x y z roll pitch yaw]

quadrotor_model_y = [x y z psi].';
quadrotor_model_y_jac_x = jacobian(quadrotor_model_y,sym_model_state);
quadrotor_model_y_jac_u = jacobian(quadrotor_model_y,sym_model_inputs);

%% build FL

if CHANGE_OF_INPUT

%     syms dphi dtheta dpsi real;
%     simplified_quadrotor_model = subs(quadrotor_model,[p q r],[0 0 0]);
%     simplified_quadrotor_model(10:12) = [dphi;dtheta;dpsi];
%     simplified_states = sym_model_state;
%     simplified_states(7:9) = [dphi dtheta dpsi].';
    
    simplified_quadrotor_model = quadrotor_model;
    simplified_states = sym_model_state;

    G0 = jacobian(simplified_quadrotor_model,sym_model_inputs);
    F0 = simplify(simplified_quadrotor_model - G0*sym_model_inputs);

    DM = Lie_derivative(G0,Lie_derivative(F0,quadrotor_model_y,simplified_states,1),simplified_states,1);

    % Feedback linearization algorithm
    syms Tdot real;
    simplified_states_aug1 = [simplified_states;T];
    simplified_quadrotor_model_aug1 = [simplified_quadrotor_model;Tdot];
    quadrotor_model_input1 = [Tdot;sym_model_inputs(2:4)];

    G1 = jacobian(simplified_quadrotor_model_aug1,quadrotor_model_input1);
    F1 = simplify(simplified_quadrotor_model_aug1 - G1*quadrotor_model_input1);

    DM1_4 = Lie_derivative(G1,Lie_derivative(F1,quadrotor_model_y(4),simplified_states_aug1,1),simplified_states_aug1,1);
    DM1_13 = Lie_derivative(G1,Lie_derivative(F1,quadrotor_model_y(1:3),simplified_states_aug1,2),simplified_states_aug1,1);

    DM1 = [DM1_13;DM1_4];

    syms Tddot real;
    simplified_states_aug2 = [simplified_states_aug1;Tdot];
    simplified_quadrotor_model_aug2 = [simplified_quadrotor_model_aug1;Tddot];
    quadrotor_model_input2 = [Tddot;sym_model_inputs(2:4)];

    G2 = jacobian(simplified_quadrotor_model_aug2,quadrotor_model_input2);
    F2 = simplify(simplified_quadrotor_model_aug2 - G2*quadrotor_model_input2);
    
    DM2_4 = Lie_derivative(G2,Lie_derivative(F2,quadrotor_model_y(4),simplified_states_aug2,1),simplified_states_aug2,1);
    DM2_13 = Lie_derivative(G2,Lie_derivative(F2,quadrotor_model_y(1:3),simplified_states_aug2,3),simplified_states_aug2,1);

    DM2 = simplify([DM2_13;DM2_4]);

    inv_DM2 = inv(DM2);
    D2 = simplify([Lie_derivative(F2,quadrotor_model_y(1:3),simplified_states_aug2,4);Lie_derivative(F2,quadrotor_model_y(4),simplified_states_aug2,2)]);

    pos_acc = simplify(Lie_derivative(F2,quadrotor_model_y(1:3),simplified_states_aug2,2));
    pos_jerk = simplify(Lie_derivative(F2,quadrotor_model_y(1:3),simplified_states_aug2,3));

    syms T1 T2 T3 T4 real;
    inv_input_mapping = [ T1 + T2 + T3 + T4;
                         (a*(T1 + T2 - T3 - T4))/2;
                         (b*(-T1 + T2 + T3 - T4))/2;
                        -(rD/rT)*(T1 - T2 + T3 - T4)];
    TM = jacobian(inv_input_mapping,[T1 T2 T3 T4]);
    input_mapping_matrix = inv(TM);
    input_mapping = simplify(input_mapping_matrix*sym_model_inputs);
    clear T1 T2 T3 T4;

    if ~exist("./FL controller","dir")
        mkdir("FL controller");
    end
    cd("FL controller");
    % convert new inputs into blades thrusts
    matlabFunction(input_mapping,'file','input_mapping.m','vars',{sym_model_inputs,sym_model_params});
    % matrix to conver thusts into new inputs
    matlabFunction(TM,'file','inv_input_mapping_matrix.m','vars',{sym_model_params});
    % compute acceleration
    matlabFunction(pos_acc,'file','compute_pos_acc.m','vars',{simplified_states_aug2,sym_model_params});
    % compute jerk
    matlabFunction(pos_jerk,'file','compute_pos_jerk.m','vars',{simplified_states_aug2,sym_model_params});
    % compute decoupling matrix inverse
    matlabFunction(inv_DM2,'file','inv_DM.m','vars',{simplified_states_aug2,sym_model_params});
    % compute feedback compensation
    matlabFunction(D2,'file','compensation_term.m','vars',{simplified_states_aug2,sym_model_params});
    cd ..

end

%% build 2nd order exponential CBF

if ~CHANGE_OF_INPUT  
    syms x_obs y_obs z_obs x_obs_dot y_obs_dot z_obs_dot x_obs_ddot y_obs_ddot z_obs_ddot d_dist k_0 k_1 real;
    
    p_obs = [x_obs y_obs z_obs].';
    p_obs_dot = [x_obs_dot y_obs_dot z_obs_dot].';
    p_obs_ddot = [x_obs_ddot y_obs_ddot z_obs_ddot].';
    
    h_f = norm([x;y;z]-p_obs) - d_dist;
    h_f_dot = simplify(jacobian(h_f,sym_model_state)*quadrotor_model + jacobian(h_f,p_obs)*p_obs_dot);
    h_f_ddot = simplify(jacobian(h_f_dot,sym_model_state)*quadrotor_model + jacobian(h_f_dot,[p_obs;p_obs_dot])*[p_obs_dot;p_obs_ddot]);
end

%% generate functions

if ~ENABLE_FUNCTION_GENERATION
    fprintf("Writing functions on file is disabled\n");
    return;
end


syms neigh_pos N_neighbours non_neigh_pos N_non_neighbours d_safe v_max real;
additional_INs = {neigh_pos, N_neighbours, non_neigh_pos, N_non_neighbours, d_safe, v_max};

% matlabFunction(lin_motion,'file',"lin_motion.m",'vars',{sym_model_state,sym_model_inputs,sym_model_params},'optimize',false);
% matlabFunction(rot_motion,'file',"rot_motion.m",'vars',{sym_model_state,sym_model_inputs,sym_model_params},'optimize',false);
matlabFunction(om,'file','om.m','vars',{sym_model_state(7:12)},'optimize',false);
matlabFunction(quadrotor_model,'file','quadrotor_model.m','vars',{sym_model_state,sym_model_inputs,additional_INs{:},sym_model_params},'optimize',false);
matlabFunction(quadrotor_model_jac_x,quadrotor_model_jac_u,'file','quadrotor_model_jac.m','vars',{sym_model_state,sym_model_inputs,additional_INs{:},sym_model_params},'Outputs',{'J_f_x','J_f_u'},'optimize',false);
matlabFunction(quadrotor_model_y,'file','quadrotor_output.m','vars',{sym_model_state,sym_model_inputs,additional_INs{:},sym_model_params},'optimize',false);
matlabFunction(quadrotor_model_y_jac_x, quadrotor_model_y_jac_u,'file','quadrotor_output_jac.m','vars',{sym_model_state,sym_model_inputs,additional_INs{:},sym_model_params},'Outputs',{'J_y_x','J_y_u'},'optimize',false);
% CBF generation
matlabFunction(-(h_f_ddot+k_1*h_f_dot+k_0*h_f),'file','real_UAV_CBF','vars',{sym_model_state,sym_model_inputs,sym_model_params,p_obs,p_obs_dot,p_obs_ddot,d_dist,k_0,k_1});

% % correct the sintax to be used in simulink
% correctSintax('./lin_motion.m');
% correctSintax('./rot_motion.m');
% correctSintax('./om.m');
% correctSintax('./quadrotor_model.m');
% correctSintax('./quadrotor_model_jac.m');
% correctSintax('./quadrotor_output.m');s
% correctSintax('./model_input_constraints.m');

fprintf("Functions correctly generated and written on file\n");