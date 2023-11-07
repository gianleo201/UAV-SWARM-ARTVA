function c_inputs = FL_step(aug_state,out_ref,model_params)
        
    if size(out_ref) == [1 4]
        ref = [out_ref;zeros(3,4)];
    else
        ref = out_ref;
    end
    
    K_x = [16 32 24 8];
    K_psi = [1 2];

    curr_state_p = [aug_state(1:3).';
                    aug_state(4:6).';
                    compute_pos_acc(aug_state,model_params).';
                    compute_pos_jerk(aug_state,model_params).'];
    
    phi_theta_psi_dot = om(aug_state(7:12));
    psi_dot = phi_theta_psi_dot(3);
    curr_state_psi = [aug_state(12);psi_dot];

    temp_x = K_x * (ref(:,1:3)-curr_state_p);
    temp_psi = K_psi * (ref(1:2,4)-curr_state_psi);

    temp = [temp_x temp_psi].';

    c_inputs = inv_DM(aug_state,model_params) * (-compensation_term(aug_state,model_params)+temp);

end