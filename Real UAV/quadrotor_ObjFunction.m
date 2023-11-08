function J = quadrotor_ObjFunction(X,U,e,data,neigh_pos,N_neighbours,non_neigh_pos,N_non_neighbours,d_safe, v_max,quadrotor_params)

    p = data.PredictionHorizon;
    Ts = data.Ts;
    max_rkO = N_neighbours+1;
    
    % position reference
    X1_ref = data.References(1:p,1);
    X2_ref = data.References(1:p,2);
    X3_ref = data.References(1:p,3);

%     % velocity reference
%     X4_ref = data.References(1:p,4);
%     X5_ref = data.References(1:p,5);
%     X6_ref = data.References(1:p,6);
    
    % yaw reference
    X4_ref = data.References(1:p,4);
    

    % position
    X1 = X(2:p+1,1);
    X2 = X(2:p+1,2);
    X3 = X(2:p+1,3);

    % velocity
    X4 = X(2:p+1,4);
    X5 = X(2:p+1,5);
    X6 = X(2:p+1,6);

    % angluar rate
    P = X(2:p+1,7);
    Q = X(2:p+1,8);
    R = X(2:p+1,9);
    
    % orientation
    PHI = X(2:p+1,10);
    THETA = X(2:p+1,11);
    PSI = X(2:p+1,12);

    % thrust
    U1 = U(1:p,1);
    U2 = U(1:p,2);
    U3 = U(1:p,3);
    U4 = U(1:p,4);

    % thrust needed to hover
    U_ref = repmat(quadrotor_params(6)*quadrotor_params(7)/4 ...
                    ,p,1);

    XNeigh = zeros(p,3*N_neighbours);
    for i = 1:N_neighbours
        XNeigh(:,3*i-2:3*i) = repmat(neigh_pos(i,1:3),p,1); % assuming constant position of obstacle along prediction horizon
    end
    XNNeigh = zeros(p,3*N_non_neighbours);
    for i = 1:N_non_neighbours
        XNNeigh(:,3*i-2:3*i) = repmat(non_neigh_pos(i,1:3),p,1); % % assuming constant position of obstacle along prediction horizon
    end

    XS = [X1 X2 X3];

    % maximize neighbours distances
    XID = 0;
    for i = 1:N_neighbours
        temp = XS-XNeigh(:,3*i-2:3*i);
        temp_dst = XNeigh(1,3*i-2:3*i)-X(1,1:3);
        curr_vel = X(1,4:6);
        var_sec_dist = 1+(curr_vel*temp_dst.'/(v_max*norm(temp_dst)));
        temp1 = 0;
        for j = 1:p
%             if norm(temp(j,:)) <= d_safe + 2*v_max * p * 0.1
%                 temp1 = temp1 + (sqrt(temp(j,:)*temp(j,:).')-(d_safe + 2*v_max * p*0.1))^2;
%             end
            if norm(temp(j,:)) <= d_safe + var_sec_dist*v_max * p * 0.1
                temp1 = temp1 + (sqrt(temp(j,:)*temp(j,:).')-(d_safe + var_sec_dist*v_max * p*0.1))^2;
            end
        end
        XID = XID + temp1 ;
    end
    % maximize dist to nearest non_niegh
    XNID = 0;
    if N_non_neighbours > 0
        nearest_non_neighbour = non_neigh_pos(1,1:3);
        nearest_index = 1;
        for i = 1:N_non_neighbours
            if norm(X(1,1:3)-non_neigh_pos(i,1:3)) < norm(X(1,1:3)-nearest_non_neighbour)
                nearest_non_neighbour = non_neigh_pos(i,1:3);
                nearest_index = i;
            end
        end
        temp_dst = nearest_non_neighbour-X(1,1:3);
        curr_vel = X(1,4:6);
        var_sec_dist = 1+(curr_vel*temp_dst.'/(v_max*norm(temp_dst)));
        if norm(X(1,1:3)-nearest_non_neighbour) <= (d_safe + 2*v_max * p*0.1)
            temp = XS-XNNeigh(:,3*nearest_index-2:3*nearest_index);
            temp1 = 0;
            for j = 1:p
%                 if norm(temp(j,:)) <= d_safe + 2*v_max * p * 0.1
%                     temp1 = temp1 + (sqrt(temp(j,:)*temp(j,:).')-(d_safe + 2*v_max * p*0.1))^2;
%                 end
                if norm(temp(j,:)) <= d_safe + var_sec_dist*v_max * p * 0.1
                    temp1 = temp1 + (sqrt(temp(j,:)*temp(j,:).')-(d_safe + var_sec_dist*v_max * p*0.1))^2;
                end
            end
            XNID = XNID + temp1 ;
        end
    end
        
    % maximize observability index
    O = zeros(10,10);
    for k = 1:p
        temp = single_H_function([X1(k) X2(k) X3(k)]);
        if max_rkO > 1
            for j = 1:max_rkO-1
                temp = [temp single_H_function(neigh_pos(j,1:3))];
            end
        end
        O = O + temp*temp.';
    end
    O = O / p;
    sigma_lower = min_sv_O(O,max_rkO);

    
    % weights
    Wx = (1/(d_safe))^2;
    Wdx = (1/(v_max))^2;
    Wu = (1/(10-U_ref(1)))^2;
    We = (1/(2*d_safe))^2;
%     Wxnid = (0.1*d_safe)^2;
    Wxnid = (d_safe)^2;
%     Wxnid = 0;
%     Wxnid = (10)^2;
    Wsigma = (1e-06)^2;
%     Wsigma = (10)^2;
%     Wsigma = 0;

%     J = Wx * sum((X1_ref-X1).^2) + ...
%         Wx * sum((X2_ref-X2).^2) + ...
%         Wx * sum((X3_ref-X3).^2) + ...
%         (1/(pi/4))^2 * sum(THETA.^2) + ...
%         (1/((pi/4)/Ts))^2 * sum(P.^2+Q.^2+R.^2) + ...
%         Wdx * sum((X4_ref-X4).^2) + ...
%         Wdx * sum((X5_ref-X5).^2) + ...
%         Wdx * sum((X6_ref-X6).^2) + ...
%         Wu * sum((U1-U_ref).^2+(U2-U_ref).^2+(U3-U_ref).^2+(U4-U_ref).^2) + ...
%         We * e.^2 + ...
%         Wxnid * (XID + XNID) - ...
%         Wsigma * sigma_lower;

    J = Wx * sum((X1_ref-X1).^2) + ...
        Wx * sum((X2_ref-X2).^2) + ...
        Wx * sum((X3_ref-X3).^2) + ...
        (1/(pi/4))^2 * sum(PHI.^2+THETA.^2) + ...
        (1/(pi/6))^2 + sum((PSI-X4_ref).^2) + ...
        (1/((0.1*pi/2)/Ts))^2 * sum(P.^2+Q.^2+R.^2) + ...
        Wdx * sum(X4.^2+X5.^2+X6.^2) + ...
        Wu * sum((U1-U_ref).^2+(U2-U_ref).^2+(U3-U_ref).^2+(U4-U_ref).^2) + ...
        We * e.^2 + ...
        Wxnid * (XID + XNID) - ...
        Wsigma * sigma_lower;

end