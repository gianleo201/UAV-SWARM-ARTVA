function J = quadrotor_ObjFunction(X,U,e,data,neigh_pos,N_neighbours,non_neigh_pos,N_non_neighbours,d_safe, v_max,quadrotor_params)

    p = data.PredictionHorizon;
    max_rkO = N_neighbours+1;
    
    % position
    X1 = X(2:p+1,1);
    X2 = X(2:p+1,2);
    X3 = X(2:p+1,3);

%     XNeigh = zeros(p,3*N_neighbours);
%     XNeigh = zeros(p,3*size(neigh_pos,1)/3);
%     for i = 1:N_neighbours
%         XNeigh(:,3*i-2:3*i) = repmat(neigh_pos(i,1:3),p,1); % assuming constant position of obstacle along prediction horizon
%     end
%     XNNeigh = zeros(p,3*N_non_neighbours);
%     for i = 1:N_non_neighbours
%         XNNeigh(:,3*i-2:3*i) = repmat(non_neigh_pos(i,1:3),p,1); % % assuming constant position of obstacle along prediction horizon
%     end
    XNNeigh = repmat(non_neigh_pos(1,1:3),p,1);

    XS = [X1 X2 X3];

    % maximize neighbours distances
    XID = 0;
%     for i = 1:N_neighbours
%         temp = XS-XNeigh(:,3*i-2:3*i);
%         temp_dst = XNeigh(1,3*i-2:3*i)-X(1,1:3);
%         curr_vel = X(1,4:6);
%         var_sec_dist = 1+(curr_vel*temp_dst.'/(v_max*norm(temp_dst)));
%         temp1 = 0;
%         for j = 1:p
% %             if norm(temp(j,:)) <= d_safe + 2*v_max * p * 0.1
% %                 temp1 = temp1 + (sqrt(temp(j,:)*temp(j,:).')-(d_safe + 2*v_max * p*0.1))^2;
% %             end
%             if norm(temp(j,:)) <= d_safe + var_sec_dist*v_max * p * 0.1
%                 temp1 = temp1 + (sqrt(temp(j,:)*temp(j,:).')-(d_safe + var_sec_dist*v_max * p*0.1))^2;
%             end
%         end
%         XID = XID + temp1 ;
%     end

    % maximize dist to nearest non_niegh
    XNID = 0;
    if N_non_neighbours > 0
        nearest_non_neighbour = non_neigh_pos(1,1:3);
        temp_dst = nearest_non_neighbour-X(1,1:3);
        curr_vel = X(1,4:6);
        var_sec_dist = 1+(curr_vel*temp_dst.'/(v_max*norm(temp_dst)));
        temp = XS-XNNeigh;
        temp1 = 0;
        for j = 1:p
            if norm(temp(j,:)) <= d_safe + var_sec_dist*v_max * p * 0.1
                temp1 = temp1 + (sqrt(temp(j,:)*temp(j,:).')-(d_safe + var_sec_dist*v_max * p*0.1))^2;
            end
        end
        XNID = XNID + temp1 ; 
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

    sigma_lower_ref = 0.9/(N_neighbours+1);
    
    sigma_lower_error = min(0,sigma_lower-sigma_lower_ref)*(sigma_lower-sigma_lower_ref);

    
%     Wxnid = (1/0.1*d_safe)^2;
    Wxnid = 0;
%     Wsigma = (1/1e-02)^2;
    Wsigma = 0;

    % objective function
    J = Wxnid * (XID + XNID) + ...
        Wsigma * sigma_lower_error;

end