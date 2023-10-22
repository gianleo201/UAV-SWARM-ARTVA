function J = ObjFunction(X,U,e,data,neigh_pos,N_neighbours,non_neigh_pos,N_non_neighbours,d_safe, v_max)

    p = data.PredictionHorizon;
    max_rkO = N_neighbours+1;

    X1_ref = data.References(1:p,1);
    X2_ref = data.References(1:p,2);
    X3_ref = data.References(1:p,3);
    X4_ref = data.References(1:p,4);

    X1 = X(2:p+1,1);
    X2 = X(2:p+1,2);
    X3 = X(2:p+1,3);
    X4 = X(2:p+1,4);

    U1 = U(1:p,1);
    U2 = U(1:p,2);

    XNeigh = zeros(p,2*N_neighbours);
    for i = 1:N_neighbours
        XNeigh(:,2*i-1:2*i) = repmat(neigh_pos(i,1:2),p,1);
    end
    XNNeigh = zeros(p,2*N_non_neighbours);
    for i = 1:N_non_neighbours
        XNNeigh(:,2*i-1:2*i) = repmat(non_neigh_pos(i,1:2),p,1);
    end

    XS = [X1 X2];

    % maximize neighbours distances
%     XID = 0;
% %     WD = 0.1 * ones(1,N_neighbours);
%     for i = 1:N_neighbours
%         temp = XS-XNeigh(:,2*i-1:2*i);
%         if norm(temp(1,:)) <= (d_safe + v_max * p*0.1)
%             temp1 = 0;
%             for j = 1:p
% %                 temp1 = temp1 + temp(j,:)*temp(j,:).';
%                 temp1 = temp1 + (sqrt(temp(j,:)*temp(j,:).')-(d_safe + v_max * p*0.1))^2;
%             end
% %             XID = XID + WD(i)*temp1 ;
%         XID = XID + temp1 ;
%         end
%     end
%     % maximize non niegh dists
%     XNID = 0;
% %     WND = 0.1 * ones(1,N_non_neighbours);
%     for i = 1:N_non_neighbours
%         temp = XS-XNNeigh(:,2*i-1:2*i);
%         if norm(temp(1,:)) <= (d_safe + 2*v_max * p*0.1) && norm(temp(1,:)) <= (d_safe + v_max * p*0.1)
%             temp1 = 0;
%             for j = 1:p
% %                 temp1 = temp1 + temp(j,:)*temp(j,:).';
%                 temp1 = temp1 + (sqrt(temp(j,:)*temp(j,:).')-(d_safe + v_max * p*0.1))^2;
%             end
% %             XNID = XNID + WND(i)*temp1 ;
%             XNID = XNID + temp1 ;
%         end
%     end


    % maximize neighbours distances
    XID = 0;
%     WD = 0.1 * ones(1,N_neighbours);
    for i = 1:N_neighbours
        temp = XS-XNeigh(:,2*i-1:2*i);
        temp1 = 0;
        for j = 1:p
%                 temp1 = temp1 + temp(j,:)*temp(j,:).';
            if norm(temp(j,:)) <= d_safe + v_max * p * 0.1
                temp1 = temp1 + (sqrt(temp(j,:)*temp(j,:).')-(d_safe + v_max * p*0.1))^2;
            end
        end
%         XID = XID + WD(i)*temp1 ;
        XID = XID + temp1 ;
    end
    % maximize dist to nearest non_niegh
    XNID = 0;
    if N_non_neighbours > 0
        nearest_non_neighbour = non_neigh_pos(1,1:2);
        nearest_index = 1;
        for i = 1:N_non_neighbours
            if norm(X(1,1:2)-non_neigh_pos(i,1:2)) < norm(X(1,1:2)-nearest_non_neighbour)
                nearest_non_neighbour = non_neigh_pos(i,1:2);
                nearest_index = i;
            end
        end
        if norm(X(1,1:2)-nearest_non_neighbour) <= (d_safe + 2*v_max * p*0.1)
            temp = XS-XNNeigh(:,2*nearest_index-1:2*nearest_index);
            temp1 = 0;
            for j = 1:p
%                 temp1 = temp1 + temp(j,:)*temp(j,:).';
                if norm(temp(j,:)) <= d_safe + v_max * p * 0.1
                    temp1 = temp1 + (sqrt(temp(j,:)*temp(j,:).')-(d_safe + v_max * p*0.1))^2;
                end
            end
%             XNID = XNID + WND(i)*temp1 ;
            XNID = XNID + temp1 ;
        end
    end
        


    O = zeros(10,10);
    for k = 1:p
        temp = single_H_function([X1(k) X2(k) 0]);
        if max_rkO > 1
            for j = 1:max_rkO-1
                temp = [temp single_H_function([neigh_pos(j,1:2) 0])];
            end
        end
        O = O + temp*temp.';
    end
    O = O / p;
    
    sigma_lower = min_sv_O(O,max_rkO);

    Wx = (1/(2*d_safe))^2;
    Wdx = (1/(v_max/2))^2;
    Wu = (1/9.81)^2;
    We = (1/(0.1*d_safe))^2;
    Wxnid = (d_safe)^2;
%     Wxnid = 0;
%     Wxnid = (10)^2;
    Wsigma = (1e-02)^2;
%     Wsigma = (10)^2;
%     Wsigma = (0)^2;

   
%     J = Wx * sum((X1_ref-X1).^2) + ...
%         Wx * sum((X2_ref-X2).^2) + ...
%         Wdx * sum((X3_ref-X3).^2) + ...
%         Wdx * sum((X4_ref-X4).^2) + ...
%         Wu * ( sum(U1.^2) + sum(U2.^2) ) + ...
%         We * e(end).^2 - ...
%         Wsigma * sigma_lower; % 1.1667 W_sigma per drone

    J = Wx * sum((X1_ref-X1).^2) + ...
        Wx * sum((X2_ref-X2).^2) + ...
        Wdx * sum((X3_ref-X3).^2) + ...
        Wdx * sum((X4_ref-X4).^2) + ...
        Wu * (sum(U1.^2)+sum(U2.^2)) + ...
        We * e.^2 + ...
        Wxnid * (XID + XNID) - ...
        Wsigma * sigma_lower; % 1.1667 W_sigma per drone

end