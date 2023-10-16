function J = ObjFunction(X,U,e,data,neigh_pos,N_neighbours,non_neigh_pos,N_non_neighbours,d_safe, v_max)
    p = data.PredictionHorizon;
    max_rkO = N_neighbours+1;
    X1_ref = data.References(1:p,1);
    X2_ref = data.References(1:p,2);
    X3_ref = data.References(1:p,3);
    X4_ref = data.References(1:p,4);
    XNeigh = zeros(p,2*N_neighbours);
    for i = 1:N_neighbours
        XNeigh(:,2*i-1:2*i) = repmat(neigh_pos(i,1:2),p,1);
    end
    XNNeigh = zeros(p,2*N_non_neighbours);
    for i = 1:N_neighbours
        XNNeigh(:,2*i-1:2*i) = repmat(neigh_pos(i,1:2),p,1);
    end
 
    X1 = X(2:p+1,1);
    X2 = X(2:p+1,2);
    X3 = X(2:p+1,3);
    X4 = X(2:p+1,4);

    XS = [X1 X2];
    % maximize neighbours distances
    XID = 0;
    WD = 1* ones(1,N_neighbours);
    for i = 1:N_neighbours
        temp = XS-XNeigh(:,2*i-1:2*i);
        temp1 = 0;
        for j = 1:p
            temp1 = temp1 + temp(j,:)*temp(j,:).';
        end
        XID = XID + WD(i)*temp1 ;
    end
    % maximize non niegh dists
    XNID = 0;
    WND = 10* ones(1,N_non_neighbours);
    for i = 1:N_non_neighbours
        temp = XS-XNNeigh(:,2*i-1:2*i);
        if norm(temp(1,:)) <= 5
            temp1 = 0;
            for j = 1:p
                temp1 = temp1 + temp(j,:)*temp(j,:).';
            end
            XNID = XNID + WND(i)*temp1 ;
        end
    end

    U1 = U(1:p,1);
    U2 = U(1:p,2);

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
    
    [~,svs,~] = svd(O,'vector');
    sigma_lower = svs(max_rkO);

    J = 5 * sum((X1_ref-X1).^2) + ...
        5 * sum((X2_ref-X2).^2) + ...
        5 * sum((X3_ref-X3).^2) + ...
        5 * sum((X4_ref-X4).^2) + ...
        10 * ( sum(U1.^2) + sum(U2.^2) ) + ...
        100 * e.^2 - ...
        XID - ...
        XNID - ...
        0.5 * sigma_lower; % 1.1667 W_sigma per drone
end