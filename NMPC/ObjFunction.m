function J = ObjFunction(X,U,e,data,params)
    p = data.PredictionHorizon;
    X1_ref = data.References(1:p,1);
    X2_ref = data.References(1:p,2);
    X3_ref = data.References(1:p,1);
    X4_ref = data.References(1:p,2);
    X1 = X(2:p+1,1);
    X2 = X(2:p+1,2);
    X3 = X(2:p+1,3);
    X4 = X(2:p+1,4);
    U1 = U(1:p,1);
    U2 = U(1:p,2);
    O = zeros(10,10);
    for k = 1:p
        temp = single_H_function([X1(k) X2(k) 0]);
        O = O + temp*temp.';
    end
    O = O / p;

%     sigma_lower = sqrt(svds(O,1,"smallestnz"));
    [~,svs,~] = svd(O,'econ','vector');
    mask = (svs>1e-12);
    nzn_sigmas = svs(mask);
    sigma_lower = sqrt(nzn_sigmas(end));

    J = sum((X1_ref-X1).^2) + ...
        sum((X2_ref-X2).^2) + ...
        sum((X3_ref-X3).^2) + ...
        sum((X4_ref-X4).^2) + ...
        0.01*( sum(U1.^2) + sum(U2.^2) ) + ...
        - 400 * sigma_lower;
end