function c_ineq = IneqConstriant(X,U,e,data,params)
    p = data.PredictionHorizon;
    x = X(1:p+1,1:2);
    dx = X(1:p+1,3:4);
    c_ineq = zeros(2*(p+1),1);
    % velocity constraint
    for k = 1 : p+1
        c_ineq(k) = norm(dx(k,:)) - 5;
    end
%     % collision avoidance constraint
%     i = 1;
%     for k = p+2 : 2*(p+1)
%         if i == 1
%             display(norm(dx(i,:)));
%         end
%         c_ineq(k) = norm(dx(i,:)) - 2;
%         i = i + 1;
%     end
end