function [L_f_g_k] = Lie_derivative(f,g,x,k)
    if k == 0
        L_f_g_k = g; return;
    end
    L_f_g_k = simplify(jacobian(g,x)*f);
    for i = 1:k-1
        L_f_g_k = simplify(jacobian(L_f_g_k,x)*f);
    end
end

