function L_b = Lie_bracket(f,g,x,k)
    L_b = Lie_derivative(f,g,x,1)-Lie_derivative(g,f,x,1);
    for i = 1 : k-1
        L_b = Lie_derivative(f,L_b,x,1) - Lie_derivative(L_b,f,x,1);
    end
end