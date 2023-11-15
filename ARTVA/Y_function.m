function Y = Y_function(N,p_r,p_t,a_b,Ms,m,w_noise)
    Y = zeros(N,1);
    for i=1:N
        Y(i) = single_Y_function(p_r(i,:),p_t,a_b,Ms,m,w_noise);
    end
end