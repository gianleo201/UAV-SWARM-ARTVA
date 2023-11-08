%% Determine best values for a and b
min_opts.Display = 'iter-detailed';
min_opts.TolX = 1e-06;
min_opts.TolFun = 1e-06;
result = fminsearch(@sum_sqr_err,[1 1],min_opts);


function sum_sqr_error = sum_sqr_err(x)
    a = x(1);
    b = x(2);
    THETA_vec = 0:0.00001:pi; % cos(x)^2 and sin(x)^2 are pi periodic functions
    sum_sqr_error = 0;
    for i = 1 : length(THETA_vec)
        THETA = THETA_vec(i);
        real_f = (1/(1+3*cos(THETA)^2)^(1/3));
        approx_function = a^(-2)*cos(THETA)^2+b^(-2)*sin(THETA)^2;
        sum_sqr_error = sum_sqr_error + (real_f - approx_function)^2;
    end
end