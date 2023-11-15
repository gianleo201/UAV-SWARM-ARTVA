function H = H_function(N,p_r)
    H = zeros(10,N);
    for i=1:N
        H(:,i) = single_H_function(p_r(i,:));
    end
end