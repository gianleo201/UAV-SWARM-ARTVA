function [EMA_x_to_ref] = EMA_const_reference(x_i,x_f,lambda,p)
%EMA_CONST_REFERENCE Summary of this function goes here
%   Detailed explanation goes here
    EMA_x_to_ref_temp = zeros(p,size(x_i,2));
    EMA_x_to_ref_temp(1,:) = x_i;
    for i = 2:p
        EMA_x_to_ref_temp(i,:) = (1-lambda) * EMA_x_to_ref_temp(i-1,:) + lambda*x_f;
    end
    EMA_x_to_ref = EMA_x_to_ref_temp(2:end,:);
end

