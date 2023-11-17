function min_sv = min_sv_O(O,max_rank)
    % Wrapper function to extract the minimum singular value of O matrix

    [~,svs,~] = svd(O,"vector");
%     min_sv = svs(max_rank);
    actual_rk = rank(O);
    if actual_rk > 0
        min_sv = svs(actual_rk);
    else
        min_sv = svs(1);
    end

end

