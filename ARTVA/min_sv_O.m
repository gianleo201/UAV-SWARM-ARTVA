function min_sv = min_sv_O(O,max_rank)
    % Wrapper function to extract the minimum singular value of O matrix

    [~,svs,~] = svd(O,"vector");
%     min_sv = svs(max_rank);
    min_sv = svs(rank(O));

end

