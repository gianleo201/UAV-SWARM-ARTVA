function min_sv = min_sv_O(O,max_rank)
    % Wrapper function to extract the minimum singular value of the
    % gramian output matrix. 

    % Use svds (not available for code generation)
%     min_sv = svds(O,1,"smallestnz");

    % svd and take the lowest nonzero (to be defined)
%     [~,svs,~] = svd(O,'vector');
%     mask = (svs >= 1e-17);
%     svs = svs(mask);
%     [min_sv,~] = min(svs);

    % svd and take what should be the first nonzero sv
    [~,svs,~] = svd(O,"vector");
    min_sv = svs(max_rank);

end

