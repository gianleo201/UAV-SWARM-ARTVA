function min_sv = min_sv_O(O,max_rank)
    % extract minimum singular value

    min_sv = sqrt(svds(O,1,"smallestnz"));

%     [~,svs,~] = svd(O,"vector");
%     min_sv = svs(max_rank);
end

