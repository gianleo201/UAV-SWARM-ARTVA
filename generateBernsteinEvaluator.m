%% set up optimization problem

EnvINIT;

% choose the weights
functional_weights = [1 1 1];

% approximate trajectories with bernstain polynomials
N_approx_bernstain = 5; % polynomial approximation degree
syms t tf real; % temporal variables
syms BnTraj_ [N 2] real;
Bns = sym('Bns',[N 2 N_approx_bernstain+1]);
for i=1:N
    eval(['syms BN_' num2str(i) '_x_ [1 N_approx_bernstain+1] real;']);
    eval(['syms BN_' num2str(i) '_y_ [1 N_approx_bernstain+1] real;']);
    Bns(i,1,:) = eval(['BN_' num2str(i) '_x_']);
    Bns(i,2,:) = eval(['BN_' num2str(i) '_y_']);
    BnTraj_(i,:) = [0 0];
    % fill the polynomial in the list
    for j=1:N_approx_bernstain+1
        eval(['BnTraj_(i,:) = BnTraj_(i,:) + [BN_' num2str(i) '_x_(j)*nchoosek(N_approx_bernstain,j-1)*(t)^(j-1)*(tf-t)^(N_approx_bernstain-j+1)/(tf)^N_approx_bernstain, BN_' num2str(i) '_y_(j)*nchoosek(N_approx_bernstain,j-1)*(t)^(j-1)*(tf-t)^(N_approx_bernstain-j+1)/(tf)^N_approx_bernstain];']);
    end
end

matlabFunction(BnTraj_,"File","Bernstein_eval","vars",{t,tf,Bns});
matlabFunction(simplify(diff(BnTraj_,t)),"File","D_Bernstein_eval","vars",{t,tf,Bns});
DD_BnTraj_ = simplify(diff(BnTraj_,t,2)); % second order derivatives
matlabFunction(DD_BnTraj_,"File","DD_Bernstein_eval","vars",{t,tf,Bns});

% symbolic expressions of integral of norms

Integral_expr = sym('Integral_expr',[N 1]);

for i=1:N
    Integral_expr(i) = simplify(int(norm(DD_BnTraj_(i,:))^2,t,0,tf));
end

matlabFunction(Integral_expr,"File","Integral_expr_eval","vars",{tf,Bns});