if exist("LAST_N.mat","file")
    LAST_N = load("LAST_N.mat");
    if LAST_N.N == N
        fprintf("Model already generated for %d UAVs\n",N);
        return;
    end
    
    save LAST_N.mat N;
    
    clear LAST_N;
else
    save LAST_N.mat N;
end

%% Symbolic variables

syms a b m real; % magnetic field params

syms p_x p_y p_z real; % position vector where the magnetic field is evaluated
p = [p_x p_y p_z].';  % by definition = p_r - p_t

syms THETA lambda norm_p real; % polar coordinates variables

syms p_r_x p_r_y p_r_z real; % generic reciever position symbolic variables

syms m_11 m_12 m_13 m_22 m_23 m_33 real; % vector of params to be estimated

syms w_noise real; % noise symbolic variable;

syms p_t_x p_t_y p_t_z real; % transimter position symbolic variables
p_t = [p_t_x p_t_y p_t_z].';

M_real = diag([b^2,a^2,a^2]); % supposing trasmitter parallel to inertial frame
p_t_bar = M_real*p_t;
rho = p_t.'*p_t_bar;

% specific reciever symbolic variable
syms p_r_x_ [N 1] real;
syms p_r_y_ [N 1] real;
syms p_r_z_ [N 1] real;
p_r = [p_r_x_ p_r_y_ p_r_z_];

%% transmitter output


X = [m_11 m_12 m_13 m_22 m_23 m_33 p_t_bar.' rho].'; % vector to be estimated

PHI = [p_r_x^2; 2*p_r_x*p_r_y; 2*p_r_x*p_r_z; p_r_y^2; 2*p_r_y*p_r_z; p_r_z^2; -2*p_r_x; -2*p_r_y; -2*p_r_z; 1];

etha = PHI.'*X;

DELTA = simplify(norm(p)^2)*( ...
    (1+3*cos(THETA)^2)^(-1/3) ...
    -a^(-2)*cos(THETA)^2 ...
    +b^(-2)*sin(THETA)^2);

temp = [2 - 3*sin(THETA)^2;
        3*cos(THETA)*sin(THETA)*cos(lambda);
        3*cos(THETA)*sin(THETA)*sin(lambda)];

temp = jacobian(temp,[THETA lambda]);

V_t = (a*b)^2*(m/(4*pi))^(2/3)*norm(p)^5*w_noise;

% transmitter output
y_t = etha + DELTA + V_t;

% build up H matrix
syms H [10 N] real;
for i=1:N
    H(:,i) = subs(PHI,[p_r_x p_r_y p_r_z],p_r(i,:));
    H(:,i) = simplify(H(:,i));
end

% build up Y output matrix
syms Y [N 1] real;
syms DELTAS [N 1] real;
for i=1:N
    current_p = p_r(i,:).'- p_t;
    Y(i) = subs(y_t,[p_r_x p_r_y p_r_z],p_r(i,:));
    Y(i) = subs(Y(i), p, current_p);
    Y(i) = subs(Y(i), THETA, acos(current_p(1)/norm(current_p)));
    Y(i) = simplify(Y(i));

    DELTAS(i)= subs(DELTA,[p_r_x p_r_y p_r_z],p_r(i,:));
    DELTAS(i) = subs(DELTAS(i), p, current_p);
    DELTAS(i) = subs(DELTAS(i),THETA, acos(current_p(1)/norm(current_p)));
    DELTAS(i) = simplify(DELTAS(i));
end

% save functions
file_path = fileparts(mfilename("fullpath"))+"\";

% matlabFunction(PHI,"File","single_H_function","vars",{[p_r_x p_r_y p_r_z]});
matlabFunction(H,"File",file_path+"H_function","vars",{p_r});

% matlabFunction(Y(1),"File","single_Y_function","vars",{p_r(1,:), p_t, [a b], [m_11 m_12 m_13 m_22 m_23 m_33], m, w_noise});
matlabFunction(Y,"File",file_path+"Y_function","vars",{p_r, p_t, [a b], [m_11 m_12 m_13 m_22 m_23 m_33], m, w_noise});

matlabFunction(DELTAS,"File",file_path+"DELTA_function","vars",{p_r, p_t, [a b]});
