function H = H_function(in1)
%H_function
%    H = H_function(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    10-Oct-2023 20:23:37

p_r_x_1 = in1(1);
p_r_x_2 = in1(2);
p_r_y_1 = in1(5);
p_r_x_3 = in1(3);
p_r_y_2 = in1(6);
p_r_z_1 = in1(9);
p_r_x_4 = in1(4);
p_r_y_3 = in1(7);
p_r_z_2 = in1(10);
p_r_y_4 = in1(8);
p_r_z_3 = in1(11);
p_r_z_4 = in1(12);
H = reshape([p_r_x_1.^2,p_r_x_1.*p_r_y_1.*2.0,p_r_x_1.*p_r_z_1.*2.0,p_r_y_1.^2,p_r_y_1.*p_r_z_1.*2.0,p_r_z_1.^2,p_r_x_1.*-2.0,p_r_y_1.*-2.0,p_r_z_1.*-2.0,1.0,p_r_x_2.^2,p_r_x_2.*p_r_y_2.*2.0,p_r_x_2.*p_r_z_2.*2.0,p_r_y_2.^2,p_r_y_2.*p_r_z_2.*2.0,p_r_z_2.^2,p_r_x_2.*-2.0,p_r_y_2.*-2.0,p_r_z_2.*-2.0,1.0,p_r_x_3.^2,p_r_x_3.*p_r_y_3.*2.0,p_r_x_3.*p_r_z_3.*2.0,p_r_y_3.^2,p_r_y_3.*p_r_z_3.*2.0,p_r_z_3.^2,p_r_x_3.*-2.0,p_r_y_3.*-2.0,p_r_z_3.*-2.0,1.0,p_r_x_4.^2,p_r_x_4.*p_r_y_4.*2.0,p_r_x_4.*p_r_z_4.*2.0,p_r_y_4.^2,p_r_y_4.*p_r_z_4.*2.0,p_r_z_4.^2,p_r_x_4.*-2.0,p_r_y_4.*-2.0,p_r_z_4.*-2.0,1.0],[10,4]);
