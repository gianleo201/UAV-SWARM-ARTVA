function DELTAS = DELTA_function(in1,in2,in3)
%DELTA_function
%    DELTAS = DELTA_function(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    10-Oct-2023 13:35:21

a = in3(:,1);
b = in3(:,2);
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
p_t_x = in2(1,:);
p_t_y = in2(2,:);
p_t_z = in2(3,:);
t2 = 1.0./a.^2;
t3 = 1.0./b.^2;
t4 = -p_t_x;
t5 = -p_t_y;
t6 = -p_t_z;
t7 = p_r_x_1+t4;
t8 = p_r_x_2+t4;
t9 = p_r_y_1+t5;
t10 = p_r_x_3+t4;
t11 = p_r_y_2+t5;
t12 = p_r_x_4+t4;
t13 = p_r_z_1+t6;
t14 = p_r_y_3+t5;
t15 = p_r_z_2+t6;
t16 = p_r_y_4+t5;
t17 = p_r_z_3+t6;
t18 = p_r_z_4+t6;
t19 = t7.^2;
t20 = t8.^2;
t21 = t9.^2;
t22 = t10.^2;
t23 = t11.^2;
t24 = t12.^2;
t25 = t13.^2;
t26 = t14.^2;
t27 = t15.^2;
t28 = t16.^2;
t29 = t17.^2;
t30 = t18.^2;
t31 = t19+t21+t25;
t32 = t20+t23+t27;
t33 = t22+t26+t29;
t34 = t24+t28+t30;
t35 = 1.0./t31;
t36 = 1.0./t32;
t37 = 1.0./t33;
t38 = 1.0./t34;
DELTAS = [-t31.*(-1.0./(t19.*t35.*3.0+1.0).^(1.0./3.0)+t3.*(t19.*t35-1.0)+t2.*t19.*t35);-t32.*(-1.0./(t20.*t36.*3.0+1.0).^(1.0./3.0)+t3.*(t20.*t36-1.0)+t2.*t20.*t36);-t33.*(-1.0./(t22.*t37.*3.0+1.0).^(1.0./3.0)+t3.*(t22.*t37-1.0)+t2.*t22.*t37);-t34.*(-1.0./(t24.*t38.*3.0+1.0).^(1.0./3.0)+t3.*(t24.*t38-1.0)+t2.*t24.*t38)];
