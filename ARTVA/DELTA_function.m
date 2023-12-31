function DELTAS = DELTA_function(in1,in2,in3)
%DELTA_function
%    DELTAS = DELTA_function(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    13-Nov-2023 18:46:10

a = in3(:,1);
b = in3(:,2);
p_r_x_1 = in1(1);
p_r_x_2 = in1(2);
p_r_y_1 = in1(6);
p_r_x_3 = in1(3);
p_r_y_2 = in1(7);
p_r_z_1 = in1(11);
p_r_x_4 = in1(4);
p_r_y_3 = in1(8);
p_r_z_2 = in1(12);
p_r_x_5 = in1(5);
p_r_y_4 = in1(9);
p_r_z_3 = in1(13);
p_r_y_5 = in1(10);
p_r_z_4 = in1(14);
p_r_z_5 = in1(15);
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
t15 = p_r_x_5+t4;
t16 = p_r_z_2+t6;
t17 = p_r_y_4+t5;
t18 = p_r_z_3+t6;
t19 = p_r_y_5+t5;
t20 = p_r_z_4+t6;
t21 = p_r_z_5+t6;
t22 = t7.^2;
t23 = t8.^2;
t24 = t9.^2;
t25 = t10.^2;
t26 = t11.^2;
t27 = t12.^2;
t28 = t13.^2;
t29 = t14.^2;
t30 = t15.^2;
t31 = t16.^2;
t32 = t17.^2;
t33 = t18.^2;
t34 = t19.^2;
t35 = t20.^2;
t36 = t21.^2;
t37 = t22+t24+t28;
t38 = t23+t26+t31;
t39 = t25+t29+t33;
t40 = t27+t32+t35;
t41 = t30+t34+t36;
t42 = 1.0./t37;
t43 = 1.0./t38;
t44 = 1.0./t39;
t45 = 1.0./t40;
t46 = 1.0./t41;
DELTAS = [-t37.*(-1.0./(t22.*t42.*3.0+1.0).^(1.0./3.0)+t3.*(t22.*t42-1.0)+t2.*t22.*t42);-t38.*(-1.0./(t23.*t43.*3.0+1.0).^(1.0./3.0)+t3.*(t23.*t43-1.0)+t2.*t23.*t43);-t39.*(-1.0./(t25.*t44.*3.0+1.0).^(1.0./3.0)+t3.*(t25.*t44-1.0)+t2.*t25.*t44);-t40.*(-1.0./(t27.*t45.*3.0+1.0).^(1.0./3.0)+t3.*(t27.*t45-1.0)+t2.*t27.*t45);-t41.*(-1.0./(t30.*t46.*3.0+1.0).^(1.0./3.0)+t3.*(t30.*t46-1.0)+t2.*t30.*t46)];
