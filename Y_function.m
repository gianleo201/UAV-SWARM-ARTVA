function Y = Y_function(in1,in2,in3,in4,m,w_noise)
%Y_function
%    Y = Y_function(IN1,IN2,IN3,IN4,M,W_NOISE)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    02-Oct-2023 12:23:04

a = in3(:,1);
b = in3(:,2);
m_11 = in4(:,1);
m_12 = in4(:,2);
m_13 = in4(:,3);
m_22 = in4(:,4);
m_23 = in4(:,5);
m_33 = in4(:,6);
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
t2 = a.^2;
t3 = b.^2;
t4 = p_t_x.^2;
t5 = p_t_y.^2;
t6 = p_t_z.^2;
t9 = -p_t_x;
t10 = -p_t_y;
t11 = -p_t_z;
t12 = 2.0.^(2.0./3.0);
t13 = m.^(2.0./3.0);
t26 = 1.0./pi.^(2.0./3.0);
t7 = 1.0./t2;
t8 = 1.0./t3;
t14 = p_r_x_1+t9;
t15 = p_r_x_2+t9;
t16 = p_r_y_1+t10;
t17 = p_r_x_3+t9;
t18 = p_r_y_2+t10;
t19 = p_r_x_4+t9;
t20 = p_r_z_1+t11;
t21 = p_r_y_3+t10;
t22 = p_r_z_2+t11;
t23 = p_r_y_4+t10;
t24 = p_r_z_3+t11;
t25 = p_r_z_4+t11;
t27 = t2.*t5;
t28 = t2.*t6;
t29 = t3.*t4;
t30 = t14.^2;
t31 = t15.^2;
t32 = t16.^2;
t33 = t17.^2;
t34 = t18.^2;
t35 = t19.^2;
t36 = t20.^2;
t37 = t21.^2;
t38 = t22.^2;
t39 = t23.^2;
t40 = t24.^2;
t41 = t25.^2;
t42 = t30+t32+t36;
t43 = t31+t34+t38;
t44 = t33+t37+t40;
t45 = t35+t39+t41;
t46 = 1.0./t42;
t47 = 1.0./t43;
t48 = 1.0./t44;
t49 = 1.0./t45;
mt1 = [t27+t28+t29+m_11.*p_r_x_1.^2+m_22.*p_r_y_1.^2+m_33.*p_r_z_1.^2-t42.*(-1.0./(t30.*t46.*3.0+1.0).^(1.0./3.0)+t8.*(t30.*t46-1.0)+t7.*t30.*t46)+m_12.*p_r_x_1.*p_r_y_1.*2.0+m_13.*p_r_x_1.*p_r_z_1.*2.0+m_23.*p_r_y_1.*p_r_z_1.*2.0-p_r_x_1.*p_t_x.*t3.*2.0-p_r_y_1.*p_t_y.*t2.*2.0-p_r_z_1.*p_t_z.*t2.*2.0+(t2.*t3.*t12.*t13.*t26.*t42.^(5.0./2.0).*w_noise)./4.0;t27+t28+t29+m_11.*p_r_x_2.^2+m_22.*p_r_y_2.^2+m_33.*p_r_z_2.^2-t43.*(-1.0./(t31.*t47.*3.0+1.0).^(1.0./3.0)+t8.*(t31.*t47-1.0)+t7.*t31.*t47)+m_12.*p_r_x_2.*p_r_y_2.*2.0+m_13.*p_r_x_2.*p_r_z_2.*2.0+m_23.*p_r_y_2.*p_r_z_2.*2.0-p_r_x_2.*p_t_x.*t3.*2.0-p_r_y_2.*p_t_y.*t2.*2.0-p_r_z_2.*p_t_z.*t2.*2.0+(t2.*t3.*t12.*t13.*t26.*t43.^(5.0./2.0).*w_noise)./4.0];
mt2 = [t27+t28+t29+m_11.*p_r_x_3.^2+m_22.*p_r_y_3.^2+m_33.*p_r_z_3.^2-t44.*(-1.0./(t33.*t48.*3.0+1.0).^(1.0./3.0)+t8.*(t33.*t48-1.0)+t7.*t33.*t48)+m_12.*p_r_x_3.*p_r_y_3.*2.0+m_13.*p_r_x_3.*p_r_z_3.*2.0+m_23.*p_r_y_3.*p_r_z_3.*2.0-p_r_x_3.*p_t_x.*t3.*2.0-p_r_y_3.*p_t_y.*t2.*2.0-p_r_z_3.*p_t_z.*t2.*2.0+(t2.*t3.*t12.*t13.*t26.*t44.^(5.0./2.0).*w_noise)./4.0;t27+t28+t29+m_11.*p_r_x_4.^2+m_22.*p_r_y_4.^2+m_33.*p_r_z_4.^2-t45.*(-1.0./(t35.*t49.*3.0+1.0).^(1.0./3.0)+t8.*(t35.*t49-1.0)+t7.*t35.*t49)+m_12.*p_r_x_4.*p_r_y_4.*2.0+m_13.*p_r_x_4.*p_r_z_4.*2.0+m_23.*p_r_y_4.*p_r_z_4.*2.0-p_r_x_4.*p_t_x.*t3.*2.0-p_r_y_4.*p_t_y.*t2.*2.0-p_r_z_4.*p_t_z.*t2.*2.0+(t2.*t3.*t12.*t13.*t26.*t45.^(5.0./2.0).*w_noise)./4.0];
Y = [mt1;mt2];
