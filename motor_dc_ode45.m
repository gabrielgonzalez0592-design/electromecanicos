
clear;
clc;
close all;

motor.Ra = 2;
motor.Kt = 0.01;
motor.b = 0.0012;
motor.La = 0.023;
motor.Ke = 0.01;
motor.J = 0.001;

va = 12;
Tload = 0;
tspan = [0 2];
x0 = [0; 0; 0]; % x = [theta_m; omega_m; i_a]

[time, state] = ode45(@(t, x) motor_dc_rhs(t, x, motor, va, Tload), tspan, x0);

theta = state(:, 1);
omega = state(:, 2);
ia = state(:, 3);

figure('Name', 'Motor DC - ode45');
subplot(3, 1, 1);
plot(time, theta, 'LineWidth', 1.5);
grid on;
ylabel('Posicion (rad)');
title('Respuesta del motor DC');

subplot(3, 1, 2);
plot(time, omega, 'LineWidth', 1.5);
grid on;
ylabel('Velocidad (rad/s)');

subplot(3, 1, 3);
plot(time, ia, 'LineWidth', 1.5);
grid on;
xlabel('Tiempo (s)');
ylabel('Corriente (A)');

fprintf('Equilibrio esperado para va = %.2f V y Tload = %.2f N.m:\n', va, Tload);
fprintf('  omega = %.4f rad/s, ia = %.6f A\n', ...
    motor.Kt * va / (motor.Ra * motor.b + motor.Kt * motor.Ke), ...
    motor.b * va / (motor.Ra * motor.b + motor.Kt * motor.Ke));

function dx = motor_dc_rhs(~, x, motor, va, Tload)
omega = x(2);
ia = x(3);

dtheta = omega;
domega = (motor.Kt * ia - motor.b * omega - Tload) / motor.J;
dia = (va - motor.Ra * ia - motor.Ke * omega) / motor.La;

dx = [dtheta; domega; dia];
end