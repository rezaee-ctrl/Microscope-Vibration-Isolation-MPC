clear; clc; close all;

% 1. بارگذاری نتایج ذخیره شده از مراحل قبل
load('PID_Results.mat');
load('MPC_Results.mat');

% 2. تبدیل جابه‌جایی محور عمودی (Z) از متر به میلی‌متر
z_open_mm = y_open(:, 3) * 1000;
z_pid_mm  = y_pid(:, 3)  * 1000;
z_mpc_mm  = y_mpc(:, 3)  * 1000;

% 3. محاسبه‌ی بیشترین لرزش و درصد بهبود
peak_open = max(abs(z_open_mm));
peak_pid  = max(abs(z_pid_mm));
peak_mpc  = max(abs(z_mpc_mm));

imp_pid = (1 - peak_pid / peak_open) * 100;
imp_mpc = (1 - peak_mpc / peak_open) * 100;

% چاپ خلاصه نتایج در Command Window (مناسب برای رزومه)
fprintf('--- Performance Summary (Z-Axis) ---\n');
fprintf('Open-Loop Peak : %.2f mm\n', peak_open);
fprintf('PID Peak       : %.2f mm (%.1f%% reduction)\n', peak_pid, imp_pid);
fprintf('MPC Peak       : %.2f mm (%.1f%% reduction)\n', peak_mpc, imp_mpc);
fprintf('Max MPC Force  : %.1f N (Limit: 300 N)\n', max(abs(u_mpc(:, 3))));

% 4. رسم دو نمودار حرفه‌ای زیر هم
figure('Position', [100, 100, 850, 600]);

% نمودار بالا: مقایسه جابه‌جایی بر حسب میلی‌متر
subplot(2, 1, 1);
plot(t, z_open_mm, 'r--', 'LineWidth', 1.2); hold on;
plot(t, z_pid_mm,  'b',   'LineWidth', 1.5);
plot(t, z_mpc_mm,  'g',   'LineWidth', 1.8);
grid on;
ylabel('Z Displacement (mm)');
title('3-DOF Microscope Vibration Isolation: Open-Loop vs PID vs MPC');
legend('Without Control', 'PID', 'MPC (300N Limit)', 'Location', 'northeast');

% نمودار پایین: نیروی موتور MPC و نمایش رعایت سقف 300 نیوتن
subplot(2, 1, 2);
plot(t, u_mpc(:, 3), 'k', 'LineWidth', 1.5); hold on;
yline(300,  'r--', 'LineWidth', 1.2);
yline(-300, 'r--', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Actuator Force Z (N)');
title('MPC Control Effort (Actuator Constraint Verification)');
ylim([-350, 350]);
legend('MPC Force (u_z)', 'Actuator Limits (\pm300 N)', 'Location', 'northeast');

% 5. ذخیره خودکار نمودار با کیفیت بالا در پوشه پروژه
exportgraphics(gcf, 'Comparison_Plot.png', 'Resolution', 300);



