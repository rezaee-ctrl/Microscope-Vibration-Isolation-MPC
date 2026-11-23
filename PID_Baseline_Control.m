clear; clc; close all;

% 1. بارگذاری مدلی که در فایل اول ساختیم
load('TableModel_3DOF.mat');

% 2. تعریف زمان شبیه‌سازی (10 ثانیه، با گام 0.01 ثانیه)
t = 0:Ts:10;
N = length(t);

% 3. ساخت یک ضربه/لرزش (مثلاً 100 نیوتن در ثانیه 1 به مدت 0.2 ثانیه)
d = zeros(N, 3);
d(t >= 1 & t <= 1.2, :) = 100;

% 4. شبیه‌سازی میز بدون کنترل‌کننده (Open-Loop)
y_open = lsim(sys_c, d, t);

% رسم جابه‌جایی محور عمودی (Z)
plot(t, y_open(:, 3), 'r', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Z Displacement (m)');
title('Microscope Vibration WITHOUT Controller');

% 5. طراحی کنترل‌کننده PID برای هر محور
Kp = 15000;  % واکنش به خطای فعلی
Ki = 5000;   % از بین بردن خطای ماندگار
Kd = 3000;   % ترمز کردن برای جلوگیری از نوسان

c_1axis = pid(Kp, Ki, Kd);
C_pid = blkdiag(c_1axis, c_1axis, c_1axis); % تکرار برای 3 محور x, y, z

% 6. بستن حلقه فیدبک و شبیه‌سازی با همان ضربه قبلی
sys_pid = feedback(sys_c, C_pid);
y_pid = lsim(sys_pid, d, t);

% 7. رسم نمودار PID روی نمودار قبلی برای مقایسه
hold on;
plot(t, y_pid(:, 3), 'b', 'LineWidth', 1.5);
legend('Without Control', 'With PID');

% ذخیره نتایج بدون کنترل و PID برای مقایسه نهایی
save('PID_Results.mat', 't', 'd', 'y_open', 'y_pid');