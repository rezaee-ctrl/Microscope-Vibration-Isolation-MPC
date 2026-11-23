clear; clc; close all;

% 1. بارگذاری مدل میز و نتایج PID (برای داشتن همان ضربه d)
load('TableModel_3DOF.mat');
load('PID_Results.mat');

% 2. استخراج ماتریس‌های گسسته (برای پیش‌بینی آینده)
Ad = sys_d.A;   % ابعاد 6x6
Bd = sys_d.B;   % ابعاد 6x3
Cd = sys_d.C;   % ابعاد 3x6

% 3. تنظیمات MPC
p = 20;         % افق پیش‌بینی: 20 قدم جلوتر (0.2 ثانیه)
u_max = 300;    % سقف نیروی هر موتور (300 نیوتن)
u_min = -300;   % کف نیروی هر موتور (-300 نیوتن)

% 4. تعریف وزن‌ها (اولویت‌های MPC)
% وزن خیلی بالا روی صفر نگه داشتن موقعیت (x, y, z) و وزن کم روی نیروی موتور
Q = diag([1e7, 100, 1e7, 100, 1e7, 100]);
R = 0.001 * eye(3);

% 5. ساخت ماتریس‌های پیش‌بینی برای 20 قدم آینده (X = F*x + G*U)
nx = 6; % تعداد حالت‌ها (موقعیت و سرعت 3 محور)
nu = 3; % تعداد موتورها
F = zeros(p*nx, nx);
G = zeros(p*nx, p*nu);

for i = 1:p
    F((i-1)*nx+1 : i*nx, :) = Ad^i;
    for j = 1:i
        G((i-1)*nx+1 : i*nx, (j-1)*nu+1 : j*nu) = Ad^(i-j) * Bd;
    end
end

% 6. آماده کردن ماتریس هزینه‌ی کل (H) برای تابع بهینه‌سازی quadprog
Q_bar = kron(eye(p), Q);
R_bar = kron(eye(p), R);
H = 2 * (G' * Q_bar * G + R_bar);
H = (H + H') / 2; % تضمین تقارن عددی برای جلوگیری از اخطار quadprog

% 7. آماده کردن محدودیت‌های موتور و متغیرهای ذخیره نتیجه
N = length(t);
x = zeros(nx, 1);       % حالت اولیه میز (ساکن در صفر)
y_mpc = zeros(N, 3);    % برای ذخیره جابه‌جایی میز با MPC
u_mpc = zeros(N, 3);    % برای ذخیره نیروی موتورها

lb = repmat(u_min, p*nu, 1); % کف نیرو (-300) برای کل 20 قدم
ub = repmat(u_max, p*nu, 1); % سقف نیرو (+300) برای کل 20 قدم
options = optimoptions('quadprog', 'Display', 'off'); % خاموش کردن پیام‌های تکراری

% 8. حلقه‌ی شبیه‌سازی MPC (قدم به قدم در طول 10 ثانیه)
for k = 1:N
    % ثبت موقعیت فعلی میز
    y_mpc(k, :) = (Cd * x)';
    
    % محاسبه‌ی بهترین نیروی موتور برای 20 قدم آینده با رعایت سقف 300 نیوتن
    f = 2 * G' * Q_bar * F * x;
    U_opt = quadprog(H, f, [], [], [], [], lb, ub, [], options);
    
    % اصل افق لغزان: فقط فرمان قدم اول را به موتور می‌دهیم
    u_k = U_opt(1:nu);
    u_mpc(k, :) = u_k';
    
    % رفتن به قدم بعدی (اثر همزمان نیروی موتور و ضربه‌ی لرزش d)
    x = Ad * x + Bd * (u_k + d(k, :)');
end

% 9. ذخیره نتایج MPC و رسم نمودار مقایسه‌ی سه روش
save('MPC_Results.mat', 'y_mpc', 'u_mpc');

figure;
plot(t, y_open(:, 3), 'r--', 'LineWidth', 1.2); hold on;
plot(t, y_pid(:, 3), 'b', 'LineWidth', 1.5);
plot(t, y_mpc(:, 3), 'g', 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Z Displacement (m)');
title('3-DOF Microscope Vibration Isolation: Open-Loop vs PID vs MPC');
legend('Without Control', 'PID', 'MPC (300N limit)');

