clc;
clear;
close all;

%% =========================================================
%  IMPLEMENTASI PID PADA PENGATURAN KECEPATAN MOTOR DC ROVER
%  TERHADAP GANGGUAN BEBAN
%  - Transfer Function & Closed-Loop
%  - Simulasi State-Space dengan Gangguan Beban
%  - Perbandingan Respon Dengan vs Tanpa Gangguan
%  - Analisis Frekuensi & Kestabilan
%  - Pembuatan & Simulasi Model Simulink Otomatis dengan TF PID
%% =========================================================

%% =========================================================
%  BAGIAN 1: PARAMETER MOTOR DC ROVER
%% =========================================================
J = 0.01;    % Momen inersia rotor (kg.m^2)
b = 0.1;     % Koefisien gesekan viskos (N.m.s)
K = 0.01;    % Konstanta motor back-EMF / torsi (V.s/rad)
R = 1;       % Resistansi jangkar (Ohm)
L = 0.5;     % Induktansi jangkar (H)

fprintf('=========================================\n');
fprintf(' PARAMETER MOTOR DC ROVER\n');
fprintf('=========================================\n');
fprintf('Momen Inersia (J)   = %.4f kg.m^2\n', J);
fprintf('Koef. Gesekan (b)   = %.4f N.m.s\n',  b);
fprintf('Konstanta Motor (K) = %.4f V.s/rad\n', K);
fprintf('Resistansi (R)      = %.4f Ohm\n',     R);
fprintf('Induktansi (L)      = %.4f H\n',       L);

%% =========================================================
%  BAGIAN 2: TRANSFER FUNCTION MOTOR DC ROVER
%  G(s) = K / [(Js+b)(Ls+R) + K^2]
%% =========================================================
s = tf('s');
G = K / ((J*s + b)*(L*s + R) + K^2);

fprintf('\n=========================================\n');
fprintf(' TRANSFER FUNCTION MOTOR DC ROVER\n');
fprintf('=========================================\n');
fprintf('G(s) = K / [(Js+b)(Ls+R) + K^2]\n');
disp(G);

%% =========================================================
%  BAGIAN 3: PARAMETER PID & KONTROLER
%% =========================================================
Kp = 100;
Ki = 200;
Kd = 10;
N = 100;  % Filter coefficient untuk derivative term

fprintf('\n=========================================\n');
fprintf(' NILAI PARAMETER PID\n');
fprintf('=========================================\n');
fprintf('Kp = %.2f\n', Kp);
fprintf('Ki = %.2f\n', Ki);
fprintf('Kd = %.2f\n', Kd);
fprintf('N  = %.2f (Filter Derivative)\n', N);

% Kontroler PID dengan filter: C(s) = Kp + Ki/s + Kd*N/(s+N)
C = pid(Kp, Ki, Kd, 1/N);
fprintf('\nKontroler PID C(s) dengan filter:\n');
disp(C);

% Transfer Function PID dalam bentuk proper (untuk Simulink)
% C(s) = (Kd*N*s^2 + (Kp*N + Kd*N^2)*s + Ki*N) / (s^2 + N*s)
num_pid = [Kd*N, (Kp*N + Kd*N^2), Ki*N];
den_pid = [1, N, 0];

fprintf('\nTransfer Function PID untuk Simulink:\n');
fprintf('C(s) = (%.2f*s^2 + %.2f*s + %.2f) / (s^2 + %.2f*s)\n', ...
    num_pid(1), num_pid(2), num_pid(3), den_pid(2));

%% =========================================================
%  BAGIAN 4: SISTEM CLOSED-LOOP & INFORMASI STEP RESPONSE
%% =========================================================
T = feedback(C*G, 1);

fprintf('\n=========================================\n');
fprintf(' SISTEM CLOSED-LOOP T(s)\n');
fprintf('=========================================\n');
disp(T);

info = stepinfo(T);
fprintf('\n=========================================\n');
fprintf(' STEP RESPONSE INFO (Tanpa Gangguan)\n');
fprintf('=========================================\n');
fprintf('Rise Time     : %.4f s\n',  info.RiseTime);
fprintf('Settling Time : %.4f s\n',  info.SettlingTime);
fprintf('Overshoot     : %.2f %%\n', info.Overshoot);
fprintf('Peak          : %.4f\n',    info.Peak);
fprintf('Steady-State  : %.4f\n',    dcgain(T));

%% =========================================================
%  BAGIAN 5: SETUP WAKTU & SINYAL
%% =========================================================
t_end          = 10;
dt             = 0.001;
t              = 0:dt:t_end;
ref_speed      = ones(size(t));
Va_max         = 24;

% Parameter gangguan beban
t_dist_start   = 3;
t_dist_end     = 7;
dist_amplitude = 0.3;

% Sinyal gangguan beban
disturbance = zeros(size(t));
disturbance(t >= t_dist_start & t < t_dist_end) = dist_amplitude;

%% =========================================================
%  BAGIAN 6: SIMULASI DENGAN GANGGUAN (STATE-SPACE + PID)
%% =========================================================
omega    = zeros(size(t));
i_a      = zeros(size(t));
Va       = zeros(size(t));
err_sig  = zeros(size(t));
int_err  = 0;
prev_err = 0;
prev_derr = 0;

for k = 1:length(t)-1
    % Hitung error
    err_sig(k) = ref_speed(k) - omega(k);
    
    % Integral error
    int_err = int_err + err_sig(k) * dt;
    
    % Derivative error dengan filter
    d_err = (err_sig(k) - prev_err) / dt;
    d_err_filtered = prev_derr + (d_err - prev_derr) * N * dt / (1 + N * dt);
    prev_derr = d_err_filtered;
    prev_err = err_sig(k);
    
    % Sinyal kontrol PID
    Va(k) = Kp*err_sig(k) + Ki*int_err + Kd*d_err_filtered;
    
    % Saturasi tegangan
    Va(k) = max(-Va_max, min(Va_max, Va(k)));
    
    % Gangguan torsi beban
    Td = disturbance(k);
    
    % Persamaan state-space motor DC dengan gangguan
    d_omega = (-b/J)*omega(k) + (K/J)*i_a(k) - (1/J)*Td;
    d_ia    = (-K/L)*omega(k) + (-R/L)*i_a(k) + (1/L)*Va(k);
    
    % Update state
    omega(k+1) = omega(k) + d_omega * dt;
    i_a(k+1)   = i_a(k)   + d_ia * dt;
end

%% =========================================================
%  BAGIAN 7: SIMULASI TANPA GANGGUAN (REFERENSI)
%% =========================================================
omega_nd  = zeros(size(t));
i_a_nd    = zeros(size(t));
int_err2  = 0;
prev_err2 = 0;
prev_derr2 = 0;

for k = 1:length(t)-1
    e2 = ref_speed(k) - omega_nd(k);
    int_err2 = int_err2 + e2 * dt;
    d_err2 = (e2 - prev_err2) / dt;
    d_err2_filtered = prev_derr2 + (d_err2 - prev_derr2) * N * dt / (1 + N * dt);
    prev_derr2 = d_err2_filtered;
    prev_err2 = e2;
    
    Va2 = Kp*e2 + Ki*int_err2 + Kd*d_err2_filtered;
    Va2 = max(-Va_max, min(Va_max, Va2));
    
    d_omega2 = (-b/J)*omega_nd(k) + (K/J)*i_a_nd(k);
    d_ia2    = (-K/L)*omega_nd(k) + (-R/L)*i_a_nd(k) + (1/L)*Va2;
    
    omega_nd(k+1) = omega_nd(k) + d_omega2 * dt;
    i_a_nd(k+1)   = i_a_nd(k) + d_ia2 * dt;
end

% Data step response dari TF
[y_step, t_step] = step(T);

%% =========================================================
%  BAGIAN 8: ANALISIS PERFORMA GANGGUAN
%% =========================================================
idx_d  = find(t >= t_dist_start, 1);
idx_de = find(t >= t_dist_end, 1);
[min_spd, ~] = min(omega(idx_d:idx_de));
drop_pct = (1 - min_spd) * 100;
idx_after = find(t >= t_dist_end + 1, 1);
if isempty(idx_after)
    idx_after = length(omega) - 300;
end
ss_after = mean(omega(idx_after : min(idx_after+300, length(omega))));

fprintf('\n=========================================\n');
fprintf(' ANALISIS PERFORMA TERHADAP GANGGUAN\n');
fprintf('=========================================\n');
fprintf('Amplitudo Gangguan            : %.2f N.m\n', dist_amplitude);
fprintf('Waktu Gangguan                : %.1f - %.1f s\n', t_dist_start, t_dist_end);
fprintf('Penurunan Kecepatan Maks      : %.4f rad/s (%.2f %%)\n', 1-min_spd, drop_pct);
fprintf('Kecepatan SS Setelah Gangguan : %.4f rad/s\n', ss_after);
fprintf('Steady-State Error            : %.6f rad/s\n', abs(1 - ss_after));

%% =========================================================
%  BAGIAN 9 - FIGURE 1: DASHBOARD UTAMA (6 SUBPLOT)
%% =========================================================
figure('Name','Fig1 - Dashboard PID Motor DC Rover', ...
    'NumberTitle','off','Position',[50 30 1300 820]);

% --- Subplot 1: Respon Kecepatan dengan Gangguan ---
subplot(3,2,1);
plot(t, ref_speed, 'r--', 'LineWidth', 1.8); hold on;
plot(t, omega, 'b-', 'LineWidth', 1.5);
patch([t_dist_start t_dist_end t_dist_end t_dist_start], ...
      [-0.2 -0.2 1.5 1.5], [1 0.8 0.8], 'FaceAlpha', 0.25, 'EdgeColor', 'none');
xline(t_dist_start, 'k:', 'LineWidth', 1.2);
xline(t_dist_end, 'k:', 'LineWidth', 1.2);
legend('Setpoint', 'Kecepatan Motor', 'Zona Gangguan', 'Location', 'southeast');
xlabel('Waktu (s)'); ylabel('Kecepatan (rad/s)');
title('Respon Kecepatan - Dengan Gangguan Beban');
grid on; ylim([-0.2 1.5]);

% --- Subplot 2: Step Response dari TF ---
subplot(3,2,2);
plot(t_step, y_step, 'b-', 'LineWidth', 1.5); hold on;
yline(1, 'r--', 'LineWidth', 1.2);
yline(1.02, 'g:', 'LineWidth', 1.0);
yline(0.98, 'g:', 'LineWidth', 1.0);
legend('Step Response', 'Setpoint', 'Batas ±2%', 'Location', 'southeast');
xlabel('Waktu (s)'); ylabel('Amplitudo');
title('Step Response Closed-Loop (TF, tanpa gangguan)');
grid on;

% --- Subplot 3: Error Kecepatan ---
subplot(3,2,3);
plot(t, err_sig, 'r-', 'LineWidth', 1.2);
patch([t_dist_start t_dist_end t_dist_end t_dist_start], ...
      [-1 -1 1 1], [1 0.8 0.8], 'FaceAlpha', 0.25, 'EdgeColor', 'none');
xline(t_dist_start, 'k:', 'LineWidth', 1.2);
xline(t_dist_end, 'k:', 'LineWidth', 1.2);
yline(0, 'k-', 'LineWidth', 0.8);
xlabel('Waktu (s)'); ylabel('Error (rad/s)');
title('Error Kecepatan (e = Setpoint - Output)');
grid on;

% --- Subplot 4: Sinyal Kontrol PID ---
subplot(3,2,4);
plot(t, Va, 'm-', 'LineWidth', 1.2);
patch([t_dist_start t_dist_end t_dist_end t_dist_start], ...
      [-25 -25 25 25], [1 0.8 0.8], 'FaceAlpha', 0.2, 'EdgeColor', 'none');
xline(t_dist_start, 'k:', 'LineWidth', 1.2);
xline(t_dist_end, 'k:', 'LineWidth', 1.2);
yline(Va_max, 'k--', 'LineWidth', 0.8);
yline(-Va_max, 'k--', 'LineWidth', 0.8);
xlabel('Waktu (s)'); ylabel('Tegangan Va (V)');
title('Sinyal Kontrol PID (Output Kontroler)');
grid on;

% --- Subplot 5: Profil Gangguan Beban ---
subplot(3,2,5);
stairs(t, disturbance, 'k-', 'LineWidth', 2.0);
xlabel('Waktu (s)'); ylabel('Torsi Gangguan (N.m)');
title('Profil Gangguan Beban (Load Disturbance)');
ylim([-0.05 dist_amplitude*1.5]); grid on;

% --- Subplot 6: Arus Jangkar ---
subplot(3,2,6);
plot(t, i_a, 'g-', 'LineWidth', 1.2);
patch([t_dist_start t_dist_end t_dist_end t_dist_start], ...
      [min(i_a)-0.1 min(i_a)-0.1 max(i_a)+0.1 max(i_a)+0.1], ...
      [1 0.8 0.8], 'FaceAlpha', 0.25, 'EdgeColor', 'none');
xline(t_dist_start, 'k:', 'LineWidth', 1.2);
xline(t_dist_end, 'k:', 'LineWidth', 1.2);
xlabel('Waktu (s)'); ylabel('Arus (A)');
title('Arus Jangkar Motor DC');
grid on;

sgtitle('IMPLEMENTASI PID - PENGATURAN KECEPATAN MOTOR DC ROVER', ...
    'FontSize', 14, 'FontWeight', 'bold');

%% =========================================================
%  BAGIAN 10 - FIGURE 2: PERBANDINGAN RESPON
%% =========================================================
figure('Name','Fig2 - Perbandingan Respon Dengan vs Tanpa Gangguan', ...
    'NumberTitle','off','Position',[100 100 980 520]);

plot(t, ref_speed, 'r--', 'LineWidth', 2.2); hold on;
plot(t, omega_nd, 'b-', 'LineWidth', 1.8);
plot(t, omega, 'g-', 'LineWidth', 1.8);
patch([t_dist_start t_dist_end t_dist_end t_dist_start], ...
      [-0.1 -0.1 1.5 1.5], [1 0.8 0.8], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
xline(t_dist_start, 'k:', 'LineWidth', 1.5, 'Label', 'Gangguan ON');
xline(t_dist_end, 'k:', 'LineWidth', 1.5, 'Label', 'Gangguan OFF');
legend('Setpoint (1 rad/s)', 'Tanpa Gangguan Beban', ...
       'Dengan Gangguan Beban', 'Zona Gangguan Aktif', 'Location', 'best');
xlabel('Waktu (s)'); ylabel('Kecepatan (rad/s)');
title({'Perbandingan Respon Kecepatan Motor DC Rover', ...
       sprintf('Gangguan Beban: %.2f N.m pada t = %.0f-%.0f s', ...
       dist_amplitude, t_dist_start, t_dist_end)});
text(t_dist_start+0.15, 0.08, sprintf('Td = %.2f N.m', dist_amplitude), ...
     'FontSize', 10, 'Color', 'r', 'FontWeight', 'bold');
grid on; ylim([-0.1 1.5]);

%% =========================================================
%  BAGIAN 11 - FIGURE 3: ANALISIS FREKUENSI & KESTABILAN
%% =========================================================
figure('Name','Fig3 - Analisis Frekuensi dan Kestabilan', ...
    'NumberTitle','off','Position',[150 60 1100 700]);

L_open = C*G;

% Diagram Bode
subplot(2,2,1);
margin(L_open);
title('Diagram Bode - Loop Terbuka (C*G)');
grid on;

% Diagram Nyquist
subplot(2,2,2);
nyquist(L_open);
title('Diagram Nyquist - Loop Terbuka');
grid on;

% Peta Pole-Zero
subplot(2,2,3);
pzmap(T);
title('Peta Pole-Zero - Sistem Closed-Loop');
grid on;

% Root Locus
subplot(2,2,4);
rlocus(L_open);
title('Root Locus - Loop Terbuka');
grid on;

sgtitle('ANALISIS FREKUENSI & KESTABILAN SISTEM PID', ...
    'FontSize', 13, 'FontWeight', 'bold');

% Margin Kestabilan
[Gm, Pm, Wcg, Wcp] = margin(L_open);
fprintf('\n=========================================\n');
fprintf(' MARGIN KESTABILAN SISTEM\n');
fprintf('=========================================\n');
fprintf('Gain Margin (Gm)     : %.4f (%.2f dB)\n', Gm, 20*log10(Gm));
fprintf('Phase Margin (Pm)    : %.2f deg\n', Pm);
fprintf('Gain Crossover (Wcp) : %.4f rad/s\n', Wcp);
fprintf('Phase Crossover (Wcg): %.4f rad/s\n', Wcg);
if Pm > 45 && Gm > 2
    fprintf('Status: SISTEM STABIL (Pm > 45 deg, Gm > 6 dB)\n');
else
    fprintf('Status: PERIKSA KESTABILAN\n');
end

%% =========================================================
%  BAGIAN 12: MEMBUAT MODEL SIMULINK OTOMATIS
%% =========================================================
fprintf('\n=========================================\n');
fprintf(' MEMBUAT MODEL SIMULINK\n');
fprintf('=========================================\n');

model = 'PID_DC_Motor_Rover';

% Tutup model jika sudah terbuka
if bdIsLoaded(model)
    close_system(model, 0);
end

% Buat model baru
new_system(model);
open_system(model);

%% --- POSISI BLOK (dirapikan) ---
pos_step      = [50, 120, 80, 150];
pos_sum_err   = [140, 118, 170, 152];
pos_pid_tf    = [220, 105, 340, 165];
pos_saturasi  = [380, 110, 430, 160];
pos_sum_dist  = [480, 118, 510, 152];
pos_motor     = [560, 108, 700, 162];
pos_dist_on   = [380, 240, 420, 270];
pos_dist_off  = [380, 300, 420, 330];
pos_sum_dsig  = [450, 258, 475, 282];
pos_scope1    = [760, 90, 810, 175];
pos_towork    = [760, 195, 840, 225];
pos_scope_va  = [760, 35, 810, 70];

%% --- TAMBAH BLOK ---

% 1. Step Input (Setpoint)
add_block('simulink/Sources/Step', [model '/Step_Input']);
set_param([model '/Step_Input'], ...
    'Time', '0', 'Before', '0', 'After', '1', ...
    'Position', pos_step);

% 2. Sum Error (Setpoint - Feedback)
add_block('simulink/Math Operations/Sum', [model '/Sum_Error']);
set_param([model '/Sum_Error'], ...
    'Inputs', '+-', ...
    'Position', pos_sum_err);

% 3. PID Controller (Transfer Function dengan filter)
add_block('simulink/Continuous/Transfer Fcn', [model '/PID_TF']);
set_param([model '/PID_TF'], ...
    'Numerator', mat2str(num_pid), ...
    'Denominator', mat2str(den_pid), ...
    'Position', pos_pid_tf);

% 4. Saturasi Tegangan
add_block('simulink/Discontinuities/Saturation', [model '/Saturasi']);
set_param([model '/Saturasi'], ...
    'UpperLimit', num2str(Va_max), ...
    'LowerLimit', num2str(-Va_max), ...
    'Position', pos_saturasi);

% 5. Sum Gangguan Beban
add_block('simulink/Math Operations/Sum', [model '/Sum_Dist']);
set_param([model '/Sum_Dist'], ...
    'Inputs', '++', ...
    'Position', pos_sum_dist);

% 6. Transfer Function Motor DC
num_motor = [K];
den_motor = [J*L, (J*R + b*L), (b*R + K^2)];
add_block('simulink/Continuous/Transfer Fcn', [model '/Motor_DC']);
set_param([model '/Motor_DC'], ...
    'Numerator', mat2str(num_motor), ...
    'Denominator', mat2str(den_motor), ...
    'Position', pos_motor);

% 7. Gangguan Beban ON
add_block('simulink/Sources/Step', [model '/Dist_ON']);
set_param([model '/Dist_ON'], ...
    'Time', num2str(t_dist_start), ...
    'Before', '0', 'After', num2str(dist_amplitude), ...
    'Position', pos_dist_on);

% 8. Gangguan Beban OFF
add_block('simulink/Sources/Step', [model '/Dist_OFF']);
set_param([model '/Dist_OFF'], ...
    'Time', num2str(t_dist_end), ...
    'Before', '0', 'After', num2str(-dist_amplitude), ...
    'Position', pos_dist_off);

% 9. Sum Sinyal Gangguan
add_block('simulink/Math Operations/Sum', [model '/Sum_DistSig']);
set_param([model '/Sum_DistSig'], ...
    'Inputs', '++', ...
    'Position', pos_sum_dsig);

% 10. Scope Utama (3 input)
add_block('simulink/Sinks/Scope', [model '/Scope_Utama']);
set_param([model '/Scope_Utama'], ...
    'NumInputPorts', '3', ...
    'Position', pos_scope1);

% 11. To Workspace
add_block('simulink/Sinks/To Workspace', [model '/ToWorkspace']);
set_param([model '/ToWorkspace'], ...
    'VariableName', 'sim_omega', ...
    'SaveFormat', 'Array', ...
    'Position', pos_towork);

% 12. Scope Sinyal Kontrol Va
add_block('simulink/Sinks/Scope', [model '/Scope_Va']);
set_param([model '/Scope_Va'], ...
    'Position', pos_scope_va);

%% --- HUBUNGKAN BLOK ---
add_line(model, 'Step_Input/1', 'Sum_Error/1', 'autorouting', 'on');
add_line(model, 'Sum_Error/1', 'PID_TF/1', 'autorouting', 'on');
add_line(model, 'PID_TF/1', 'Saturasi/1', 'autorouting', 'on');
add_line(model, 'Saturasi/1', 'Sum_Dist/1', 'autorouting', 'on');
add_line(model, 'Saturasi/1', 'Scope_Va/1', 'autorouting', 'on');
add_line(model, 'Sum_Dist/1', 'Motor_DC/1', 'autorouting', 'on');
add_line(model, 'Motor_DC/1', 'Scope_Utama/1', 'autorouting', 'on');
add_line(model, 'Motor_DC/1', 'ToWorkspace/1', 'autorouting', 'on');
add_line(model, 'Motor_DC/1', 'Sum_Error/2', 'autorouting', 'on');
add_line(model, 'Dist_ON/1', 'Sum_DistSig/1', 'autorouting', 'on');
add_line(model, 'Dist_OFF/1', 'Sum_DistSig/2', 'autorouting', 'on');
add_line(model, 'Sum_DistSig/1', 'Sum_Dist/2', 'autorouting', 'on');
add_line(model, 'Step_Input/1', 'Scope_Utama/2', 'autorouting', 'on');
add_line(model, 'Sum_DistSig/1', 'Scope_Utama/3', 'autorouting', 'on');

%% --- KONFIGURASI SOLVER ---
set_param(model, ...
    'Solver', 'ode45', ...
    'StopTime', num2str(t_end), ...
    'MaxStep', '0.01', ...
    'RelTol', '1e-5');

% Simpan model
save_system(model);
fprintf('Model Simulink "%s.slx" berhasil dibuat dan disimpan!\n', model);
fprintf('Kontroler PID menggunakan Transfer Function dengan filter:\n');
fprintf('  C(s) = (%.2f*s^2 + %.2f*s + %.2f) / (s^2 + %.2f*s)\n', ...
    num_pid(1), num_pid(2), num_pid(3), den_pid(2));

%% =========================================================
%  BAGIAN 13: JALANKAN SIMULINK & FIGURE 4
%% =========================================================
fprintf('\nMenjalankan simulasi Simulink...\n');
try
    simOut = sim(model, 'StopTime', num2str(t_end));
    t_sim = simOut.tout;
    omega_sim = sim_omega;
    fprintf('Simulasi Simulink selesai!\n');
    
    % Figure 4: Perbandingan
    figure('Name','Fig4 - Hasil Simulink vs State-Space Manual', ...
        'NumberTitle','off','Position',[200 80 1050 500]);
    
    subplot(1,2,1);
    plot(t, ref_speed, 'r--', 'LineWidth', 2.0); hold on;
    plot(t, omega, 'b-', 'LineWidth', 1.6);
    patch([t_dist_start t_dist_end t_dist_end t_dist_start], ...
          [-0.1 -0.1 1.5 1.5], [1 0.8 0.8], 'FaceAlpha', 0.25, 'EdgeColor', 'none');
    legend('Setpoint', 'State-Space Manual', 'Zona Gangguan', 'Location', 'best');
    xlabel('Waktu (s)'); ylabel('Kecepatan (rad/s)');
    title('Simulasi State-Space Manual (PID)');
    grid on; ylim([-0.1 1.5]);
    
    subplot(1,2,2);
    plot(t_sim, ones(size(t_sim)), 'r--', 'LineWidth', 2.0); hold on;
    plot(t_sim, omega_sim, 'g-', 'LineWidth', 1.6);
    patch([t_dist_start t_dist_end t_dist_end t_dist_start], ...
          [-0.1 -0.1 1.5 1.5], [1 0.8 0.8], 'FaceAlpha', 0.25, 'EdgeColor', 'none');
    legend('Setpoint', 'Output Simulink', 'Zona Gangguan', 'Location', 'best');
    xlabel('Waktu (s)'); ylabel('Kecepatan (rad/s)');
    title('Simulasi Simulink (PID TF + Gangguan Beban)');
    grid on; ylim([-0.1 1.5]);
    
    sgtitle('PERBANDINGAN: STATE-SPACE MANUAL vs SIMULINK', ...
        'FontSize', 13, 'FontWeight', 'bold');
    
catch ME
    fprintf('[PERINGATAN] Simulasi Simulink gagal: %s\n', ME.message);
    fprintf('Buka model "%s" dan jalankan manual di Simulink.\n', model);
end

%% =========================================================
fprintf('\n=========================================\n');
fprintf(' SELESAI - Semua tahap berhasil!\n');
fprintf('=========================================\n');
fprintf('Figure yang dihasilkan:\n');
fprintf('  Fig 1 : Dashboard Utama (6 subplot)\n');
fprintf('  Fig 2 : Perbandingan Dengan vs Tanpa Gangguan\n');
fprintf('  Fig 3 : Analisis Frekuensi & Kestabilan\n');
fprintf('  Fig 4 : Hasil Simulink vs State-Space Manual\n');
fprintf('\nModel Simulink : %s.slx\n', model);
fprintf('Kontroler PID  : Transfer Function dengan Derivative Filter\n');
fprintf('                 C(s) = Kp + Ki/s + Kd*N/(s+N)\n');
fprintf('                 N = %.0f (Filter Coefficient)\n', N);
fprintf('=========================================\n');
