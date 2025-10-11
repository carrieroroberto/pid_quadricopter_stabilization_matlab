% Studente: Carriero Roberto
% Matricola: 587640
% Corso di Laurea: Ingegneria Informatica e dell'Automazione (P-TECH)
%
% Tema d'Anno in Programmazione dei Sistemi Avionici - a.a. 2024/2025
% Docente: Prof. Antonio Satriano
%
% Titolo: Simulazione di un Sistema di Controllo PID per la Stabilizzazione
% di un Drone Quadricottero in Ambiente MATLAB®
%
% Funzione main: quadricottero_pid_main.m

clc; % Pulisce la Command Window.
clear; % Pulisce tutte le variabili dal Workspace.

% --- Parametri del Drone ---
m = 1; % Massa del drone (kg).
g = 9.81; % Accelerazione di gravità (m/s^2).
L = 0.5; % Lunghezza del braccio dal centro al motore (m).

% Calcolo dei momenti di inerzia (resistenza alla rotazione), assumendo
% motori puntiformi ai vertici dei bracci.
Ixx = 2 * (m / 4) * L^2; % Inerzia asse X (rollio). Dipende da 2 motori.
Iyy = 2 * (m / 4) * L^2; % Inerzia asse Y (beccheggio). Dipende da 2 motori.
Izz = Ixx + Iyy; % Inerzia asse Z (imbardata). Approssimazione per simmetria (Teorema assi perpendicolari).

% Stampa dei momenti di inerzia
fprintf('Momenti di Inerzia calcolati:\n');
fprintf('  Ixx: %.4f kg*m^2\n', Ixx);
fprintf('  Iyy: %.4f kg*m^2\n', Iyy);
fprintf('  Izz: %.4f kg*m^2\n', Izz);

% Passo temporale della simulazione (in secondi).
t = 0;
dt = 0.1;

% --- Parametri Iniziali PID ---
% sim_data è una struttura dati per gestire i parametri PID, aggiornabili da GUI.
sim_data = struct();

% Guadagni PID per il controllo di posizione.
sim_data.Kp_pos = 10;
sim_data.Ki_pos = 0.1;
sim_data.Kd_pos = 7;

% Guadagni PID per il controllo di assetto.
sim_data.Kp_ass = 8;
sim_data.Ki_ass = 0.08;
sim_data.Kd_ass = 3;

% --- Variabili di Stato ---
x = 0; y = 0; z = 0.01; % Posizione [m].
vx = 0; vy = 0; vz = 0; % Velocità lineari [m/s].
phi = 0; theta = 0; psi = 0; % Angoli di Eulero [rad].
p = 0; q = 0; r = 0; % Velocità angolari [rad/s].

% --- Variabili Target (Setpoint) ---
x_ref = 0; y_ref = 0; z_ref = 5; % Posizione desiderata [m].
psi_ref = 0; % Assetto di imbardata desiderato [rad].

% --- Variabili Vento Casuale ---
wind_active = false; % Flag per vento attivo.
wind_start = 0; % Tempo inizio folata.
wind_duration = 0; % Durata folata.
wind_speed_kmh = 0; % Velocità vento.
wind_direction = 0; % Direzione vento.
wind_torque_coeff_yaw = 0.005; % Coefficiente coppia torcente di imbardata.
wind_torque_coeff_roll = 0.002; % Coefficiente coppia torcente di rollio.
wind_torque_coeff_pitch = 0.002; % Coefficiente coppia torcente di beccheggio.
next_wind_time = rand * (4 - 2) + 2; % Prossima folata di vento casuale (tra 2 e 4 secondi).

% --- Limitazione e filtraggio per stabilità (anti-windup e filtro derivativo) --- %
integral_pos = [0; 0; 0]; % Accumulatore errore integrale posizione.
integral_ass = [0; 0; 0]; % Accumulatore errore integrale assetto.
integral_limit = 5; % Limite per l'integrale PID (anti-windup).
last_integral_reset = 0; % Ultimo tempo di reset.
integral_reset_interval = 5; % Intervallo di reset (secondi).
tau = [0; 0; 0]; % Coppie generate dal controllore di assetto.
sim_data.p_filt = 0; % Velocità angolare filtrata di rollio.
sim_data.q_filt = 0; % Velocità angolare filtrata di beccheggio.
sim_data.r_filt = 0; % Velocità angolare filtrata di imbardata.
sim_data.alpha_d = 0.5; % Coefficiente del filtro.
angle_limit = deg2rad(45); % Limite fisico degli angoli di rollio/beccheggio [rad].
angular_vel_limit = deg2rad(120); % Limite velocità angolari [rad/s].
max_angle_pos_control = deg2rad(20); % Limite angoli di assetto richiesti dal controllore di posizione.
command_limit = 35; % Limite per i comandi di controllo.

% --- Dati per Grafici ---
% Vettori per memorizzare i dati da visualizzare nei grafici per PID ed errori.
t_vec = [];
P_pos_vec = []; I_pos_vec = []; D_pos_vec = [];
P_ass_vec = []; I_ass_vec = []; D_ass_vec = [];
ex_vec = []; ey_vec = []; ez_vec = [];
ephi_vec = []; etheta_vec = []; epsi_vec = [];

% --- Configurazione Figura Principale (Quadricottero) ---
fig_drone = figure('Position', [50, 100, 600, 600], 'Name', 'Simulazione Quadricottero');
h_drone = plot3(0, 0, 0, 'ro', 'MarkerSize', 10, 'LineWidth', 2); % Corpo drone.
hold on;
h_arm1 = plot3([0 0], [0 0], [0 0], 'b-', 'LineWidth', 2); % Braccio 1.
h_arm2 = plot3([0 0], [0 0], [0 0], 'b-', 'LineWidth', 2); % Braccio 2.
h_target = plot3(x_ref, y_ref, z_ref, 'g*', 'MarkerSize', 10); % Posizione target.
h_wind_arrow = quiver3(0, 0, 0, 0, 0, 0, 'k-', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Freccia vento.
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)'); % Etichetta di posizione.
title('Simulazione Quadricottero');
grid on;
axis([-2 2 -2 2 0 10]);         % Limiti degli assi.
view(45, 30);                   % Angolo di visione (45° orizzontale, 30° elevazione).
h_wind_label = text(-1.5, -1.5, 0, 'Vento: 0 km/h, Direzione: 0°', 'FontSize', 10, 'Color', 'k'); % Etichetta vento.

% --- Figura Grafici Termini PID ---
fig_pid = figure('Position', [680, 100, 600, 600], 'Name', 'Grafici Termini PID');
T_window = 10; % Finestra temporale visualizzata nei grafici (10s).

% Subplot per i termini P, I, D di posizione e assetto.
subplot(3, 2, 1); h_P_pos = plot(NaN, NaN, 'r-', 'LineWidth', 2); xlabel('Tempo (s)'); ylabel('P_pos'); title('Proporzionale (pos)'); grid on;
subplot(3, 2, 3); h_I_pos = plot(NaN, NaN, 'g-', 'LineWidth', 2); xlabel('Tempo (s)'); ylabel('I_pos'); title('Integrale (pos)'); grid on;
subplot(3, 2, 5); h_D_pos = plot(NaN, NaN, 'b-', 'LineWidth', 2); xlabel('Tempo (s)'); ylabel('D_pos'); title('Derivativo (pos)'); grid on;
subplot(3, 2, 2); h_P_ass = plot(NaN, NaN, 'r-', 'LineWidth', 2); xlabel('Tempo (s)'); ylabel('P_ass'); title('Proporzionale (ass)'); grid on;
subplot(3, 2, 4); h_I_ass = plot(NaN, NaN, 'g-', 'LineWidth', 2); xlabel('Tempo (s)'); ylabel('I_ass'); title('Integrale (ass)'); grid on;
subplot(3, 2, 6); h_D_ass = plot(NaN, NaN, 'b-', 'LineWidth', 2); xlabel('Tempo (s)'); ylabel('D_ass'); title('Derivativo (ass)'); grid on;

% --- Figura Grafici Errore ---
fig_error = figure('Position', [1310, 100, 600, 600], 'Name', 'Grafici Errore');

% Subplot per gli errori di posizione e assetto.
subplot(3, 2, 1); h_ex = plot(NaN, NaN, 'r-', 'LineWidth', 1.5); xlabel('Tempo (s)'); ylabel('e_x (m)'); title('Errore Posizione X'); grid on;
subplot(3, 2, 3); h_ey = plot(NaN, NaN, 'g-', 'LineWidth', 1.5); xlabel('Tempo (s)'); ylabel('e_y (m)'); title('Errore Posizione Y'); grid on;
subplot(3, 2, 5); h_ez = plot(NaN, NaN, 'b-', 'LineWidth', 1.5); xlabel('Tempo (s)'); ylabel('e_z (m)'); title('Errore Posizione Z'); grid on;
subplot(3, 2, 2); h_ephi = plot(NaN, NaN, 'r-', 'LineWidth', 1.5); xlabel('Tempo (s)'); ylabel('e_\phi (rad)'); title('Errore Rollio (\phi)'); grid on;
subplot(3, 2, 4); h_etheta = plot(NaN, NaN, 'g-', 'LineWidth', 1.5); xlabel('Tempo (s)'); ylabel('e_\theta (rad)'); title('Errore Beccheggio (\theta)'); grid on;
subplot(3, 2, 6); h_epsi = plot(NaN, NaN, 'b-', 'LineWidth', 1.5); xlabel('Tempo (s)'); ylabel('e_\psi (rad)'); title('Errore Imbardata (\psi)'); grid on;

% --- Controlli Interattivi (Slider) ---
% Slider per modificare i guadagni PID in tempo reale.
% I callback chiamano la funzione esterna
% "quadricottero_pid_ui_callbacks.m" per aggiornare i parametri nella
% struttura dati memorizzata nella figura.
sim_data.text_kp_pos = uicontrol('Parent', fig_drone, 'Style', 'text', 'Position', [50, 70, 120, 20], 'String', sprintf('Kp_pos: %.1f', sim_data.Kp_pos));
uicontrol('Parent', fig_drone, 'Style', 'slider', 'Position', [50, 50, 120, 20], 'Min', 0, 'Max', 20, 'Value', sim_data.Kp_pos, 'Callback', @(src, ~)quadricottero_pid_ui_callbacks('updateKpPos', src, fig_drone));
sim_data.text_ki_pos = uicontrol('Parent', fig_drone, 'Style', 'text', 'Position', [180, 70, 120, 20], 'String', sprintf('Ki_pos: %.2f', sim_data.Ki_pos));
uicontrol('Parent', fig_drone, 'Style', 'slider', 'Position', [180, 50, 120, 20], 'Min', 0, 'Max', 1, 'Value', sim_data.Ki_pos, 'Callback', @(src, ~)quadricottero_pid_ui_callbacks('updateKiPos', src, fig_drone));
sim_data.text_kd_pos = uicontrol('Parent', fig_drone, 'Style', 'text', 'Position', [310, 70, 120, 20], 'String', sprintf('Kd_pos: %.1f', sim_data.Kd_pos));
uicontrol('Parent', fig_drone, 'Style', 'slider', 'Position', [310, 50, 120, 20], 'Min', 0, 'Max', 10, 'Value', sim_data.Kd_pos, 'Callback', @(src, ~)quadricottero_pid_ui_callbacks('updateKdPos', src, fig_drone));
sim_data.text_kp_ass = uicontrol('Parent', fig_drone, 'Style', 'text', 'Position', [50, 30, 120, 20], 'String', sprintf('Kp_ass: %.1f', sim_data.Kp_ass));
uicontrol('Parent', fig_drone, 'Style', 'slider', 'Position', [50, 10, 120, 20], 'Min', 0, 'Max', 20, 'Value', sim_data.Kp_ass, 'Callback', @(src, ~)quadricottero_pid_ui_callbacks('updateKpAss', src, fig_drone));
sim_data.text_ki_ass = uicontrol('Parent', fig_drone, 'Style', 'text', 'Position', [180, 30, 120, 20], 'String', sprintf('Ki_ass: %.2f', sim_data.Ki_ass));
uicontrol('Parent', fig_drone, 'Style', 'slider', 'Position', [180, 10, 120, 20], 'Min', 0, 'Max', 1, 'Value', sim_data.Ki_ass, 'Callback', @(src, ~)quadricottero_pid_ui_callbacks('updateKiAss', src, fig_drone));
sim_data.text_kd_ass = uicontrol('Parent', fig_drone, 'Style', 'text', 'Position', [310, 30, 120, 20], 'String', sprintf('Kd_ass: %.1f', sim_data.Kd_ass));
uicontrol('Parent', fig_drone, 'Style', 'slider', 'Position', [310, 10, 120, 20], 'Min', 0, 'Max', 10, 'Value', sim_data.Kd_ass, 'Callback', @(src, ~)quadricottero_pid_ui_callbacks('updateKdAss', src, fig_drone));

% Salva la struct nella proprietà UserData della figura principale per accesso globale.
set(fig_drone, 'UserData', sim_data);

% --- Loop Principale di Simulazione ---
while ishandle(fig_drone) && ishandle(fig_pid) && ishandle(fig_error) % Continua finché le finestre sono aperte.

    % Recupera i parametri PID aggiornati dagli slider.
    current_sim_data = get(fig_drone, 'UserData');
    t = t + dt;     % Avanza il tempo.

    % Reset dell'integrale.
    if t - last_integral_reset >= integral_reset_interval
        integral_pos = [0; 0; 0]; % Reset integrale posizione.
        integral_ass = [0; 0; 0]; % Reset integrale assetto.
        last_integral_reset = t;
    end

    % Generazione delle folate di vento casuali.
    tau_wind_rot = [0; 0; 0]; % Coppie di disturbo dal vento.
    F_wind = [0; 0; 0]; % Forze di disturbo dal vento.
    
    if ~wind_active && t >= next_wind_time % Inizia una nuova folata.
        wind_active = true;
        wind_start = t;
        wind_duration = rand * (5 - 2) + 2; % Durata casuale.
        wind_speed_kmh = rand * (8 - 2) + 2; % Velocità casuale.
        wind_direction = rand * 360; % Direzione casuale.

        % Stampa di riepilogo del vento generato
        fprintf('\nFolata di Vento (t = %.2f s)\n', t);
        fprintf('  Intensità: %.1f km/h\n', wind_speed_kmh);
        fprintf('  Durata: %.1f s\n', wind_duration);
        fprintf('  Direzione: %.0f°\n', wind_direction);

        % Generazione delle forze e coppie agenti sul quadricottero durante
        % la folata di vento
        wind_speed_ms = wind_speed_kmh * 0.2778; % Conversione da km/h a m/s.
        F_wind = [wind_speed_ms * cosd(wind_direction); wind_speed_ms * sind(wind_direction); 0]; % Forza applicata
        tau_wind_rot(1) = wind_speed_ms * wind_torque_coeff_roll * sind(wind_direction); % Coppia di rollio.
        tau_wind_rot(2) = wind_speed_ms * wind_torque_coeff_pitch * cosd(wind_direction); % Coppia di beccheggio
        tau_wind_rot(3) = wind_speed_ms * wind_torque_coeff_yaw * (rand - 0.5) * 2; % Coppia di imbardata
        set(h_wind_label, 'String', sprintf('Vento: %.1f km/h, Direzione: %.0f°', wind_speed_kmh, wind_direction));
       
    elseif wind_active && (t - wind_start) >= wind_duration % Folata finita
        wind_active = false;
        set(h_wind_label, 'String', 'Vento: 0 km/h, Direzione: 0°');
        next_wind_time = t + rand * (4 - 2) + 2; % Prossima folata dopo un delay casuale.

    elseif wind_active % Folata ancora attiva (le forze e le coppie devono continuare a essere applicate).
        wind_speed_ms = wind_speed_kmh * 0.2778;
        F_wind = [wind_speed_ms * cosd(wind_direction); wind_speed_ms * sind(wind_direction); 0];
        tau_wind_rot(1) = wind_speed_ms * wind_torque_coeff_roll * sind(wind_direction);
        tau_wind_rot(2) = wind_speed_ms * wind_torque_coeff_pitch * cosd(wind_direction);
        tau_wind_rot(3) = wind_speed_ms * wind_torque_coeff_yaw * (rand - 0.5) * 2;
    end

    % --- Controllo PID di Posizione (x, y, z) ---
    e_pos = [x_ref - x; y_ref - y; z_ref - z]; % Calcolo errore di posizione.
    integral_pos = integral_pos + e_pos * dt; % Accumulo integrale.
    integral_pos = max(min(integral_pos, integral_limit), -integral_limit); % Anti-windup (integral_pos: [-integral_limit, +integral_limit]).
    d_pos = [vx; vy; vz]; % Derivativo (velocità attuale).

    P_pos = current_sim_data.Kp_pos * e_pos; % Componente Proporzionale.
    I_pos = current_sim_data.Ki_pos * integral_pos; % Componente Integrale.
    D_pos = current_sim_data.Kd_pos * (-d_pos); % Componente Derivativa (negativa perchè il riferimento è 0).

    u_pos = P_pos + I_pos + D_pos; % Output del controllore di posizione.
    u_pos = max(min(u_pos, command_limit), -command_limit); % Limite sull'output (u_pos: [+command_limit, -command_limit]).

    % Calcolo degli angoli di rollio e beccheggio desiderati.
    % atan2 permette al drone di inclinarsi per generare forza.
    phi_d = atan2(-u_pos(2), g * m); % Rollio desiderato per controllare Y.
    theta_d = atan2(u_pos(1), g * m); % Beccheggio desiderato per controllare X.

    % Limita gli angoli desiderati per sicurezza tra [-max_angle_pos_control, +max_angle_pos_control].
    phi_d = max(min(phi_d, max_angle_pos_control), -max_angle_pos_control);
    theta_d = max(min(theta_d, max_angle_pos_control), -max_angle_pos_control);
    psi_d = psi_ref; % Imbardata desiderata.

    % --- Dinamica Rotazionale ---
    coppie_totali = tau + tau_wind_rot; % Coppie da controllore + vento.

    % Equazioni di Eulero per la dinamica rotazionale
    p_dot = (coppie_totali(1) + (Iyy - Izz) * q * r) / Ixx; % Accelerazione angolare rollio.
    q_dot = (coppie_totali(2) + (Izz - Ixx) * p * r) / Iyy; % Accelerazione angolare beccheggio.
    r_dot = (coppie_totali(3) + (Ixx - Iyy) * p * q) / Izz; % Accelerazione angolare imbardata.

    % Limita le accelerazioni angolari tra [-angular_vel_limit, +angular_vel_limit].
    p_dot = max(min(p_dot, angular_vel_limit/dt), -angular_vel_limit/dt);
    q_dot = max(min(q_dot, angular_vel_limit/dt), -angular_vel_limit/dt);
    r_dot = max(min(r_dot, angular_vel_limit/dt), -angular_vel_limit/dt);

    % Integrazione delle accelerazioni per ottenere le velocità angolari.
    p = p + p_dot * dt;
    q = q + q_dot * dt;
    r = r + r_dot * dt;

    % Limita le velocità angolari sempre tra [-angular_vel_limit, +angular_vel_limit].
    p = max(min(p, angular_vel_limit), -angular_vel_limit);
    q = max(min(q, angular_vel_limit), -angular_vel_limit);
    r = max(min(r, angular_vel_limit), -angular_vel_limit);

    % --- Filtro Passa-Basso per il Termine Derivativo ---
    % Applica un filtro esponenziale alle velocità angolari per ridurre il rumore
    current_sim_data.p_filt = current_sim_data.alpha_d * current_sim_data.p_filt + (1 - current_sim_data.alpha_d) * p;
    current_sim_data.q_filt = current_sim_data.alpha_d * current_sim_data.q_filt + (1 - current_sim_data.alpha_d) * q;
    current_sim_data.r_filt = current_sim_data.alpha_d * current_sim_data.r_filt + (1 - current_sim_data.alpha_d) * r;
    
    % Aggiorna i dati filtrati nella figura.
    set(fig_drone, 'UserData', current_sim_data);

    % --- Controllo PID di Assetto ---
    e_ass = [phi_d - phi; theta_d - theta; psi_d - psi]; % Calcolo errore di assetto.
    e_ass(3) = atan2(sin(e_ass(3)), cos(e_ass(3))); % Normalizza errore di imbardata tra [-pi, pi].

    integral_ass = integral_ass + e_ass * dt; % Accumulo integrale.
    integral_ass = max(min(integral_ass, integral_limit), -integral_limit); % Anti-windup (integral_pos: [-integral_limit, +integral_limit]).
    d_ass = [current_sim_data.p_filt; current_sim_data.q_filt; current_sim_data.r_filt];  % Derivativo (velocità angolari filtrate).

    P_ass = current_sim_data.Kp_ass * e_ass; % Componente Proporzionale.
    I_ass = current_sim_data.Ki_ass * integral_ass; % Componente Integrale.
    D_ass = current_sim_data.Kd_ass * (-d_ass); % Componente Derivativa.

    tau = P_ass + I_ass + D_ass; % Output del controllore di assetto.
    tau = max(min(tau, command_limit), -command_limit); % Limite sull'output (tau: [+command_limit, -command_limit]).

    % --- Calcolo Spinta Totale ---
    T = u_pos(3) + m * g; % Spinta totale: componente Z dal PID posizione + gravità.
    T = max(T, 0); % La spinta non può essere negativa.

    % --- Dinamica di Traslazione 6 gradi di libertà (6-DoF) ---
    % Modella il movimento del drone nello spazio 3D.
    % Matrice di rotazione R 3x3: converte vettori dal sistema drone a quello globale.
    R = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi); ...
         cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi); ...
         -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];

    % Calcolo accelerazione lineare su Z nel sistema globale.
    acc = (R * [0; 0; T] + F_wind) / m - [0; 0; g];
 
    % Integrazione delle accelerazioni per ottenere velocità lineari.
    vx = vx + acc(1) * dt;
    vy = vy + acc(2) * dt;
    vz = vz + acc(3) * dt;

    % Integrazione delle velocità per ottenere le posizioni.
    x = x + vx * dt;
    y = y + vy * dt;
    z = z + vz * dt;
    z = max(z, 0); % Il drone non può andare sotto terra.

    % --- Aggiornamento Assetto ---
    % Matrice di trasformazione da velocità angolari a tassi di Eulero.
    omega_to_eulerrate = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
                          0, cos(phi),           -sin(phi);
                          0, sin(phi)/cos(theta),  cos(phi)/cos(theta)];
    euler_rates = omega_to_eulerrate * [p; q; r];

    % Integrazione dei tassi di Eulero per aggiornare gli angoli.
    phi = phi + euler_rates(1) * dt;
    theta = theta + euler_rates(2) * dt;
    psi = psi + euler_rates(3) * dt;

    % Limita gli angoli per mantenere stabilità
    phi = max(min(phi, angle_limit), -angle_limit);
    theta = max(min(theta, angle_limit), -angle_limit);
    psi = atan2(sin(psi), cos(psi)); % Normalizza l'angolo di imbardata a [-pi, pi].

    % --- Aggiorna i Dati per i Grafici ---
    % Aggiunge i dati attuali ai vettori per i grafici.
    t_vec = [t_vec, t];
    P_pos_vec = [P_pos_vec, P_pos(3)]; I_pos_vec = [I_pos_vec, I_pos(3)]; D_pos_vec = [D_pos_vec, D_pos(3)];
    P_ass_vec = [P_ass_vec, P_ass(1)]; I_ass_vec = [I_ass_vec, I_ass(1)]; D_ass_vec = [D_ass_vec, D_ass(1)];
    ex_vec = [ex_vec, e_pos(1)]; ey_vec = [ey_vec, e_pos(2)]; ez_vec = [ez_vec, e_pos(3)];
    ephi_vec = [ephi_vec, e_ass(1)]; etheta_vec = [etheta_vec, e_ass(2)]; epsi_vec = [epsi_vec, e_ass(3)];

    % Aggiorna i dati nella finestra temporale T_window (finestra temporale scorrevole).
    idx = t_vec >= (t - T_window);
    t_vec = t_vec(idx);
    P_pos_vec = P_pos_vec(idx); I_pos_vec = I_pos_vec(idx); D_pos_vec = D_pos_vec(idx);
    P_ass_vec = P_ass_vec(idx); I_ass_vec = I_ass_vec(idx); D_ass_vec = D_ass_vec(idx);
    ex_vec = ex_vec(idx); ey_vec = ey_vec(idx); ez_vec = ez_vec(idx);
    ephi_vec = ephi_vec(idx); etheta_vec = etheta_vec(idx); epsi_vec = epsi_vec(idx);

    % --- Aggiorna Grafici Termini PID ---
    % Aggiorna le linee dei grafici con i nuovi dati.
    set(h_P_pos, 'XData', t_vec, 'YData', P_pos_vec);
    set(h_I_pos, 'XData', t_vec, 'YData', I_pos_vec);
    set(h_D_pos, 'XData', t_vec, 'YData', D_pos_vec);
    set(h_P_ass, 'XData', t_vec, 'YData', P_ass_vec);
    set(h_I_ass, 'XData', t_vec, 'YData', I_ass_vec);
    set(h_D_ass, 'XData', t_vec, 'YData', D_ass_vec);

    % Aggiorna i limiti dell'asse X (tempo) per seguire la finestra.
    set(get(h_P_pos, 'Parent'), 'XLim', [max(0, t - T_window), max(t, T_window)]);
    set(get(h_I_pos, 'Parent'), 'XLim', [max(0, t - T_window), max(t, T_window)]);
    set(get(h_D_pos, 'Parent'), 'XLim', [max(0, t - T_window), max(t, T_window)]);
    set(get(h_P_ass, 'Parent'), 'XLim', [max(0, t - T_window), max(t, T_window)]);
    set(get(h_I_ass, 'Parent'), 'XLim', [max(0, t - T_window), max(t, T_window)]);
    set(get(h_D_ass, 'Parent'), 'XLim', [max(0, t - T_window), max(t, T_window)]);

    % Aggiorna i limiti dell'asse Y per i grafici PID nella visualizzazione
    graph_limit_pid = 50;
    if ~isempty(P_pos_vec)
        min_P_pos = max(min(P_pos_vec), -graph_limit_pid); max_P_pos = min(max(P_pos_vec), graph_limit_pid);
        min_I_pos = max(min(I_pos_vec), -graph_limit_pid); max_I_pos = min(max(I_pos_vec), graph_limit_pid);
        min_D_pos = max(min(D_pos_vec), -graph_limit_pid); max_D_pos = min(max(D_pos_vec), graph_limit_pid);
        min_P_ass = max(min(P_ass_vec), -graph_limit_pid); max_P_ass = min(max(P_ass_vec), graph_limit_pid);
        min_I_ass = max(min(I_ass_vec), -graph_limit_pid); max_I_ass = min(max(I_ass_vec), graph_limit_pid);
        min_D_ass = max(min(D_ass_vec), -graph_limit_pid); max_D_ass = min(max(D_ass_vec), graph_limit_pid);
        
        % Regola minimi/massimi se sono troppo vicini.
        if min_P_pos >= max_P_pos, min_P_pos = max_P_pos - 0.1; end
        if min_I_pos >= max_I_pos, min_I_pos = max_I_pos - 0.1; end
        if min_D_pos >= max_D_pos, min_D_pos = max_D_pos - 0.1; end
        if min_P_ass >= max_P_ass, min_P_ass = max_P_ass - 0.1; end
        if min_I_ass >= max_I_ass, min_I_ass = max_I_ass - 0.1; end
        if min_D_ass >= max_D_ass, min_D_ass = max_D_ass - 0.1; end
        
        % Applica i nuovi limiti Y.
        set(get(h_P_pos, 'Parent'), 'YLim', [min_P_pos - 1, max_P_pos + 1]);
        set(get(h_I_pos, 'Parent'), 'YLim', [min_I_pos - 1, max_I_pos + 1]);
        set(get(h_D_pos, 'Parent'), 'YLim', [min_D_pos - 1, max_D_pos + 1]);
        set(get(h_P_ass, 'Parent'), 'YLim', [min_P_ass - 0.1, max_P_ass + 0.1]);
        set(get(h_I_ass, 'Parent'), 'YLim', [min_I_ass - 0.1, max_I_ass + 0.1]);
        set(get(h_D_ass, 'Parent'), 'YLim', [min_D_ass - 0.1, max_D_ass + 0.1]);
    end

    % --- Aggiorna Grafici Errore (fig_error) ---
    % Aggiorna le linee dei grafici degli errori.
    set(h_ex, 'XData', t_vec, 'YData', ex_vec);
    set(h_ey, 'XData', t_vec, 'YData', ey_vec);
    set(h_ez, 'XData', t_vec, 'YData', ez_vec);
    set(h_ephi, 'XData', t_vec, 'YData', ephi_vec);
    set(h_etheta, 'XData', t_vec, 'YData', etheta_vec);
    set(h_epsi, 'XData', t_vec, 'YData', epsi_vec);

    % Aggiorna i limiti dell'asse X (tempo) per gli errori.
    set(get(h_ex, 'Parent'), 'XLim', [max(0, t - T_window), max(t, T_window)]);
    set(get(h_ey, 'Parent'), 'XLim', [max(0, t - T_window), max(t, T_window)]);
    set(get(h_ez, 'Parent'), 'XLim', [max(0, t - T_window), max(t, T_window)]);
    set(get(h_ephi, 'Parent'), 'XLim', [max(0, t - T_window), max(t, T_window)]);
    set(get(h_etheta, 'Parent'), 'XLim', [max(0, t - T_window), max(t, T_window)]);
    set(get(h_epsi, 'Parent'), 'XLim', [max(0, t - T_window), max(t, T_window)]);

    % Limita gli assi Y per i grafici degli errori.
    graph_limit_error_pos = 1; % Limite per errori di posizione.
    graph_limit_error_ass = deg2rad(15); % Limite per errori di assetto.
    if ~isempty(ex_vec)
        min_ex = max(min(ex_vec), -graph_limit_error_pos); max_ex = min(max(ex_vec), graph_limit_error_pos);
        min_ey = max(min(ey_vec), -graph_limit_error_pos); max_ey = min(max(ey_vec), graph_limit_error_pos);
        min_ez = max(min(ez_vec), -graph_limit_error_pos); max_ez = min(max(ez_vec), graph_limit_error_pos);
        
        % Regola minimi/massimi se sono troppo vicini.
        if min_ex >= max_ex, min_ex = max_ex - 0.05; end
        if min_ey >= max_ey, min_ey = max_ey - 0.05; end
        if min_ez >= max_ez, min_ez = max_ez - 0.05; end
        set(get(h_ex, 'Parent'), 'YLim', [min_ex - 0.05, max_ex + 0.05]);
        set(get(h_ey, 'Parent'), 'YLim', [min_ey - 0.05, max_ey + 0.05]);
        set(get(h_ez, 'Parent'), 'YLim', [min_ez - 0.05, max_ez + 0.05]);
    end
    
    if ~isempty(ephi_vec)
        min_ephi = max(min(ephi_vec), -graph_limit_error_ass); max_ephi = min(max(ephi_vec), graph_limit_error_ass);
        min_etheta = max(min(etheta_vec), -graph_limit_error_ass); max_etheta = min(max(etheta_vec), graph_limit_error_ass);
        min_epsi = max(min(epsi_vec), -graph_limit_error_ass); max_epsi = min(max(epsi_vec), graph_limit_error_ass);
        
        % Regola minimi/massimi se sono troppo vicini.
        if min_ephi >= max_ephi, min_ephi = max_ephi - 0.01; end
        if min_etheta >= max_etheta, min_etheta = max_etheta - 0.01; end
        if min_epsi >= max_epsi, min_epsi = max_epsi - 0.01; end
        set(get(h_ephi, 'Parent'), 'YLim', [min_ephi - 0.01, max_ephi + 0.01]);
        set(get(h_etheta, 'Parent'), 'YLim', [min_etheta - 0.01, max_etheta + 0.01]);
        set(get(h_epsi, 'Parent'), 'YLim', [min_epsi - 0.01, max_epsi + 0.01]);
    end

    % --- Animazione Drone ---
    % Aggiorna la posizione e l'orientamento grafico del drone.
    % R_drone è la matrice di rotazione attuale del drone nello spazio.
    R_drone = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi); ...
               cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi); ...
               -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];
    
    % Definisce le posizioni locali dei bracci.
    arm1_local = [L; 0; 0];
    arm2_local = [0; L; 0];
    
    % Ruota i bracci dal sistema corpo a quello globale.
    arm1_rotated = R_drone * arm1_local;
    arm2_rotated = R_drone * arm2_local;
    
    % Aggiorna le coordinate del drone e dei bracci.
    set(h_drone, 'XData', x, 'YData', y, 'ZData', z);
    set(h_arm1, 'XData', [x-arm1_rotated(1) x+arm1_rotated(1)], 'YData', [y-arm1_rotated(2) y+arm1_rotated(2)], 'ZData', [z-arm1_rotated(3) z+arm1_rotated(3)]);
    set(h_arm2, 'XData', [x-arm2_rotated(1) x+arm2_rotated(1)], 'YData', [y-arm2_rotated(2) y+arm2_rotated(2)], 'ZData', [z-arm2_rotated(3) z+arm2_rotated(3)]);
    
    % Mostra/nasconde la freccia del vento in base allo stato.
    if wind_active
        arrow_scale = wind_speed_kmh / 8; % Scala la lunghezza della freccia in base alla velocità.
        u_wind_arrow = arrow_scale * cosd(wind_direction);
        v_wind_arrow = arrow_scale * sind(wind_direction);
        w_wind_arrow = 0;
        set(h_wind_arrow, 'XData', x, 'YData', y, 'ZData', z, ...
                          'UData', u_wind_arrow, 'VData', v_wind_arrow, 'WData', w_wind_arrow, 'Visible', 'on');
    else
        set(h_wind_arrow, 'Visible', 'off');
    end

    % Forza l'aggiornamento della grafica.
    drawnow;
    
end
