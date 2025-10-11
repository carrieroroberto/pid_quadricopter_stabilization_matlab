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
% Funzione esterna: quadricottero_pid_ui_callbacks.m

function quadricottero_pid_ui_callbacks(type, hObject, fig_main_drone)
% Questa funzione gestisce gli eventi generati dagli elementi della GUI (slider).
%
% Parametri:
%   type:           Indica quale tipo di evento è avvenuto (es. 'updateKpPos' per lo slider Kp di posizione).
%   hObject:        Handle dell'oggetto GUI che ha generato l'evento (es. lo slider).
%   fig_main_drone: Handle della figura principale del drone, dove sono memorizzati i dati.

% Recupera la struttura dati sim_data dalla proprietà UserData della figura principale.
% Questa struct contiene tutti i parametri dinamici e PID del drone.
sim_data = get(fig_main_drone, 'UserData');
handles = sim_data; % copia il contenuto di sim_data in handles

% Utilizza il costrutto switch per gestire i diversi tipi di callback.
switch type
    % Aggiorna il guadagno Kp per la posizione.
    case 'updateKpPos'
        sim_data.Kp_pos = get(hObject, 'Value'); % Prende il valore dallo slider.
        set(handles.text_kp_pos, 'String', sprintf('Kp_pos: %.2f', sim_data.Kp_pos)); % Aggiorna il testo sulla GUI.
    
    % Aggiorna il guadagno Ki per la posizione.
    case 'updateKiPos'
        sim_data.Ki_pos = get(hObject, 'Value');
        set(handles.text_ki_pos, 'String', sprintf('Ki_pos: %.2f', sim_data.Ki_pos));
    
    % Aggiorna il guadagno Kd per la posizione.
    case 'updateKdPos'
        sim_data.Kd_pos = get(hObject, 'Value');
        set(handles.text_kd_pos, 'String', sprintf('Kd_pos: %.2f', sim_data.Kd_pos));

    % Aggiorna il guadagno Kp per l'assetto.
    case 'updateKpAss'
        sim_data.Kp_ass = get(hObject, 'Value');
        set(handles.text_kp_ass, 'String', sprintf('Kp_ass: %.2f', sim_data.Kp_ass));

    % Aggiorna il guadagno Ki per l'assetto.
    case 'updateKiAss'
        sim_data.Ki_ass = get(hObject, 'Value');
        set(handles.text_ki_ass, 'String', sprintf('Ki_ass: %.2f', sim_data.Ki_ass));

    % Aggiorna il guadagno Kd per l'assetto.
    case 'updateKdAss'
        sim_data.Kd_ass = get(hObject, 'Value');
        set(handles.text_kd_ass, 'String', sprintf('Kd_ass: %.2f', sim_data.Kd_ass));
   
    % Inverte lo stato di attivazione del vento.
    case 'toggle_wind'
        sim_data.wind_active = ~sim_data.wind_active;
        if sim_data.wind_active
            % Resetta i parametri del vento se attivato (saranno ricalcolati nel main).
            sim_data.wind_speed_kmh = 0;
            sim_data.wind_direction = 0;
            disp('Vento casuale attivato. La forza e direzione cambieranno periodicamente.');
        else
            % Disattiva il vento.
            sim_data.wind_speed_kmh = 0;
            sim_data.wind_direction = 0;
            disp('Vento casuale disattivato.');
        end

    % Aggiorna la coordinata X di riferimento.
    case 'x_ref'
        sim_data.x_ref = get(hObject, 'Value');
        disp(['Target X: ', num2str(sim_data.x_ref, '%.2f')]);
    
    % Aggiorna la coordinata Y di riferimento.
    case 'y_ref'
        sim_data.y_ref = get(hObject, 'Value');
        disp(['Target Y: ', num2str(sim_data.y_ref, '%.2f')]);

    % Aggiorna la coordinata Z di riferimento.
    case 'z_ref'
        sim_data.z_ref = get(hObject, 'Value');
        disp(['Target Z: ', num2str(sim_data.z_ref, '%.2f')]);
    
    % Gestisce tutti i casi non previsti (default)
    otherwise
        disp('Funzione callback non riconosciuta');
end

% Salva la struttura sim_data aggiornata nella proprietà UserData della
% figura principale.
set(fig_main_drone, 'UserData', sim_data);

% Aggiorna le altre figure per mantenere la coerenza dei dati.
fig_pid_handle = findobj('Name', 'Grafici Termini PID');
set(fig_pid_handle, 'UserData', sim_data);

fig_error_handle = findobj('Name', 'Grafici Errore');
set(fig_error_handle, 'UserData', sim_data);

end