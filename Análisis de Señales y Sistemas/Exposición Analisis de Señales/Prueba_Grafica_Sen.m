% 1. Parámetros de la señal
A = 170;                              % Amplitud
f = 60;                             % Frecuencia (10 Hz)
w = 2 * pi * f;                     % Frecuencia angular (rad/s)
T = 1 / f;                          % Periodo (0.1 s)

% 2. Base de tiempo continuo (muestreo denso y suave)
t_cont = 0 : 0.0001 : 2*T;          
y_cont = A * sin(w*t_cont);

% 3. Base de tiempo discreto (muestreo a intervalos Ts fijos)
Fs = 600;                           % Frecuencia de muestreo (600 Hz = 60 muestras/ciclo)
Ts = 1 / Fs;                        % Periodo de muestreo
t_disc = 0 : Ts : 2*T;              % Instantes de muestreo n*Ts
y_disc = A * cos(w*t_disc);

% 4. Gráfica 1: Tiempo continuo
subplot(2, 1, 1);
plot(t_cont, y_cont, 'b', 'LineWidth', 1.5);
grid on;
title('Sen(t)');
xlabel('Tiempo [s]'); ylabel('Amplitud');

% 5. Gráfica 2: Tiempo discreto
subplot(2, 1, 2);
stem(t_disc, y_disc, 'r', 'filled', 'LineWidth', 1.2);
grid on;
title('Sen[n]');
xlabel('Tiempo [s]'); ylabel('Amplitud');

