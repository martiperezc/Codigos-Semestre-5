%Definir parametros de nuestra funcion

A = 170;
f = 2;
w = 2*pi*f;

%Definir a Vector tiempo para muestreo de señal:

t = 0:0.01: 2*(1/f);

%Definir funcion a graficar
y_sen = A * sin(w*t);
y_cos = A * cos(w*t);

%Mostrar Gráfica del sen

plot(t, y_sen, 'b', t, y_cos, 'r', 'LineWidth', 1.5);
grid on;

legend('Sen', 'Cos');
xlabel('tiempo(s)');
ylabel('Amplitud');

