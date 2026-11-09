%Definimos nuestros parametros iniciales

A = 170;
f = 60;
w = 2*pi*f;
%Creamos nuestro vector tiempo

t= 0:0.0001 : 2*(1/f); %Definimos cuantos periodos queremos mostrar (2)

%Definimos las ecuaciones para las funciones

y_sen = A *sin(w*t);
y_cos = A *cos(w*t + deg2rad(30));

%Mostramos la gráficas:
plot(t,y_sen, 'b', t, y_cos, 'r--', 'LineWidth', 1.5);
grid on;
legend('Seno', 'Cos(-90)');
xlabel('Tiempo(s)');
ylabel('Amplitud');


