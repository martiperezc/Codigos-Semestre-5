import serial
import struct
import matplotlib.pyplot as plt
from collections import deque

print("Inicio")

# Timeout bajo para que el teclado responda rápido
ser = serial.Serial("COM7", 115200)

N = 100
# El deque mantiene un tamaño fijo, descartando automáticamente el dato más viejo
ydata = deque([0]*N, maxlen=N) 
xdata = list(range(N))

plt.ion()
fig, ax = plt.subplots()
line, = ax.plot(xdata, list(ydata))

# Límites y etiquetas de la gráfica
ax.set_ylim(0, 3.3)
ax.set_xlim(0, N-1)
ax.set_ylabel('Voltage')
ax.set_xlabel('Muestras')
ax.set_title('Lectura UART en Tiempo Real')

try:
    while True:
        data = ser.read(4)

        if len(data) == 4:
            valor = struct.unpack("<f", data)[0]
            print(f"Voltaje: {valor:.2f}")

            ydata.append(valor)
            line.set_ydata(list(ydata))
            
            # Actualización directa del lienzo en cada iteración
            fig.canvas.draw()
            fig.canvas.flush_events()

except KeyboardInterrupt:
    print("\nDetenido con éxito.")
    ser.close()       
    plt.close('all')