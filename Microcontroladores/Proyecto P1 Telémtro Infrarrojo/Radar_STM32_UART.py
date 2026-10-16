"""Radar de 180 grados en tiempo real para STM32F103.

Protocolo UART: una trama por línea, terminada en \n o \r\n.
    ANGULO,DISTANCIA
Ejemplos: 84,7.2   /   85,100

Dependencias: Python 3 + pyserial (pip install pyserial).
Ejecutar: python Radar_STM32_UART.py
Prueba sin placa: python Radar_STM32_UART.py --demo
"""

import argparse
from collections import deque
import math
import queue
import statistics
import threading
import time
import tkinter as tk
from typing import Optional, Tuple

# ---------------- CONFIGURACIÓN ----------------
PUERTO_SERIAL = "COM7"
BAUD_RATE = 115200
DISTANCIA_MIN_CM = 4.0
DISTANCIA_MAX_CM = 30.0
DISTANCIA_SIN_OBJETO = 100.0

# El STM32 envía una muestra cada 20 ms (50 Hz).
# La UART se lee continuamente en otro hilo: NO se duerme 40 ms entre lecturas.
REFRESCO_GUI_MS = 40             # 25 imágenes/s; procesa todas las tramas recibidas.
RETENCION_DISTANCIA_S = 0.35     # Muestra el último eco válido durante 350 ms.
PERSISTENCIA_PUNTOS_S = 9.0     # Deja ver una ida/vuelta del servo (ajustable).
ESPERA_RECONEXION_S = 1.5

FONDO = "#0b1210"
PANEL = "#101e18"
VERDE = "#62f65c"
VERDE_SUAVE = "#2b9243"
VERDE_OSCURO = "#1b4930"
ROJO = "#ff4050"
BLANCO = "#e6f5e9"
GRIS = "#7ca58b"
AMARILLO = "#ffc86a"


def analizar_trama(linea: str) -> Optional[Tuple[int, float]]:
    """Devuelve (ángulo, distancia) solo si la trama recibida es válida."""
    datos = linea.strip().split(",")
    if len(datos) != 2:
        return None
    try:
        angulo_actual = int(datos[0].strip())
        distancia_actual = float(datos[1].strip())
    except ValueError:
        return None

    if not (0 <= angulo_actual <= 180 and math.isfinite(distancia_actual)):
        return None
    if not (
        DISTANCIA_MIN_CM <= distancia_actual <= DISTANCIA_MAX_CM
        or distancia_actual == DISTANCIA_SIN_OBJETO
    ):
        return None
    return angulo_actual, distancia_actual


def mezclar_color(color_a: str, color_b: str, proporcion: float) -> str:
    """Mezcla colores RGB para desvanecer los puntos sin librerías externas."""
    t = max(0.0, min(1.0, proporcion))
    a = [int(color_a[i:i + 2], 16) for i in (1, 3, 5)]
    b = [int(color_b[i:i + 2], 16) for i in (1, 3, 5)]
    return "#{:02x}{:02x}{:02x}".format(
        *(round(x + (y - x) * t) for x, y in zip(a, b))
    )


class LectorSerial(threading.Thread):
    """Lee TODAS las tramas de UART fuera del hilo de Tkinter."""

    def __init__(self, puerto: str, baud: int, mensajes: queue.Queue,
                 detener: threading.Event):
        super().__init__(daemon=True)
        self.puerto = puerto
        self.baud = baud
        self.mensajes = mensajes
        self.detener = detener

    def publicar(self, mensaje):
        try:
            self.mensajes.put_nowait(mensaje)
        except queue.Full:
            # Protección si el sistema gráfico se detiene durante mucho tiempo.
            try:
                self.mensajes.get_nowait()
            except queue.Empty:
                pass
            try:
                self.mensajes.put_nowait(mensaje)
            except queue.Full:
                pass

    def run(self):
        try:
            import serial  # pip install pyserial
        except ImportError:
            self.publicar(("estado", "Falta pyserial: pip install pyserial", False))
            return

        while not self.detener.is_set():
            try:
                # timeout corto para poder detener/reconectar sin bloquear la GUI.
                with serial.Serial(self.puerto, self.baud, timeout=0.15) as conexion:
                    self.publicar(("estado", f"Conectado a {self.puerto}", True))
                    while not self.detener.is_set():
                        crudo = conexion.readline()
                        if not crudo:
                            continue
                        trama = analizar_trama(crudo.decode("utf-8", errors="ignore"))
                        if trama is not None:
                            self.publicar(("dato", trama[0], trama[1], time.monotonic()))
            except (serial.SerialException, OSError) as error:
                self.publicar(("estado", f"Puerto no disponible: {error}", False))
                self.detener.wait(ESPERA_RECONEXION_S)


class LectorSimulado(threading.Thread):
    """Genera ejemplos de un objeto ancho y otro estrecho a 50 Hz."""

    def __init__(self, mensajes: queue.Queue, detener: threading.Event):
        super().__init__(daemon=True)
        self.mensajes = mensajes
        self.detener = detener

    def run(self):
        self.mensajes.put(("estado", "Modo simulación · 50 Hz", True))
        angulo = 0
        sentido = 1
        while not self.detener.is_set():
            # Dos cuerpos que ocupan varios ángulos consecutivos.
            if 55 <= angulo <= 79:
                distancia = 12.2 + 0.55 * math.sin(angulo / 4)
            elif 112 <= angulo <= 126:
                distancia = 22.4 + 0.38 * math.sin(angulo / 3)
            else:
                distancia = DISTANCIA_SIN_OBJETO
            try:
                self.mensajes.put_nowait(
                    ("dato", angulo, distancia, time.monotonic())
                )
            except queue.Full:
                pass
            angulo += sentido
            if angulo >= 180:
                angulo = 180
                sentido = -1
            elif angulo <= 0:
                angulo = 0
                sentido = 1
            self.detener.wait(0.02)


class InterfazRadar:
    def __init__(self, ventana: tk.Tk, puerto: str, baud: int, demo: bool):
        self.ventana = ventana
        self.puerto = puerto
        self.baud = baud
        self.demo = demo
        self.mensajes = queue.Queue(maxsize=3000)
        self.detener = threading.Event()
        self.lector = None

        # Un único punto por ángulo; cuando el sensor barre un objeto amplio
        # aparecen varios puntos rojos en ángulos consecutivos.
        # eco[angulo] = (distancia_en_cm, instante_monotonic)
        self.ecos = {}
        self.historial_distancias = deque(maxlen=12)
        self.angulo_actual = None
        self.distancia_uart = None
        self.sentido = 1
        self.ultimo_paquete = 0.0
        self.ultimo_eco = 0.0
        self.conectado = False
        self.estado_conexion = "Iniciando conexión..."

        ventana.title("RADAR UART | STM32F103 · 180°")
        ventana.geometry("1100x720")
        ventana.minsize(790, 535)
        ventana.configure(bg=FONDO)
        ventana.protocol("WM_DELETE_WINDOW", self.cerrar)

        cabecera = tk.Frame(ventana, bg=FONDO, padx=22, pady=11)
        cabecera.pack(side="top", fill="x")
        tk.Label(
            cabecera, text="RADAR 180°", font=("Segoe UI", 22, "bold"),
            fg=VERDE, bg=FONDO
        ).pack(side="left")
        tk.Label(
            cabecera, text="STM32F103  /  SENSOR DE DISTANCIA",
            font=("Consolas", 10), fg=GRIS, bg=FONDO, padx=18
        ).pack(side="left", pady=9)

        tk.Button(
            cabecera, text="LIMPIAR PUNTOS", command=self.limpiar,
            font=("Segoe UI", 9, "bold"), fg=VERDE, bg=PANEL,
            activebackground=VERDE_OSCURO, activeforeground=BLANCO,
            relief="flat", padx=13, pady=9, cursor="hand2"
        ).pack(side="right")

        self.lienzo = tk.Canvas(
            ventana, bg=FONDO, highlightthickness=0, bd=0
        )
        self.lienzo.pack(side="top", fill="both", expand=True, padx=9)
        self.lienzo.bind("<Configure>", lambda _evento: self.dibujar())

        barra = tk.Frame(ventana, bg=PANEL, padx=13, pady=15)
        barra.pack(side="bottom", fill="x")
        self.campos = {}
        definiciones = [
            ("ESTADO", "SIN ECO", 16),
            ("ÁNGULO", "--°", 22),
            ("DISTANCIA", "-- cm", 22),
            ("PUNTOS", "0", 19),
            ("MÍNIMA", "-- cm", 16),
            ("PROMEDIO", "-- cm", 16),
            ("MÁXIMA", "-- cm", 16),
        ]
        for columna, (titulo, valor, tamanio) in enumerate(definiciones):
            barra.columnconfigure(columna, weight=1, uniform="datos")
            contenedor = tk.Frame(barra, bg=PANEL)
            contenedor.grid(row=0, column=columna, sticky="nsew")
            tk.Label(
                contenedor, text=titulo, font=("Segoe UI", 9, "bold"),
                fg=GRIS, bg=PANEL
            ).pack()
            etiqueta = tk.Label(
                contenedor, text=valor,
                font=("Segoe UI", tamanio, "bold"),
                fg=VERDE if columna < 3 else BLANCO,
                bg=PANEL
            )
            etiqueta.pack(pady=(5, 0))
            self.campos[titulo] = etiqueta

        pie = tk.Frame(ventana, bg=FONDO, padx=14, pady=5)
        pie.pack(side="bottom", fill="x")
        self.etiqueta_conexion = tk.Label(
            pie, text="●  Iniciando...", font=("Consolas", 10),
            fg=AMARILLO, bg=FONDO, anchor="w"
        )
        self.etiqueta_conexion.pack(side="left", fill="x", expand=True)
        tk.Label(
            pie, text="UART: 20 ms  ·  GUI: 40 ms  ·  Retención: 350 ms  ·  Barrido: 0°–180°",
            font=("Consolas", 9), fg=GRIS, bg=FONDO
        ).pack(side="right")

        self.lector = (LectorSimulado(self.mensajes, self.detener)
                       if demo else LectorSerial(puerto, baud, self.mensajes, self.detener))
        self.lector.start()
        self.actualizar()

    def limpiar(self):
        self.ecos.clear()
        self.historial_distancias.clear()
        self.dibujar()

    def actualizar(self):
        """Único acceso a widgets: hilo principal de Tkinter."""
        ahora = time.monotonic()
        procesados = 0
        while procesados < 1500:
            try:
                mensaje = self.mensajes.get_nowait()
            except queue.Empty:
                break
            procesados += 1
            if mensaje[0] == "estado":
                _, self.estado_conexion, self.conectado = mensaje
            elif mensaje[0] == "dato":
                _, angulo, distancia, instante = mensaje
                if self.angulo_actual is not None and angulo != self.angulo_actual:
                    self.sentido = 1 if angulo > self.angulo_actual else -1
                self.angulo_actual = angulo
                self.distancia_uart = distancia
                self.ultimo_paquete = instante
                if DISTANCIA_MIN_CM <= distancia <= DISTANCIA_MAX_CM:
                    # Ligero filtrado por ángulo para reducir el ruido de distancia.
                    anterior = self.ecos.get(angulo)
                    if anterior is not None and instante - anterior[1] < 1.0:
                        distancia_punto = 0.65 * distancia + 0.35 * anterior[0]
                    else:
                        distancia_punto = distancia
                    self.ecos[angulo] = (distancia_punto, instante)
                    self.historial_distancias.append((instante, distancia))
                    self.ultimo_eco = instante
                elif distancia == DISTANCIA_SIN_OBJETO:
                    # 100 cm significa AUSENCIA: elimina el punto de ese ángulo.
                    self.ecos.pop(angulo, None)

        # Los puntos de otros ángulos permanecen hasta ser revisados o expirar.
        for angulo, (_, instante) in list(self.ecos.items()):
            if ahora - instante > PERSISTENCIA_PUNTOS_S:
                del self.ecos[angulo]

        self.etiqueta_conexion.configure(
            text=("●  " if self.conectado else "○  ") + self.estado_conexion,
            fg=VERDE if self.conectado else AMARILLO
        )
        self.actualizar_estadisticas(ahora)
        self.dibujar(ahora)
        if not self.detener.is_set():
            self.ventana.after(REFRESCO_GUI_MS, self.actualizar)

    def actualizar_estadisticas(self, ahora: float):
        muestras_recientes = [
            distancia for instante, distancia in self.historial_distancias
            if ahora - instante <= RETENCION_DISTANCIA_S
        ]
        hay_eco_actual = (
            self.distancia_uart is not None
            and DISTANCIA_MIN_CM <= self.distancia_uart <= DISTANCIA_MAX_CM
            and ahora - self.ultimo_paquete < 1.0
        )
        enlace_activo = self.conectado and ahora - self.ultimo_paquete < 1.0
        if not self.conectado:
            estado, color = "SIN UART", AMARILLO
        elif not enlace_activo:
            estado, color = "SIN DATOS", AMARILLO
        elif hay_eco_actual:
            estado, color = "EN RANGO", VERDE
        else:
            estado, color = "SIN ECO", GRIS

        self.campos["ESTADO"].configure(text=estado, fg=color)
        self.campos["ÁNGULO"].configure(
            text=f"{self.angulo_actual}°" if self.angulo_actual is not None else "--°"
        )
        # Mediana de las últimas 3 lecturas válidas recientes; no detiene UART.
        if muestras_recientes and enlace_activo:
            distancia_estable = statistics.median(muestras_recientes[-3:])
            self.campos["DISTANCIA"].configure(
                text=f"{distancia_estable:.1f} cm", fg=VERDE
            )
        else:
            self.campos["DISTANCIA"].configure(text="-- cm", fg=GRIS)

        distancias_en_mapa = [d for d, _ in self.ecos.values()]
        self.campos["PUNTOS"].configure(text=str(len(distancias_en_mapa)))
        for titulo, operacion in (
            ("MÍNIMA", min),
            ("PROMEDIO", statistics.mean),
            ("MÁXIMA", max),
        ):
            texto = (f"{operacion(distancias_en_mapa):.1f} cm"
                     if distancias_en_mapa else "-- cm")
            self.campos[titulo].configure(text=texto)

    def dibujar(self, ahora: Optional[float] = None):
        if ahora is None:
            ahora = time.monotonic()
        c = self.lienzo
        w, h = c.winfo_width(), c.winfo_height()
        if w < 200 or h < 190:
            return
        c.delete("all")
        cx, cy = w / 2, h - 43
        radio = min((w - 116) / 2, h - 102)
        if radio <= 0:
            return

        def posicion(angulo: float, distancia_cm: float):
            t = math.radians(angulo)
            r = distancia_cm / DISTANCIA_MAX_CM * radio
            return cx + r * math.cos(t), cy - r * math.sin(t)

        # Superficie radar, horizonte y arcos de rango.
        c.create_line(cx - radio - 12, cy, cx + radio + 12, cy,
                      fill=VERDE, width=2)
        for cm in (5, 10, 15, 20, 25, 30):
            r = radio * cm / DISTANCIA_MAX_CM
            color = "#244f31" if cm % 10 else VERDE_SUAVE
            c.create_arc(cx - r, cy - r, cx + r, cy + r,
                         start=0, extent=180, style="arc", outline=color,
                         width=1.7 if cm % 10 == 0 else 1)

        # Líneas radiales y graduación angular.
        for angulo in range(0, 181, 15):
            x, y = posicion(angulo, DISTANCIA_MAX_CM)
            color = VERDE_SUAVE if angulo % 30 == 0 else VERDE_OSCURO
            c.create_line(cx, cy, x, y, fill=color,
                          width=1.6 if angulo % 30 == 0 else 1)
        for angulo in range(0, 181, 5):
            a = math.radians(angulo)
            t1 = radio + 2
            t2 = radio + (10 if angulo % 30 == 0 else 5)
            c.create_line(cx + t1 * math.cos(a), cy - t1 * math.sin(a),
                          cx + t2 * math.cos(a), cy - t2 * math.sin(a),
                          fill=VERDE_SUAVE if angulo % 30 == 0 else VERDE_OSCURO)

        for angulo in (0, 30, 60, 90, 120, 150, 180):
            a = math.radians(angulo)
            r = radio + 25
            x, y = cx + r * math.cos(a), cy - r * math.sin(a)
            c.create_text(x, y, text=f"{angulo}°", fill=VERDE,
                          font=("Consolas", 11, "bold"))
        for distancia in (10, 20, 30):
            x, _ = posicion(0, distancia)
            c.create_text(x - 1, cy - 15, text=f"{distancia} cm",
                          fill=VERDE, anchor="e", font=("Consolas", 10, "bold"))
        c.create_oval(cx - 4, cy - 4, cx + 4, cy + 4,
                      fill=VERDE, outline="")

        # Haz móvil: refleja el ÚLTIMO ángulo recibido (sin animación artificial).
        if self.angulo_actual is not None and ahora - self.ultimo_paquete <= 1.2:
            for i in range(9, -1, -1):
                a = self.angulo_actual - self.sentido * i * 1.3
                if 0 <= a <= 180:
                    x, y = posicion(a, DISTANCIA_MAX_CM)
                    intensidad = (10 - i) / 10
                    c.create_line(cx, cy, x, y,
                                  fill=mezclar_color(FONDO, VERDE, 0.12 + 0.52 * intensidad),
                                  width=1 if i else 2)

        # Cada muestra válida deja un punto rojo en su ubicación polar.
        # 100 cm nunca genera un punto. Los puntos viejos se atenúan.
        for angulo, (distancia, instante) in sorted(self.ecos.items()):
            edad = max(0.0, ahora - instante)
            intensidad = max(0.0, 1 - edad / PERSISTENCIA_PUNTOS_S)
            x, y = posicion(angulo, distancia)
            halo = mezclar_color(FONDO, ROJO, 0.20 + 0.20 * intensidad)
            nucleo = mezclar_color(FONDO, ROJO, 0.35 + 0.65 * intensidad)
            c.create_oval(x - 7, y - 7, x + 7, y + 7, outline="", fill=halo)
            c.create_oval(x - 3.8, y - 3.8, x + 3.8, y + 3.8,
                          outline="", fill=nucleo)

        # Leyenda superior discreta, fuera del sector de medición.
        c.create_oval(22, 15, 30, 23, fill=ROJO, outline="")
        c.create_text(39, 19, text="ECOS DETECTADOS", anchor="w",
                      font=("Consolas", 9), fill=GRIS)
        c.create_line(205, 19, 227, 19, fill=VERDE, width=2)
        c.create_text(235, 19, text="ÁNGULO ACTUAL", anchor="w",
                      font=("Consolas", 9), fill=GRIS)

    def cerrar(self):
        self.detener.set()
        self.ventana.destroy()


def main():
    parser = argparse.ArgumentParser(description="Radar semicircular STM32 por UART")
    parser.add_argument("--port", default=PUERTO_SERIAL,
                        help="Puerto serie (predeterminado COM7)")
    parser.add_argument("--baud", type=int, default=BAUD_RATE,
                        help="Velocidad UART (predeterminado 115200)")
    parser.add_argument("--demo", action="store_true",
                        help="Probar la interfaz con datos simulados, sin conectar UART")
    args = parser.parse_args()
    ventana = tk.Tk()
    InterfazRadar(ventana, args.port, args.baud, args.demo)
    ventana.mainloop()


if __name__ == "__main__":
    main()
