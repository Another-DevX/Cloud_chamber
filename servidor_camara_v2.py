# -*- coding: utf-8 -*-
"""
servidor_camara.py — Panel de control para la cámara de niebla
===============================================================

Sirve una página web local para monitorear y controlar la temperatura de la
cámara de niebla de difusión:

  * Lee las temperaturas de dos termocuplas desde un ESP32 (JSON por serial).
  * Controla la fuente OWON ODP3032 por serial usando su protocolo
    &comando,args,checksum# (ver ODP3032.txt).
  * Corre dos lazos PID independientes:
      - Zona FRÍA  (CH1, Peltier):   más voltaje => más frío.
      - Zona CALIENTE (CH2, calefactor): más voltaje => más caliente.
  * Incluye protecciones: límites de voltaje/corriente, rampa de voltaje
    (para no golpear térmicamente la Peltier), y watchdog que apaga las
    salidas si dejan de llegar datos de temperatura.

Uso:
    pip install flask pyserial
    python servidor_camara.py
    -> abrir http://localhost:8000 en el navegador

Todo está en este único archivo para que sea fácil de copiar a la máquina
del laboratorio.
"""

import json
import math
import threading
import time
from collections import deque

import serial                       # solo para el ESP32 (CDC/serie)
import serial.tools.list_ports
import usb.core                      # para la fuente OWON (USB en bruto)
import usb.util
from flask import Flask, jsonify, request, Response

# ============================================================================
# Configuración general
# ============================================================================

PUERTO_WEB = 8000

# --- Identificación USB de la fuente OWON (igual que en el script original) ---
# La ODP3032 NO es un puerto serie: es un dispositivo USB en bruto. Por eso se
# maneja con pyusb y no aparece en la lista de puertos COM/tty.
OWON_VID = 0x5345   # Vendor ID de OWON
OWON_PID = 0x1234   # Product ID (puede cambiar entre modelos/firmware)
EP_OUT = 0x03       # Endpoint Bulk OUT (escritura)
EP_IN = 0x81        # Endpoint Bulk IN  (lectura)

# Canales de la fuente OWON
CANAL_FRIO = 1      # Peltier de etapa fina
CANAL_CALIENTE = 2  # Calefactor

# Límites duros de la ODP3032 (30 V / 3 A por canal). Los límites "blandos"
# se configuran desde la interfaz, pero nunca podrán superar estos.
V_MAX_HW = 30.0
I_MAX_HW = 3.0

# Periodo del lazo de control (s)
PERIODO_CONTROL = 1.0

# Watchdog: si los datos de temperatura tienen más de este tiempo (s),
# se apagan las salidas y se desactivan los PID.
WATCHDOG_S = 5.0

# Rampa máxima de voltaje por segundo (protege la Peltier de choques térmicos)
RAMPA_V_POR_S = 0.5

# ============================================================================
# PID con anti-windup y derivada sobre la medición
# ============================================================================

class PID:
    """
    PID discreto pensado para actuadores unidireccionales (solo puede
    enfriar o solo puede calentar), que es exactamente nuestra limitación:
    la fuente no puede invertir polaridad.

      - Salida acotada a [out_min, out_max] (voltios).
      - Anti-windup por "clamping": el integrador solo acumula cuando la
        salida no está saturada, o cuando el error empuja de vuelta.
      - Derivada calculada sobre la medición (no sobre el error) para que
        un cambio de setpoint no produzca un pico de salida.
    """

    def __init__(self, kp, ki, kd, out_min=0.0, out_max=12.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.reset()

    def reset(self):
        self._integral = 0.0
        self._last_meas = None
        self._last_out = 0.0

    def update(self, error, medicion, dt):
        if dt <= 0:
            return self._last_out

        p = self.kp * error

        # Derivada sobre la medición (signo negativo: si la medición se mueve
        # hacia el setpoint, frena la salida).
        if self._last_meas is None or self.kd == 0:
            d = 0.0
        else:
            d = -self.kd * (medicion - self._last_meas) / dt
        self._last_meas = medicion

        # Candidato de salida con el integrador actual
        u = p + self._integral + d

        # Anti-windup: integrar solo si no estamos saturados, o si el error
        # ayuda a salir de la saturación.
        saturado_arriba = u >= self.out_max and error > 0
        saturado_abajo = u <= self.out_min and error < 0
        if not (saturado_arriba or saturado_abajo):
            self._integral += self.ki * error * dt
            # Límite absoluto del integrador por si acaso
            self._integral = max(-self.out_max, min(self.out_max, self._integral))

        u = p + self._integral + d
        u = max(self.out_min, min(self.out_max, u))
        self._last_out = u
        return u


# ============================================================================
# Driver de la fuente OWON ODP3032 (protocolo &cmd,args,checksum#)
# ============================================================================

class FuenteOWON:
    """
    Driver USB de la fuente OWON ODP3032. Usa pyusb con endpoints bulk crudos,
    exactamente como el script original que ya les funcionaba en Windows:
    escribe en EP_OUT (0x03) y lee de EP_IN (0x81), con el framing
    &payload,checksum#\\r\\n.
    """

    def __init__(self):
        self._dev = None
        self._lock = threading.Lock()
        self.etiqueta = None        # "5345:1234" del dispositivo conectado
        self.ultima_respuesta = ""

    @property
    def conectada(self):
        return self._dev is not None

    def conectar(self, ident=None):
        """
        `ident` es "bus.address" (lo que manda la interfaz). Si viene vacío,
        se busca por VID/PID como en el script original.
        """
        self.desconectar()

        if ident and "." in ident:
            bus, addr = (int(x) for x in ident.split("."))
            dev = usb.core.find(custom_match=lambda d: d.bus == bus and d.address == addr)
        else:
            dev = usb.core.find(idVendor=OWON_VID, idProduct=OWON_PID)

        if dev is None:
            raise RuntimeError("No se encontró la fuente OWON. Verifica el cable USB.")

        # En Linux puede haber un driver del kernel tomado; lo soltamos.
        # En Windows esto no aplica y lanza excepción, así que lo ignoramos.
        try:
            if dev.is_kernel_driver_active(0):
                dev.detach_kernel_driver(0)
        except Exception:
            pass

        try:
            dev.set_configuration()
        except usb.core.USBError:
            # Si ya estaba configurada, seguimos.
            pass

        self._dev = dev
        self.etiqueta = f"{dev.idVendor:04x}:{dev.idProduct:04x}"

    def desconectar(self):
        if self._dev is not None:
            try:
                usb.util.dispose_resources(self._dev)
            except Exception:
                pass
        self._dev = None
        self.etiqueta = None

    @staticmethod
    def _checksum(payload):
        # Igual que en el script original: suma de ord() de (payload + ',')
        return sum(ord(c) for c in (payload + ','))

    def _enmarcar(self, payload):
        return f"&{payload},{self._checksum(payload)}#\r\n".encode()

    def enviar(self, payload, leer=False):
        """Envía &payload,checksum# y opcionalmente lee la respuesta."""
        if not self.conectada:
            raise RuntimeError("La fuente no está conectada")
        with self._lock:
            self._dev.write(EP_OUT, self._enmarcar(payload))
            time.sleep(0.1)
            if leer:
                datos = self._dev.read(EP_IN, 512, timeout=5000)
                self.ultima_respuesta = ''.join(chr(x) for x in datos).strip()
                return self.ultima_respuesta
        return None

    # ---- Comandos de la especificación (ODP3032.txt) ----

    def salida(self, canal, encendida):
        # SW{CH},{0,1}: 0 activa, 1 desactiva (¡ojo, está invertido!)
        self.enviar(f"SW{canal},{0 if encendida else 1}")

    def set_voltaje(self, canal, voltios):
        voltios = max(0.0, min(V_MAX_HW, voltios))
        self.enviar(f"SCH{canal}V,{voltios:.2f}")

    def set_corriente(self, canal, amperios):
        amperios = max(0.0, min(I_MAX_HW, amperios))
        self.enviar(f"SCH{canal}C,{amperios:.2f}")

    def sincronizar(self):
        """SYNCHRO,0 devuelve información de la fuente. El formato exacto de
        la respuesta no está documentado en la especificación que tenemos,
        así que por ahora solo la guardamos y la mostramos cruda en la
        interfaz. Cuando la vean en pantalla, es fácil escribir el parser."""
        try:
            return self.enviar("SYNCHRO,0", leer=True)
        except Exception as e:
            return f"error: {e}"


# ============================================================================
# Lector del ESP32
# ============================================================================

class LectorESP32:
    def __init__(self, estado):
        self._ser = None
        self._hilo = None
        self._detener = threading.Event()
        self.estado = estado
        self.puerto = None

    @property
    def conectado(self):
        return self._ser is not None and self._ser.is_open

    def conectar(self, puerto, baudios=115200):
        self.desconectar()
        self._ser = serial.Serial(puerto, baudios, timeout=2)
        self.puerto = puerto
        self._detener.clear()
        self._hilo = threading.Thread(target=self._bucle, daemon=True)
        self._hilo.start()

    def desconectar(self):
        self._detener.set()
        if self._hilo is not None:
            self._hilo.join(timeout=3)
            self._hilo = None
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self.puerto = None

    def _bucle(self):
        while not self._detener.is_set():
            try:
                linea = self._ser.readline().decode(errors="replace").strip()
            except Exception:
                # El puerto se cayó (desconectaron el cable, etc.)
                self.estado.registrar_error("Se perdió la conexión con el ESP32")
                break
            if not linea:
                continue
            try:
                dato = json.loads(linea)
            except json.JSONDecodeError:
                continue  # línea corrupta, ignorar
            t_cold = dato.get("t_cold")
            t_hot = dato.get("t_hot")
            self.estado.nueva_muestra(t_cold, t_hot)


# ============================================================================
# Estado global compartido
# ============================================================================

class Estado:
    def __init__(self):
        self.lock = threading.Lock()

        # Última medición
        self.t_cold = None
        self.t_hot = None
        self.t_dato = 0.0  # time.time() de la última muestra válida

        # Historia para gráficas y estabilidad: (timestamp, t_cold, t_hot)
        self.historia = deque(maxlen=3600)

        # Setpoints (°C)
        self.sp_cold = -25.0
        self.sp_hot = 30.0

        # PIDs y sus parámetros por zona
        self.pid_cold = PID(kp=1.5, ki=0.05, kd=2.0, out_min=0.0, out_max=12.0)
        self.pid_hot = PID(kp=1.0, ki=0.03, kd=1.0, out_min=0.0, out_max=12.0)
        self.pid_cold_on = False
        self.pid_hot_on = False

        # Límites de corriente que se programan en la fuente al activar
        self.i_lim_cold = 3.0
        self.i_lim_hot = 1.5

        # Últimos voltajes enviados a la fuente (para rampa y monitoreo)
        self.v_cold = 0.0
        self.v_hot = 0.0

        # Mensajes de error / eventos para mostrar en la interfaz
        self.eventos = deque(maxlen=20)

    def registrar_error(self, msg):
        with self.lock:
            self.eventos.appendleft((time.time(), msg))

    def nueva_muestra(self, t_cold, t_hot):
        ahora = time.time()
        with self.lock:
            self.t_cold = t_cold
            self.t_hot = t_hot
            if t_cold is not None or t_hot is not None:
                self.t_dato = ahora
            self.historia.append((ahora, t_cold, t_hot))

    def estabilidad(self, zona, ventana_s=60.0):
        """
        Devuelve (sigma, delta, estado) para la zona:
          sigma  = desviación estándar de la temperatura en la ventana
          delta  = |promedio - setpoint|
          estado = 'sin_datos' | 'estabilizando' | 'estable'
        Criterio de 'estable': sigma < 0.3 °C y delta < 0.5 °C.
        """
        ahora = time.time()
        idx = 1 if zona == "cold" else 2
        sp = self.sp_cold if zona == "cold" else self.sp_hot
        with self.lock:
            vals = [m[idx] for m in self.historia
                    if ahora - m[0] <= ventana_s and m[idx] is not None]
        if len(vals) < 10:
            return None, None, "sin_datos"
        media = sum(vals) / len(vals)
        var = sum((v - media) ** 2 for v in vals) / len(vals)
        sigma = math.sqrt(var)
        delta = abs(media - sp)
        estado = "estable" if (sigma < 0.3 and delta < 0.5) else "estabilizando"
        return sigma, delta, estado


estado = Estado()
fuente = FuenteOWON()
lector = LectorESP32(estado)


# ============================================================================
# Lazo de control (hilo)
# ============================================================================

def apagar_salidas(motivo=None):
    """Pone ambos canales a 0 V y apaga las salidas. Nunca lanza excepción."""
    estado.pid_cold_on = False
    estado.pid_hot_on = False
    estado.pid_cold.reset()
    estado.pid_hot.reset()
    estado.v_cold = 0.0
    estado.v_hot = 0.0
    if motivo:
        estado.registrar_error(motivo)
    if fuente.conectada:
        try:
            fuente.set_voltaje(CANAL_FRIO, 0.0)
            fuente.set_voltaje(CANAL_CALIENTE, 0.0)
            fuente.salida(CANAL_FRIO, False)
            fuente.salida(CANAL_CALIENTE, False)
        except Exception as e:
            estado.registrar_error(f"No se pudo apagar la fuente: {e}")


def _rampa(v_actual, v_deseado, dt):
    """Limita el cambio de voltaje a RAMPA_V_POR_S voltios por segundo."""
    dv_max = RAMPA_V_POR_S * dt
    dv = v_deseado - v_actual
    if dv > dv_max:
        dv = dv_max
    elif dv < -dv_max:
        dv = -dv_max
    return v_actual + dv


def lazo_control():
    ultimo = time.time()
    ultimo_synchro = 0.0
    while True:
        time.sleep(PERIODO_CONTROL)
        ahora = time.time()
        dt = ahora - ultimo
        ultimo = ahora

        pid_activo = estado.pid_cold_on or estado.pid_hot_on

        # --- Watchdog: datos de temperatura frescos ---
        if pid_activo and (ahora - estado.t_dato) > WATCHDOG_S:
            apagar_salidas("WATCHDOG: sin datos de temperatura, salidas apagadas")
            continue

        if not fuente.conectada:
            if pid_activo:
                estado.pid_cold_on = False
                estado.pid_hot_on = False
                estado.registrar_error("La fuente se desconectó; PID desactivado")
            continue

        try:
            # --- Zona fría: más voltaje en la Peltier => baja la temperatura.
            # Error positivo cuando está más caliente que el setpoint.
            if estado.pid_cold_on and estado.t_cold is not None:
                err = estado.t_cold - estado.sp_cold
                v = estado.pid_cold.update(err, estado.t_cold, dt)
                v = _rampa(estado.v_cold, v, dt)
                estado.v_cold = v
                fuente.set_voltaje(CANAL_FRIO, v)

            # --- Zona caliente: más voltaje en el calefactor => sube la temp.
            # Error positivo cuando está más fría que el setpoint.
            if estado.pid_hot_on and estado.t_hot is not None:
                err = estado.sp_hot - estado.t_hot
                v = estado.pid_hot.update(err, estado.t_hot, dt)
                v = _rampa(estado.v_hot, v, dt)
                estado.v_hot = v
                fuente.set_voltaje(CANAL_CALIENTE, v)

            # --- Consulta periódica de la fuente (cruda, ver nota en driver)
            if ahora - ultimo_synchro > 10.0:
                ultimo_synchro = ahora
                fuente.sincronizar()

        except Exception as e:
            apagar_salidas(f"Error comunicándose con la fuente: {e}")


# ============================================================================
# Servidor web
# ============================================================================

app = Flask(__name__)


def listar_usb():
    """Enumera dispositivos USB para elegir la fuente. Devuelve la lista y,
    si el backend de libusb no está disponible, un mensaje de error."""
    lista = []
    try:
        dispositivos = usb.core.find(find_all=True)
    except Exception as e:
        return [], f"No se pudo acceder al USB (¿falta libusb?): {e}"

    for d in dispositivos:
        try:
            producto = usb.util.get_string(d, d.iProduct) or ""
        except Exception:
            producto = ""
        try:
            fabricante = usb.util.get_string(d, d.iManufacturer) or ""
        except Exception:
            fabricante = ""
        es_owon = (d.idVendor == OWON_VID)
        nombre = (producto or fabricante or "Dispositivo USB").strip()
        etiqueta = f"{nombre} ({d.idVendor:04x}:{d.idProduct:04x})"
        lista.append({
            "id": f"{d.bus}.{d.address}",
            "etiqueta": ("★ " if es_owon else "") + etiqueta,
            "owon": es_owon,
        })
    # OWON primero para que quede preseleccionada.
    lista.sort(key=lambda x: (not x["owon"], x["etiqueta"]))
    return lista, None


@app.get("/api/puertos")
def api_puertos():
    serie = [{"device": p.device, "descripcion": p.description}
             for p in serial.tools.list_ports.comports()]
    usb_devs, usb_error = listar_usb()
    return jsonify({"serial": serie, "usb": usb_devs, "usb_error": usb_error})


@app.post("/api/conectar")
def api_conectar():
    datos = request.get_json(force=True)
    dispositivo = datos.get("dispositivo")
    puerto = datos.get("puerto")
    baudios = int(datos.get("baudios", 115200))
    try:
        if dispositivo == "esp32":
            lector.conectar(puerto, baudios)
        elif dispositivo == "fuente":
            # `puerto` aquí es el id USB "bus.address" (no un puerto COM).
            fuente.conectar(puerto)
            # Programar límites de corriente de una vez, con salidas apagadas
            fuente.set_voltaje(CANAL_FRIO, 0.0)
            fuente.set_voltaje(CANAL_CALIENTE, 0.0)
            fuente.set_corriente(CANAL_FRIO, estado.i_lim_cold)
            fuente.set_corriente(CANAL_CALIENTE, estado.i_lim_hot)
        else:
            return jsonify({"ok": False, "error": "dispositivo desconocido"}), 400
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500
    return jsonify({"ok": True})


@app.post("/api/desconectar")
def api_desconectar():
    datos = request.get_json(force=True)
    dispositivo = datos.get("dispositivo")
    if dispositivo == "esp32":
        lector.desconectar()
    elif dispositivo == "fuente":
        apagar_salidas()
        fuente.desconectar()
    return jsonify({"ok": True})


@app.post("/api/setpoints")
def api_setpoints():
    datos = request.get_json(force=True)
    with estado.lock:
        if "cold" in datos:
            estado.sp_cold = float(datos["cold"])
        if "hot" in datos:
            estado.sp_hot = float(datos["hot"])
    return jsonify({"ok": True})


@app.post("/api/pid")
def api_pid():
    """Configura ganancias/límites y activa o desactiva cada lazo."""
    datos = request.get_json(force=True)
    zona = datos.get("zona")
    pid = estado.pid_cold if zona == "cold" else estado.pid_hot

    for campo in ("kp", "ki", "kd"):
        if campo in datos:
            setattr(pid, campo, float(datos[campo]))
    if "vmax" in datos:
        pid.out_max = max(0.0, min(V_MAX_HW, float(datos["vmax"])))
    if "imax" in datos:
        imax = max(0.0, min(I_MAX_HW, float(datos["imax"])))
        if zona == "cold":
            estado.i_lim_cold = imax
        else:
            estado.i_lim_hot = imax
        if fuente.conectada:
            canal = CANAL_FRIO if zona == "cold" else CANAL_CALIENTE
            fuente.set_corriente(canal, imax)

    if "activo" in datos:
        activo = bool(datos["activo"])
        canal = CANAL_FRIO if zona == "cold" else CANAL_CALIENTE
        if activo:
            if not fuente.conectada:
                return jsonify({"ok": False, "error": "Conecta la fuente primero"}), 400
            if (time.time() - estado.t_dato) > WATCHDOG_S:
                return jsonify({"ok": False,
                                "error": "No hay datos de temperatura recientes"}), 400
            pid.reset()
            try:
                fuente.set_voltaje(canal, 0.0)
                fuente.salida(canal, True)
            except Exception as e:
                return jsonify({"ok": False, "error": str(e)}), 500
            if zona == "cold":
                estado.v_cold = 0.0
                estado.pid_cold_on = True
            else:
                estado.v_hot = 0.0
                estado.pid_hot_on = True
        else:
            if zona == "cold":
                estado.pid_cold_on = False
                estado.v_cold = 0.0
            else:
                estado.pid_hot_on = False
                estado.v_hot = 0.0
            if fuente.conectada:
                try:
                    fuente.set_voltaje(canal, 0.0)
                    fuente.salida(canal, False)
                except Exception as e:
                    return jsonify({"ok": False, "error": str(e)}), 500
    return jsonify({"ok": True})


@app.post("/api/parada")
def api_parada():
    apagar_salidas("Parada de emergencia desde la interfaz")
    return jsonify({"ok": True})


@app.get("/api/estado")
def api_estado():
    ahora = time.time()
    sig_c, del_c, est_c = estado.estabilidad("cold")
    sig_h, del_h, est_h = estado.estabilidad("hot")
    with estado.lock:
        # Historia de los últimos 15 minutos, decimada a ~450 puntos
        corte = ahora - 900
        hist = [m for m in estado.historia if m[0] >= corte]
        paso = max(1, len(hist) // 450)
        hist = hist[::paso]
        return jsonify({
            "ahora": ahora,
            "esp32": {"conectado": lector.conectado, "puerto": lector.puerto,
                      "datos_frescos": (ahora - estado.t_dato) < WATCHDOG_S},
            "fuente": {"conectada": fuente.conectada, "puerto": fuente.etiqueta,
                       "respuesta_cruda": fuente.ultima_respuesta},
            "zonas": {
                "cold": {"t": estado.t_cold, "sp": estado.sp_cold,
                         "pid_on": estado.pid_cold_on, "v": estado.v_cold,
                         "imax": estado.i_lim_cold,
                         "kp": estado.pid_cold.kp, "ki": estado.pid_cold.ki,
                         "kd": estado.pid_cold.kd, "vmax": estado.pid_cold.out_max,
                         "sigma": sig_c, "delta": del_c, "estab": est_c},
                "hot": {"t": estado.t_hot, "sp": estado.sp_hot,
                        "pid_on": estado.pid_hot_on, "v": estado.v_hot,
                        "imax": estado.i_lim_hot,
                        "kp": estado.pid_hot.kp, "ki": estado.pid_hot.ki,
                        "kd": estado.pid_hot.kd, "vmax": estado.pid_hot.out_max,
                        "sigma": sig_h, "delta": del_h, "estab": est_h},
            },
            "historia": [[round(m[0], 1), m[1], m[2]] for m in hist],
            "eventos": [{"t": e[0], "msg": e[1]} for e in list(estado.eventos)],
        })


@app.get("/")
def index():
    return Response(PAGINA, mimetype="text/html")


# ============================================================================
# Interfaz web
# ============================================================================

PAGINA = r"""<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Cámara de niebla — Control térmico</title>
<style>
  :root {
    --fondo: #0d1220;
    --panel: #141b2e;
    --panel-borde: #1f2942;
    --texto: #e8edf5;
    --tenue: #8a96ab;
    --frio: #6fd6e8;
    --frio-suave: rgba(111, 214, 232, 0.12);
    --calor: #f2a65a;
    --calor-suave: rgba(242, 166, 90, 0.12);
    --ok: #7bd88f;
    --alerta: #e86a6a;
    --mono: ui-monospace, "Cascadia Mono", "JetBrains Mono", Consolas, monospace;
    --sans: system-ui, -apple-system, "Segoe UI", Roboto, sans-serif;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0;
    background: var(--fondo);
    color: var(--texto);
    font-family: var(--sans);
    font-size: 15px;
    padding-bottom: 40px;
  }
  header {
    display: flex; align-items: baseline; gap: 14px;
    padding: 18px 28px 6px;
  }
  header h1 {
    font-size: 17px; font-weight: 600; letter-spacing: 0.06em;
    text-transform: uppercase; margin: 0;
  }
  header .sub { color: var(--tenue); font-size: 13px; }

  .contenedor { max-width: 1180px; margin: 0 auto; padding: 0 22px; }

  /* --- Barra de conexiones --- */
  .conexiones {
    display: grid; grid-template-columns: 1fr 1fr; gap: 14px; margin: 14px 0 20px;
  }
  .conexion {
    background: var(--panel); border: 1px solid var(--panel-borde);
    border-radius: 10px; padding: 12px 14px;
    display: flex; align-items: center; gap: 10px; flex-wrap: wrap;
  }
  .conexion .nombre { font-weight: 600; min-width: 70px; }
  .punto {
    width: 9px; height: 9px; border-radius: 50%;
    background: var(--alerta); flex: none;
  }
  .punto.on { background: var(--ok); }
  select, input[type=number] {
    background: #0f1526; color: var(--texto);
    border: 1px solid var(--panel-borde); border-radius: 7px;
    padding: 6px 9px; font-family: var(--mono); font-size: 13px;
  }
  select { min-width: 170px; }
  button {
    background: #202b48; color: var(--texto);
    border: 1px solid #2c3a5e; border-radius: 7px;
    padding: 7px 14px; font-size: 13px; font-weight: 600; cursor: pointer;
  }
  button:hover { background: #283656; }
  button:focus-visible, select:focus-visible, input:focus-visible {
    outline: 2px solid var(--frio); outline-offset: 1px;
  }
  button.peligro { background: #4a1f24; border-color: #7a3038; color: #ffd9d9; }
  button.peligro:hover { background: #5d262d; }

  /* --- Zonas --- */
  .zonas { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; }
  .zona {
    background: var(--panel); border: 1px solid var(--panel-borde);
    border-radius: 12px; padding: 18px 20px; position: relative; overflow: hidden;
  }
  .zona::before {
    content: ""; position: absolute; inset: 0 auto 0 0; width: 4px;
  }
  .zona.fria::before { background: var(--frio); }
  .zona.caliente::before { background: var(--calor); }
  .zona h2 {
    margin: 0 0 4px; font-size: 13px; letter-spacing: 0.12em;
    text-transform: uppercase; font-weight: 600;
  }
  .zona.fria h2 { color: var(--frio); }
  .zona.caliente h2 { color: var(--calor); }
  .zona .canal { color: var(--tenue); font-size: 12px; }

  .lectura {
    font-family: var(--mono); font-size: 52px; font-weight: 500;
    font-variant-numeric: tabular-nums; line-height: 1.1; margin: 10px 0 2px;
  }
  .zona.fria .lectura { color: var(--frio); }
  .zona.caliente .lectura { color: var(--calor); }
  .lectura .unidad { font-size: 22px; color: var(--tenue); }

  .estab {
    display: inline-flex; align-items: center; gap: 7px;
    font-size: 12.5px; padding: 4px 10px; border-radius: 20px;
    background: #0f1526; border: 1px solid var(--panel-borde); margin: 6px 0 12px;
  }
  .estab .p { width: 7px; height: 7px; border-radius: 50%; background: var(--tenue); }
  .estab.estable .p { background: var(--ok); animation: pulso 2s infinite; }
  .estab.estabilizando .p { background: var(--calor); }
  @keyframes pulso { 50% { opacity: 0.35; } }
  @media (prefers-reduced-motion: reduce) { .estab.estable .p { animation: none; } }

  .fila { display: flex; align-items: center; gap: 10px; margin: 8px 0; flex-wrap: wrap; }
  .fila label { color: var(--tenue); font-size: 13px; min-width: 118px; }
  .fila input[type=number] { width: 90px; }

  .medidas {
    display: flex; gap: 22px; font-family: var(--mono); font-size: 13px;
    color: var(--tenue); margin-top: 10px;
    border-top: 1px solid var(--panel-borde); padding-top: 10px;
  }
  .medidas b { color: var(--texto); font-weight: 500; }

  details.avanzado { margin-top: 10px; }
  details.avanzado summary {
    cursor: pointer; color: var(--tenue); font-size: 12.5px;
    user-select: none;
  }
  details.avanzado .fila input { width: 72px; }

  .interruptor { margin-left: auto; }
  .interruptor button.on { background: #1d3d2a; border-color: #2e5c40; color: #c9f2d5; }

  /* --- Gráfica --- */
  .grafica {
    background: var(--panel); border: 1px solid var(--panel-borde);
    border-radius: 12px; padding: 14px 18px 6px; margin-top: 14px;
  }
  .grafica h3 {
    margin: 0 0 8px; font-size: 12px; letter-spacing: 0.1em;
    text-transform: uppercase; color: var(--tenue); font-weight: 600;
  }
  canvas { width: 100%; height: 240px; display: block; }

  /* --- Pie: parada y eventos --- */
  .pie { display: flex; gap: 14px; margin-top: 14px; align-items: flex-start; }
  .eventos {
    flex: 1; background: var(--panel); border: 1px solid var(--panel-borde);
    border-radius: 10px; padding: 10px 14px; font-size: 12.5px;
    color: var(--tenue); max-height: 120px; overflow-y: auto;
    font-family: var(--mono);
  }
  .eventos div { padding: 2px 0; }
  .parada button { font-size: 15px; padding: 14px 22px; border-radius: 10px; }

  @media (max-width: 860px) {
    .zonas, .conexiones { grid-template-columns: 1fr; }
    .lectura { font-size: 42px; }
  }
</style>
</head>
<body>
<header>
  <h1>Cámara de niebla</h1>
  <span class="sub">control térmico · OWON ODP3032 · ESP32</span>
</header>

<div class="contenedor">

  <!-- Conexiones -->
  <div class="conexiones">
    <div class="conexion">
      <span class="punto" id="punto-esp32"></span>
      <span class="nombre">ESP32</span>
      <select id="puerto-esp32"></select>
      <button id="btn-esp32">Conectar</button>
    </div>
    <div class="conexion">
      <span class="punto" id="punto-fuente"></span>
      <span class="nombre">Fuente</span>
      <select id="puerto-fuente"></select>
      <button id="btn-fuente">Conectar</button>
      <button id="btn-puertos" title="Volver a buscar puertos">⟳</button>
    </div>
  </div>
  <div id="aviso-usb" style="display:none; color:var(--alerta); font-size:12.5px;
       font-family:var(--mono); margin:-8px 0 16px;"></div>

  <!-- Zonas -->
  <div class="zonas">

    <section class="zona fria">
      <h2>Zona fría</h2>
      <div class="canal">Peltier · canal 1</div>
      <div class="lectura" id="t-cold">--.-<span class="unidad"> °C</span></div>
      <div class="estab" id="estab-cold"><span class="p"></span><span class="txt">sin datos</span></div>

      <div class="fila">
        <label for="sp-cold">Temperatura objetivo</label>
        <input type="number" id="sp-cold" step="0.5" value="-25.0"> °C
        <span class="interruptor"><button id="pid-cold">Activar control</button></span>
      </div>

      <details class="avanzado">
        <summary>Ajustes avanzados (PID y límites)</summary>
        <div class="fila"><label>Kp / Ki / Kd</label>
          <input type="number" id="kp-cold" step="0.1">
          <input type="number" id="ki-cold" step="0.01">
          <input type="number" id="kd-cold" step="0.1">
        </div>
        <div class="fila"><label>V máx / I máx</label>
          <input type="number" id="vmax-cold" step="0.5"> V
          <input type="number" id="imax-cold" step="0.1"> A
          <button id="guardar-cold">Guardar</button>
        </div>
      </details>

      <div class="medidas">
        <span>V programado <b id="v-cold">0.00 V</b></span>
        <span>σ(60 s) <b id="sigma-cold">—</b></span>
        <span>ΔT <b id="delta-cold">—</b></span>
      </div>
    </section>

    <section class="zona caliente">
      <h2>Zona caliente</h2>
      <div class="canal">Calefactor · canal 2</div>
      <div class="lectura" id="t-hot">--.-<span class="unidad"> °C</span></div>
      <div class="estab" id="estab-hot"><span class="p"></span><span class="txt">sin datos</span></div>

      <div class="fila">
        <label for="sp-hot">Temperatura objetivo</label>
        <input type="number" id="sp-hot" step="0.5" value="30.0"> °C
        <span class="interruptor"><button id="pid-hot">Activar control</button></span>
      </div>

      <details class="avanzado">
        <summary>Ajustes avanzados (PID y límites)</summary>
        <div class="fila"><label>Kp / Ki / Kd</label>
          <input type="number" id="kp-hot" step="0.1">
          <input type="number" id="ki-hot" step="0.01">
          <input type="number" id="kd-hot" step="0.1">
        </div>
        <div class="fila"><label>V máx / I máx</label>
          <input type="number" id="vmax-hot" step="0.5"> V
          <input type="number" id="imax-hot" step="0.1"> A
          <button id="guardar-hot">Guardar</button>
        </div>
      </details>

      <div class="medidas">
        <span>V programado <b id="v-hot">0.00 V</b></span>
        <span>σ(60 s) <b id="sigma-hot">—</b></span>
        <span>ΔT <b id="delta-hot">—</b></span>
      </div>
    </section>
  </div>

  <!-- Gráfica -->
  <div class="grafica">
    <h3>Últimos 15 minutos</h3>
    <canvas id="lienzo"></canvas>
  </div>

  <!-- Pie -->
  <div class="pie">
    <div class="parada">
      <button class="peligro" id="btn-parada">⏻ Apagar todo</button>
    </div>
    <div class="eventos" id="eventos">Sin eventos.</div>
  </div>

</div>

<script>
"use strict";

const $ = id => document.getElementById(id);
let ultimoEstado = null;
let editando = new Set();   // inputs que el usuario está tocando: no sobreescribir

// Marcar inputs en edición para que el refresco no pise lo que escribe el usuario
document.querySelectorAll("input").forEach(inp => {
  inp.addEventListener("focus", () => editando.add(inp.id));
  inp.addEventListener("blur", () => editando.delete(inp.id));
});

async function api(ruta, cuerpo) {
  const opciones = cuerpo
    ? { method: "POST", headers: {"Content-Type": "application/json"},
        body: JSON.stringify(cuerpo) }
    : {};
  const r = await fetch(ruta, opciones);
  const d = await r.json();
  if (d && d.ok === false) alert(d.error || "Error");
  return d;
}

// ---------- Puertos y conexiones ----------
function llenarSelect(sel, opciones, previo) {
  sel.innerHTML = "";
  if (opciones.length === 0) {
    const o = document.createElement("option");
    o.value = ""; o.textContent = "(ninguno detectado)";
    sel.appendChild(o);
    return;
  }
  opciones.forEach(op => {
    const o = document.createElement("option");
    o.value = op.value; o.textContent = op.texto;
    sel.appendChild(o);
  });
  if (previo) sel.value = previo;
}

async function cargarPuertos() {
  const d = await api("/api/puertos");

  // ESP32: puertos serie (COM3, /dev/ttyUSB0, ...)
  llenarSelect($("puerto-esp32"),
    d.serial.map(p => ({ value: p.device, texto: p.device + " — " + p.descripcion })),
    $("puerto-esp32").value);

  // Fuente OWON: dispositivos USB en bruto (no aparece como puerto serie).
  // La OWON viene marcada con ★ y ordenada primero, así que queda preseleccionada.
  llenarSelect($("puerto-fuente"),
    d.usb.map(u => ({ value: u.id, texto: u.etiqueta })),
    $("puerto-fuente").value);

  const aviso = $("aviso-usb");
  if (d.usb_error) {
    aviso.textContent = d.usb_error;
    aviso.style.display = "block";
  } else {
    aviso.style.display = "none";
  }
}
$("btn-puertos").onclick = cargarPuertos;

function botonConexion(dispositivo, botonId, selectorId, conectadoAhora) {
  return async () => {
    if (conectadoAhora()) {
      await api("/api/desconectar", { dispositivo });
    } else {
      await api("/api/conectar", { dispositivo, puerto: $(selectorId).value });
    }
    refrescar();
  };
}
$("btn-esp32").onclick = botonConexion("esp32", "btn-esp32", "puerto-esp32",
  () => ultimoEstado && ultimoEstado.esp32.conectado);
$("btn-fuente").onclick = botonConexion("fuente", "btn-fuente", "puerto-fuente",
  () => ultimoEstado && ultimoEstado.fuente.conectada);

// ---------- Setpoints ----------
for (const zona of ["cold", "hot"]) {
  $("sp-" + zona).addEventListener("change", async e => {
    await api("/api/setpoints", { [zona]: parseFloat(e.target.value) });
  });
  $("guardar-" + zona).onclick = async () => {
    await api("/api/pid", {
      zona,
      kp: parseFloat($("kp-" + zona).value),
      ki: parseFloat($("ki-" + zona).value),
      kd: parseFloat($("kd-" + zona).value),
      vmax: parseFloat($("vmax-" + zona).value),
      imax: parseFloat($("imax-" + zona).value),
    });
  };
  $("pid-" + zona).onclick = async () => {
    const encendido = ultimoEstado && ultimoEstado.zonas[zona].pid_on;
    await api("/api/pid", { zona, activo: !encendido });
    refrescar();
  };
}

$("btn-parada").onclick = async () => {
  await api("/api/parada");
  refrescar();
};

// ---------- Refresco ----------
function fmt(v, dec = 1) {
  return (v === null || v === undefined || Number.isNaN(v))
    ? "--.-" : v.toFixed(dec);
}

function pintarZona(zona, d) {
  $("t-" + zona).innerHTML = fmt(d.t, 2) + '<span class="unidad"> °C</span>';
  $("v-" + zona).textContent = d.v.toFixed(2) + " V";
  $("sigma-" + zona).textContent = d.sigma === null ? "—" : d.sigma.toFixed(2) + " °C";
  $("delta-" + zona).textContent = d.delta === null ? "—" : d.delta.toFixed(2) + " °C";

  const est = $("estab-" + zona);
  est.className = "estab " + (d.estab === "sin_datos" ? "" : d.estab);
  est.querySelector(".txt").textContent =
    { sin_datos: "sin datos", estabilizando: "estabilizando…", estable: "estable" }[d.estab];

  const btn = $("pid-" + zona);
  btn.textContent = d.pid_on ? "Control activo — detener" : "Activar control";
  btn.classList.toggle("on", d.pid_on);

  // Rellenar inputs solo si el usuario no los está editando
  const campos = { sp: d.sp, kp: d.kp, ki: d.ki, kd: d.kd, vmax: d.vmax, imax: d.imax };
  for (const [k, v] of Object.entries(campos)) {
    const inp = $(k + "-" + zona);
    if (!editando.has(inp.id) && document.activeElement !== inp) inp.value = v;
  }
}

function pintarConexion(puntoId, botonId, info, conectado) {
  $(puntoId).classList.toggle("on", conectado);
  $(botonId).textContent = conectado ? "Desconectar" : "Conectar";
}

async function refrescar() {
  let d;
  try {
    d = await (await fetch("/api/estado")).json();
  } catch { return; }
  ultimoEstado = d;

  pintarConexion("punto-esp32", "btn-esp32", d.esp32, d.esp32.conectado && d.esp32.datos_frescos);
  pintarConexion("punto-fuente", "btn-fuente", d.fuente, d.fuente.conectada);
  pintarZona("cold", d.zonas.cold);
  pintarZona("hot", d.zonas.hot);

  const ev = $("eventos");
  if (d.eventos.length === 0) {
    ev.textContent = "Sin eventos.";
  } else {
    ev.innerHTML = d.eventos.map(e => {
      const hora = new Date(e.t * 1000).toLocaleTimeString();
      return `<div>[${hora}] ${e.msg}</div>`;
    }).join("");
  }

  dibujar(d);
}

// ---------- Gráfica (canvas, sin dependencias) ----------
const lienzo = $("lienzo");
const ctx = lienzo.getContext("2d");

function dibujar(d) {
  const dpr = window.devicePixelRatio || 1;
  const w = lienzo.clientWidth, h = lienzo.clientHeight;
  lienzo.width = w * dpr; lienzo.height = h * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);

  const hist = d.historia;
  if (!hist || hist.length < 2) {
    ctx.fillStyle = "#8a96ab"; ctx.font = "13px system-ui";
    ctx.fillText("Esperando datos…", 16, h / 2);
    return;
  }

  const margen = { izq: 46, der: 10, arr: 10, aba: 22 };
  const t0 = hist[0][0], t1 = hist[hist.length - 1][0];
  const spans = [d.zonas.cold.sp, d.zonas.hot.sp];
  let vals = spans.slice();
  hist.forEach(m => { if (m[1] !== null) vals.push(m[1]); if (m[2] !== null) vals.push(m[2]); });
  let vMin = Math.min(...vals), vMax = Math.max(...vals);
  const holgura = Math.max(1, (vMax - vMin) * 0.1);
  vMin -= holgura; vMax += holgura;

  const X = t => margen.izq + (t - t0) / Math.max(1, t1 - t0) * (w - margen.izq - margen.der);
  const Y = v => margen.arr + (vMax - v) / (vMax - vMin) * (h - margen.arr - margen.aba);

  // Rejilla y eje Y
  ctx.strokeStyle = "#1f2942"; ctx.fillStyle = "#8a96ab";
  ctx.font = "11px ui-monospace, monospace"; ctx.lineWidth = 1;
  const nLineas = 5;
  for (let i = 0; i <= nLineas; i++) {
    const v = vMin + (vMax - vMin) * i / nLineas;
    const y = Y(v);
    ctx.beginPath(); ctx.moveTo(margen.izq, y); ctx.lineTo(w - margen.der, y); ctx.stroke();
    ctx.fillText(v.toFixed(1), 4, y + 4);
  }

  // Setpoints (líneas punteadas)
  const punteada = (v, color) => {
    ctx.save(); ctx.setLineDash([5, 5]); ctx.strokeStyle = color; ctx.globalAlpha = 0.5;
    ctx.beginPath(); ctx.moveTo(margen.izq, Y(v)); ctx.lineTo(w - margen.der, Y(v));
    ctx.stroke(); ctx.restore();
  };
  punteada(d.zonas.cold.sp, "#6fd6e8");
  punteada(d.zonas.hot.sp, "#f2a65a");

  // Trazas
  const traza = (idx, color) => {
    ctx.strokeStyle = color; ctx.lineWidth = 1.8; ctx.beginPath();
    let pluma = false;
    hist.forEach(m => {
      if (m[idx] === null) { pluma = false; return; }
      const x = X(m[0]), y = Y(m[idx]);
      if (!pluma) { ctx.moveTo(x, y); pluma = true; } else { ctx.lineTo(x, y); }
    });
    ctx.stroke();
  };
  traza(1, "#6fd6e8");
  traza(2, "#f2a65a");

  // Eje X: hora inicial y final
  ctx.fillStyle = "#8a96ab";
  const hora = t => new Date(t * 1000).toLocaleTimeString().slice(0, 5);
  ctx.fillText(hora(t0), margen.izq, h - 6);
  ctx.fillText(hora(t1), w - margen.der - 34, h - 6);
}

// ---------- Arranque ----------
cargarPuertos();
refrescar();
setInterval(refrescar, 1000);
</script>
</body>
</html>
"""

# ============================================================================
# Arranque
# ============================================================================

if __name__ == "__main__":
    hilo = threading.Thread(target=lazo_control, daemon=True)
    hilo.start()
    print(f"Panel de control en http://localhost:{PUERTO_WEB}")
    # threaded=True para que las peticiones no bloqueen; use_reloader=False
    # para no duplicar los hilos de control.
    app.run(host="0.0.0.0", port=PUERTO_WEB, threaded=True, use_reloader=False)
