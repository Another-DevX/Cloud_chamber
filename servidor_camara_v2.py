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
import logging
import math
import threading
import time
from collections import deque

# ============================================================================
# Logging
# ============================================================================

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(name)s — %(message)s",
    datefmt="%H:%M:%S",
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler("cloud_chamber.log", encoding="utf-8"),
    ],
)
log = logging.getLogger("cloud_chamber")
log_fuente = logging.getLogger("cloud_chamber.fuente")
log_esp32  = logging.getLogger("cloud_chamber.esp32")
log_pid    = logging.getLogger("cloud_chamber.pid")


class _UILogHandler(logging.Handler):
    """Captura entradas de log recientes para exponerlas en la interfaz web."""
    def __init__(self, maxlen=300):
        super().__init__()
        self._buf = deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self.setFormatter(logging.Formatter("%(message)s"))

    def emit(self, record):
        try:
            src = record.name.replace("cloud_chamber.", "").replace("cloud_chamber", "app")
            with self._lock:
                self._buf.appendleft({
                    "t":   record.created,
                    "lvl": record.levelname,
                    "src": src,
                    "msg": self.format(record),
                })
        except Exception:
            self.handleError(record)

    def entries(self, n=200):
        with self._lock:
            return list(self._buf)[:n]


_ui_log = _UILogHandler(maxlen=400)
logging.getLogger("cloud_chamber").addHandler(_ui_log)

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
# NOTA: si el OVP de hardware está configurado en 5.5 V, ajustar V_MAX_HW
# a 5.0 para que el PID no intente comandar voltajes imposibles.
V_MAX_HW = 5.0
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
        self.canales = {}           # datos parseados del último SYNCHRO: {1: {...}, 2: {...}}

    @property
    def conectada(self):
        return self._dev is not None

    def conectar(self, ident=None):
        """
        `ident` es "bus.address" (lo que manda la interfaz). Si viene vacío,
        se busca por VID/PID como en el script original.
        """
        log_fuente.info("Intentando conectar fuente OWON (ident=%s)", ident or "auto")
        self.desconectar()

        if ident and "." in ident:
            bus, addr = (int(x) for x in ident.split("."))
            dev = usb.core.find(custom_match=lambda d: d.bus == bus and d.address == addr)
            log_fuente.debug("Buscando por bus=%d address=%d", bus, addr)
        else:
            dev = usb.core.find(idVendor=OWON_VID, idProduct=OWON_PID)
            log_fuente.debug("Buscando por VID=0x%04x PID=0x%04x", OWON_VID, OWON_PID)

        if dev is None:
            log_fuente.error("No se encontró la fuente OWON en el bus USB")
            raise RuntimeError("No se encontró la fuente OWON. Verifica el cable USB.")

        # En Linux puede haber un driver del kernel tomado; lo soltamos.
        # En Windows esto no aplica y lanza excepción, así que lo ignoramos.
        try:
            if dev.is_kernel_driver_active(0):
                log_fuente.debug("Soltando driver del kernel para interfaz 0")
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

        # Reclamar la interfaz explícitamente para poder soltarla al desconectar.
        try:
            usb.util.claim_interface(dev, 0)
        except usb.core.USBError as e:
            log_fuente.debug("claim_interface: %s (puede ser normal)", e)

        log_fuente.info("Fuente conectada: %s (bus=%d address=%d)",
                        self.etiqueta, dev.bus, dev.address)

        # Intentar desbloquear el panel frontal (REMOTE mode).
        # La ODP3032 puede entrar en modo remoto al recibir comandos USB,
        # lo que congela los botones físicos. KEYLOCK,0 lo libera (si el
        # firmware lo soporta; si no, la excepción se ignora).
        log_fuente.info("Intentando desbloquear panel frontal (KEYLOCK,0)…")
        try:
            self.enviar("KEYLOCK,0")
            log_fuente.info("Panel frontal desbloqueado (KEYLOCK,0 aceptado)")
        except Exception as e:
            log_fuente.warning(
                "KEYLOCK,0 no aceptado (%s) — el panel frontal puede quedar "
                "bloqueado en REMOTE mode mientras haya conexión USB activa", e
            )

    def desconectar(self):
        if self._dev is not None:
            log_fuente.info("Desconectando fuente (%s)", self.etiqueta)
            # Intentar restaurar el control local antes de soltar el USB.
            try:
                self.enviar("KEYLOCK,0")
                log_fuente.info("Panel frontal restaurado (KEYLOCK,0)")
            except Exception:
                pass
            try:
                usb.util.release_interface(self._dev, 0)
            except Exception:
                pass
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
        """Envía &payload,checksum# y opcionalmente lee la respuesta.

        El threading.Lock se suelta antes del sleep en comandos write-only,
        para que el lazo PID no bloquee las peticiones de la API durante
        los ~100 ms de espera del dispositivo.
        """
        if not self.conectada:
            raise RuntimeError("La fuente no está conectada")
        trama = self._enmarcar(payload)
        log_fuente.debug("TX → %s", trama.decode(errors="replace").strip())
        with self._lock:
            self._dev.write(EP_OUT, trama)
            if leer:
                # Mantener el lock durante el sleep para que otro write
                # no corrompa nuestra respuesta pendiente.
                time.sleep(0.12)
                datos = self._dev.read(EP_IN, 512, timeout=5000)
                self.ultima_respuesta = ''.join(chr(x) for x in datos).strip()
                log_fuente.debug("RX ← %s", self.ultima_respuesta)
                return self.ultima_respuesta
        # Write-only: soltar el lock ANTES del sleep para no bloquear otros hilos.
        time.sleep(0.02)
        return None

    # ---- Comandos de la especificación (ODP3032.txt) ----

    def salida(self, canal, encendida):
        # SW{CH},{0,1}: 0 activa, 1 desactiva (¡ojo, está invertido!)
        log_fuente.info("Salida CH%d → %s", canal, "ON" if encendida else "OFF")
        self.enviar(f"SW{canal},{0 if encendida else 1}")

    def set_voltaje(self, canal, voltios):
        voltios = max(0.0, min(V_MAX_HW, voltios))
        log_fuente.debug("set_voltaje CH%d → %.2f V", canal, voltios)
        self.enviar(f"SCH{canal}V,{voltios:.2f}")

    def set_corriente(self, canal, amperios):
        amperios = max(0.0, min(I_MAX_HW, amperios))
        log_fuente.info("set_corriente CH%d → %.2f A", canal, amperios)
        self.enviar(f"SCH{canal}C,{amperios:.2f}")

    def sincronizar(self):
        """SYNCHRO,0 devuelve el estado completo de la fuente.
        Formato esperado:
          $COMMON,{modo},{on},{cc_cv},{f3},{v_set},{i_set},{v_out},{i_out},{ovp},{ocp},   (x2)  checksum#
        """
        try:
            resp = self.enviar("SYNCHRO,0", leer=True)
            canales = self._parsear_synchro(resp)
            if canales:
                self.canales = canales
                for ch, c in canales.items():
                    modo = "CC" if c["cc_cv"] else "CV"
                    log_fuente.info(
                        "SYNCHRO CH%d: on=%d modo=%s v_set=%.3fV v_out=%.3fV "
                        "i_out=%.3fA ovp=%.3fV ocp=%.3fA",
                        ch, c["on"], modo,
                        c["v_set"], c["v_out"], c["i_out"], c["ovp"], c["ocp"],
                    )
            else:
                log_fuente.warning("SYNCHRO: respuesta no parseable: %r", resp)
            return resp
        except Exception as e:
            log_fuente.error("SYNCHRO falló: %s", e)
            return f"error: {e}"

    @staticmethod
    def _parsear_synchro(resp):
        """
        Parsea: $COMMON,{modo},{f1},{f2},{f3},{v_set},{i_set},{v_out},{i_out},{ovp},{ocp},
                              {f1},{f2},{f3},{v_set},{i_set},{v_out},{i_out},{ovp},{ocp},{checksum}#
        9 campos por canal; devuelve {1: dict, 2: dict} o None si falla.
        """
        try:
            inner = resp.lstrip("$").rstrip("#").strip()
            p = inner.split(",")
            # p[0]=COMMON, p[1]=modo, p[2..10]=CH1 (9 campos), p[11..19]=CH2, p[20]=checksum
            if len(p) < 21:
                return None

            def _ch(off):
                return {
                    "on":    int(p[off]),
                    "cc_cv": int(p[off + 1]),   # 0 = CV (voltaje), 1 = CC (corriente)
                    "f3":    int(p[off + 2]),
                    "v_set": float(p[off + 3]),
                    "i_set": float(p[off + 4]),
                    "v_out": float(p[off + 5]),
                    "i_out": float(p[off + 6]),
                    "ovp":   float(p[off + 7]),
                    "ocp":   float(p[off + 8]),
                }
            return {1: _ch(2), 2: _ch(11)}
        except Exception:
            return None


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
        log_esp32.info("Conectando ESP32 en %s a %d baudios", puerto, baudios)
        self.desconectar()
        self._ser = serial.Serial(puerto, baudios, timeout=2)
        self.puerto = puerto
        log_esp32.info("ESP32 conectado en %s", puerto)
        self._detener.clear()
        self._hilo = threading.Thread(target=self._bucle, daemon=True)
        self._hilo.start()

    def desconectar(self):
        if self._ser is not None:
            log_esp32.info("Desconectando ESP32 (%s)", self.puerto)
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
        log_esp32.debug("Hilo de lectura ESP32 iniciado")
        while not self._detener.is_set():
            try:
                linea = self._ser.readline().decode(errors="replace").strip()
            except Exception as exc:
                # El puerto se cayó (desconectaron el cable, etc.)
                log_esp32.error("Error leyendo del ESP32: %s", exc)
                self.estado.registrar_error("Se perdió la conexión con el ESP32")
                break
            if not linea:
                continue
            try:
                dato = json.loads(linea)
            except json.JSONDecodeError:
                log_esp32.warning("Línea no parseable del ESP32: %r", linea)
                continue  # línea corrupta, ignorar
            t_cold = dato.get("t_cold")
            t_hot = dato.get("t_hot")
            log_esp32.debug("Muestra recibida: t_cold=%s °C  t_hot=%s °C", t_cold, t_hot)
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
        self.pid_cold = PID(kp=1.5, ki=0.05, kd=2.0, out_min=0.0, out_max=4.5)
        self.pid_hot = PID(kp=1.0, ki=0.03, kd=1.0, out_min=0.0, out_max=4.5)
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
    if motivo:
        log_pid.warning("apagar_salidas: %s", motivo)
    else:
        log_pid.info("apagar_salidas llamado")
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
            log_pid.warning("Watchdog disparado: última muestra hace %.1f s",
                            ahora - estado.t_dato)
            apagar_salidas("WATCHDOG: sin datos de temperatura, salidas apagadas")
            continue

        if not fuente.conectada:
            if pid_activo:
                log_pid.error("Fuente desconectada con PID activo; desactivando PID")
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
                log_pid.debug("PID FRÍO  t=%.2f°C sp=%.2f°C err=%+.2f v_ramp=%.3f V",
                              estado.t_cold, estado.sp_cold, err, v)
                estado.v_cold = v
                fuente.set_voltaje(CANAL_FRIO, v)

            # --- Zona caliente: más voltaje en el calefactor => sube la temp.
            # Error positivo cuando está más fría que el setpoint.
            if estado.pid_hot_on and estado.t_hot is not None:
                err = estado.sp_hot - estado.t_hot
                v = estado.pid_hot.update(err, estado.t_hot, dt)
                v = _rampa(estado.v_hot, v, dt)
                log_pid.debug("PID CALOR t=%.2f°C sp=%.2f°C err=%+.2f v_ramp=%.3f V",
                              estado.t_hot, estado.sp_hot, err, v)
                estado.v_hot = v
                fuente.set_voltaje(CANAL_CALIENTE, v)

            # --- Consulta periódica de la fuente (cruda, ver nota en driver)
            if ahora - ultimo_synchro > 10.0:
                ultimo_synchro = ahora
                fuente.sincronizar()
                # Diagnóstico: verificar que la fuente aplicó los voltajes
                for canal, zona, v_cmd in [
                    (CANAL_FRIO,     "FRÍO",  estado.v_cold),
                    (CANAL_CALIENTE, "CALOR", estado.v_hot),
                ]:
                    info = fuente.canales.get(canal)
                    if info is None:
                        continue
                    # ¿Comando de voltaje ignorado?
                    if v_cmd > 0.05 and abs(info["v_set"] - v_cmd) > 0.2:
                        log_pid.warning(
                            "DIAG CH%d %s: v_cmd=%.2fV pero fuente v_set=%.3fV "
                            "— comando SCH%dV posiblemente ignorado",
                            canal, zona, v_cmd, info["v_set"], canal,
                        )
                    # ¿OVP demasiado bajo para el setpoint PID?
                    pid = estado.pid_cold if canal == CANAL_FRIO else estado.pid_hot
                    if info["ovp"] > 0 and pid.out_max > info["ovp"]:
                        log_pid.warning(
                            "DIAG CH%d %s: OVP=%.1fV < vmax_pid=%.1fV "
                            "— la fuente cortará la salida al superar la protección",
                            canal, zona, info["ovp"], pid.out_max,
                        )
                    # ¿Modo CC cuando se esperaba CV?
                    if info["on"] and info["cc_cv"] == 1:
                        log_pid.warning(
                            "DIAG CH%d %s: fuente en modo CC (limitada por corriente), "
                            "v_out=%.3fV < v_set=%.3fV — verificar límite de corriente",
                            canal, zona, info["v_out"], info["v_set"],
                        )

        except Exception as e:
            log_pid.exception("Excepción en el lazo de control")
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
    log.info("API /conectar — dispositivo=%s puerto=%s", dispositivo, puerto)
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
            log.info("Fuente inicializada con i_lim_cold=%.2f A  i_lim_hot=%.2f A",
                     estado.i_lim_cold, estado.i_lim_hot)
        else:
            log.warning("API /conectar — dispositivo desconocido: %s", dispositivo)
            return jsonify({"ok": False, "error": "dispositivo desconocido"}), 400
    except Exception as e:
        log.exception("Error al conectar %s", dispositivo)
        return jsonify({"ok": False, "error": str(e)}), 500
    return jsonify({"ok": True})


@app.post("/api/desconectar")
def api_desconectar():
    datos = request.get_json(force=True)
    dispositivo = datos.get("dispositivo")
    log.info("API /desconectar — dispositivo=%s", dispositivo)
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
            log.info("Setpoint FRÍO → %.2f°C", estado.sp_cold)
        if "hot" in datos:
            estado.sp_hot = float(datos["hot"])
            log.info("Setpoint CALOR → %.2f°C", estado.sp_hot)
    return jsonify({"ok": True})


@app.post("/api/pid")
def api_pid():
    """Configura ganancias/límites y activa o desactiva cada lazo."""
    datos = request.get_json(force=True)
    zona = datos.get("zona")
    pid = estado.pid_cold if zona == "cold" else estado.pid_hot

    for campo in ("kp", "ki", "kd"):
        if campo in datos:
            nuevo = float(datos[campo])
            log_pid.info("PID %s: %s = %.4f", zona.upper(), campo, nuevo)
            setattr(pid, campo, nuevo)
    if "vmax" in datos:
        pid.out_max = max(0.0, min(V_MAX_HW, float(datos["vmax"])))
        log_pid.info("PID %s: vmax = %.2f V", zona.upper(), pid.out_max)
    if "imax" in datos:
        imax = max(0.0, min(I_MAX_HW, float(datos["imax"])))
        log_pid.info("PID %s: imax = %.2f A", zona.upper(), imax)
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
        log_pid.info("PID %s: solicitud activo=%s", zona.upper(), activo)
        if activo:
            if not fuente.conectada:
                log_pid.warning("PID %s: no se puede activar, fuente no conectada", zona.upper())
                return jsonify({"ok": False, "error": "Conecta la fuente primero"}), 400
            if (time.time() - estado.t_dato) > WATCHDOG_S:
                log_pid.warning("PID %s: no se puede activar, sin datos de temperatura", zona.upper())
                return jsonify({"ok": False,
                                "error": "No hay datos de temperatura recientes"}), 400
            pid.reset()
            try:
                fuente.set_voltaje(canal, 0.0)
                fuente.salida(canal, True)
            except Exception as e:
                log_pid.exception("Error activando PID %s", zona.upper())
                return jsonify({"ok": False, "error": str(e)}), 500
            if zona == "cold":
                estado.v_cold = 0.0
                estado.pid_cold_on = True
            else:
                estado.v_hot = 0.0
                estado.pid_hot_on = True
            log_pid.info("PID %s activado", zona.upper())
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
                    log_pid.exception("Error desactivando PID %s", zona.upper())
                    return jsonify({"ok": False, "error": str(e)}), 500
            log_pid.info("PID %s desactivado", zona.upper())
    return jsonify({"ok": True})


@app.post("/api/parada")
def api_parada():
    log.warning("API /parada — parada de emergencia solicitada")
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
                       "respuesta_cruda": fuente.ultima_respuesta,
                       "canales": fuente.canales},
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


@app.get("/api/logs")
def api_logs():
    """Devuelve las últimas entradas de log capturadas en memoria."""
    n = min(int(request.args.get("n", 200)), 400)
    return jsonify({"logs": _ui_log.entries(n)})


@app.post("/api/cmd")
def api_cmd():
    """Envía un comando crudo a la fuente (para depuración)."""
    datos = request.get_json(force=True)
    payload = (datos.get("payload") or "").strip()
    leer = bool(datos.get("leer", True))
    if not payload:
        return jsonify({"ok": False, "error": "payload vacío"}), 400
    if not fuente.conectada:
        return jsonify({"ok": False, "error": "Fuente no conectada"}), 400
    log_fuente.info("CMD manual → payload=%r leer=%s", payload, leer)
    try:
        resp = fuente.enviar(payload, leer=leer)
        return jsonify({"ok": True, "respuesta": resp or "(sin respuesta)"})
    except Exception as e:
        log_fuente.error("CMD manual falló: %s", e)
        return jsonify({"ok": False, "error": str(e)}), 500


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

  /* --- Panel de logs --- */
  .log-panel {
    margin-top: 14px; background: var(--panel); border: 1px solid var(--panel-borde);
    border-radius: 12px; overflow: hidden;
  }
  .log-header {
    display: flex; align-items: center; gap: 8px; padding: 8px 14px;
    background: #0d1220; border-bottom: 1px solid var(--panel-borde);
    font-size: 12px; font-family: var(--mono);
  }
  .log-header span { color: var(--tenue); margin-right: auto; }
  .log-filter {
    padding: 2px 8px; font-size: 11px; border-radius: 4px;
    background: #1f2942; color: var(--tenue); border: 1px solid #2a3a5a;
  }
  .log-filter.activo { background: #2a3a6a; color: var(--texto); border-color: #4a6aaa; }
  .log-entries {
    height: 220px; overflow-y: auto; font-family: var(--mono);
    font-size: 12px; padding: 6px 0;
  }
  .log-entries .le {
    padding: 1px 14px; display: grid;
    grid-template-columns: 6.5em 4.5em 5em 1fr; gap: 6px; align-items: baseline;
    border-bottom: 1px solid rgba(255,255,255,0.03);
  }
  .le:hover { background: rgba(255,255,255,0.03); }
  .le .lt { color: #5a6880; }
  .le .ls { color: #7090b0; }
  .le .ll { font-weight: 600; font-size: 10px; padding: 1px 4px; border-radius: 3px; }
  .ll.DEBUG   { color: #5a6880; }
  .ll.INFO    { color: #7bd88f; background: rgba(123,216,143,0.08); }
  .ll.WARNING { color: #f2c94c; background: rgba(242,201,76,0.12); }
  .ll.ERROR, .ll.CRITICAL { color: #e86a6a; background: rgba(232,106,106,0.12); }
  .le .lm { color: var(--texto); white-space: pre-wrap; word-break: break-all; }

  /* --- Consola de comandos crudos --- */
  .debug-consola {
    margin-top: 14px; background: var(--panel); border: 1px solid var(--panel-borde);
    border-radius: 12px; padding: 0;
  }
  .debug-consola summary {
    padding: 10px 16px; cursor: pointer; font-size: 12.5px; color: var(--tenue);
    font-family: var(--mono); user-select: none; list-style: none;
  }
  .debug-consola summary::before { content: "▶  "; font-size: 10px; }
  .debug-consola[open] summary::before { content: "▼  "; }
  .debug-consola .debug-body { padding: 0 16px 14px; display: flex; flex-direction: column; gap: 8px; }
  .debug-consola .fila-cmd { display: flex; gap: 8px; align-items: center; }
  .debug-consola input[type=text] {
    flex: 1; background: #0d1220; border: 1px solid var(--panel-borde);
    color: var(--texto); border-radius: 6px; padding: 6px 10px;
    font-family: var(--mono); font-size: 13px;
  }
  .debug-consola label { font-size: 12px; color: var(--tenue); white-space: nowrap; }
  .cmd-resp {
    background: #0d1220; border: 1px solid var(--panel-borde); border-radius: 6px;
    padding: 8px 12px; font-family: var(--mono); font-size: 12px; color: #7bd88f;
    min-height: 2em; white-space: pre-wrap; word-break: break-all;
  }

  /* --- Medidas reales de la fuente en las zonas --- */
  .medidas-fuente {
    display: flex; flex-wrap: wrap; gap: 6px 14px; margin-top: 6px;
    padding: 6px 10px; background: rgba(0,0,0,0.2); border-radius: 8px;
    font-size: 12px; font-family: var(--mono); color: var(--tenue);
  }
  .medidas-fuente b { font-weight: 600; }
  .medidas-fuente .mf-alerta { color: var(--alerta); }

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
      <div class="medidas-fuente" id="mf-cold">
        <span>v_set <b id="mf-vset-cold">—</b></span>
        <span>v_out <b id="mf-vout-cold">—</b></span>
        <span>i_out <b id="mf-iout-cold">—</b></span>
        <span>modo <b id="mf-modo-cold">—</b></span>
        <span>OVP <b id="mf-ovp-cold">—</b></span>
      </div>

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
      <div class="medidas-fuente" id="mf-hot">
        <span>v_set <b id="mf-vset-hot">—</b></span>
        <span>v_out <b id="mf-vout-hot">—</b></span>
        <span>i_out <b id="mf-iout-hot">—</b></span>
        <span>modo <b id="mf-modo-hot">—</b></span>
        <span>OVP <b id="mf-ovp-hot">—</b></span>
      </div>
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

  <!-- Panel de logs -->
  <div class="log-panel">
    <div class="log-header">
      <span>Log en tiempo real</span>
      <button class="log-filter activo" data-lvl="">TODO</button>
      <button class="log-filter" data-lvl="INFO">INFO+</button>
      <button class="log-filter" data-lvl="WARNING">WARN+</button>
      <button id="log-clear" style="margin-left:8px;font-size:11px;padding:2px 8px;
        border-radius:4px;background:#1f2942;color:#8a96ab;border:1px solid #2a3a5a;">Limpiar</button>
    </div>
    <div class="log-entries" id="log-entries"><div style="padding:8px 14px;color:#5a6880">Esperando logs…</div></div>
  </div>

  <!-- Consola de comandos crudos -->
  <details class="debug-consola">
    <summary>Consola de comandos crudos (debug fuente)</summary>
    <div class="debug-body">
      <div class="fila-cmd">
        <input type="text" id="cmd-payload" placeholder="SCH1V,5.00  ó  SYNCHRO,0">
        <label><input type="checkbox" id="cmd-leer" checked> Leer resp.</label>
        <button id="cmd-enviar">Enviar</button>
      </div>
      <div class="cmd-resp" id="cmd-resp">(respuesta aparece aquí)</div>
    </div>
  </details>

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

// ---------- Datos reales de la fuente (SYNCHRO) ----------
const CANAL_ZONA = { cold: 1, hot: 2 };
function pintarFuente(zona, canales) {
  if (!canales) return;
  const ch = canales[CANAL_ZONA[zona]];
  if (!ch) return;
  const modo = ch.cc_cv === 1 ? "CC" : "CV";
  const vCmd = ultimoEstado ? ultimoEstado.zonas[zona].v : 0;
  const desfase = Math.abs(ch.v_set - vCmd) > 0.2 && vCmd > 0.05;
  const ovpBajo = ch.ovp > 0 && ultimoEstado &&
        ch.ovp < ultimoEstado.zonas[zona].vmax;

  $("mf-vset-" + zona).textContent = ch.v_set.toFixed(3) + " V";
  $("mf-vset-" + zona).className = desfase ? "mf-alerta" : "";
  $("mf-vout-" + zona).textContent = ch.v_out.toFixed(3) + " V";
  $("mf-iout-" + zona).textContent = ch.i_out.toFixed(3) + " A";
  $("mf-modo-" + zona).textContent = modo;
  $("mf-modo-" + zona).className = ch.cc_cv === 1 ? "mf-alerta" : "";
  const ovpEl = $("mf-ovp-" + zona);
  ovpEl.textContent = ch.ovp.toFixed(1) + " V";
  ovpEl.className = ovpBajo ? "mf-alerta" : "";
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
  if (d.fuente.canales) {
    pintarFuente("cold", d.fuente.canales);
    pintarFuente("hot",  d.fuente.canales);
  }

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

// ---------- Panel de logs ----------
let _logFiltro = "";       // nivel mínimo: "" = todo, "INFO", "WARNING", "ERROR"
let _logBuf = [];           // buffer local para no re-pedir todo en cada tick
let _logPausado = false;    // true cuando el usuario hace scroll hacia arriba

const NIVELES = ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"];
function nivelNum(s) { return NIVELES.indexOf(s); }

const logEl = $("log-entries");
logEl.addEventListener("scroll", () => {
  // Si el usuario sube el scroll pausamos el auto-scroll
  const enFondo = logEl.scrollTop + logEl.clientHeight >= logEl.scrollHeight - 20;
  _logPausado = !enFondo;
});

function renderLogs() {
  const minNivel = nivelNum(_logFiltro || "DEBUG");
  const filtrados = _logBuf.filter(e => nivelNum(e.lvl) >= minNivel);
  logEl.innerHTML = filtrados.map(e => {
    const hora = new Date(e.t * 1000).toLocaleTimeString();
    return `<div class="le">
      <span class="lt">${hora}</span>
      <span class="ll ${e.lvl}">${e.lvl}</span>
      <span class="ls">${e.src}</span>
      <span class="lm">${e.msg.replace(/</g,"&lt;")}</span>
    </div>`;
  }).join("") || '<div style="padding:8px 14px;color:#5a6880">Sin entradas para este filtro.</div>';
  if (!_logPausado) logEl.scrollTop = logEl.scrollHeight;
}

async function fetchLogs() {
  try {
    const d = await (await fetch("/api/logs?n=200")).json();
    _logBuf = d.logs || [];
    renderLogs();
  } catch {}
}
fetchLogs();
setInterval(fetchLogs, 2000);

// Filtros
document.querySelectorAll(".log-filter").forEach(btn => {
  btn.onclick = () => {
    document.querySelectorAll(".log-filter").forEach(b => b.classList.remove("activo"));
    btn.classList.add("activo");
    _logFiltro = btn.dataset.lvl;
    _logPausado = false;
    renderLogs();
  };
});
$("log-clear").onclick = () => { _logBuf = []; renderLogs(); };

// ---------- Consola de comandos crudos ----------
$("cmd-enviar").onclick = async () => {
  const payload = $("cmd-payload").value.trim();
  const leer = $("cmd-leer").checked;
  if (!payload) return;
  const resp = $("cmd-resp");
  resp.textContent = "enviando…";
  resp.style.color = "#8a96ab";
  try {
    const d = await api("/api/cmd", { payload, leer });
    if (d.ok) {
      resp.textContent = d.respuesta || "(sin respuesta)";
      resp.style.color = "#7bd88f";
    } else {
      resp.textContent = "ERROR: " + d.error;
      resp.style.color = "#e86a6a";
    }
  } catch (e) {
    resp.textContent = "Error de red: " + e;
    resp.style.color = "#e86a6a";
  }
};
$("cmd-payload").addEventListener("keydown", e => {
  if (e.key === "Enter") $("cmd-enviar").click();
});
</script>
</body>
</html>
"""

# ============================================================================
# Arranque
# ============================================================================

if __name__ == "__main__":
    log.info("Iniciando servidor de la cámara de niebla (puerto web %d)", PUERTO_WEB)
    hilo = threading.Thread(target=lazo_control, daemon=True)
    hilo.start()
    log.info("Hilo de control PID iniciado")
    print(f"Panel de control en http://localhost:{PUERTO_WEB}")
    # threaded=True para que las peticiones no bloqueen; use_reloader=False
    # para no duplicar los hilos de control.
    app.run(host="0.0.0.0", port=PUERTO_WEB, threaded=True, use_reloader=False)
