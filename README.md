# Cloud Chamber

Servidor local para monitorizar y controlar térmicamente una cámara de niebla
de difusión. Lee dos termocuplas desde un ESP32 y controla una fuente OWON
ODP3032 mediante USB:

- CH1: Peltier de la zona fría.
- CH2: calentador de la zona caliente.
- Dos controladores PID independientes.
- Interfaz web local para configurar, monitorizar y diagnosticar el sistema.

El servidor principal está implementado en `servidor_camara_v2.py` y escucha
por defecto en `http://localhost:8000`.

## Requisitos

- Python 3.
- Flask.
- pyserial.
- PyUSB y un backend libusb compatible.
- Acceso al dispositivo USB de la OWON.

Instalación básica de dependencias:

```bash
pip install flask pyserial pyusb
```

En Windows puede ser necesario instalar/configurar libusb para la OWON. El
driver reconoce los formatos de timeout utilizados tanto por libusb1 como por
libusb0.

## Ejecución

```bash
python servidor_camara_v2.py
```

Después, abrir `http://localhost:8000`, conectar el ESP32 y la OWON desde la
interfaz y activar por separado los controles frío y caliente.

Cuando se reemplace el archivo en otro equipo, hay que cerrar completamente el
proceso Python anterior y volver a iniciarlo. Actualizar únicamente el navegador
no carga los cambios del servidor.

## Logging

El sistema registra actividad simultáneamente en la consola y en
`cloud_chamber.log`.

- Loggers separados para aplicación, fuente, ESP32 y PID.
- Niveles `DEBUG`, `INFO`, `WARNING` y `ERROR`.
- Registro de muestras, comandos USB, respuestas, estados PID, watchdog y
  diagnósticos de protección.
- `_UILogHandler` conserva los eventos recientes en memoria.
- `/api/logs` expone esos eventos a la interfaz web.

## Driver de la OWON

`FuenteOWON` controla la ODP3032 directamente mediante PyUSB. La fuente no se
trata como puerto serie: se utilizan endpoints USB bulk.

Las órdenes tienen este formato:

```text
&comando,argumentos,checksum#
```

El driver incluye:

- `claim_interface()` al conectar.
- `release_interface()` y `dispose_resources()` al desconectar.
- Un lock para evitar que distintos hilos mezclen comandos y respuestas.
- Límites de voltaje y corriente antes de transmitir una orden.
- Cálculo automático del checksum del protocolo.

### Escritura y ACK

Los setters (`SCH1V`, `SCH2V`, `SCH1C`, `SCH2C`, `SW1` y `SW2`) se envían sin
esperar un ACK. Algunos firmwares no confirman estas órdenes y esperar una
respuesta provocaba timeouts de libusb0 en Windows, abortando la activación del
PID antes de comenzar a enviar voltajes.

Antes de una consulta que sí requiere respuesta, como `SYNCHRO`, el driver
descarta ACK antiguos del endpoint de entrada. El timeout corto usado para
detectar que el endpoint ya está vacío es normal y no representa un fallo de
comunicación.

`_es_timeout_usb()` reconoce:

- `USBTimeoutError`.
- Errores 60 y 110.
- Errores genéricos de libusb0 cuyo texto contiene `timeout`.

### KEYLOCK y panel frontal

No se envía `KEYLOCK,0` automáticamente al conectar, periódicamente ni al
desconectar. La semántica puede variar entre versiones de firmware y el comando
podía mantener el panel bloqueado.

La fuente todavía puede mostrar modo remoto o limitar los botones mientras
recibe tráfico USB; esto puede ser comportamiento normal de la ODP3032.

### SYNCHRO

`sincronizar()` consulta el estado completo de la fuente:

- Reintenta cuando recibe `$ACK,2` (dispositivo ocupado).
- Se ejecuta después de una espera preventiva de 150 ms.
- `_parsear_synchro()` convierte `$COMMON,...` en datos independientes para
  cada canal.

Los datos incluyen:

- Estado de la salida.
- Modo CC/CV.
- Voltaje y corriente configurados.
- Voltaje y corriente reales.
- OVP y OCP.

En la respuesta `SYNCHRO`, `on=1` significa salida activa y `on=0` salida
apagada.

## Control térmico

El servidor ejecuta dos PID independientes:

- Zona fría: cuanto mayor es el voltaje de CH1, mayor es el enfriamiento de la
  Peltier.
- Zona caliente: cuanto mayor es el voltaje de CH2, mayor es el calentamiento.

Los PID incluyen:

- Salida unidireccional.
- Anti-windup por clamping.
- Derivada sobre la medición.
- Límites configurables desde la interfaz.
- Rampa máxima de `0.5 V/s`.

El watchdog apaga ambas salidas y desactiva los PID si dejan de llegar datos de
temperatura durante más de cinco segundos.

Cada diez segundos se consulta y diagnostica el estado real de la fuente:

- Salida apagada o posible disparo de OCP.
- Voltaje configurado con salida real en cero.
- OVP inferior al máximo del PID.
- Comando de voltaje aparentemente rechazado.

## Límites eléctricos

La configuración actual es:

```python
V_MAX_HW = 12.0
pid_cold.out_max = 12.0
pid_hot.out_max = 4.5
```

El máximo de 12 V corresponde a la tensión nominal de la Peltier conectada a
CH1. Debido a la rampa de `0.5 V/s`, subir de 0 a 12 V tarda aproximadamente 24
segundos.

### Advertencia sobre OVP

Los registros mostraron una OVP de 5.5 V en CH1. Para alcanzar 12 V, la OVP de
ese canal debe configurarse por encima de 12 V, por ejemplo entre 12.5 y 13 V.
Solo debe hacerse si la Peltier, el cableado, las conexiones y la refrigeración
soportan la tensión y corriente correspondientes.

Si OVP permanece en 5.5 V, la fuente cortará la salida antes de alcanzar el
máximo solicitado por el PID.

## API

La aplicación ofrece, entre otros, los siguientes recursos:

- `/api/estado`: estado del ESP32, PID y datos reales de la fuente obtenidos
  mediante `SYNCHRO`.
- `/api/logs`: logs recientes almacenados en memoria.
- `/api/cmd`: envío manual de comandos crudos a la OWON.

Los endpoints permiten configurar setpoints, ganancias PID, voltaje máximo,
corriente máxima y activación independiente de cada zona.

## Interfaz web

La interfaz muestra por canal:

- `v_set`.
- `v_out`.
- `i_out`.
- Modo CC/CV.
- OVP y OCP.
- Estado y parámetros del PID.

También incluye:

- Alertas visuales cuando OVP es inferior al máximo solicitado.
- Panel de logs con filtros y desplazamiento automático.
- Consola de comandos crudos con la respuesta en pantalla.
- Controles independientes para las zonas fría y caliente.

## Diagnóstico rápido

Con el PID activo deberían aparecer órdenes como:

```text
PID FRÍO ... v_ramp=...
TX → &SCH1V,...
PID CALOR ... v_ramp=...
TX → &SCH2V,...
```

No es necesario recibir un `RX` después de cada setter. Aproximadamente cada
diez segundos sí debería aparecer:

```text
TX → &SYNCHRO,0,...
RX ← $COMMON,...
```

Si el traceback contiene todavía esta expresión, se está ejecutando una versión
antigua del servidor:

```python
timeout=5000 if leer else 250
```
