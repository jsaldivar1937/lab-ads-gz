"""ECU virtual del vehículo: genera el tráfico del bus CAN a partir de la simulación.

- Mensajes definidos en config/lab_ads.dbc (motor, dirección, velocidad,
  frenos, ruedas, estabilidad, carrocería), codificados con cantools.
- Diagnóstico OBD-II emulado en 0x7DF (petición) / 0x7E8 (respuesta):
  modo 01 (datos en vivo) y modo 03 (códigos de falla).
- Las fallas inyectadas (fallas.py) alteran las señales como lo haría un
  sensor o módulo dañado real.

Tópicos:
  /<auto>/can/rx       can_msgs/Frame   tramas crudas
  /<auto>/ecu/estado   std_msgs/String  valores decodificados + DTC (JSON)
"""
import json
import math
import random

import cantools
from can_msgs.msg import Frame
from std_msgs.msg import Header, String

import fallas
from conversiones import a_stamp

PERIODOS = {            # s (tasas típicas de un vehículo real, limitadas por el paso de simulación)
    'ENGINE_1': 0.05, 'STEERING': 0.05, 'VEHICLE_SPEED': 0.1, 'BRAKE': 0.1,
    'WHEEL_SPEEDS': 0.1, 'ESP_IMU': 0.1, 'BODY': 1.0,
}
OBD_PETICION, OBD_RESPUESTA = 0x7DF, 0x7E8
PIDS_MODO01 = [0x0C, 0x0D, 0x05, 0x11, 0x2F, 0x42, 0x01]
PERIODO_OBD = 0.5
RELLENO = 0x55


def dtc_a_bytes(codigo):
    """'P0118' → (0x01, 0x18) según SAE J2012."""
    letra = 'PCBU'.index(codigo[0])
    b1 = (letra << 6) | (int(codigo[1], 16) << 4) | int(codigo[2], 16)
    return b1, int(codigo[3:5], 16)


def trama(datos):
    datos = list(datos)[:8]
    return datos + [RELLENO] * (8 - len(datos))


class ECU:
    def __init__(self, nodo, equipo, vehiculo, ruta_dbc):
        self.db = cantools.database.load_file(ruta_dbc)
        self.mensajes = {m.name: m for m in self.db.messages}
        self.pub_can = nodo.create_publisher(Frame, f'/{equipo}/can/rx', 200)
        self.pub_estado = nodo.create_publisher(String, f'/{equipo}/ecu/estado', 10)

        fis = vehiculo.get_physics_control()
        try:
            self.relaciones = [g.ratio for g in fis.forward_gears] or [3.5]
        except (AttributeError, RuntimeError):
            self.relaciones = [3.5]
        self.final = float(getattr(fis, 'final_ratio', 0) or 4.0)
        self.rpm_max = float(getattr(fis, 'max_rpm', 0) or 6000.0)
        self.max_giro_rueda = float(fis.wheels[0].max_steer_angle)   # grados
        self.relacion_volante = 15.0

        self.refrigerante = 25.0
        self.combustible = 72.0
        self.odometro = 12345.6
        self.contador = 0
        self.ultimo = {}
        self.volante_prev = 0.0
        self.volante_congelado = None
        self.vx_prev = 0.0
        self.acel_lon = 0.0
        self.t_obd = 0.0
        self.i_obd = 0
        self.t_estado = 0.0
        self.valores = {}

    # ------------------------------------------------------------ señales
    def actualizar(self, ts, dt, vx, wz, control, omegas, radio):
        throttle, brake = float(control.throttle), float(control.brake)
        marcha = int(control.gear)
        reversa = bool(control.reverse) or marcha < 0
        vel = abs(vx) * 3.6

        rpm_rueda = abs(vx) / (2 * math.pi * radio) * 60.0
        rel = self.relaciones[min(max(marcha, 1), len(self.relaciones)) - 1]
        rpm = max(750.0 + 450.0 * throttle, rpm_rueda * rel * self.final)
        rpm = min(rpm + random.gauss(0, 8), self.rpm_max)

        self.refrigerante += ((90.0 - self.refrigerante) / 120.0 + 0.02 * throttle) * dt
        self.combustible = max(0.0, self.combustible - (0.0004 + 0.004 * throttle) * dt)
        self.odometro += abs(vx) * dt / 1000.0
        self.acel_lon = 0.8 * self.acel_lon + 0.2 * max(-12.0, min(12.0, (vx - self.vx_prev) / max(dt, 1e-3)))
        self.vx_prev = vx

        volante = -float(control.steer) * self.max_giro_rueda * self.relacion_volante  # + izquierda
        if fallas.activa('C0051'):
            if self.volante_congelado is None:
                self.volante_congelado = volante
            volante = self.volante_congelado
        else:
            self.volante_congelado = None
        tasa_volante = (volante - self.volante_prev) / max(dt, 1e-3)
        self.volante_prev = volante

        ruedas = [abs(w) * radio * 3.6 for w in omegas]
        if fallas.activa('C0035'):
            ruedas[0] = 0.0

        estado_dtc = fallas.estado()
        v = {
            'rpm': rpm,
            'acelerador': 0.0 if fallas.activa('P0122') else throttle * 100.0,
            'pedal': throttle * 100.0,
            'refrigerante': 140.0 if fallas.activa('P0118') else self.refrigerante,
            'carga': min(100.0, 15.0 + 70.0 * throttle + 0.1 * vel),
            'volante': volante, 'tasa_volante': tasa_volante,
            'velocidad': vel, 'marcha': marcha, 'reversa': reversa, 'odometro': self.odometro,
            'freno': brake * 100.0, 'presion': brake * 120.0, 'freno_mano': bool(control.hand_brake),
            'ruedas': ruedas,
            'guinada': math.degrees(wz), 'acel_lat': vx * wz, 'acel_lon': self.acel_lon,
            'bateria': 14.1 + random.gauss(0, 0.04),
            'combustible': self.combustible,
            'mil': estado_dtc['mil'], 'n_dtc': len(fallas.codigos_dtc()),
        }
        self.valores = v
        self.enviar_dbc(ts, v)
        self.enviar_obd(ts, v)

        if ts - self.t_estado >= 0.2:
            self.t_estado = ts
            redondeo = {k: (round(x, 2) if isinstance(x, float) else x) for k, x in v.items()}
            redondeo['ruedas'] = [round(x, 2) for x in ruedas]
            redondeo['dtc'] = estado_dtc
            self.pub_estado.publish(String(data=json.dumps(redondeo)))

    # ------------------------------------------------------------ DBC
    def enviar_dbc(self, ts, v):
        sin_abs = fallas.activa('U0121')
        datos = {
            'ENGINE_1': {'EngineRPM': v['rpm'], 'ThrottlePos': v['acelerador'], 'AccelPedal': v['pedal'],
                         'CoolantTemp': v['refrigerante'], 'EngineLoad': v['carga'],
                         'Counter': self.contador % 16},
            'STEERING': {'SteeringAngle': v['volante'], 'SteeringRate': v['tasa_volante']},
            'VEHICLE_SPEED': {'VehicleSpeed': v['velocidad'], 'Gear': 15 if v['reversa'] else max(v['marcha'], 0),
                              'Reverse': int(v['reversa']), 'Odometer': v['odometro']},
            'BRAKE': {'BrakePedal': v['freno'], 'BrakePressure': v['presion'], 'HandBrake': int(v['freno_mano'])},
            'WHEEL_SPEEDS': dict(zip(['WheelSpeedFL', 'WheelSpeedFR', 'WheelSpeedRL', 'WheelSpeedRR'], v['ruedas'])),
            'ESP_IMU': {'YawRate': v['guinada'], 'LatAccel': v['acel_lat'], 'LongAccel': v['acel_lon']},
            'BODY': {'BatteryVoltage': v['bateria'], 'FuelLevel': v['combustible'],
                     'MIL': int(v['mil']), 'DTCCount': v['n_dtc']},
        }
        for nombre, periodo in PERIODOS.items():
            if sin_abs and nombre in ('BRAKE', 'WHEEL_SPEEDS'):
                continue
            if ts - self.ultimo.get(nombre, -1e9) < periodo - 1e-6:
                continue
            self.ultimo[nombre] = ts
            msg = self.mensajes[nombre]
            valores = {}
            for s in msg.signals:
                x = datos[nombre].get(s.name, 0)
                lo = s.minimum if s.minimum is not None else x
                hi = s.maximum if s.maximum is not None else x
                valores[s.name] = min(max(x, lo), hi)
            self.publicar(ts, msg.frame_id, msg.encode(valores, strict=False))
            if nombre == 'ENGINE_1':
                self.contador += 1

    # ------------------------------------------------------------ OBD-II
    def enviar_obd(self, ts, v):
        if ts - self.t_obd < PERIODO_OBD:
            return
        self.t_obd = ts
        self.i_obd += 1
        if self.i_obd % 8 == 0:                       # modo 03: DTC almacenados
            codigos = fallas.codigos_dtc()[:2]
            resp = [0x43, len(fallas.codigos_dtc())]
            for c in codigos:
                resp.extend(dtc_a_bytes(c))
            self.publicar(ts, OBD_PETICION, trama([0x01, 0x03]))
            self.publicar(ts, OBD_RESPUESTA, trama([len(resp)] + resp))
            return
        pid = PIDS_MODO01[self.i_obd % len(PIDS_MODO01)]
        if pid == 0x0C:
            a = int(v['rpm'] * 4)
            datos = [a >> 8 & 0xFF, a & 0xFF]
        elif pid == 0x0D:
            datos = [min(int(v['velocidad']), 255)]
        elif pid == 0x05:
            datos = [int(min(max(v['refrigerante'] + 40, 0), 255))]
        elif pid == 0x11:
            datos = [int(v['acelerador'] * 255 / 100)]
        elif pid == 0x2F:
            datos = [int(v['combustible'] * 255 / 100)]
        elif pid == 0x42:
            mv = int(v['bateria'] * 1000)
            datos = [mv >> 8 & 0xFF, mv & 0xFF]
        else:                                         # 0x01: estado de monitores
            datos = [(0x80 if v['mil'] else 0) | min(v['n_dtc'], 0x7F), 0x07, 0x65, 0x00]
        resp = [0x41, pid] + datos
        self.publicar(ts, OBD_PETICION, trama([0x02, 0x01, pid]))
        self.publicar(ts, OBD_RESPUESTA, trama([len(resp)] + resp))

    def publicar(self, ts, can_id, datos):
        f = Frame()
        f.header = Header(stamp=a_stamp(ts), frame_id='can0')
        f.id = can_id
        f.is_rtr = f.is_extended = f.is_error = False
        f.dlc = 8
        f.data = list(bytes(datos))[:8] + [0] * max(0, 8 - len(datos))
        self.pub_can.publish(f)
