"""Catálogo de fallas inyectables y su estado (compartido por núcleo, sensores y ECU).

Los códigos P/C/U siguen la nomenclatura OBD-II; los B1Axx son códigos
propios del laboratorio para sensores ADAS (no estándar).
"""
import threading

CATALOGO = {
    'P0118': {'sistema': 'Motor', 'desc': 'Sensor de temperatura del refrigerante: señal alta'},
    'P0122': {'sistema': 'Motor', 'desc': 'Sensor de posición del acelerador: señal baja'},
    'C0035': {'sistema': 'ABS', 'desc': 'Sensor de velocidad de rueda delantera izquierda'},
    'C0051': {'sistema': 'Dirección', 'desc': 'Sensor de ángulo del volante: señal congelada'},
    'U0121': {'sistema': 'Red CAN', 'desc': 'Pérdida de comunicación con el módulo ABS'},
    'B1A01': {'sistema': 'ADAS', 'desc': 'Cámara frontal bloqueada (imagen negra)'},
    'B1A02': {'sistema': 'ADAS', 'desc': 'LiDAR degradado (pérdida de puntos y ruido)'},
    'B1A03': {'sistema': 'ADAS', 'desc': 'GNSS sin señal'},
    'B1A04': {'sistema': 'ADAS', 'desc': 'IMU con sesgo en giroscopio y acelerómetro'},
    'B1A05': {'sistema': 'ADAS', 'desc': 'Ultrasonido delantero sin respuesta'},
}

_lock = threading.Lock()
_activas = set()       # fallas inyectadas ahora
_historicas = set()    # DTC almacenados (persisten hasta borrarlos)


def activa(codigo):
    return codigo in _activas


def poner(codigo, valor):
    if codigo not in CATALOGO:
        return False
    with _lock:
        if valor:
            _activas.add(codigo)
            _historicas.add(codigo)
        else:
            _activas.discard(codigo)
    return True


def borrar_dtc():
    """Modo 04: borra los almacenados que ya no están activos."""
    with _lock:
        _historicas.intersection_update(_activas)


def estado():
    with _lock:
        activas, historicas = set(_activas), set(_historicas)
    lista = lambda cods: [{'codigo': c, **CATALOGO[c]} for c in sorted(cods)]  # noqa: E731
    return {'activos': lista(activas), 'almacenados': lista(historicas - activas),
            'mil': bool(activas)}


def codigos_dtc():
    with _lock:
        return sorted(_historicas)
