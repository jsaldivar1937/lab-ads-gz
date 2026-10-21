"""Geometría 2D de la ciudad para los sensores emulados (ultrasonido y radar)
y para detectar invasión de carril. Todo vectorizado con numpy."""
import json
import math

import numpy as np


class Geometria:
    def __init__(self, ruta_json):
        d = json.loads(open(ruta_json, encoding='utf-8').read())
        self.datos = d
        obs = d['obstaculos']
        self.estaticos = np.array([[o['x'], o['y'], o['yaw'], o['sx'] / 2, o['sy'] / 2] for o in obs],
                                  dtype=float).reshape(-1, 5)
        self.marcas = d['marcas']
        self.seg = np.array([m[:4] for m in self.marcas], dtype=float).reshape(-1, 4)
        self.rutas = {k: np.array(v, dtype=float) for k, v in d['rutas'].items()}

    # ------------------------------------------------------------ rayos
    @staticmethod
    def rayos_vs_cajas(orig, dirs, cajas, alcance):
        """orig, dirs: (N,2); cajas: (B,5) [cx, cy, yaw, hx, hy].
        Devuelve (distancia (N,), índice de caja (N,) o -1)."""
        n = len(orig)
        if len(cajas) == 0:
            return np.full(n, np.inf), np.full(n, -1)
        cx, cy, yaw, hx, hy = (cajas[:, i][None, :] for i in range(5))
        c, s = np.cos(yaw), np.sin(yaw)
        ox = orig[:, 0:1] - cx
        oy = orig[:, 1:2] - cy
        px, py = c * ox + s * oy, -s * ox + c * oy                    # origen en marco de la caja
        dx, dy = c * dirs[:, 0:1] + s * dirs[:, 1:2], -s * dirs[:, 0:1] + c * dirs[:, 1:2]
        eps = 1e-9
        dx = np.where(np.abs(dx) < eps, eps, dx)
        dy = np.where(np.abs(dy) < eps, eps, dy)
        t1x, t2x = (-hx - px) / dx, (hx - px) / dx
        t1y, t2y = (-hy - py) / dy, (hy - py) / dy
        tmin = np.maximum(np.minimum(t1x, t2x), np.minimum(t1y, t2y))
        tmax = np.minimum(np.maximum(t1x, t2x), np.maximum(t1y, t2y))
        golpe = (tmax >= np.maximum(tmin, 0.0))
        t = np.where(tmin > 0, tmin, 0.0)
        t = np.where(golpe & (t <= alcance), t, np.inf)
        idx = np.argmin(t, axis=1)
        dist = t[np.arange(n), idx]
        idx = np.where(np.isfinite(dist), idx, -1)
        return dist, idx

    def rayos(self, orig, dirs, alcance, dinamicas=None):
        """Rayos contra obstáculos fijos y cajas dinámicas (autos). Devuelve (dist, idx, es_dinamica)."""
        cajas = self.estaticos
        nest = len(cajas)
        if dinamicas is not None and len(dinamicas):
            cajas = np.vstack([cajas, dinamicas])
        dist, idx = self.rayos_vs_cajas(orig, dirs, cajas, alcance)
        return dist, idx, idx >= nest, nest

    # ------------------------------------------------------------ marcas viales
    def cruces_de_marca(self, p0, p1):
        """Índices de marcas cruzadas por los segmentos p0[i]→p1[i] (esquinas del auto)."""
        if len(self.seg) == 0:
            return set()
        cruzadas = set()
        a, b = self.seg[:, 0:2], self.seg[:, 2:4]
        for q0, q1 in zip(p0, p1):
            if np.allclose(q0, q1):
                continue
            r = q1 - q0
            s = b - a
            den = r[0] * s[:, 1] - r[1] * s[:, 0]
            ok = np.abs(den) > 1e-9
            qa = a - q0
            t = np.where(ok, (qa[:, 0] * s[:, 1] - qa[:, 1] * s[:, 0]) / np.where(ok, den, 1), -1)
            u = np.where(ok, (qa[:, 0] * r[1] - qa[:, 1] * r[0]) / np.where(ok, den, 1), -1)
            hits = np.nonzero(ok & (t >= 0) & (t <= 1) & (u >= 0) & (u <= 1))[0]
            cruzadas.update(int(h) for h in hits)
        return cruzadas


def esquinas(x, y, yaw, largo=4.5, ancho=1.85):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for lx, ly in ((largo / 2, ancho / 2), (largo / 2, -ancho / 2), (-largo / 2, ancho / 2), (-largo / 2, -ancho / 2)):
        pts.append([x + c * lx - s * ly, y + s * lx + c * ly])
    return np.array(pts)
