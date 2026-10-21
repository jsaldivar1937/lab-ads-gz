/* Laboratorio Virtual ADS — visualizador web (un vehículo, muchos espectadores)
 * Telemetría por Server-Sent Events (/api/stream); cámara por MJPEG;
 * LiDAR y radar por /api/nube (float32); comandos por /api/cmd con permiso.
 */
'use strict';

const S = {
  cfg: null, token: null, camara: null, control: {},
  estado: null, tray: [], mapa: null, fuente: null,
  teclas: {}, giro: 0, reversa: false, conducirYo: false,
  pestana: 'conduccion', lab: {}, ecu: null, canPausa: false, canFiltro: '',
};
const $ = (id) => document.getElementById(id);
const VISTA = document.body.dataset.vista === 'profesor' ? 'profesor' : 'alumno';
const K = (clave) => `lab_${VISTA}_${clave}`;   // cada vista guarda su propia sesión
const rgb = (s) => `rgb(${s})`;
const MAX_TRAY = 2000;

/* ======================================================= utilidades */
async function api(ruta, datos) {
  const op = datos === undefined ? {} : {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ token: S.token, ...datos }),
  };
  const r = await fetch(ruta, op);
  let cuerpo = {};
  try { cuerpo = await r.json(); } catch (e) { /* sin cuerpo */ }
  return { ok: r.ok, status: r.status, ...cuerpo };
}

function guardar(clave, valor, sesion = false) {
  try { (sesion ? sessionStorage : localStorage).setItem(clave, valor); } catch (e) { /* privado */ }
}
function leer(clave, sesion = false) {
  try { return (sesion ? sessionStorage : localStorage).getItem(clave); } catch (e) { return null; }
}

/* ======================================================= inicio */
async function iniciar() {
  S.cfg = await (await fetch('api/config')).json();
  $('sub-mapa').textContent = `${S.cfg.simulador || 'ROS 2'} · ${S.cfg.mapa}`;
  document.documentElement.style.setProperty('--eq', rgb(S.cfg.color));

  const selCam = $('sel-camara');
  for (const c of S.cfg.camaras) selCam.add(new Option(`${c.nombre} (${c.tipo})`, c.nombre));
  S.camara = S.cfg.camaras.length ? S.cfg.camaras[0].nombre : null;
  selCam.onchange = () => { S.camara = selCam.value; actualizarVideo(); };

  const selClima = $('sel-clima');
  for (const c of S.cfg.climas) selClima.add(new Option(c, c));
  selClima.value = S.cfg.clima;
  selClima.onchange = () => api('api/admin/accion', { accion: 'clima', valor: selClima.value });

  $('btn-auto').onclick = () => api('api/admin/accion', {
    accion: 'autopilot', valor: !(S.estado && S.estado.modo === 'autopilot') });
  $('btn-reset').onclick = () => { S.tray = []; api('api/admin/accion', { accion: 'reset' }); };
  $('btn-quitar').onclick = () => api('api/admin/control', { uid: null });
  $('btn-conducir-yo').onclick = () => {
    S.conducirYo = !S.conducirYo;
    $('btn-conducir-yo').classList.toggle('on', S.conducirYo);
    $('btn-conducir-yo').textContent = S.conducirYo ? 'Dejar de conducir' : 'Conducir yo';
    aplicarRol();
  };
  $('btn-vista').onclick = vistaSuperior;
  $('form-entrada').onsubmit = entrar;
  $('btn-salir').onclick = salir;

  iniciar3D();
  iniciarMapa();
  iniciarGraficas();
  iniciarMandos();
  iniciarPestanas();
  iniciarSensores();
  iniciarECU();
  cargarMapa();

  S.token = leer(K('token'));
  const yo = S.token ? await api(`api/yo?token=${encodeURIComponent(S.token)}`) : { ok: false };
  if (yo.ok && (VISTA === 'alumno' || yo.rol === 'profesor')) arrancar();
  else if (VISTA === 'profesor' && leer(K('clave'))) reingresar();
  else mostrarEntrada();

  setInterval(actualizarNubes, 500);
  setInterval(cicloManejo, 100);
  requestAnimationFrame(dibujar);
}

function mostrarEntrada(error = '') {
  $('entrada').hidden = false;
  $('error-entrada').textContent = error;
  const campo = VISTA === 'profesor' ? $('in-usuario') : $('in-nombre');
  campo.value = leer(K('nombre')) || campo.value;
  campo.focus();
}

/* Alumno: solo nombre. Profesor: usuario y clave (se recuerdan en esta computadora). */
async function iniciarSesion(nombre, usuario, clave) {
  const r = await api('api/unirse', { nombre });
  if (!r.ok) return 'No se pudo entrar; intenta de nuevo.';
  S.token = r.token;
  if (VISTA === 'profesor') {
    const p = await api('api/profesor', { usuario, clave });
    if (!p.ok) return 'Usuario o clave incorrectos.';
    guardar(K('clave'), clave);
  }
  guardar(K('token'), r.token);
  guardar(K('nombre'), VISTA === 'profesor' ? usuario : nombre);
  return null;
}

async function entrar(ev) {
  ev.preventDefault();
  let error;
  if (VISTA === 'profesor') {
    const usuario = $('in-usuario').value.trim();
    error = await iniciarSesion(`Prof. ${usuario}`, usuario, $('in-clave').value);
  } else {
    const nombre = $('in-nombre').value.trim();
    error = await iniciarSesion(nombre);
  }
  if (error) { $('error-entrada').textContent = error; return; }
  arrancar();
}

/* Si el servidor se reinició, el token deja de valer: volver a entrar en silencio */
async function reingresar() {
  const nombre = leer(K('nombre'));
  if (!nombre) { mostrarEntrada(); return; }
  const error = VISTA === 'profesor'
    ? await iniciarSesion(`Prof. ${nombre}`, nombre, leer(K('clave')) || '')
    : await iniciarSesion(nombre);
  if (error) mostrarEntrada(error);
  else arrancar();
}

function salir() {
  if (S.fuente) S.fuente.close();
  S.token = null;
  S.control = {};
  S.conducirYo = false;
  try { localStorage.removeItem(K('token')); localStorage.removeItem(K('clave')); } catch (e) { /* privado */ }
  $('btn-salir').hidden = true;
  if ($('in-clave')) $('in-clave').value = '';
  aplicarRol();
  mostrarEntrada();
}

function arrancar() {
  $('entrada').hidden = true;
  $('btn-salir').hidden = false;
  actualizarVideo();
  conectarStream();
}

function actualizarVideo() {
  if (S.camara) $('img-camara').src = `stream/${S.camara}?t=${Date.now()}`;
}

/* ======================================================= telemetría (SSE) */
function conectarStream() {
  if (S.fuente) S.fuente.close();
  const f = new EventSource(`api/stream?token=${encodeURIComponent(S.token)}`);
  S.fuente = f;
  f.onmessage = (ev) => recibir(JSON.parse(ev.data));
  f.onerror = async () => {
    pill('pill-sim', false, 'sin conexión');
    const yo = await api(`api/yo?token=${encodeURIComponent(S.token)}`).catch(() => ({ ok: false, status: 0 }));
    if (yo.status === 401) { f.close(); reingresar(); }
  };
}

function recibir(d) {
  if (d.tray) S.tray = d.tray;
  if (d.estado) {
    const e = d.estado;
    S.estado = e;
    const u = S.tray[S.tray.length - 1];
    if (!u || Math.hypot(e.x - u[0], e.y - u[1]) > 0.5) {
      S.tray.push([e.x, e.y]);
      if (S.tray.length > MAX_TRAY) S.tray.shift();
    }
    actualizarTelemetria(e);
  }
  if (d.imu) { S.graf.acc.agregar(d.imu.ax); S.graf.gyr.agregar(d.imu.gz); }
  $('gnss').textContent = d.gnss
    ? `${d.gnss[0].toFixed(6)}, ${d.gnss[1].toFixed(6)} · ${d.gnss[2].toFixed(1)} m` : 'sin señal';
  if (d.enc) {
    const n = ['DI', 'DD', 'TI', 'TD'];
    $('enc').textContent = d.enc.map((w, i) => `${n[i]} ${w.toFixed(1)}`).join('  ') + '  rad/s';
  }
  if (d.lab) pill('pill-sim', d.lab.rtf > 0.85, `${d.lab.fps.toFixed(0)} FPS · ${d.lab.rtf.toFixed(2)}x`);
  else pill('pill-sim', true, 'conectado');
  for (const ev of d.eventos || []) agregarEvento(ev);
  if (d.control) actualizarControl(d.control);
  if (d.lab) { S.lab = d.lab; actualizarEstadoSensores(); actualizarExtra(); }
  S.ultimo = d;
  if (S.pestana === 'sensores') pintarSensores(d);
  if (d.ecu) { S.ecu = d.ecu; if (S.pestana === 'ecu') pintarECU(d.ecu); }
  if (d.can && S.pestana === 'ecu') pintarTablaCAN(d.can);
  if (d.can_log && S.pestana === 'ecu') agregarLogCAN(d.can_log);
}

function pill(id, ok, texto) {
  const p = $(id);
  p.classList.toggle('ok', ok);
  p.classList.toggle('err', !ok);
  if (texto) p.lastChild.textContent = texto;
}

function actualizarTelemetria(d) {
  $('vel').textContent = Math.abs(d.v * 3.6).toFixed(0);
  $('b-acel').style.width = `${d.acel * 100}%`;
  $('b-freno').style.width = `${d.freno * 100}%`;
  const g = $('b-giro'), ancho = Math.min(Math.abs(d.giro), 1) * 50;
  g.style.width = `${ancho}%`;
  g.style.left = d.giro >= 0 ? `${50 - ancho}%` : '50%';
  const modo = $('modo');
  modo.textContent = d.reversa ? `${d.modo} · R` : d.modo;
  modo.className = `modo ${d.modo}`;
  $('btn-auto').classList.toggle('on', d.modo === 'autopilot');
  S.graf.vel.agregar(d.v * 3.6);
}

function agregarEvento(ev) {
  const lista = $('eventos');
  const vacio = lista.querySelector('.vacio');
  if (vacio) vacio.remove();
  const li = document.createElement('li');
  const txt = ev.tipo === 'colision'
    ? `Colisión con ${ev.con} (impulso ${ev.impulso})`
    : `Invasión de carril: ${ev.marcas.join(', ')}`;
  li.innerHTML = `<time>t=${ev.t.toFixed(1)}</time><span class="${ev.tipo}"></span>`;
  li.lastChild.textContent = txt;
  lista.prepend(li);
  while (lista.children.length > 60) lista.lastChild.remove();
}

/* ======================================================= control y permisos */
function actualizarControl(c) {
  S.control = c;
  const p = $('pill-conduce');
  p.lastChild.textContent = c.conductor ? `Conduce: ${c.conductor}` : 'Nadie conduce';
  p.classList.toggle('ok', !!c.conductor);
  $('pill-gente').lastChild.textContent = `${c.conectados} conectado${c.conectados === 1 ? '' : 's'}`;
  $('yo').textContent = c.rol === 'profesor' ? `${c.yo} · profesor` : c.yo || '';
  document.body.classList.toggle('es-profesor', c.rol === 'profesor');
  aplicarRol();
  if (c.rol === 'profesor') pintarUsuarios(c.usuarios || []);
}

function puedoConducirAhora() {
  const c = S.control;
  if (c.rol === 'profesor') return S.conducirYo;
  return !!c.conduzco;
}

function aplicarRol() {
  const c = S.control, prof = c.rol === 'profesor';
  $('v-profesor').hidden = !prof;
  $('lbl-clima').hidden = !prof || !S.cfg.climas.length;
  $('v-observador').hidden = prof || c.conduzco;
  $('v-mandos').hidden = !puedoConducirAhora();
  $('aviso-control').querySelector('b').textContent = prof ? 'Estás conduciendo (profesor)' : 'Tienes el control';
  $('tit-ctrl').textContent = prof ? 'Control · profesor' : 'Control';
  if (!puedoConducirAhora()) { S.teclas = {}; S.reversa = false; $('btn-rev').classList.remove('on'); }
}

function pintarUsuarios(lista) {
  const ul = $('usuarios');
  const firma = JSON.stringify(lista);
  if (ul.dataset.firma === firma) return;
  ul.dataset.firma = firma;
  ul.innerHTML = '';
  for (const u of lista) {
    const li = document.createElement('li');
    const nombre = document.createElement('span');
    nombre.textContent = u.nombre + (u.rol === 'profesor' ? ' (profesor)' : '');
    li.appendChild(nombre);
    if (u.rol !== 'profesor') {
      const b = document.createElement('button');
      b.textContent = u.conduce ? 'Quitar control' : 'Dar control';
      b.className = u.conduce ? 'on' : '';
      b.onclick = () => api('api/admin/control', { uid: u.conduce ? null : u.uid });
      li.appendChild(b);
    } else {
      const t = document.createElement('em');
      t.textContent = 'siempre puede conducir';
      li.appendChild(t);
    }
    ul.appendChild(li);
  }
  if (!lista.length) ul.innerHTML = '<li class="vacio">Nadie más conectado</li>';
}

/* ======================================================= manejo */
function iniciarMandos() {
  addEventListener('keydown', (e) => tecla(e, true));
  addEventListener('keyup', (e) => tecla(e, false));
  addEventListener('blur', () => { S.teclas = {}; });
  for (const b of document.querySelectorAll('#pad button')) {
    const k = b.dataset.k;
    if (k === 'x') { b.onclick = () => alternarReversa(); continue; }
    const on = (e) => { e.preventDefault(); S.teclas[k] = true; b.classList.add('pulsado'); };
    const off = () => { S.teclas[k] = false; b.classList.remove('pulsado'); };
    b.addEventListener('pointerdown', on);
    b.addEventListener('pointerup', off);
    b.addEventListener('pointerleave', off);
    b.addEventListener('pointercancel', off);
  }
}

function alternarReversa() {
  S.reversa = !S.reversa;
  $('btn-rev').classList.toggle('on', S.reversa);
}

function tecla(e, abajo) {
  if (!puedoConducirAhora() || ['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return;
  const mapa = { arrowup: 'w', arrowdown: 's', arrowleft: 'a', arrowright: 'd' };
  let k = e.key.toLowerCase();
  k = mapa[k] || k;
  if (!['w', 'a', 's', 'd', 'x'].includes(k)) return;
  e.preventDefault();
  if (k === 'x') { if (abajo && !e.repeat) alternarReversa(); return; }
  S.teclas[k] = abajo;
}

let enviando = false;
async function cicloManejo() {
  if (!S.token || !puedoConducirAhora() || document.hidden || enviando) return;
  const t = S.teclas;
  const objetivo = t.a ? 0.55 : t.d ? -0.55 : 0;
  S.giro += (objetivo - S.giro) * 0.3;
  enviando = true;
  try {
    await api('api/cmd', { d: [t.w ? 0.55 : 0, S.giro, t.s ? 0.8 : 0, S.reversa ? 1 : 0] });
  } catch (e) { /* red */ }
  enviando = false;
}

/* ======================================================= gráficas */
class Grafica {
  constructor(canvas, titulo, unidad, color, n = 200) {
    Object.assign(this, { c: canvas, titulo, unidad, color, n, datos: [] });
  }
  agregar(v) { this.datos.push(v); if (this.datos.length > this.n) this.datos.shift(); this.sucio = true; }
  dibujar() {
    if (!this.sucio) return;
    this.sucio = false;
    const c = this.c, dpr = devicePixelRatio || 1, w = c.clientWidth, h = c.clientHeight;
    if (!w || !h) return;
    if (c.width !== w * dpr) { c.width = w * dpr; c.height = h * dpr; }
    const x = c.getContext('2d');
    x.setTransform(dpr, 0, 0, dpr, 0, 0);
    x.clearRect(0, 0, w, h);
    const d = this.datos;
    let lo = Math.min(...d, 0), hi = Math.max(...d, 0);
    if (hi - lo < 1e-3) { hi += 1; lo -= 1; }
    const Y = (v) => h - 6 - ((v - lo) / (hi - lo)) * (h - 22);
    x.strokeStyle = '#233040'; x.beginPath(); x.moveTo(0, Y(0)); x.lineTo(w, Y(0)); x.stroke();
    if (d.length > 1) {
      x.strokeStyle = this.color; x.lineWidth = 1.5; x.beginPath();
      d.forEach((v, i) => { const px = (i / (this.n - 1)) * w; i ? x.lineTo(px, Y(v)) : x.moveTo(px, Y(v)); });
      x.stroke();
    }
    x.font = '11px IBM Plex Sans, sans-serif'; x.fillStyle = '#7f91a3'; x.fillText(this.titulo, 6, 12);
    x.textAlign = 'right'; x.fillStyle = '#d7e1ea'; x.font = '11px JetBrains Mono, monospace';
    x.fillText(`${(d.length ? d[d.length - 1] : 0).toFixed(2)} ${this.unidad}`, w - 6, 12);
    x.textAlign = 'left';
  }
}

function iniciarGraficas() {
  S.graf = {
    vel: new Grafica($('g-vel'), 'Velocidad', 'km/h', '#3ccf8e'),
    acc: new Grafica($('g-acc'), 'IMU acel. x', 'm/s²', '#f5a524'),
    gyr: new Grafica($('g-gyr'), 'IMU giro z', 'rad/s', '#4aa3ff'),
  };
}

/* ======================================================= vista 3D */
const MAX_LIDAR = 6000, MAX_RADAR = 2000;

function iniciar3D() {
  const cont = $('vista3d');
  const r = new THREE.WebGLRenderer({ antialias: true });
  r.setPixelRatio(Math.min(devicePixelRatio, 2));
  r.setClearColor(0x070a0e);
  cont.appendChild(r.domElement);
  const escena = new THREE.Scene();
  const cam = new THREE.PerspectiveCamera(55, 1, 0.1, 500);
  cam.up.set(0, 0, 1);
  cam.position.set(-16, -10, 12);
  const ctl = new THREE.OrbitControls(cam, r.domElement);
  ctl.target.set(6, 0, 0);
  ctl.update();

  const rejilla = new THREE.GridHelper(120, 24, 0x2a3644, 0x161f29);
  rejilla.rotation.x = Math.PI / 2;
  escena.add(rejilla);
  const auto = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(4.8, 2.0, 1.5)),
    new THREE.LineBasicMaterial({ color: new THREE.Color(rgb(S.cfg.color)) }));
  auto.position.z = 0.75;
  escena.add(auto);
  escena.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(2.4, 0, 0.75), 2, 0xd7e1ea));

  const crearNube = (max, tam) => {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(max * 3), 3));
    g.setAttribute('color', new THREE.BufferAttribute(new Float32Array(max * 3), 3));
    g.setDrawRange(0, 0);
    const p = new THREE.Points(g, new THREE.PointsMaterial({ size: tam, vertexColors: true, sizeAttenuation: false }));
    escena.add(p);
    return p;
  };
  S.lidar3d = crearNube(MAX_LIDAR, 2);
  S.radar3d = crearNube(MAX_RADAR, 6);
  if (S.cfg.lidar.length) S.lidar3d.position.set(...S.cfg.lidar[0].pos);
  if (S.cfg.radar.length) S.radar3d.position.set(...S.cfg.radar[0].pos);

  S.r3d = { r, escena, cam, ctl };
  new ResizeObserver(() => {
    const w = cont.clientWidth, h = cont.clientHeight;
    if (!w || !h) return;
    r.setSize(w, h);
    cam.aspect = w / h;
    cam.updateProjectionMatrix();
  }).observe(cont);
}

function vistaSuperior() {
  const { cam, ctl } = S.r3d;
  cam.position.set(0.01, 0, 70);
  ctl.target.set(0, 0, 0);
  ctl.update();
}

function colorAltura(z) {
  const t = Math.max(0, Math.min(1, (z + 0.3) / 3.3));
  if (t < 0.5) { const k = t / 0.5; return [0.23, 0.42 + k * 0.39, 1.0 - k * 0.44]; }
  const k = (t - 0.5) / 0.5; return [0.24 + k * 0.72, 0.81 - k * 0.16, 0.56 - k * 0.42];
}

function llenarNube(obj, d, max, color) {
  const n = Math.min(d.length / 4, max) | 0;
  const g = obj.geometry, pos = g.attributes.position.array, col = g.attributes.color.array;
  for (let i = 0; i < n; i++) {
    pos[i * 3] = d[i * 4]; pos[i * 3 + 1] = d[i * 4 + 1]; pos[i * 3 + 2] = d[i * 4 + 2];
    const c = color(d, i);
    col[i * 3] = c[0]; col[i * 3 + 1] = c[1]; col[i * 3 + 2] = c[2];
  }
  g.setDrawRange(0, n);
  g.attributes.position.needsUpdate = g.attributes.color.needsUpdate = true;
}

let pidiendo = false;
async function actualizarNubes() {
  if (pidiendo || document.hidden || !S.token) return;
  pidiendo = true;
  try {
    if (S.cfg.lidar.length) {
      const l = S.cfg.lidar[0];
      const d = new Float32Array(await (await fetch(`api/nube/${l.nombre}`, { cache: 'no-store' })).arrayBuffer());
      llenarNube(S.lidar3d, d, MAX_LIDAR, (a, i) => colorAltura(a[i * 4 + 2] + l.pos[2]));
    }
    if (S.cfg.radar.length) {
      const d = new Float32Array(await (await fetch(`api/nube/${S.cfg.radar[0].nombre}`, { cache: 'no-store' })).arrayBuffer());
      llenarNube(S.radar3d, d, MAX_RADAR, (a, i) => {
        const v = a[i * 4 + 3];
        return v < -0.2 ? [1, 0.36, 0.36] : v > 0.2 ? [0.29, 0.64, 1] : [0.85, 0.85, 0.85];
      });
    }
  } catch (e) { /* servidor reiniciando */ }
  pidiendo = false;
}

/* ======================================================= mapa 2D */
function iniciarMapa() {
  const c = $('mapa');
  S.vistaMapa = { esc: 1, cx: 0, cy: 0, ajustado: false };
  S.capaCarriles = document.createElement('canvas');
  new ResizeObserver(() => { S.carrilesSucio = true; }).observe(c);
  c.addEventListener('wheel', (e) => {
    e.preventDefault();
    const v = S.vistaMapa, k = e.deltaY < 0 ? 1.15 : 1 / 1.15;
    const [wx, wy] = aMundo(e.offsetX, e.offsetY);
    v.esc *= k;
    const [px, py] = aPantalla(wx, wy);
    v.cx += (px - e.offsetX) / v.esc; v.cy -= (py - e.offsetY) / v.esc;
    S.carrilesSucio = true;
  }, { passive: false });
  let arrastre = null;
  c.addEventListener('pointerdown', (e) => { arrastre = [e.clientX, e.clientY]; $('chk-seguir').checked = false; });
  addEventListener('pointerup', () => { arrastre = null; });
  addEventListener('pointermove', (e) => {
    if (!arrastre) return;
    const v = S.vistaMapa;
    v.cx -= (e.clientX - arrastre[0]) / v.esc; v.cy += (e.clientY - arrastre[1]) / v.esc;
    arrastre = [e.clientX, e.clientY];
    S.carrilesSucio = true;
  });
  c.addEventListener('dblclick', () => { S.vistaMapa.ajustado = false; });
}

async function cargarMapa() {
  try {
    const r = await fetch('api/mapa');
    if (!r.ok) throw new Error();
    S.mapa = await r.json();
    S.vistaMapa.ajustado = false;
  } catch (e) { setTimeout(cargarMapa, 3000); }
}

function aPantalla(x, y) {
  const v = S.vistaMapa, c = $('mapa');
  return [(x - v.cx) * v.esc + c.clientWidth / 2, -(y - v.cy) * v.esc + c.clientHeight / 2];
}
function aMundo(px, py) {
  const v = S.vistaMapa, c = $('mapa');
  return [(px - c.clientWidth / 2) / v.esc + v.cx, -(py - c.clientHeight / 2) / v.esc + v.cy];
}

function ajustarMapa() {
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  for (const l of S.mapa.carriles) for (const [x, y] of l) {
    x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y);
  }
  const c = $('mapa'), v = S.vistaMapa;
  v.cx = (x0 + x1) / 2; v.cy = (y0 + y1) / 2;
  v.esc = 0.92 * Math.min(c.clientWidth / (x1 - x0), c.clientHeight / (y1 - y0));
  v.ajustado = true;
  S.carrilesSucio = true;
}

function dibujarMapa() {
  const c = $('mapa'), dpr = devicePixelRatio || 1, w = c.clientWidth, h = c.clientHeight;
  if (!w || !h) return;
  if (c.width !== w * dpr || c.height !== h * dpr) { c.width = w * dpr; c.height = h * dpr; S.carrilesSucio = true; }
  const x = c.getContext('2d');
  x.setTransform(dpr, 0, 0, dpr, 0, 0);
  x.clearRect(0, 0, w, h);
  if (!S.mapa) {
    x.fillStyle = '#7f91a3'; x.font = '13px IBM Plex Sans, sans-serif';
    x.fillText('Esperando el mapa del núcleo de simulación…', 16, 28);
    return;
  }
  if (!S.vistaMapa.ajustado) ajustarMapa();
  const e = S.estado;
  if ($('chk-seguir').checked && e) {
    const v = S.vistaMapa;
    if (Math.hypot(v.cx - e.x, v.cy - e.y) > 0.05) { v.cx = e.x; v.cy = e.y; S.carrilesSucio = true; }
  }
  if (S.carrilesSucio) dibujarCarriles(w, h, dpr);
  x.drawImage(S.capaCarriles, 0, 0, w, h);

  if (S.tray.length > 1) {
    x.strokeStyle = rgb(S.cfg.color); x.globalAlpha = 0.9; x.lineWidth = 2;
    x.beginPath();
    S.tray.forEach(([wx, wy], i) => { const [px, py] = aPantalla(wx, wy); i ? x.lineTo(px, py) : x.moveTo(px, py); });
    x.stroke(); x.globalAlpha = 1;
  }
  if (e) {
    const [px, py] = aPantalla(e.x, e.y);
    const tam = Math.max(8, Math.min(16, 2.4 * S.vistaMapa.esc));
    x.save(); x.translate(px, py); x.rotate(-e.yaw);
    x.beginPath(); x.moveTo(tam, 0); x.lineTo(-tam * 0.7, tam * 0.6); x.lineTo(-tam * 0.7, -tam * 0.6); x.closePath();
    x.fillStyle = rgb(S.cfg.color); x.fill(); x.lineWidth = 2; x.strokeStyle = '#fff'; x.stroke();
    x.restore();
  }
  x.fillStyle = '#7f91a3'; x.font = '11px JetBrains Mono, monospace';
  x.fillText('50 m', 12, h - 12);
  x.strokeStyle = '#7f91a3'; x.beginPath(); x.moveTo(48, h - 16); x.lineTo(48 + 50 * S.vistaMapa.esc, h - 16); x.stroke();
}

function dibujarCarriles(w, h, dpr) {
  const c = S.capaCarriles;
  c.width = w * dpr; c.height = h * dpr;
  const x = c.getContext('2d');
  x.setTransform(dpr, 0, 0, dpr, 0, 0);
  x.strokeStyle = '#2b3a4b';
  x.lineWidth = Math.max(1, Math.min(3.2 * S.vistaMapa.esc, 14));
  x.lineCap = 'round'; x.lineJoin = 'round';
  x.beginPath();
  for (const l of S.mapa.carriles) {
    l.forEach(([wx, wy], i) => { const [px, py] = aPantalla(wx, wy); i ? x.lineTo(px, py) : x.moveTo(px, py); });
  }
  x.stroke();
  x.strokeStyle = '#3d4f63'; x.lineWidth = 0.6; x.stroke();
  S.carrilesSucio = false;
}

/* ======================================================= bucle */
function dibujar() {
  const { r, escena, cam, ctl } = S.r3d;
  ctl.update();
  r.render(escena, cam);
  dibujarMapa();
  Object.values(S.graf).forEach((g) => g.dibujar());
  requestAnimationFrame(dibujar);
}


/* ======================================================= pestañas */
function iniciarPestanas() {
  for (const b of document.querySelectorAll('#pestanas button')) b.onclick = () => mostrarPestana(b.dataset.tab);
}

function mostrarPestana(tab) {
  S.pestana = tab;
  for (const t of ['conduccion', 'sensores', 'ecu', 'terminal']) $(`tab-${t}`).hidden = t !== tab;
  for (const b of document.querySelectorAll('#pestanas button')) b.classList.toggle('activa', b.dataset.tab === tab);
  actualizarExtra();
  if (tab === 'sensores' && S.ultimo) pintarSensores(S.ultimo);
  if (tab === 'ecu' && S.ecu) pintarECU(S.ecu);
  if (tab === 'terminal') abrirTerminal();
  else { cerrarEco(); clearInterval(S.relojTopicos); S.relojTopicos = null; }
  S.carrilesSucio = true;
}

/* ======================================================= terminal y datos crudos */
const EJEMPLOS = [
  ['Listar tópicos disponibles', 'ros2 topic list'],
  ['Ver la IMU en crudo', 'ros2 topic echo /auto/imu/data'],
  ['Frecuencia real del LiDAR', 'ros2 topic hz /auto/lidar/points'],
  ['Estructura de un mensaje', 'ros2 interface show sensor_msgs/msg/NavSatFix'],
  ['Práctica 1 · IMU', 'python3 leer_imu.py'],
  ['Práctica 2 · GNSS a coordenadas locales', 'python3 leer_gnss.py'],
  ['Práctica 3 · LiDAR por sectores', 'python3 leer_lidar.py'],
  ['Práctica 4 · Cámara (guarda imágenes)', 'python3 leer_camara.py'],
  ['Práctica 5 · Ultrasonido', 'python3 leer_ultrasonido.py'],
  ['Práctica 6 · Radar', 'python3 leer_radar.py'],
  ['Práctica 7 · Odometría con encoders', 'python3 leer_encoders.py'],
  ['Práctica 8 · Sniffer CAN con DBC', 'python3 sniffer_can.py'],
  ['Práctica 9 · Lector OBD-II', 'python3 lector_obd.py'],
  ['Eco crudo de cualquier tópico', 'python3 eco_crudo.py /auto/gnss/fix'],
  ['Práctica 10 · Filtra la cámara y publica tu tópico', 'python3 procesar_camara.py bordes'],
  ['Práctica 10 · Detectar el carril', 'python3 procesar_camara.py carril'],
  ['Ver el código de una práctica', 'cat leer_imu.py'],
  ['Editar una práctica', 'nano leer_imu.py'],
  ['Ver mis archivos', 'ls -l'],
  ['Correr un archivo que subí', 'python3 mi_codigo.py'],
];

async function abrirTerminal() {
  const f = $('iframe-terminal');
  if (!f.src) {
    const r = await cargarArchivos();
    const url = r && r.carpeta ? `terminal/?arg=${encodeURIComponent(r.carpeta)}` : 'terminal/';
    f.src = url;
    $('lnk-terminal').href = url;
    iniciarSubidas();
  } else cargarArchivos();
  const ul = $('lista-ejemplos');
  if (!ul.children.length) {
    for (const [desc, cmd] of EJEMPLOS) {
      const li = document.createElement('li');
      li.innerHTML = '<span></span><code></code>';
      li.firstChild.textContent = desc;
      li.lastChild.textContent = cmd;
      li.onclick = async () => {
        try { await navigator.clipboard.writeText(cmd); } catch (e) { /* sin permiso */ }
        li.classList.add('copiado');
        setTimeout(() => li.classList.remove('copiado'), 900);
      };
      ul.appendChild(li);
    }
    $('btn-topicos').onclick = cargarTopicos;
    $('sel-topico').onchange = () => verTopico($('sel-topico').value);
  }
  await cargarTopicos();
  if ($('sel-topico').value) verTopico($('sel-topico').value);
  // La lista se actualiza sola: los tópicos de los alumnos aparecen cuando corren su programa
  clearInterval(S.relojTopicos);
  S.relojTopicos = setInterval(cargarTopicos, 3000);
}

/* ---- archivos del alumno: se guardan en su carpeta de la terminal ---- */
const tam = (b) => b < 1024 ? `${b} B` : b < 1048576 ? `${(b / 1024).toFixed(1)} KB` : `${(b / 1048576).toFixed(1)} MB`;

async function cargarArchivos(nuevos = []) {
  const r = await api(`api/archivos?token=${encodeURIComponent(S.token)}`);
  if (!r.ok) return null;
  S.maxMb = r.max_mb;
  S.carpeta = r.carpeta;
  $('ruta-alumno').innerHTML = `Tu carpeta en la terminal: <code>${r.ruta}</code> · arrastra aquí tu código o usa <b>Subir archivo</b> (máx. ${r.max_mb} MB).`;
  const ul = $('lista-archivos');
  ul.innerHTML = '';
  for (const a of r.archivos) {
    const li = document.createElement('li');
    if (nuevos.includes(a.nombre)) li.className = 'nuevo';
    const enl = document.createElement('a');
    enl.textContent = a.nombre;
    enl.title = 'Descargar';
    enl.href = `api/archivos/${encodeURIComponent(a.nombre)}?token=${encodeURIComponent(S.token)}`;
    const det = document.createElement('small');
    det.textContent = tam(a.bytes);
    li.append(enl, det);
    ul.appendChild(li);
  }
  return r;
}

async function subirArchivos(lista) {
  if (!lista || !lista.length) return;
  const fd = new FormData();
  fd.append('token', S.token);
  for (const f of lista) fd.append('archivo', f);
  const p = $('ruta-alumno');
  p.textContent = `Subiendo ${lista.length} archivo(s)…`;
  let r = {};
  try {
    const resp = await fetch('api/archivos', { method: 'POST', body: fd });
    try { r = await resp.json(); } catch (e) { r = { error: resp.status === 413 ? `El archivo pasa de ${S.maxMb || 10} MB` : `Error ${resp.status}` }; }
  } catch (e) { r = { error: 'Sin conexión con el servidor' }; }
  await cargarArchivos(r.guardados || []);
  if (!r.ok) p.innerHTML += ` <span style="color:var(--err)">${r.error || 'No se pudo subir'}</span>`;
  else {
    const prim = r.guardados[0];
    const cmd = prim.endsWith('.py') ? `python3 ${prim}` : prim.endsWith('.sh') ? `bash ${prim}` : `cat ${prim}`;
    p.innerHTML += ` <span style="color:var(--ok)">Listo. En la terminal: <code>${cmd}</code></span>`;
  }
}

function iniciarSubidas() {
  const inp = $('in-archivo'), zona = $('zona-subir');
  $('btn-subir').onclick = () => inp.click();
  inp.onchange = () => { subirArchivos([...inp.files]); inp.value = ''; };
  zona.ondragover = (e) => { e.preventDefault(); zona.classList.add('encima'); };
  zona.ondragleave = () => zona.classList.remove('encima');
  zona.ondrop = (e) => { e.preventDefault(); zona.classList.remove('encima'); subirArchivos([...e.dataTransfer.files]); };
}

async function cargarTopicos() {
  const r = await api(`api/topicos?token=${encodeURIComponent(S.token)}`);
  if (!r.ok) return;
  const sel = $('sel-topico'), actual = sel.value;
  const lista = Object.values(r).filter((x) => x && x.nombre);
  const firma = lista.map((t) => t.nombre).join('|');
  if (firma === S.firmaTopicos) return;                 // sin cambios: no cerrar el menú abierto
  const antes = S.tipos || {};
  S.firmaTopicos = firma;
  S.tipos = Object.fromEntries(lista.map((t) => [t.nombre, t.tipo]));
  const opc = (t) => `<option value="${t.nombre}">${t.nombre}  (${t.tipo.replace('/msg/', '/')})</option>`;
  const alumnos = lista.filter((t) => t.nombre.startsWith('/alumnos/'));
  const lab = lista.filter((t) => !t.nombre.startsWith('/alumnos/'));
  sel.innerHTML = '<option value="">Elige un tópico…</option>' +
    (alumnos.length ? `<optgroup label="Publicados por alumnos">${alumnos.map(opc).join('')}</optgroup>` : '') +
    `<optgroup label="Laboratorio">${lab.map(opc).join('')}</optgroup>`;
  sel.value = actual;
  if (actual && sel.value !== actual) {                  // el tópico ya no existe (se detuvo el programa)
    sel.insertAdjacentHTML('beforeend', `<option value="${actual}">${actual}  (detenido)</option>`);
    sel.value = actual;
  }
  // Tópico nuevo del propio alumno → mostrarlo automáticamente
  const mio = S.carpeta && lista.find((t) => t.nombre.startsWith(`/alumnos/${S.carpeta}/`) && !(t.nombre in antes));
  if (mio && Object.keys(antes).length && sel.value !== mio.nombre) {
    sel.value = mio.nombre;
    verTopico(mio.nombre);
  }
}

function cerrarEco() {
  if (S.eco) { S.eco.close(); S.eco = null; }
  const img = $('eco-img');
  if (img && img.src) { img.removeAttribute('src'); img.hidden = true; }
}

function verTopico(topico) {
  cerrarEco();
  if (!topico) return;
  if ((S.tipos || {})[topico] === 'sensor_msgs/msg/Image') {     // imágenes: además, video en vivo
    const img = $('eco-img');
    img.src = `api/eco_video?token=${encodeURIComponent(S.token)}&topico=${encodeURIComponent(topico)}&t=${Date.now()}`;
    img.hidden = false;
  }
  $('eco-texto').textContent = 'Conectando…';
  const f = new EventSource(`api/eco?token=${encodeURIComponent(S.token)}&topico=${encodeURIComponent(topico)}`);
  S.eco = f;
  f.onmessage = (ev) => {
    const d = JSON.parse(ev.data);
    $('eco-meta').textContent = `${d.tipo} · ${d.hz} Hz · ${d.n} mensajes`;
    $('eco-texto').textContent = d.texto;
  };
  f.onerror = () => { $('eco-meta').textContent = 'Sin conexión con el tópico'; };
}

function valores(el, lista) {
  el.innerHTML = lista.map(([etq, val, unidad = '', alerta = false]) =>
    `<div class="valor${alerta ? ' alerta' : ''}"><span>${etq}</span><b>${val}<small>${unidad}</small></b></div>`).join('');
}
const fmt = (x, d = 2) => (x === null || x === undefined || Number.isNaN(x)) ? '—' : Number(x).toFixed(d);
const fallasActivas = () => new Set(((S.ecu && S.ecu.dtc && S.ecu.dtc.activos) || []).map((f) => f.codigo));

/* ======================================================= sensores */
const FALLA_SENSOR = { camara: 'B1A01', lidar: 'B1A02', gnss: 'B1A03', imu: 'B1A04', ultrasonido: 'B1A05' };

function iniciarSensores() {
  const sel = $('sel-extra');
  sel.add(new Option('Ninguna (ahorra GPU)', ''));
  for (const c of S.cfg.camaras.filter((x) => x.bajo_demanda)) sel.add(new Option(`${c.nombre} (${c.tipo})`, c.nombre));
  sel.onchange = () => {
    api('api/admin/camara', { nombre: sel.value });
    S.lab = { ...S.lab, camara_extra: sel.value };
    actualizarExtra();
  };
  actualizarEstadoSensores();
}

function actualizarExtra() {
  const nombre = (S.lab && S.lab.camara_extra) || '';
  const cam = S.cfg.camaras.find((c) => c.nombre === nombre);
  $('lbl-extra').textContent = cam ? `${cam.nombre} (${cam.tipo})` : 'Ninguna activa';
  if (document.activeElement !== $('sel-extra')) $('sel-extra').value = nombre;
  const img = $('img-extra');
  const deseado = S.pestana === 'sensores' && nombre ? `stream/${nombre}` : '';
  if (S.extraSrc !== deseado) {
    S.extraSrc = deseado;
    if (deseado) img.src = `${deseado}?t=${Date.now()}`; else img.removeAttribute('src');
  }
}

function actualizarEstadoSensores() {
  const tbody = $('tabla-sensores').querySelector('tbody');
  const reales = {};
  for (const x of (S.lab && S.lab.sensores) || []) reales[x.nombre] = x.hz;
  const fallas = fallasActivas();
  const filas = [];
  for (const s of S.cfg.sensores) {
    if (['sensor.other.collision', 'sensor.other.lane_invasion'].includes(s.tipo)) continue;
    const grupo = s.cantidad > 1;
    const hz = grupo
      ? Math.max(0, ...Object.entries(reales).filter(([n]) => n.startsWith(s.nombre + '_')).map(([, v]) => v))
      : reales[s.nombre];
    const clave = Object.keys(FALLA_SENSOR).find((k) => s.nombre.startsWith(k));
    let estado = ['OK', 'ok'];
    if (clave && fallas.has(FALLA_SENSOR[clave])) estado = [`FALLA ${FALLA_SENSOR[clave]}`, 'err'];
    else if (s.bajo_demanda && (S.lab.camara_extra || '') !== s.nombre) estado = ['apagada', ''];
    else if (!hz) estado = ['sin datos', 'aviso-t'];
    filas.push(`<tr><td>${s.nombre}${grupo ? ` ×${s.cantidad}` : ''}<small>${s.tipo.replace('sensor.', '')}</small></td>
      <td class="nowrap">${s.hz ? s.hz + ' Hz' : 'eventos'}</td><td class="nowrap">${hz ? hz.toFixed(1) + ' Hz' : '—'}</td>
      <td class="nowrap ${estado[1]}">${estado[0]}</td></tr>`);
  }
  tbody.innerHTML = filas.join('');
}

function pintarSensores(d) {
  const i = d.imu || {};
  valores($('val-imu'), [
    ['Acel. x', fmt(i.ax), 'm/s²'], ['Acel. y', fmt(i.ay), 'm/s²'], ['Acel. z', fmt(i.az), 'm/s²'],
    ['Giro x', fmt(i.gx, 3), 'rad/s'], ['Giro y', fmt(i.gy, 3), 'rad/s'], ['Giro z', fmt(i.gz, 3), 'rad/s'],
  ]);
  const g = d.gnss;
  const chip = $('chip-gnss');
  chip.textContent = g ? 'FIX' : 'SIN SEÑAL';
  chip.className = `estado-chip ${g ? 'ok' : 'err'}`;
  valores($('val-gnss'), [
    ['Latitud', g ? g[0].toFixed(7) : '—', '°'], ['Longitud', g ? g[1].toFixed(7) : '—', '°'],
    ['Altitud', g ? g[2].toFixed(1) : '—', 'm', !g],
  ]);
  const r = d.radar || {};
  valores($('val-radar'), [
    ['Detecciones radar', r.n ?? '—'], ['Objeto más cercano', fmt(r.dist, 1), 'm'],
    ['Velocidad relativa', fmt(r.vel, 1), 'm/s', r.vel !== null && r.vel < -3],
    ['Puntos LiDAR / barrido', d.lidar_n ?? '—', '', fallasActivas().has('B1A02')],
  ]);
  const n = ['Del. izq.', 'Del. der.', 'Tras. izq.', 'Tras. der.'];
  const v = (d.estado && Math.abs(d.estado.v)) || 0;
  valores($('val-enc'), (d.enc || [null, null, null, null]).map((w, k) =>
    [n[k], fmt(w, 1), 'rad/s', w !== null && Math.abs(w) < 0.05 && v > 1.5]));
  dibujarUltrasonido((d.estado && d.estado.us) || {});
}

function dibujarUltrasonido(us) {
  const c = $('us-canvas'), dpr = devicePixelRatio || 1, w = c.clientWidth, h = c.clientHeight;
  if (!w || !h) return;
  if (c.width !== w * dpr || c.height !== h * dpr) { c.width = w * dpr; c.height = h * dpr; }
  const x = c.getContext('2d');
  x.setTransform(dpr, 0, 0, dpr, 0, 0);
  x.clearRect(0, 0, w, h);
  const esc = Math.min((h - 30) / 15.5, w / 9), cx = w / 2, cy = h / 2;
  const P = (px, py) => [cx - py * esc, cy - px * esc];
  // vehículo
  const [ax, ay] = P(2.4, 1.0), [bx, by] = P(-2.4, -1.0);
  x.fillStyle = '#16202b'; x.strokeStyle = rgb(S.cfg.color); x.lineWidth = 2;
  x.beginPath(); x.roundRect ? x.roundRect(ax, ay, bx - ax, by - ay, 8) : x.rect(ax, ay, bx - ax, by - ay);
  x.fill(); x.stroke();
  x.fillStyle = '#7f91a3'; x.font = '11px JetBrains Mono, monospace'; x.textAlign = 'center';
  x.textAlign = 'left'; x.fillText('▲ frente', 10, 16); x.textAlign = 'center';
  for (const s of S.cfg.ultrasonido) {
    const d = us[s.nombre];
    const [sx, sy] = P(s.pos[0], s.pos[1]);
    const yaw = (s.rot[2] * Math.PI) / 180;
    const ang = Math.atan2(-Math.cos(yaw), -Math.sin(yaw));
    const libre = d === undefined || d === -1, falla = d === -2;
    const rad = (libre || falla ? 5 : d) * esc;
    const color = falla ? '#55606d' : libre ? 'rgba(60,207,142,.18)'
      : d < 0.5 ? '#ff5d5d' : d < 1.5 ? '#f5a524' : d < 3 ? '#e6d94a' : '#3ccf8e';
    x.beginPath(); x.moveTo(sx, sy); x.arc(sx, sy, rad, ang - 0.22, ang + 0.22); x.closePath();
    x.fillStyle = color; x.globalAlpha = libre ? 1 : 0.75; x.fill(); x.globalAlpha = 1;
    const tx = sx + Math.cos(ang) * (rad + 12), ty = sy + Math.sin(ang) * (rad + 12);
    x.fillStyle = falla ? '#ff5d5d' : '#d7e1ea';
    x.fillText(falla ? 'falla' : libre ? '' : `${d.toFixed(1)}`, tx, ty + 4);
  }
  x.textAlign = 'left';
}

/* ======================================================= ECU / CAN */
function iniciarECU() {
  const ul = $('lista-fallas');
  ul.innerHTML = '';
  for (const f of S.cfg.fallas) {
    const li = document.createElement('li');
    li.innerHTML = `<b>${f.codigo}</b><span>${f.desc}<small>${f.sistema}</small></span><button data-codigo="${f.codigo}">Inyectar</button>`;
    li.querySelector('button').onclick = () => api('api/admin/falla',
      { codigo: f.codigo, activo: !fallasActivas().has(f.codigo) });
    ul.appendChild(li);
  }
  $('btn-borrar-dtc').onclick = () => api('api/admin/borrar_dtc', {});
  const filtro = $('filtro-can');
  for (const m of S.cfg.can_mensajes) filtro.add(new Option(`0x${m.id.toString(16).toUpperCase().padStart(3, '0')} ${m.nombre}`, m.id));
  filtro.add(new Option('0x7DF OBD petición', 0x7DF));
  filtro.add(new Option('0x7E8 OBD respuesta', 0x7E8));
  filtro.onchange = () => { S.canFiltro = filtro.value; $('tabla-log').querySelector('tbody').innerHTML = ''; };
  $('btn-pausa-can').onclick = () => {
    S.canPausa = !S.canPausa;
    $('btn-pausa-can').textContent = S.canPausa ? 'Reanudar' : 'Pausar';
    $('btn-pausa-can').classList.toggle('on', S.canPausa);
  };
}

function marchaTexto(e) { return e.reversa ? 'R' : e.marcha <= 0 ? 'N' : String(e.marcha); }

function pintarECU(e) {
  dibujarMedidor($('m-rpm'), e.rpm, 7000, 'RPM', '×1000', 5500, 1000);
  dibujarMedidor($('m-vel'), e.velocidad, 180, 'Velocidad', 'km/h', null, 1);
  $('mil').classList.toggle('on', !!e.mil);
  const r = e.ruedas || [];
  const v = e.velocidad || 0;
  valores($('val-ecu'), [
    ['Marcha', marchaTexto(e)], ['Acelerador (sensor)', fmt(e.acelerador, 0), '%', e.pedal > 10 && e.acelerador < 1],
    ['Pedal', fmt(e.pedal, 0), '%'], ['Freno', fmt(e.freno, 0), '%'], ['Presión de freno', fmt(e.presion, 1), 'bar'],
    ['Volante', fmt(e.volante, 1), '°'], ['Refrigerante', fmt(e.refrigerante, 0), '°C', e.refrigerante > 115],
    ['Carga motor', fmt(e.carga, 0), '%'], ['Combustible', fmt(e.combustible, 1), '%'],
    ['Batería', fmt(e.bateria, 2), 'V'], ['Odómetro', fmt(e.odometro, 1), 'km'],
    ['Guiñada', fmt(e.guinada, 1), '°/s'], ['Acel. lateral', fmt(e.acel_lat), 'm/s²'], ['Acel. long.', fmt(e.acel_lon), 'm/s²'],
    ...['DI', 'DD', 'TI', 'TD'].map((n, k) => [`Rueda ${n}`, fmt(r[k], 1), 'km/h', r[k] !== undefined && r[k] < 0.5 && v > 5]),
  ]);
  const dtc = e.dtc || { activos: [], almacenados: [] };
  const li = (f, clase, etq) => `<li class="${clase}"><b>${f.codigo}</b><span>${f.desc}</span><em>${etq} · ${f.sistema}</em></li>`;
  const html = dtc.activos.map((f) => li(f, 'activo', 'activo')).concat(dtc.almacenados.map((f) => li(f, 'almacenado', 'almacenado')));
  $('lista-dtc').innerHTML = html.join('') || '<li class="vacio">Sin códigos</li>';
  const activos = new Set(dtc.activos.map((f) => f.codigo));
  for (const b of document.querySelectorAll('#lista-fallas button')) {
    const on = activos.has(b.dataset.codigo);
    b.classList.toggle('on', on);
    b.textContent = on ? 'Quitar' : 'Inyectar';
  }
  actualizarEstadoSensores();
}

const hexId = (id) => `0x${id.toString(16).toUpperCase().padStart(3, '0')}`;

function pintarTablaCAN(tabla) {
  const tb = $('tabla-can').querySelector('tbody');
  if (!tabla.length) return;
  tb.innerHTML = tabla.map((f) => `<tr class="${f.vivo ? '' : 'muerto'}"><td>${hexId(f.id)}</td><td>${f.nombre}</td>
    <td>${f.dlc}</td><td>${f.datos}</td><td>${f.vivo ? f.hz.toFixed(1) : 'sin tramas'}</td>
    <td class="senales">${f.senales}</td></tr>`).join('');
}

function agregarLogCAN(tramas) {
  if (S.canPausa || !tramas.length) return;
  const tb = $('tabla-log').querySelector('tbody');
  const filtro = S.canFiltro ? Number(S.canFiltro) : null;
  const frag = document.createDocumentFragment();
  for (const f of tramas.slice().reverse()) {
    if (filtro !== null && f.id !== filtro) continue;
    const tr = document.createElement('tr');
    tr.className = 'nuevo';
    tr.innerHTML = `<td>${f.t.toFixed(2)}</td><td>${hexId(f.id)}</td><td>${f.datos}</td><td class="senales"></td>`;
    tr.lastChild.textContent = `${f.nombre}: ${f.senales}`;
    frag.appendChild(tr);
  }
  tb.prepend(frag);
  while (tb.children.length > 150) tb.lastChild.remove();
}

function dibujarMedidor(c, valor, max, titulo, unidad, rojo, div) {
  const dpr = devicePixelRatio || 1, w = c.clientWidth, h = c.clientHeight;
  if (!w || !h) return;
  if (c.width !== w * dpr || c.height !== h * dpr) { c.width = w * dpr; c.height = h * dpr; }
  const x = c.getContext('2d');
  x.setTransform(dpr, 0, 0, dpr, 0, 0);
  x.clearRect(0, 0, w, h);
  const cx = w / 2, cy = h * 0.6, r = Math.min(w / 2, h * 0.55) - 8;
  const a0 = Math.PI * 0.8, a1 = Math.PI * 2.2;
  const A = (v) => a0 + (Math.min(Math.max(v, 0), max) / max) * (a1 - a0);
  x.lineWidth = 9; x.lineCap = 'round';
  x.strokeStyle = '#1d2835'; x.beginPath(); x.arc(cx, cy, r, a0, a1); x.stroke();
  if (rojo) { x.strokeStyle = 'rgba(255,93,93,.45)'; x.beginPath(); x.arc(cx, cy, r, A(rojo), a1); x.stroke(); }
  x.strokeStyle = rojo && valor > rojo ? '#ff5d5d' : '#f5a524';
  x.beginPath(); x.arc(cx, cy, r, a0, A(valor || 0)); x.stroke();
  x.fillStyle = '#7f91a3'; x.font = '10px JetBrains Mono, monospace'; x.textAlign = 'center';
  const paso = max / 7;
  for (let k = 0; k <= 7; k++) {
    const a = A(k * paso);
    x.fillText(String(Math.round((k * paso) / div)), cx + Math.cos(a) * (r - 18), cy + Math.sin(a) * (r - 18) + 3);
  }
  x.fillStyle = '#d7e1ea'; x.font = '600 22px JetBrains Mono, monospace';
  x.fillText(valor === undefined ? '—' : Math.round(valor), cx, cy + 6);
  x.fillStyle = '#7f91a3'; x.font = '11px IBM Plex Sans, sans-serif';
  x.fillText(`${titulo} (${unidad})`, cx, cy + 24);
  x.textAlign = 'left';
}

iniciar().catch((e) => {
  document.body.insertAdjacentHTML('afterbegin',
    `<p style="background:#5a1d1d;margin:0;padding:8px 16px">No se pudo iniciar el visualizador: ${e.message}</p>`);
});
