/* =========================================================================
 * Simulador CanSat · Desafío CanSat ESA · Fase regional de Andalucía
 *
 * Núcleo sin DOM (motor físico, sensores, trama CSV, canal de radio,
 * parser de estación de tierra y validación) + interfaz para navegador.
 * El núcleo también corre en Node para tests:
 *   const S = require('./simulacion.js');
 *   const sim = new S.CanSatSim({ seed: 42 }); sim.runToEnd();
 *
 * Convenciones:
 *   - Mundo NED (Norte, Este, Abajo). Cuerpo FRD (X adelante, Y derecha, Z abajo).
 *     Colgada y quieta, la lata mide Az ≈ −g.
 *   - h = altura sobre la rampa (m, positiva hacia arriba); v = velocidad
 *     vertical positiva hacia abajo (como en la ecuación del enunciado).
 *   - Toda la aleatoriedad sale de PRNG con semilla: misma semilla y mismos
 *     parámetros → misma trama, byte a byte.
 * ========================================================================= */
(function (global) {
  'use strict';

  /* ───────────── 1. Constantes físicas ───────────── */
  const G = 9.81;                 // m/s²
  const R_UNIV = 8.31447;         // J/(mol·K)
  const M_AIR = 0.0289644;        // kg/mol
  const R_SPEC = 287.05;          // J/(kg·K), aire seco
  const L_ISA = 0.0065;           // K/m, el gradiente que supone el firmware
  const BARO_EXP = (G * M_AIR) / (R_UNIV * L_ISA); // ≈ 5.2576
  const DEG = Math.PI / 180;
  const R_EARTH = 6371000;        // m

  /* Parámetros editables desde la interfaz (unidades amigables). */
  const DEFAULTS = Object.freeze({
    teamId: 'SEV01',
    seed: 42,
    rateHz: 5,            // frecuencia de la trama (1–5 Hz)
    massG: 330,           // masa del CanSat (g)
    cd: 1.0,              // Cd del paracaídas semiesférico
    chuteCm: 28.8,        // diámetro proyectado del paracaídas (cm) → A ≈ 0.065 m²
    riserM: 0.8,          // distancia paracaídas–lata (m), fija la frecuencia del péndulo
    T0C: 22,              // temperatura en la rampa (°C)
    P0hPa: 1015.0,        // presión en la rampa (hPa, QFE del día)
    apogeeM: 1000,        // apogeo sobre la rampa (m)
    lapseKkm: 6.5,        // gradiente térmico REAL de la atmósfera simulada (K/km)
    windMs: 4,            // viento medio a 10 m (m/s)
    windFromDeg: 250,     // procedencia del viento (°, 250 = poniente)
    lossPct: 3,           // pérdida media del enlace en vuelo (%)
    baud: 9600,           // velocidad aérea de la radio (bit/s)
    gpsHz: 5,             // tasa de actualización del GPS
    batteryMah: 600,      // LiPo 1S
    padWaitMin: 0,        // minutos encendido en rampa antes de empezar a registrar
    lat0: 37.3891,        // coordenadas de ejemplo (área de Sevilla)
    lon0: -5.9845,
    elevM: 10,            // elevación del campo sobre el nivel del mar (m)
    faults: false,        // inyectar fallos de sensor (test de robustez)
  });

  /* Supuestos fijos del modelo (declarados, no editables en la interfaz). */
  const MODEL = Object.freeze({
    dt: 0.01,                 // paso de integración física (s), 100 Hz
    tMax: 900,                // tope de seguridad de la simulación (s)
    bootMs: 2350,             // millis() al entrar en loop(): init de SD, sensores, GPS y calibración
    tPad: 10,                 // s registrando en rampa antes del lanzamiento
    // Cohete (perfil de ascenso simplificado: no es objeto de la misión)
    tBurn: 1.5,               // s de motor con aceleración neta constante
    rocketDrag: 5e-4,         // c = ρ·Cd·A/(2m) del cohete (1/m)
    rollMaxDps: 90,           // alabeo del cohete en el apagado del motor (°/s)
    // Expulsión y paracaídas
    tumbleDps: 320,           // volteo medio impartido en la expulsión (°/s)
    tDeploy: 0.8,             // s desde la expulsión hasta la tensión de cordones
    tInflate: 0.6,            // s de inflado del paracaídas
    canCdA: 0.8 * Math.PI * 0.033 * 0.033, // arrastre de la propia lata, Cd 0.8 × 34 cm²
    tPost: 8,                 // s registrados tras el aterrizaje
    tTip: 0.6,                // s que tarda la lata en volcar al aterrizar
    // Péndulo y actitud
    swingZeta: 0.04,          // amortiguamiento lineal del balanceo
    swingQuad: 0.6,           // amortiguamiento cuadrático (1/rad): frena los balanceos grandes
    turbSigma: 0.35,          // turbulencia que excita el péndulo (m/s²)
    turbTau: 1.2,             // s
    alignHz: 1.6,             // rigidez con la que la lata sigue al cordaje
    alignZeta: 0.3,
    spinDamp: 0.5,            // 1/s, relajación del giro en Z hacia el del paracaídas
    spinMeanDps: 40,          // giro medio del paracaídas (°/s)
    spinVarDps: 8, spinVarTau: 6,
    // Viento
    gustFrac: 0.2, gustTau: 4,
    // Sensores
    probeTauIn: 30,           // s, sonda de temperatura dentro del cohete
    probeTauOut: 3,           // s, sonda expuesta al flujo
    sigmaP: 1.5,              // Pa, ruido del barómetro (BMP280 ultra-alta resolución)
    sigmaT: 0.15,             // °C (±0.3 °C a 2σ)
    sigmaAcc: 0.04,           // m/s²
    vibBoost: 2.5,            // m/s², vibración del motor
    sigmaGyro: 0.08,          // °/s
    sigmaMag: 0.3,            // µT
    accRange: 16 * 9.81,      // ±16 g
    gyroRange: 2000,          // ±2000 °/s
    magH: 27.3, magZ: 34.3, magDecl: 0.0, // campo terrestre aprox. en Sevilla (µT, °)
    // Energía (corrientes en mA)
    iMcu: 48, iSensors: 5.5, iGpsTrack: 25, iGpsAcq: 31,
    iRadioIdle: 16, iRadioTx: 100, iSdIdle: 1, iSdWrite: 60, iBuzzer: 28,
    r0: 0.11, r1: 0.04, tau1: 60,       // Thévenin 1RC de la LiPo (Ω, Ω, s)
    rTh: 22, tauTh: 150,                // °C/W y s del modelo térmico de la CPU
    sdFlushS: 0.015, sdStallP: 0.03,    // escritura en SD cada 1 s y latencias ocasionales
    radioOverhead: 0.008,               // s de preámbulo/arranque del transmisor por trama
    // GPS
    gpsSigmaH: 1.0, gpsSigmaV: 2.0, gpsTau: 40,
    satsBase: 10, noFixRate: 1 / 35,
    // Radio (Gilbert–Elliott)
    pBadLoss: 0.8, pBadToGood: 0.35, pCorrupt: 0.004,
    extraLossAscent: 0.15, extraLossLanded: 0.35,
  });

  /* ───────────── 2. Formato de la trama ───────────── */
  const FIELDS = ['TEAM_ID', 'PACKET_COUNT', 'TIMESTAMP', 'TEMP', 'PRESSURE', 'ALTITUDE',
    'AX', 'AY', 'AZ', 'GX', 'GY', 'GZ', 'MX', 'MY', 'MZ',
    'V_BAT', 'I_DRAW', 'TEMP_CPU',
    'LAT', 'LON', 'GPS_ALT', 'SATS', 'FIX_FLAG'];
  const DECIMALS = [null, 0, 0, 1, 2, 1, 2, 2, 2, 1, 1, 1, 1, 1, 1, 3, 1, 1, 6, 6, 1, 0, 0];
  const MISSING = -999;
  const CSV_HEADER = FIELDS.join(',');

  function fmtNum(v, d) {
    if (v === null || v === undefined || !Number.isFinite(v)) return String(MISSING);
    const s = v.toFixed(d);
    return /^-0(\.0+)?$/.test(s) ? s.slice(1) : s;
  }

  function encodeFrame(teamId, count, ts, values) {
    const parts = [teamId, String(count), String(ts)];
    for (let i = 3; i < FIELDS.length; i++) parts.push(fmtNum(values[FIELDS[i]], DECIMALS[i]));
    return parts.join(',') + '\n';
  }

  /* ───────────── 3. Utilidades: PRNG, OU, vectores, cuaterniones ───────────── */
  function hashSeed(seed, stream) {
    let h = (2166136261 ^ (seed >>> 0)) >>> 0;
    for (let i = 0; i < stream.length; i++) h = Math.imul(h ^ stream.charCodeAt(i), 16777619);
    h = Math.imul(h ^ (h >>> 16), 2246822507);
    h = Math.imul(h ^ (h >>> 13), 3266489909);
    return (h ^ (h >>> 16)) >>> 0;
  }

  /* mulberry32 con un flujo independiente por subsistema: cambiar la
     frecuencia de la trama no altera el viento ni la actitud. */
  class Rng {
    constructor(seed, stream) { this.a = hashSeed(seed, stream); this.spare = null; }
    next() {
      let a = (this.a = (this.a + 0x6D2B79F5) | 0);
      let t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    }
    uniform(lo, hi) { return lo + (hi - lo) * this.next(); }
    chance(p) { return this.next() < p; }
    gauss(sigma = 1) {
      if (this.spare !== null) { const s = this.spare; this.spare = null; return s * sigma; }
      let u = 0;
      while (u <= 1e-12) u = this.next();
      const r = Math.sqrt(-2 * Math.log(u));
      const th = 2 * Math.PI * this.next();
      this.spare = r * Math.sin(th);
      return r * Math.cos(th) * sigma;
    }
  }

  /* Proceso de Ornstein–Uhlenbeck con discretización exacta: ruido
     correlado en el tiempo (ráfagas, deriva del GPS…). */
  class OU {
    constructor(rng, sigma, tau) { this.rng = rng; this.sigma = sigma; this.tau = tau; this.x = rng.gauss(sigma); }
    step(dt) {
      const a = Math.exp(-dt / this.tau);
      this.x = a * this.x + this.sigma * Math.sqrt(1 - a * a) * this.rng.gauss();
      return this.x;
    }
  }

  const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const smoothstep = (x) => x * x * (3 - 2 * x);

  function qMul(a, b) {
    return [
      a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
      a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
      a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
      a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ];
  }
  function qNorm(q) { const n = Math.hypot(q[0], q[1], q[2], q[3]); return [q[0] / n, q[1] / n, q[2] / n, q[3] / n]; }
  function qFromAxisAngle(ax, ang) { const s = Math.sin(ang / 2); return [Math.cos(ang / 2), ax[0] * s, ax[1] * s, ax[2] * s]; }
  /* Rota v del cuerpo al mundo: v_w = q ⊗ v_b ⊗ q*. */
  function qRot(q, v) {
    const w = q[0], x = q[1], y = q[2], z = q[3];
    const tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
    return [v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)];
  }
  function qRotInv(q, v) { return qRot([q[0], -q[1], -q[2], -q[3]], v); }
  /* Integra la velocidad angular en ejes cuerpo: q ← q ⊗ exp(ω·dt/2). */
  function qIntegrate(q, w, dt) {
    const mag = Math.hypot(w[0], w[1], w[2]);
    if (mag < 1e-12) return q;
    return qNorm(qMul(q, qFromAxisAngle([w[0] / mag, w[1] / mag, w[2] / mag], mag * dt)));
  }

  function rk4(s, f, dt) {
    const k1 = f(s);
    const k2 = f(s.map((x, i) => x + 0.5 * dt * k1[i]));
    const k3 = f(s.map((x, i) => x + 0.5 * dt * k2[i]));
    const k4 = f(s.map((x, i) => x + dt * k3[i]));
    return s.map((x, i) => x + (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
  }

  /* ───────────── 4. Atmósfera (ISA ajustada al día) ───────────── */
  function makeAtmosphere(T0K, P0Pa, lapse) {
    const iso = Math.abs(lapse) < 1e-7;
    const n = iso ? 0 : (G * M_AIR) / (R_UNIV * lapse);
    const T = (h) => T0K - lapse * h;
    const p = (h) => (iso
      ? P0Pa * Math.exp((-G * M_AIR * h) / (R_UNIV * T0K))
      : P0Pa * Math.pow(1 - (lapse * h) / T0K, n));
    const rho = (h) => p(h) / (R_SPEC * T(h));
    return { T, p, rho, T0K, P0Pa, lapse };
  }

  /* Inversa barométrica: lo que ejecuta el firmware (siempre con L ISA). */
  function baroAltitude(pPa, P0Pa, T0K) {
    return (T0K / L_ISA) * (1 - Math.pow(pPa / P0Pa, 1 / BARO_EXP));
  }

  /* Tensión en circuito abierto de una LiPo 1S frente al estado de carga. */
  const LIPO_OCV = [[0, 3.27], [0.05, 3.61], [0.1, 3.69], [0.15, 3.71], [0.2, 3.73], [0.25, 3.75],
    [0.3, 3.77], [0.35, 3.79], [0.4, 3.8], [0.45, 3.82], [0.5, 3.84], [0.55, 3.85], [0.6, 3.87],
    [0.65, 3.91], [0.7, 3.95], [0.75, 3.98], [0.8, 4.02], [0.85, 4.08], [0.9, 4.11], [0.95, 4.15], [1, 4.2]];
  function ocv(soc) {
    const s = clamp(soc, 0, 1);
    for (let i = 1; i < LIPO_OCV.length; i++) {
      const [s1, v1] = LIPO_OCV[i];
      if (s <= s1) {
        const [s0, v0] = LIPO_OCV[i - 1];
        return v0 + ((v1 - v0) * (s - s0)) / (s1 - s0);
      }
    }
    return 4.2;
  }

  /* ───────────── 5. Configuración y magnitudes derivadas ───────────── */
  function normalizeConfig(c) {
    const o = Object.assign({}, DEFAULTS, c || {});
    const num = (v, d, lo, hi) => { const x = Number(v); return Number.isFinite(x) ? clamp(x, lo, hi) : d; };
    const pick = (v, list, d) => (list.includes(Number(v)) ? Number(v) : d);
    const team = String(o.teamId == null ? '' : o.teamId).replace(/[^A-Za-z0-9_-]/g, '').slice(0, 12);
    return {
      teamId: team || DEFAULTS.teamId,
      seed: Math.trunc(num(o.seed, DEFAULTS.seed, 0, 4294967295)),
      rateHz: Math.round(num(o.rateHz, DEFAULTS.rateHz, 1, 5)),
      massG: num(o.massG, DEFAULTS.massG, 150, 600),
      cd: num(o.cd, DEFAULTS.cd, 0.3, 2.0),
      chuteCm: num(o.chuteCm, DEFAULTS.chuteCm, 10, 120),
      riserM: num(o.riserM, DEFAULTS.riserM, 0.2, 3),
      T0C: num(o.T0C, DEFAULTS.T0C, -20, 45),
      P0hPa: num(o.P0hPa, DEFAULTS.P0hPa, 900, 1060),
      apogeeM: num(o.apogeeM, DEFAULTS.apogeeM, 200, 3000),
      lapseKkm: num(o.lapseKkm, DEFAULTS.lapseKkm, -5, 9.8),
      windMs: num(o.windMs, DEFAULTS.windMs, 0, 20),
      windFromDeg: ((num(o.windFromDeg, DEFAULTS.windFromDeg, -360, 720) % 360) + 360) % 360,
      lossPct: num(o.lossPct, DEFAULTS.lossPct, 0, 60),
      baud: pick(o.baud, [1200, 2400, 4800, 9600, 19200, 38400, 57600], DEFAULTS.baud),
      gpsHz: pick(o.gpsHz, [1, 5, 10], DEFAULTS.gpsHz),
      batteryMah: num(o.batteryMah, DEFAULTS.batteryMah, 100, 5000),
      padWaitMin: num(o.padWaitMin, DEFAULTS.padWaitMin, 0, 180),
      lat0: num(o.lat0, DEFAULTS.lat0, -85, 85),
      lon0: num(o.lon0, DEFAULTS.lon0, -180, 180),
      elevM: num(o.elevM, DEFAULTS.elevM, -100, 4000),
      faults: o.faults === true || o.faults === 'true' || o.faults === 'on',
    };
  }

  function coastHeight(v, c) { return Math.log(1 + (c * v * v) / G) / (2 * c); }
  /* Aceleración neta del motor que lleva el cohete justo al apogeo pedido. */
  function solveBoost(apogee) {
    const tb = MODEL.tBurn, c = MODEL.rocketDrag;
    let lo = 1, hi = 5000;
    for (let i = 0; i < 80; i++) {
      const mid = (lo + hi) / 2;
      if (0.5 * mid * tb * tb + coastHeight(mid * tb, c) > apogee) hi = mid; else lo = mid;
    }
    return (lo + hi) / 2;
  }

  /* Trama típica a mitad de descenso para estimar bytes y ocupación del enlace. */
  function typicalFrameBytes(cfg) {
    const v = { TEMP: 18.4, PRESSURE: 985.27, ALTITUDE: 512.3, AX: -0.12, AY: 0.31, AZ: -9.74,
      GX: -12.4, GY: 8.7, GZ: 41.3, MX: 21.4, MY: -14.8, MZ: 38.2, V_BAT: 4.172, I_DRAW: 148.3,
      TEMP_CPU: 27.6, LAT: cfg.lat0 + 0.0021, LON: cfg.lon0 + 0.0043, GPS_ALT: 524.6, SATS: 9, FIX_FLAG: 1 };
    return encodeFrame(cfg.teamId, 312, 64350, v).length;
  }

  function derive(cfgIn) {
    const cfg = normalizeConfig(cfgIn);
    const m = cfg.massG / 1000;
    const A = (Math.PI * (cfg.chuteCm / 100) ** 2) / 4;
    const cdA = cfg.cd * A + MODEL.canCdA;
    const atm = makeAtmosphere(cfg.T0C + 273.15, cfg.P0hPa * 100, cfg.lapseKkm / 1000);
    const vt = (h) => Math.sqrt((2 * m * G) / (atm.rho(h) * cdA));
    // Estimación 1D del descenso (sin viento), mismo modelo de apertura que la simulación
    let h = cfg.apogeeM, v = 0, t = 0;
    const dt = 0.02;
    while (h > 0 && t < MODEL.tMax) {
      const phi = t < MODEL.tDeploy ? 0 : smoothstep(Math.min(1, (t - MODEL.tDeploy) / MODEL.tInflate));
      const k = (atm.rho(h) * (cfg.cd * A * phi + MODEL.canCdA)) / (2 * m);
      v += (G - k * v * v) * dt;
      h -= v * dt;
      t += dt;
    }
    const aBoost = solveBoost(cfg.apogeeM);
    const c = MODEL.rocketDrag, vbo = aBoost * MODEL.tBurn;
    const tAscent = MODEL.tBurn + Math.atan(vbo * Math.sqrt(c / G)) / Math.sqrt(G * c);
    const period = 1 / cfg.rateHz;
    const frameBytes = typicalFrameBytes(cfg);
    const airtime = (frameBytes * 10) / cfg.baud + MODEL.radioOverhead;
    return {
      m, A, cdA,
      rhoTop: atm.rho(cfg.apogeeM), rhoGround: atm.rho(0),
      vtTop: vt(cfg.apogeeM), vtGround: vt(0),
      tDescent: t, tAscent, aBoost,
      fSwing: Math.sqrt(G / cfg.riserM) / (2 * Math.PI),
      frameBytes, airtime, occupancy: airtime / period,
      baroExp: BARO_EXP,
      tEnd: MODEL.tPad + tAscent + t + MODEL.tPost,
    };
  }

  /* ───────────── 6. Simulación de la misión ───────────── */
  const PHASES = { PAD: 'En rampa', ASCENSO: 'Ascenso', EXPULSION: 'Expulsión', DESCENSO: 'Descenso', ATERRIZADO: 'Aterrizado' };

  class CanSatSim {
    constructor(cfgIn) {
      const cfg = (this.cfg = normalizeConfig(cfgIn));
      const d = (this.derived = derive(cfg));
      this.m = d.m;
      this.A = d.A;
      this.atm = makeAtmosphere(cfg.T0C + 273.15, cfg.P0hPa * 100, cfg.lapseKkm / 1000);
      this.period = 1 / cfg.rateHz;
      const S = (name) => new Rng(cfg.seed, name);
      this.rng = { wind: S('viento'), turb: S('turbulencia'), att: S('actitud'), baro: S('baro'),
        imu: S('imu'), pwr: S('energia'), gps: S('gps'), gpsEv: S('gps-eventos'),
        radio: S('radio'), fw: S('firmware'), fault: S('fallos') };

      // Cohete
      this.aBoost = d.aBoost;
      this.vBurnout = this.aBoost * MODEL.tBurn;
      this.vUp = 0;

      // Viento, turbulencia y giro del paracaídas
      const gs = MODEL.gustFrac * Math.max(cfg.windMs, 0.5);
      this.gust = [new OU(this.rng.wind, gs, MODEL.gustTau), new OU(this.rng.wind, gs, MODEL.gustTau)];
      this.turb = [new OU(this.rng.turb, MODEL.turbSigma, MODEL.turbTau), new OU(this.rng.turb, MODEL.turbSigma, MODEL.turbTau)];
      const ra = this.rng.att;
      this.spinMean = (ra.chance(0.5) ? 1 : -1) * Math.max(10, MODEL.spinMeanDps + ra.gauss(10)) * DEG;
      this.spinVar = new OU(ra, MODEL.spinVarDps * DEG, MODEL.spinVarTau);

      // Campo magnético (NED, µT) y errores fijos de la IMU
      const D = MODEL.magDecl * DEG;
      this.Bw = [MODEL.magH * Math.cos(D), MODEL.magH * Math.sin(D), MODEL.magZ];
      const ri = this.rng.imu;
      this.accBias = [ri.gauss(0.08), ri.gauss(0.08), ri.gauss(0.08)];
      this.gyroBias = [ri.gauss(0.4), ri.gauss(0.4), ri.gauss(0.4)];
      this.hardIron = [ri.gauss(8), ri.gauss(8), ri.gauss(8)];

      // Estado de traslación
      this.t = 0;
      this.phase = 'PAD';
      this.h = 0; this.v = 0; this.aDown = 0;
      this.uN = 0; this.uE = 0;          // velocidad horizontal del sistema
      this.pN = 0; this.pE = 0;          // posición del sistema (paracaídas) respecto a la rampa
      this.canN = 0; this.canE = 0;      // posición de la lata (incluye el péndulo)
      this.aC = [0, 0];                  // aceleración horizontal del paracaídas
      this.aW = [0, 0, 0];               // aceleración de la lata en NED
      this.phi = 0;                      // fracción de inflado

      // Péndulo y actitud
      this.th = [0, 0]; this.thd = [0, 0]; this.thdd = [0, 0];
      this.swingOn = false; this.rPrev = [0, 0, 1];
      this.q = qFromAxisAngle([0, 0, 1], ra.uniform(0, 2 * Math.PI));
      this.w = [0, 0, 0];

      // Hitos
      this.tLaunch = MODEL.tPad;
      this.tBurnout = MODEL.tPad + MODEL.tBurn;
      this.tApogee = null; this.hApogee = null; this.tOpen = null; this.tLand = null; this.tEnd = null;
      this.vImpact = 0; this.impactUntil = -1; this.tipUntil = -1; this.tipAxis = null;
      this.events = [];
      this.burnoutLogged = false;

      // Sonda de temperatura y calibración del firmware en rampa (32 muestras)
      this.Tprobe = this.atm.T(0) - 273.15;
      let sp = 0, st = 0;
      for (let i = 0; i < 32; i++) {
        sp += this.atm.p(0) + this.rng.baro.gauss(MODEL.sigmaP);
        st += this.Tprobe + this.rng.baro.gauss(MODEL.sigmaT);
      }
      this.P0cal = sp / 32;
      this.T0cal = st / 32 + 273.15;

      // Energía
      const waitS = cfg.padWaitMin * 60;
      const iIdleA = (MODEL.iMcu + MODEL.iSensors + MODEL.iGpsTrack + MODEL.iRadioIdle + MODEL.iSdIdle) / 1000;
      this.capAs = (cfg.batteryMah / 1000) * 3600;
      this.c1 = MODEL.tau1 / MODEL.r1;
      this.soc = clamp(1 - (iIdleA * waitS) / this.capAs, 0, 1);
      this.v1 = iIdleA * MODEL.r1 * (1 - Math.exp(-waitS / MODEL.tau1));
      const p0W = (ocv(this.soc) - iIdleA * MODEL.r0 - this.v1) * iIdleA;
      this.Tcpu = this.Tprobe + MODEL.rTh * p0W * (1 - Math.exp(-(waitS + 5) / MODEL.tauTh));
      this.qAcc = 0; this.tAccStart = 0;
      this.txUntil = -1; this.sdBusyUntil = -1; this.buzzerOn = false;

      // GPS
      const rg = this.rng.gps;
      this.gpsErr = [new OU(rg, MODEL.gpsSigmaH, MODEL.gpsTau), new OU(rg, MODEL.gpsSigmaH, MODEL.gpsTau), new OU(rg, MODEL.gpsSigmaV, MODEL.gpsTau)];
      this.satsOU = new OU(rg, 0.8, 20);
      this.noFixUntil = -1; this.noFixWindows = [];
      this.gpsFix = true; this.boostNoFix = false;
      this.nextGpsT = 0; this.gpsSol = null;

      // Canal de radio (Gilbert–Elliott): estado "malo" que produce ráfagas
      const target = cfg.lossPct / 100;
      const pG = Math.min(target, 0.005);
      const piB = target <= pG ? 0 : Math.min(0.95, (target - pG) / (MODEL.pBadLoss - pG));
      this.ch = { bad: false, pG, pB: MODEL.pBadLoss, b2g: MODEL.pBadToGood, g2b: Math.min(1, (piB * MODEL.pBadToGood) / (1 - piB)) };

      // Fallos de sensor (se programan en la expulsión)
      this.faultWindows = [];

      // Registros
      this.k = 0;
      this.nextSched = 0;
      this.nextAcq = 0;
      this.sd = [];      // lo que se escribe en la MicroSD (todas las tramas)
      this.truth = [];   // estado real del modelo en el instante de cada trama
      this.rx = [];      // lo que sale por la radio y qué le pasa en el canal
      this.done = false;

      this.event(0, 'boot', 'Inicio del registro');
      this.stepGps(0, 0);
      this.emitFrame();
    }

    event(t, type, label) { this.events.push({ t, ts: MODEL.bootMs + Math.round(t * 1000), type, label }); }

    windAt(h) {
      const c = this.cfg;
      const k = Math.min(Math.pow(Math.max(h, 2) / 10, 1 / 7), 2);
      const speed = c.windMs * k;
      const to = (c.windFromDeg + 180) * DEG;
      return [speed * Math.cos(to) + this.gust[0].x, speed * Math.sin(to) + this.gust[1].x];
    }

    tiltDeg() {
      const zb = qRot(this.q, [0, 0, 1]);
      return Math.acos(clamp(zb[2], -1, 1)) / DEG;
    }

    inFault(kind, t) {
      return this.faultWindows.some((f) => f.kind === kind && t >= f.start && t < f.end);
    }

    startNoFix(t, dur, reason) {
      const last = this.noFixWindows[this.noFixWindows.length - 1];
      if (last && t <= last.end) { last.end = Math.max(last.end, t + dur); }
      else this.noFixWindows.push({ start: t, end: t + dur, reason });
      this.noFixUntil = Math.max(this.noFixUntil, t + dur);
    }

    onEjection(t) {
      const ra = this.rng.att;
      const u = ra.uniform(-1, 1), az = ra.uniform(0, 2 * Math.PI), s = Math.sqrt(1 - u * u);
      const mag = Math.max(120, MODEL.tumbleDps + ra.gauss(60)) * DEG;
      this.w = [this.w[0] + mag * s * Math.cos(az), this.w[1] + mag * s * Math.sin(az), this.w[2] + mag * u];
      this.startNoFix(t, this.rng.gpsEv.uniform(2, 6), 'Volteo tras la expulsión');
      if (this.cfg.faults) {
        const rf = this.rng.fault;
        const b = t + rf.uniform(15, 50), i = t + rf.uniform(25, 70);
        this.faultWindows.push({ kind: 'baro', start: b, end: b + 3, label: 'BMP280 sin respuesta (I²C)' });
        this.faultWindows.push({ kind: 'imu', start: i, end: i + 2, label: 'IMU sin respuesta (I²C)' });
      }
      this.event(t, 'apogee', 'Apogeo y expulsión');
    }

    /* Un paso de 10 ms de todo el sistema. */
    step() {
      const dt = MODEL.dt, cfg = this.cfg;
      const t = this.t + dt;
      this.gust[0].step(dt); this.gust[1].step(dt);
      this.turb[0].step(dt); this.turb[1].step(dt);
      const L = cfg.riserM;

      if (this.phase === 'PAD') {
        this.aW = [0, 0, 0];
        if (t >= this.tLaunch) { this.phase = 'ASCENSO'; this.event(t, 'launch', 'Lanzamiento'); }
      } else if (this.phase === 'ASCENSO') {
        const tb = t - this.tLaunch;
        if (tb <= MODEL.tBurn) {
          this.vUp = this.aBoost * tb;
          this.h = 0.5 * this.aBoost * tb * tb;
          this.aDown = -this.aBoost;
        } else {
          if (!this.burnoutLogged) { this.burnoutLogged = true; this.event(t, 'burnout', 'Fin de motor'); }
          const c = MODEL.rocketDrag;
          const f = (s) => [s[1], -G - c * s[1] * Math.abs(s[1])];
          const s1 = rk4([this.h, this.vUp], f, dt);
          this.h = s1[0]; this.vUp = s1[1];
          this.aDown = -f(s1)[1];
        }
        this.v = -this.vUp;
        this.w = [0, 0, (MODEL.rollMaxDps * DEG * Math.max(0, this.vUp)) / this.vBurnout];
        this.aW = [0, 0, this.aDown];
        if (tb > MODEL.tBurn && this.vUp <= 0) {
          this.vUp = 0; this.v = 0;
          this.phase = 'EXPULSION';
          this.tApogee = t; this.hApogee = this.h;
          this.onEjection(t);
        }
      } else if (this.phase === 'EXPULSION' || this.phase === 'DESCENSO') {
        const te = t - this.tApogee;
        this.phi = te < MODEL.tDeploy ? 0 : smoothstep(Math.min(1, (te - MODEL.tDeploy) / MODEL.tInflate));
        if (this.phase === 'EXPULSION' && this.phi >= 1) {
          this.phase = 'DESCENSO'; this.tOpen = t; this.event(t, 'open', 'Paracaídas abierto');
        }
        const [wN, wE] = this.windAt(this.h);
        const k = (cfg.cd * this.A * this.phi + MODEL.canCdA) / (2 * this.m);
        // Arrastre 3D: a = g·ẑ + (ρ·CdA/2m)·|V_rel|·V_rel, con V_rel = viento − velocidad
        const deriv = (s) => {
          const rho = this.atm.rho(Math.max(s[0], 0));
          const rN = wN - s[2], rE = wE - s[3];
          const vRel = Math.hypot(s[1], rN, rE);
          const kk = rho * k * vRel;
          return [-s[1], G - kk * s[1], kk * rN, kk * rE];
        };
        const s0 = [this.h, this.v, this.uN, this.uE];
        const s1 = rk4(s0, deriv, dt);
        const ds = deriv(s1);
        this.pN += 0.5 * (s0[2] + s1[2]) * dt;
        this.pE += 0.5 * (s0[3] + s1[3]) * dt;
        [this.h, this.v, this.uN, this.uE] = s1;
        this.aDown = ds[1];
        this.aC = [ds[2], ds[3]];

        // Péndulo lata–paracaídas (dos planos): Lθ'' = −g·sinθ − a_c·cosθ − amortiguamiento
        if (this.phi > 0 && !this.swingOn) {
          this.swingOn = true;
          this.th = [gaussDeg(this, 20), gaussDeg(this, 20)];
          this.thd = [this.rng.att.gauss(0.6), this.rng.att.gauss(0.6)];
          this.rPrev = this.riserDir();
        }
        let aN = this.aC[0], aE = this.aC[1], aD = this.aDown;
        if (this.swingOn) {
          const wp = Math.sqrt(G / L);
          for (let i = 0; i < 2; i++) {
            const th = this.th[i], thd = this.thd[i];
            const drive = this.aC[i] + this.turb[i].x;
            const acc = (-G / L) * Math.sin(th) - (drive / L) * Math.cos(th)
              - 2 * MODEL.swingZeta * wp * thd - MODEL.swingQuad * thd * Math.abs(thd);
            this.thd[i] = thd + acc * dt;
            this.th[i] = th + this.thd[i] * dt;
            this.thdd[i] = acc;
          }
          const [a0, a1] = this.th, [d0, d1] = this.thd, [dd0, dd1] = this.thdd;
          aN += L * (dd0 * Math.cos(a0) - d0 * d0 * Math.sin(a0));
          aE += L * (dd1 * Math.cos(a1) - d1 * d1 * Math.sin(a1));
          aD += -L * (dd0 * Math.sin(a0) + d0 * d0 * Math.cos(a0) + dd1 * Math.sin(a1) + d1 * d1 * Math.cos(a1));
        }
        this.aW = [aN, aE, aD];
        this.canN = this.pN + (this.swingOn ? L * Math.sin(this.th[0]) : 0);
        this.canE = this.pE + (this.swingOn ? L * Math.sin(this.th[1]) : 0);

        if (this.h <= 0) {
          this.h = 0;
          this.vImpact = this.v;
          this.v = 0; this.uN = 0; this.uE = 0;
          this.pN = this.canN; this.pE = this.canE;
          this.swingOn = false;
          this.phase = 'ATERRIZADO';
          this.tLand = t;
          this.impactUntil = t + 0.05;
          const az = this.rng.att.uniform(0, 2 * Math.PI);
          this.tipAxis = [Math.cos(az), Math.sin(az), 0];
          this.tipUntil = t + MODEL.tTip;
          this.event(t, 'land', 'Aterrizaje');
        }
      } else if (this.phase === 'ATERRIZADO') {
        this.aDown = t < this.impactUntil ? -this.vImpact / 0.05 : 0;
        this.aW = [0, 0, this.aDown];
        if (!this.buzzerOn && t >= this.tLand + 2) this.buzzerOn = true;
        if (t >= this.tLand + MODEL.tPost) {
          this.done = true; this.tEnd = t;
          this.event(t, 'end', 'Fin del registro');
        }
      }

      this.stepAttitude(dt, t);
      this.stepProbe(dt);
      this.stepPower(dt, t);
      this.stepGps(dt, t);
      this.t = t;
      if (t > MODEL.tMax) { this.done = true; this.tEnd = t; }
      if (t + 1e-9 >= this.nextAcq) this.emitFrame();
    }

    riserDir() {
      const s0 = Math.sin(this.th[0]), s1 = Math.sin(this.th[1]);
      return [s0, s1, Math.sqrt(Math.max(0, 1 - s0 * s0 - s1 * s1))];
    }

    /* Actitud: volteo libre tras la expulsión; con el paracaídas tenso, la
       lata se alinea con el cordaje (que oscila) y gira con el paracaídas. */
    stepAttitude(dt, t) {
      const ph = this.phase;
      if (ph === 'PAD') { this.w = [0, 0, 0]; return; }
      if (ph === 'ATERRIZADO') {
        if (t <= this.tipUntil) {
          const rate = Math.PI / 2 / MODEL.tTip;
          this.w = qRotInv(this.q, [this.tipAxis[0] * rate, this.tipAxis[1] * rate, 0]);
          this.q = qIntegrate(this.q, this.w, dt);
        } else this.w = [0, 0, 0];
        return;
      }
      if ((ph === 'EXPULSION' || ph === 'DESCENSO') && this.swingOn) {
        const r = this.riserDir();
        const rDot = [(r[0] - this.rPrev[0]) / dt, (r[1] - this.rPrev[1]) / dt, (r[2] - this.rPrev[2]) / dt];
        this.rPrev = r;
        const wRiser = cross(r, rDot);
        const zb = qRot(this.q, [0, 0, 1]);
        const wn = 2 * Math.PI * MODEL.alignHz;
        const tauB = qRotInv(this.q, cross(zb, r).map((x) => x * wn * wn * this.phi));
        this.spinVar.step(dt);
        const wt = qRotInv(this.q, wRiser);
        wt[2] += this.spinMean + this.spinVar.x;
        const cxy = 2 * MODEL.alignZeta * wn * this.phi, cz = MODEL.spinDamp * this.phi;
        this.w = [
          this.w[0] + (tauB[0] + cxy * (wt[0] - this.w[0])) * dt,
          this.w[1] + (tauB[1] + cxy * (wt[1] - this.w[1])) * dt,
          this.w[2] + (tauB[2] + cz * (wt[2] - this.w[2])) * dt,
        ];
      }
      this.q = qIntegrate(this.q, this.w, dt);
    }

    stepProbe(dt) {
      const inside = this.phase === 'PAD' || this.phase === 'ASCENSO';
      const tau = inside ? MODEL.probeTauIn : MODEL.probeTauOut;
      const Tair = this.atm.T(this.h) - 273.15;
      this.Tprobe += (Tair - this.Tprobe) * (1 - Math.exp(-dt / tau));
    }

    stepPower(dt, t) {
      const M = MODEL;
      const I = M.iMcu + M.iSensors
        + (this.gpsFix ? M.iGpsTrack : M.iGpsAcq)
        + (t <= this.txUntil ? M.iRadioTx : M.iRadioIdle)
        + (t <= this.sdBusyUntil ? M.iSdWrite : M.iSdIdle)
        + (this.buzzerOn ? M.iBuzzer : 0);
      const IA = I / 1000;
      this.soc = Math.max(0, this.soc - (IA * dt) / this.capAs);
      this.v1 += dt * (IA / this.c1 - this.v1 / (M.r1 * this.c1));
      const vTerm = ocv(this.soc) - IA * M.r0 - this.v1;
      const Tair = this.atm.T(this.h) - 273.15;
      this.Tcpu += (dt * (Tair + M.rTh * vTerm * IA - this.Tcpu)) / M.tauTh;
      this.qAcc += I * dt;
    }

    stepGps(dt, t) {
      const ph = this.phase, rv = this.rng.gpsEv;
      if (ph === 'ASCENSO' && !this.boostNoFix && t >= this.tLaunch + 0.2) {
        this.boostNoFix = true;
        this.startNoFix(t, this.tBurnout - t + rv.uniform(2, 4), 'Aceleración > 4 g (límite dinámico del receptor)');
      }
      if (ph === 'DESCENSO' && t >= this.noFixUntil + 3) {
        const tilt = this.tiltDeg();
        const lambda = MODEL.noFixRate * (1 + (tilt / 12) ** 2);
        if (rv.chance(lambda * dt)) {
          this.startNoFix(t, rv.uniform(2, 6), tilt > 12 ? 'Inclinación de la lata' : 'Antena tapada por el paracaídas');
        }
      }
      this.gpsFix = t >= this.noFixUntil;
      if (t + 1e-9 < this.nextGpsT) return;
      const dtE = 1 / this.cfg.gpsHz;
      this.nextGpsT += dtE;
      for (const e of this.gpsErr) e.step(dtE);
      this.satsOU.step(dtE);
      const cfg = this.cfg, rg = this.rng.gps;
      if (this.gpsFix) {
        const occl = Math.min(5, this.tiltDeg() / 15);
        const sats = clamp(Math.round(MODEL.satsBase + this.satsOU.x - occl), 5, 12);
        const n = this.canN + this.gpsErr[0].x, e = this.canE + this.gpsErr[1].x;
        this.gpsSol = {
          LAT: cfg.lat0 + n / R_EARTH / DEG,
          LON: cfg.lon0 + e / (R_EARTH * Math.cos(cfg.lat0 * DEG)) / DEG,
          GPS_ALT: cfg.elevM + this.h + this.gpsErr[2].x,
          SATS: sats, FIX_FLAG: 1,
        };
      } else {
        const sats = clamp(Math.round(1.5 + rg.gauss(1)), 0, 3);
        this.gpsSol = Object.assign({}, this.gpsSol, { SATS: sats, FIX_FLAG: 0 });
      }
    }

    /* Lectura de sensores tal como la vería el firmware en este instante. */
    sampleSensors(t, iAvg) {
      const M = MODEL, rb = this.rng.baro, ri = this.rng.imu, rp = this.rng.pwr;
      const pMeas = this.atm.p(this.h) + rb.gauss(M.sigmaP);
      const T = this.Tprobe + rb.gauss(M.sigmaT);
      const alt = baroAltitude(pMeas, this.P0cal, this.T0cal);
      const fB = qRotInv(this.q, [this.aW[0], this.aW[1], this.aW[2] - G]); // fuerza específica
      const vib = this.phase === 'ASCENSO' && t - this.tLaunch <= M.tBurn ? M.vibBoost : 0;
      const acc = fB.map((f, i) => clamp(f + this.accBias[i] + ri.gauss(M.sigmaAcc) + ri.gauss(vib), -M.accRange, M.accRange));
      const gyr = this.w.map((w, i) => clamp(w / DEG + this.gyroBias[i] + ri.gauss(M.sigmaGyro), -M.gyroRange, M.gyroRange));
      const mB = qRotInv(this.q, this.Bw);
      const mag = mB.map((b, i) => b + this.hardIron[i] + ri.gauss(M.sigmaMag));
      const vbat = ocv(this.soc) - (iAvg / 1000) * M.r0 - this.v1 + rp.gauss(0.001);
      const idraw = iAvg + rp.gauss(0.6);
      const tcpu = this.Tcpu + rp.gauss(0.25);
      const baroOk = !this.inFault('baro', t), imuOk = !this.inFault('imu', t);
      const g = this.gpsSol;
      return {
        TEMP: T,
        PRESSURE: baroOk ? pMeas / 100 : null,
        ALTITUDE: baroOk ? alt : null,
        AX: imuOk ? acc[0] : null, AY: imuOk ? acc[1] : null, AZ: imuOk ? acc[2] : null,
        GX: imuOk ? gyr[0] : null, GY: imuOk ? gyr[1] : null, GZ: imuOk ? gyr[2] : null,
        MX: imuOk ? mag[0] : null, MY: imuOk ? mag[1] : null, MZ: imuOk ? mag[2] : null,
        V_BAT: Math.round(vbat / 0.004) * 0.004,   // LSB de 4 mV del INA219
        I_DRAW: Math.round(idraw * 10) / 10,
        TEMP_CPU: tcpu,
        LAT: g.LAT, LON: g.LON, GPS_ALT: g.GPS_ALT, SATS: g.SATS, FIX_FLAG: g.FIX_FLAG,
      };
    }

    channel() {
      const r = this.rng.radio, ch = this.ch;
      if (ch.bad) { if (r.chance(ch.b2g)) ch.bad = false; } else if (r.chance(ch.g2b)) ch.bad = true;
      let pLoss = ch.bad ? ch.pB : ch.pG;
      if (this.phase === 'ASCENSO') pLoss += MODEL.extraLossAscent;
      if (this.phase === 'ATERRIZADO') pLoss += MODEL.extraLossLanded;
      const u = r.next(), u2 = r.next();
      if (u < Math.min(0.98, pLoss)) return 'lost';
      if (u2 < MODEL.pCorrupt) return 'corrupt';
      return 'ok';
    }

    /* Ciclo del firmware: leer → empaquetar → MicroSD → radio. */
    emitFrame() {
      const t = this.t, cfg = this.cfg, fw = this.rng.fw;
      this.k += 1;
      const k = this.k;
      const ts = MODEL.bootMs + Math.round(t * 1000) + Math.floor(fw.uniform(0, 3));
      const span = t - this.tAccStart;
      const iAvg = span > 1e-6 ? this.qAcc / span : MODEL.iMcu + MODEL.iSensors + MODEL.iGpsTrack + MODEL.iRadioIdle + MODEL.iSdIdle;
      this.qAcc = 0; this.tAccStart = t;
      const values = this.sampleSensors(t, iAvg);
      const line = encodeFrame(cfg.teamId, k, ts, values);
      const bytes = line.length;

      // MicroSD: búfer en RAM y volcado cada segundo (bloqueante)
      let sdStall = false;
      if (k % cfg.rateHz === 0) {
        let busy = MODEL.sdFlushS;
        if (fw.chance(MODEL.sdStallP)) { busy += fw.uniform(0.02, 0.12); sdStall = true; }
        this.sdBusyUntil = t + busy;
      }

      // Radio: si el transmisor sigue ocupado con tramas anteriores, la cola se desborda
      const airtime = (bytes * 10) / cfg.baud + MODEL.radioOverhead;
      const txStart = Math.max(t, this.txUntil);
      let status, tRx;
      if (txStart - t > this.period) { status = 'overflow'; tRx = t; }
      else { this.txUntil = txStart + airtime; status = this.channel(); tRx = txStart + airtime; }
      const rxLine = status === 'corrupt' ? corruptLine(line, this.rng.radio) : line;

      this.sd.push({ k, t, ts, line, values, bytes, sdStall });
      this.truth.push({
        k, t, ts, phase: this.phase, h: this.h, v: this.v, tilt: this.tiltDeg(),
        canN: this.canN, canE: this.canE, Tair: this.atm.T(this.h) - 273.15,
        p: this.atm.p(this.h), fix: this.gpsFix ? 1 : 0, soc: this.soc,
      });
      this.rx.push({ k, t, tRx, status, line: rxLine, airtime });

      this.nextSched += this.period;
      this.nextAcq = Math.max(this.nextSched, this.sdBusyUntil);
    }

    advanceTo(tTarget) {
      while (!this.done && this.t + MODEL.dt <= tTarget + 1e-9) this.step();
    }

    runToEnd() {
      while (!this.done) this.step();
      return this;
    }

    sdCsv() { return CSV_HEADER + '\n' + this.sd.map((r) => r.line).join(''); }
  }

  /* Ángulo inicial del péndulo en la tensión de cordones (± grados, gaussiano). */
  function gaussDeg(sim, sdDeg) { return sim.rng.att.gauss(sdDeg * DEG); }

  /* Errores de canal: bytes perdidos (trama truncada) o un byte basura. */
  function corruptLine(line, r) {
    const body = line.replace(/\n$/, '');
    if (r.chance(0.5)) return body.slice(0, Math.floor(r.uniform(0.25, 0.9) * body.length)) + '\n';
    const pos = Math.floor(r.uniform(0, body.length));
    const junk = '#?~@%&*^'[Math.floor(r.uniform(0, 8))];
    return body.slice(0, pos) + junk + body.slice(pos + 1) + '\n';
  }

  /* ───────────── 7. Estación de tierra: parser y contabilidad ───────────── */
  const NUM_RE = /^-?\d+(\.\d+)?$/;
  const INT_FIELDS = ['PACKET_COUNT', 'TIMESTAMP', 'SATS', 'FIX_FLAG'];

  function parseFrame(line, teamId) {
    const raw = String(line).replace(/\r?\n$/, '');
    const parts = raw.split(',');
    if (parts.length !== FIELDS.length) return { ok: false, error: `${parts.length} campos; se esperan ${FIELDS.length}` };
    if (teamId && parts[0] !== teamId) return { ok: false, error: `TEAM_ID «${parts[0]}» no coincide` };
    const rec = { TEAM_ID: parts[0] };
    for (let i = 1; i < parts.length; i++) {
      const s = parts[i];
      if (!NUM_RE.test(s)) return { ok: false, error: `${FIELDS[i]} no es numérico («${s}»)` };
      const x = Number(s);
      if (INT_FIELDS.includes(FIELDS[i]) && !Number.isInteger(x)) return { ok: false, error: `${FIELDS[i]} debe ser entero` };
      rec[FIELDS[i]] = x === MISSING ? null : x;
    }
    if (rec.PACKET_COUNT === null || rec.TIMESTAMP === null) return { ok: false, error: 'PACKET_COUNT o TIMESTAMP ausentes' };
    return { ok: true, rec };
  }

  class GroundStation {
    constructor(teamId, rateHz, elevM) {
      this.teamId = teamId;
      this.period = 1 / rateHz;
      this.elevM = elevM;
      this.records = [];
      this.log = [];          // entradas de consola {kind, text, note}
      this.rawLines = [];     // todo lo recibido, tal cual
      this.stats = { ok: 0, corrupt: 0, lost: 0, gaps: 0, resets: 0, timing: 0, missing: 0 };
      this.last = null;
      this.corruptPending = 0;
    }

    ingest(line) {
      this.rawLines.push(line);
      const res = parseFrame(line, this.teamId);
      if (!res.ok) {
        this.stats.corrupt++;
        this.corruptPending++;
        this.log.push({ kind: 'bad', text: line.replace(/\n$/, ''), note: 'Descartada: ' + res.error });
        return res;
      }
      const r = res.rec;
      const prevCount = this.last ? this.last.PACKET_COUNT : 0;
      const dc = r.PACKET_COUNT - prevCount;
      if (this.last && dc <= 0) {
        this.stats.resets++;
        this.log.push({ kind: 'warn', text: `PACKET_COUNT retrocede (${prevCount} → ${r.PACKET_COUNT}): ¿reinicio del microcontrolador?` });
      } else if (dc > 1) {
        const missing = dc - 1;
        this.stats.missing += missing;
        this.stats.lost += Math.max(0, missing - this.corruptPending);
        this.stats.gaps++;
        const range = missing === 1 ? `#${prevCount + 1}` : `#${prevCount + 1}–#${r.PACKET_COUNT - 1}`;
        this.log.push({ kind: 'gap', text: `— ${missing} ${missing === 1 ? 'paquete no recibido' : 'paquetes no recibidos'} (${range}) —` });
      }
      if (this.last && dc > 0) {
        const dts = (r.TIMESTAMP - this.last.TIMESTAMP) / 1000;
        if (Math.abs(dts - dc * this.period) > 0.5 * this.period) {
          this.stats.timing++;
          this.log.push({ kind: 'warn', text: `Hueco temporal sin salto de contador: Δt = ${dts.toFixed(3)} s` });
        }
      }
      this.corruptPending = 0;
      r.x = r.TIMESTAMP / 1000;
      r.GPS_AGL = r.GPS_ALT === null ? null : r.GPS_ALT - this.elevM;
      r.vBaro = this.verticalSpeed(r);
      this.records.push(r);
      this.last = r;
      this.stats.ok++;
      this.log.push({ kind: 'ok', text: line.replace(/\n$/, '') });
      return res;
    }

    /* Velocidad vertical (positiva bajando) por mínimos cuadrados sobre ~1 s de altitud. */
    verticalSpeed(r) {
      if (r.ALTITUDE === null) return null;
      const win = Math.max(1.0, 3 * this.period);
      const pts = [{ x: r.x, y: r.ALTITUDE }];
      for (let i = this.records.length - 1; i >= 0; i--) {
        const p = this.records[i];
        if (r.x - p.x > win) break;
        if (p.ALTITUDE !== null) pts.push({ x: p.x, y: p.ALTITUDE });
      }
      if (pts.length < 3) return null;
      return -linfit(pts).slope;
    }

    received() { return this.stats.ok + this.stats.corrupt; }
  }

  function linfit(pts) {
    const n = pts.length;
    let sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (const p of pts) { sx += p.x; sy += p.y; sxx += p.x * p.x; sxy += p.x * p.y; }
    const den = n * sxx - sx * sx;
    const slope = den === 0 ? 0 : (n * sxy - sx * sy) / den;
    return { slope, intercept: (sy - slope * sx) / n };
  }

  /* ───────────── 8. Validación (criterios de la sección 7) ───────────── */
  function validate(sim, gs) {
    const d = sim.derived, M = MODEL, T = sim.period;
    const inRange = (x) => x >= 8 && x <= 11;
    const landed = sim.tLand !== null;
    const sd = sim.sd, truth = sim.truth;
    const fmt = (x, n = 1) => x.toFixed(n).replace('.', ',');
    const out = [];
    const descentRows = (from) => sd.map((r, i) => ({ r, tr: truth[i] }))
      .filter(({ r }) => sim.tApogee !== null && r.t >= sim.tApogee + from && (sim.tLand === null || r.t < sim.tLand));

    // 1 · Velocidad terminal
    {
      const theoOk = inRange(d.vtTop) && inRange(d.vtGround);
      let measured = null;
      if (landed) {
        const rows = descentRows(M.tDeploy + M.tInflate + 4).filter(({ r }) => r.t < sim.tLand - 1 && r.values.ALTITUDE !== null);
        if (rows.length > 5) measured = -linfit(rows.map(({ r }) => ({ x: r.t, y: r.values.ALTITUDE }))).slope;
      }
      const ok = theoOk && (measured === null || inRange(measured));
      out.push({
        id: 'vt', title: 'Velocidad terminal entre 8 y 11 m/s',
        value: `Teórica ${fmt(d.vtTop, 2)} → ${fmt(d.vtGround, 2)} m/s` + (measured !== null ? ` · medida ${fmt(measured, 2)} m/s` : ''),
        status: !theoOk ? 'fail' : !landed ? 'pending' : ok ? 'ok' : 'fail',
      });
    }

    // 2 · Altitud barométrica frente a la del modelo (±5 m)
    {
      const rows = descentRows(0).filter(({ r }) => r.values.ALTITUDE !== null);
      let maxErr = 0, sq = 0;
      for (const { r, tr } of rows) { const e = r.values.ALTITUDE - tr.h; sq += e * e; if (Math.abs(e) > Math.abs(maxErr)) maxErr = e; }
      const rms = rows.length ? Math.sqrt(sq / rows.length) : 0;
      out.push({
        id: 'alt', title: 'Altitud barométrica = altitud real ± 5 m',
        value: rows.length ? `Error máximo ${maxErr >= 0 ? '+' : ''}${fmt(maxErr, 2)} m · RMS ${fmt(rms, 2)} m (${rows.length} tramas)` : 'Sin datos de descenso todavía',
        status: !rows.length ? 'pending' : Math.abs(maxErr) > 5 ? 'fail' : landed ? 'ok' : 'pending',
      });
    }

    // 3 · Presión monótona creciente en el descenso
    {
      const tol = 0.05; // hPa ≈ 3σ del sensor
      const rows = descentRows(M.tDeploy).filter(({ r }) => r.values.PRESSURE !== null);
      let inv = 0, strict = 0;
      for (let i = 1; i < rows.length; i++) {
        const dp = rows[i].r.values.PRESSURE - rows[i - 1].r.values.PRESSURE;
        if (dp < 0) strict++;
        if (dp < -tol) inv++;
      }
      const dP = rows.length > 1 ? rows[rows.length - 1].r.values.PRESSURE - rows[0].r.values.PRESSURE : 0;
      out.push({
        id: 'p', title: 'Presión creciente durante todo el descenso',
        value: rows.length > 1 ? `+${fmt(dP, 2)} hPa · ${inv} inversiones mayores que ${fmt(tol, 2)} hPa (3σ) · ${strict} estrictas` : 'Sin datos de descenso todavía',
        status: rows.length < 2 ? 'pending' : inv > 0 ? 'fail' : landed ? 'ok' : 'pending',
      });
    }

    // 4 · Temperatura aumenta al bajar
    {
      const rows = descentRows(5);
      let slope = null;
      if (rows.length > 5) {
        const pts = rows.filter(({ r }) => r.values.ALTITUDE !== null).map(({ r }) => ({ x: r.values.ALTITUDE / 1000, y: r.values.TEMP }));
        if (pts.length > 5) slope = linfit(pts).slope;
      }
      out.push({
        id: 'T', title: 'La temperatura externa sube al descender',
        value: slope === null ? 'Sin datos de descenso todavía' : `Gradiente medido ${fmt(slope, 2)} °C/km · atmósfera del día ${fmt(-sim.cfg.lapseKkm || 0, 1)} °C/km · ISA −6,5 °C/km`,
        status: slope === null ? 'pending' : slope >= 0 ? 'fail' : landed ? 'ok' : 'pending',
      });
    }

    // 5 · Frecuencia de emisión y huecos
    {
      const n = sd.length;
      let maxDev = 0, unjust = 0;
      for (let i = 1; i < n; i++) {
        const dts = (sd[i].ts - sd[i - 1].ts) / 1000;
        maxDev = Math.max(maxDev, Math.abs(dts - T));
        if (dts > 1.5 * T || sd[i].k !== sd[i - 1].k + 1) unjust++;
      }
      const rate = n > 1 ? (n - 1) / ((sd[n - 1].ts - sd[0].ts) / 1000) : 0;
      const groundUnjust = gs ? gs.stats.timing + gs.stats.resets : 0;
      const lostTxt = gs ? ` · Tierra: ${gs.stats.missing} sin recibir (${gs.stats.lost} perdidos, ${gs.stats.corrupt} corruptos), todos localizados por PACKET_COUNT` : '';
      out.push({
        id: 'rate', title: `Tramas a ${sim.cfg.rateHz} Hz sin huecos injustificados`,
        value: `MicroSD: ${n} tramas a ${fmt(rate, 3)} Hz, desviación máx. ${Math.round(maxDev * 1000)} ms, ${unjust} huecos${lostTxt}`,
        status: unjust > 0 || groundUnjust > 0 || Math.abs(rate - sim.cfg.rateHz) / sim.cfg.rateHz > 0.02 ? 'fail' : sim.done ? 'ok' : 'pending',
      });
    }

    // Comprobaciones informativas
    const info = [];
    if (landed) {
      const tDesc = sim.tLand - sim.tApogee;
      info.push({ id: 'dur', title: 'Duración del descenso (objetivo 90–120 s)', value: `${fmt(tDesc, 1)} s`, status: tDesc >= 90 && tDesc <= 120 ? 'ok' : 'warn' });
      const dist = Math.hypot(sim.pN, sim.pE);
      const brg = ((Math.atan2(sim.pE, sim.pN) / DEG) + 360) % 360;
      info.push({ id: 'drift', title: 'Deriva por viento', value: `${Math.round(dist)} m hacia ${Math.round(brg)}°`, status: 'info' });
    }
    const occ = d.occupancy;
    info.push({
      id: 'link', title: 'Ocupación del enlace de radio',
      value: `${Math.round(occ * 100)} % (${d.frameBytes} B por trama a ${sim.cfg.baud} bit/s)` + (sim.rx.some((e) => e.status === 'overflow') ? ' · la cola se desborda y se pierden tramas' : ''),
      status: occ >= 1 ? 'fail' : occ > 0.7 ? 'warn' : 'ok',
    });
    return { criteria: out, info };
  }

  /* ───────────── API pública del núcleo ───────────── */
  const Core = {
    G, R_UNIV, M_AIR, R_SPEC, L_ISA, BARO_EXP, DEFAULTS, MODEL, FIELDS, DECIMALS, MISSING, CSV_HEADER, PHASES,
    Rng, makeAtmosphere, baroAltitude, ocv, normalizeConfig, derive, encodeFrame, parseFrame,
    CanSatSim, GroundStation, validate, linfit,
  };
  if (typeof module !== 'undefined' && module.exports) module.exports = Core;
  global.CanSatCore = Core;

  if (typeof document === 'undefined') return;

  /* =========================================================================
   * 9. Interfaz de navegador
   * ========================================================================= */
  const $ = (s, r = document) => r.querySelector(s);
  const $$ = (s, r = document) => Array.from(r.querySelectorAll(s));
  const es = (x, n = 1) => (x === null || x === undefined || !Number.isFinite(x) ? '—' : x.toFixed(n).replace('.', ','));

  const ui = {
    sim: null, gs: null, rxIdx: 0, playing: false, paused: false, speed: 2,
    clock: 0, lastNow: 0, raf: 0, view: 'descent', logTab: 'rx', mode: 'panel',
    charts: [], dirtyCharts: true, dirtyLog: true, lastChartDraw: 0, lastAnimData: 0, animEvents: -1, theme: null,
    reducedMotion: typeof window.matchMedia === 'function' && window.matchMedia('(prefers-reduced-motion: reduce)').matches,
  };

  /* Paleta leída de los tokens CSS (se relee al cambiar de tema). */
  function readTheme() {
    const cs = getComputedStyle(document.documentElement);
    const v = (n) => cs.getPropertyValue(n).trim();
    ui.theme = {
      panel: v('--panel'), ink: v('--ink'), ink2: v('--ink-2'), muted: v('--muted'), grid: v('--grid'), axis: v('--axis'),
      s1: v('--s1'), s2: v('--s2'), s3: v('--s3'), ref: v('--ref'), band: v('--band'), shade: v('--shade'),
      accent: v('--accent'), good: v('--good'), warn: v('--warn'), bad: v('--bad'), seg1: v('--seg-1'), seg2: v('--seg-2'),
      line: v('--line'), accentSoft: v('--accent-soft'), accentInk: v('--accent-ink'),
      skyTop: v('--sky-top'), skyBot: v('--sky-bot'), skyGrid: v('--sky-grid'), hill: v('--hill'), ground: v('--ground'),
      streak: v('--streak'), smoke: v('--smoke'), rocket: v('--rocket'), can: v('--can'),
      mono: v('--font-mono') || 'monospace', body: v('--font-body') || 'sans-serif',
    };
  }

  /* ─── Definición de gráficas ─── */
  const truthX = (tr) => tr.ts / 1000;
  const CHART_SPECS = {
    alt: { unit: 'm', series: [
      { key: 'ALTITUDE', label: 'Barométrica (ALTITUDE)', color: 's1' },
      { key: 'GPS_AGL', label: 'GPS sobre la rampa', color: 's2' },
      { key: 'h', label: 'Modelo', color: 'ref', truth: true, dash: [5, 4] }] },
    vel: { unit: 'm/s', band: [8, 11], yDomain: [-2, 16], series: [
      { key: 'vBaro', label: 'Derivada en tierra', color: 's1' },
      { key: 'v', label: 'Modelo', color: 'ref', truth: true, dash: [5, 4] }] },
    pres: { unit: 'hPa', decimals: 2, series: [{ key: 'PRESSURE', label: 'PRESSURE', color: 's1' }] },
    temp: { unit: '°C', series: [
      { key: 'TEMP', label: 'Sonda (TEMP)', color: 's1' },
      { key: 'Tair', label: 'Aire del modelo', color: 'ref', truth: true, dash: [5, 4] }] },
    acc: { unit: 'm/s²', series: [
      { key: 'AX', label: 'AX', color: 's1' }, { key: 'AY', label: 'AY', color: 's2' }, { key: 'AZ', label: 'AZ', color: 's3' }] },
    gyr: { unit: '°/s', series: [
      { key: 'GX', label: 'GX', color: 's1' }, { key: 'GY', label: 'GY', color: 's2' }, { key: 'GZ', label: 'GZ', color: 's3' }] },
    mag: { unit: 'µT', series: [
      { key: 'MX', label: 'MX', color: 's1' }, { key: 'MY', label: 'MY', color: 's2' }, { key: 'MZ', label: 'MZ', color: 's3' }] },
    vbat: { unit: 'V', decimals: 3, minSpan: 0.05, series: [{ key: 'V_BAT', label: 'V_BAT', color: 's1' }] },
    idraw: { unit: 'mA', series: [{ key: 'I_DRAW', label: 'I_DRAW', color: 's1' }] },
    tcpu: { unit: '°C', series: [
      { key: 'TEMP_CPU', label: 'TEMP_CPU', color: 's1' },
      { key: 'Tair', label: 'Aire del modelo', color: 'ref', truth: true, dash: [5, 4] }] },
    sats: { unit: 'satélites', step: true, decimals: 0, yDomain: [0, 13], noFixShade: true, series: [{ key: 'SATS', label: 'SATS', color: 's1' }] },
  };

  function niceStep(range, target) {
    const raw = range / Math.max(1, target);
    const mag = Math.pow(10, Math.floor(Math.log10(raw)));
    const n = raw / mag;
    return (n < 1.5 ? 1 : n < 3 ? 2 : n < 7 ? 5 : 10) * mag;
  }

  function setupCanvas(canvas) {
    const dpr = window.devicePixelRatio || 1;
    const w = Math.max(10, canvas.clientWidth), h = Math.max(10, canvas.clientHeight);
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
    }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    return { ctx, w, h };
  }

  /* Rango temporal común a todas las gráficas (s desde el arranque = TIMESTAMP). */
  function xRange() {
    const sim = ui.sim, boot = MODEL.bootMs / 1000;
    if (ui.view === 'descent' && sim.tApogee !== null) {
      const end = sim.tLand !== null ? sim.tLand + 3 : sim.tApogee + sim.derived.tDescent + 3;
      return [boot + sim.tApogee - 2, boot + Math.max(end, sim.t)];
    }
    return [boot, boot + Math.max(sim.derived.tEnd, sim.t)];
  }

  class LineChart {
    constructor(fig) {
      this.fig = fig;
      this.id = fig.dataset.chart;
      this.spec = CHART_SPECS[this.id];
      this.canvas = $('canvas', fig);
      this.legend = $('.legend', fig);
      this.hoverX = null;
      this.items = [];
      this.legend.textContent = '';
      for (const s of this.spec.series) {
        const li = document.createElement('li');
        const key = document.createElement('i');
        key.className = 'key' + (s.dash ? ' key-dash' : '');
        key.style.setProperty('--c', `var(--${s.color})`);
        const lab = document.createElement('span');
        lab.textContent = s.label;
        const val = document.createElement('b');
        val.textContent = '—';
        li.append(key, lab, val);
        this.legend.append(li);
        this.items.push(val);
      }
      if (this.spec.series.length === 1) this.legend.classList.add('legend-single');
      const move = (e) => {
        const rect = this.canvas.getBoundingClientRect();
        this.hoverX = e.clientX - rect.left;
        this.draw();
      };
      this.canvas.addEventListener('pointermove', move);
      this.canvas.addEventListener('pointerdown', move);
      this.canvas.addEventListener('pointerleave', () => { this.hoverX = null; this.draw(); });
    }

    seriesData(s) {
      if (s.truth) {
        const pts = ui.sim.truth.map((tr) => ({ x: truthX(tr), y: tr[s.key] }));
        return pts;
      }
      return ui.gs.records.map((r) => ({ x: r.x, y: r[s.key] }));
    }

    draw() {
      const th = ui.theme, spec = this.spec;
      const { ctx, w, h } = setupCanvas(this.canvas);
      const pad = { l: 48, r: 12, t: 10, b: 22 };
      const [x0, x1] = xRange();
      const data = spec.series.map((s) => this.seriesData(s).filter((p) => p.x >= x0 - 1 && p.x <= x1 + 1));
      // Escala Y
      let y0, y1;
      if (spec.yDomain) [y0, y1] = spec.yDomain;
      else {
        y0 = Infinity; y1 = -Infinity;
        for (const arr of data) for (const p of arr) if (p.y !== null && Number.isFinite(p.y)) { y0 = Math.min(y0, p.y); y1 = Math.max(y1, p.y); }
        if (spec.band) { y0 = Math.min(y0, spec.band[0]); y1 = Math.max(y1, spec.band[1]); }
        if (!Number.isFinite(y0)) { y0 = 0; y1 = 1; }
        const span = Math.max(y1 - y0, spec.minSpan || 0.5);
        const mid = (y0 + y1) / 2;
        y0 = mid - span * 0.56; y1 = mid + span * 0.56;
      }
      const yStep = niceStep(y1 - y0, h > 200 ? 5 : 4);
      const X = (x) => pad.l + ((x - x0) / (x1 - x0)) * (w - pad.l - pad.r);
      const Y = (y) => h - pad.b - ((y - y0) / (y1 - y0)) * (h - pad.t - pad.b);

      ctx.font = `11px ${th.mono}`;
      ctx.textBaseline = 'middle';
      // Rejilla horizontal y etiquetas Y
      ctx.lineWidth = 1;
      for (let v = Math.ceil(y0 / yStep) * yStep; v <= y1 + 1e-9; v += yStep) {
        const yy = Math.round(Y(v)) + 0.5;
        ctx.strokeStyle = th.grid;
        ctx.beginPath(); ctx.moveTo(pad.l, yy); ctx.lineTo(w - pad.r, yy); ctx.stroke();
        ctx.fillStyle = th.muted;
        ctx.textAlign = 'right';
        const dec = yStep < 0.01 ? 3 : yStep < 0.1 ? 2 : yStep < 1 ? 1 : 0;
        ctx.fillText(es(v, dec), pad.l - 6, yy);
      }
      // Eje X
      const xStep = niceStep(x1 - x0, Math.max(3, Math.floor((w - pad.l) / 90)));
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      for (let v = Math.ceil(x0 / xStep) * xStep; v <= x1 + 1e-9; v += xStep) {
        const xx = Math.round(X(v)) + 0.5;
        ctx.strokeStyle = th.grid;
        ctx.beginPath(); ctx.moveTo(xx, pad.t); ctx.lineTo(xx, h - pad.b); ctx.stroke();
        ctx.fillStyle = th.muted;
        ctx.fillText(String(Math.round(v)), xx, h - pad.b + 5);
      }
      ctx.strokeStyle = th.axis;
      ctx.beginPath(); ctx.moveTo(pad.l, h - pad.b + 0.5); ctx.lineTo(w - pad.r, h - pad.b + 0.5); ctx.stroke();

      ctx.save();
      ctx.beginPath(); ctx.rect(pad.l, pad.t, w - pad.l - pad.r, h - pad.t - pad.b); ctx.clip();
      // Banda objetivo
      if (spec.band) {
        ctx.fillStyle = th.band;
        ctx.fillRect(pad.l, Y(spec.band[1]), w - pad.l - pad.r, Y(spec.band[0]) - Y(spec.band[1]));
        ctx.fillStyle = th.ink2;
        ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';
        ctx.fillText(`objetivo ${spec.band[0]}–${spec.band[1]} m/s`, pad.l + 6, Y(spec.band[1]) - 2);
      }
      // Ventanas sin fix
      if (spec.noFixShade) {
        ctx.fillStyle = th.shade;
        for (const r of ui.gs.records) if (r.FIX_FLAG === 0) {
          const a = X(r.x - ui.sim.period / 2), b = X(r.x + ui.sim.period / 2);
          ctx.fillRect(a, pad.t, b - a + 0.5, h - pad.t - pad.b);
        }
      }
      // Hitos de vuelo
      ctx.setLineDash([2, 3]);
      ctx.strokeStyle = th.axis;
      for (const e of ui.sim.events) if (['launch', 'apogee', 'land'].includes(e.type)) {
        const xx = Math.round(X(e.ts / 1000)) + 0.5;
        ctx.beginPath(); ctx.moveTo(xx, pad.t); ctx.lineTo(xx, h - pad.b); ctx.stroke();
      }
      ctx.setLineDash([]);
      // Series
      const maxGap = 3.5 * ui.sim.period;
      spec.series.forEach((s, i) => {
        const pts = data[i];
        ctx.strokeStyle = th[s.color];
        ctx.lineWidth = s.dash ? 1.5 : 2;
        ctx.lineJoin = 'round'; ctx.lineCap = 'round';
        ctx.setLineDash(s.dash || []);
        ctx.beginPath();
        let pen = false, px = null, py = null;
        for (const p of pts) {
          if (p.y === null || !Number.isFinite(p.y) || (px !== null && p.x - px > maxGap && !s.truth)) { pen = false; if (p.y === null) { px = p.x; continue; } }
          const xx = X(p.x), yy = Y(p.y);
          if (!pen) { ctx.moveTo(xx, yy); pen = true; }
          else if (spec.step) { ctx.lineTo(xx, Y(py)); ctx.lineTo(xx, yy); }
          else ctx.lineTo(xx, yy);
          px = p.x; py = p.y;
        }
        ctx.stroke();
      });
      ctx.setLineDash([]);
      ctx.restore();

      // Capa de lectura: cursor o último valor
      let hx = null;
      if (this.hoverX !== null && this.hoverX >= pad.l && this.hoverX <= w - pad.r) {
        hx = x0 + ((this.hoverX - pad.l) / (w - pad.l - pad.r)) * (x1 - x0);
        ctx.strokeStyle = th.ink2; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(Math.round(this.hoverX) + 0.5, pad.t); ctx.lineTo(Math.round(this.hoverX) + 0.5, h - pad.b); ctx.stroke();
        ctx.fillStyle = th.ink; ctx.textAlign = 'left'; ctx.textBaseline = 'top';
        ctx.font = `11px ${th.mono}`;
        const label = `${es(hx, 1)} s`;
        const tx = Math.min(this.hoverX + 6, w - pad.r - ctx.measureText(label).width - 2);
        ctx.fillText(label, tx, pad.t + 2);
      }
      const dec = spec.decimals !== undefined ? spec.decimals : 1;
      spec.series.forEach((s, i) => {
        const pts = data[i].filter((p) => p.y !== null && Number.isFinite(p.y));
        let p = null;
        if (hx !== null) {
          let best = Infinity;
          for (const q of pts) { const dd = Math.abs(q.x - hx); if (dd < best) { best = dd; p = q; } }
          if (p) {
            ctx.fillStyle = th[s.color]; ctx.strokeStyle = th.panel; ctx.lineWidth = 2;
            ctx.beginPath(); ctx.arc(X(p.x), clamp(Y(p.y), pad.t, h - pad.b), 4, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
          }
        } else p = pts[pts.length - 1] || null;
        this.items[i].textContent = p ? `${es(p.y, dec)} ${spec.unit === 'satélites' ? '' : spec.unit}`.trim() : '—';
      });
    }
  }

  /* ─── Línea de tiempo de la misión: fase, fix GPS y enlace de radio ─── */
  function drawTimeline() {
    const canvas = $('#cv-timeline');
    if (!canvas) return;
    const th = ui.theme, sim = ui.sim, boot = MODEL.bootMs / 1000;
    const { ctx, w, h } = setupCanvas(canvas);
    const padL = 74, padR = 12;
    const x0 = boot, x1 = boot + Math.max(sim.derived.tEnd, sim.t);
    const X = (x) => padL + ((x - x0) / (x1 - x0)) * (w - padL - padR);
    const rows = [{ label: 'Fase', y: 6, hgt: 22 }, { label: 'GPS', y: 36, hgt: 14 }, { label: 'Radio', y: 58, hgt: 18 }];
    ctx.font = `600 11px ${th.body}`;
    ctx.textBaseline = 'middle';
    ctx.textAlign = 'left';
    ctx.fillStyle = th.ink2;
    for (const r of rows) ctx.fillText(r.label.toUpperCase(), 0, r.y + r.hgt / 2);

    // Fases
    const now = boot + sim.t;
    const segs = [
      ['En rampa', 0, sim.tLaunch], ['Ascenso', sim.tLaunch, sim.tApogee], ['Expulsión', sim.tApogee, sim.tOpen],
      ['Descenso', sim.tOpen, sim.tLand], ['Aterrizado', sim.tLand, sim.tEnd]];
    ctx.font = `500 11px ${th.body}`;
    segs.forEach(([name, a, b], i) => {
      if (a === null || a > sim.t) return;
      const xa = X(boot + a), xb = X(Math.min(now, boot + (b === null ? sim.t : b)));
      ctx.fillStyle = i % 2 ? th.seg2 : th.seg1;
      ctx.fillRect(xa, rows[0].y, Math.max(1, xb - xa - 1), rows[0].hgt);
      ctx.fillStyle = th.ink;
      if (ctx.measureText(name).width + 8 < xb - xa) ctx.fillText(name, xa + 5, rows[0].y + rows[0].hgt / 2);
    });
    // GPS: fix / sin fix
    const r1 = rows[1];
    ctx.fillStyle = th.good;
    ctx.fillRect(X(boot), r1.y + 4, X(now) - X(boot), r1.hgt - 8);
    for (const win of sim.noFixWindows) {
      if (win.start > sim.t) continue;
      const xa = X(boot + win.start), xb = X(boot + Math.min(win.end, sim.t));
      ctx.fillStyle = th.panel;
      ctx.fillRect(xa, r1.y, xb - xa, r1.hgt);
      ctx.fillStyle = th.warn;
      ctx.fillRect(xa, r1.y, xb - xa, r1.hgt);
    }
    // Radio: un trazo por paquete
    const r2 = rows[2];
    for (let i = 0; i < ui.rxIdx; i++) {
      const e = sim.rx[i];
      const xx = Math.round(X(boot + e.t)) + 0.5;
      if (e.status === 'ok') { ctx.strokeStyle = th.axis; ctx.beginPath(); ctx.moveTo(xx, r2.y + 5); ctx.lineTo(xx, r2.y + r2.hgt - 5); ctx.stroke(); }
    }
    for (let i = 0; i < ui.rxIdx; i++) {
      const e = sim.rx[i];
      if (e.status === 'ok') continue;
      const xx = Math.round(X(boot + e.t)) + 0.5;
      ctx.strokeStyle = e.status === 'corrupt' ? th.warn : th.bad;
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(xx, r2.y); ctx.lineTo(xx, r2.y + r2.hgt); ctx.stroke();
      ctx.lineWidth = 1;
    }
    // Instante actual
    if (!sim.done) {
      ctx.strokeStyle = th.accent; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(X(now), 2); ctx.lineTo(X(now), h - 16); ctx.stroke();
      ctx.lineWidth = 1;
    }
    // Eje de tiempo
    ctx.font = `11px ${th.mono}`;
    ctx.fillStyle = th.muted; ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
    const step = niceStep(x1 - x0, Math.max(3, Math.floor((w - padL) / 80)));
    for (let v = Math.ceil(x0 / step) * step; v <= x1; v += step) ctx.fillText(`${Math.round(v)} s`, X(v), h);
  }

  /* ─── Mapa en planta: deriva por viento y posiciones congeladas sin fix ─── */
  function drawMap() {
    const canvas = $('#cv-map');
    if (!canvas) return;
    const th = ui.theme, sim = ui.sim, cfg = sim.cfg;
    const { ctx, w, h } = setupCanvas(canvas);
    const toNE = (lat, lon) => [(lat - cfg.lat0) * DEG * R_EARTH, (lon - cfg.lon0) * DEG * R_EARTH * Math.cos(cfg.lat0 * DEG)];
    const gps = ui.gs.records.filter((r) => r.LAT !== null && r.LON !== null).map((r) => ({ ne: toNE(r.LAT, r.LON), fix: r.FIX_FLAG }));
    const tru = sim.truth.map((tr) => [tr.canN, tr.canE]);
    let nMin = -50, nMax = 50, eMin = -50, eMax = 50;
    for (const p of tru.concat(gps.map((g) => g.ne))) { nMin = Math.min(nMin, p[0]); nMax = Math.max(nMax, p[0]); eMin = Math.min(eMin, p[1]); eMax = Math.max(eMax, p[1]); }
    // Incluye el punto de aterrizaje previsto para que el mapa no salte durante la reproducción
    const tDesc = sim.derived.tDescent, wind = cfg.windMs * 1.6, to = (cfg.windFromDeg + 180) * DEG;
    const pred = [wind * tDesc * Math.cos(to), wind * tDesc * Math.sin(to)];
    nMin = Math.min(nMin, pred[0]); nMax = Math.max(nMax, pred[0]); eMin = Math.min(eMin, pred[1]); eMax = Math.max(eMax, pred[1]);
    const pad = 28;
    const spanE = (eMax - eMin) * 1.15, spanN = (nMax - nMin) * 1.15;
    const scale = Math.min((w - 2 * pad) / spanE, (h - 2 * pad) / spanN);
    const cE = (eMin + eMax) / 2, cN = (nMin + nMax) / 2;
    const X = (e) => w / 2 + (e - cE) * scale;
    const Y = (n) => h / 2 - (n - cN) * scale;
    // Rejilla
    const gstep = niceStep(Math.max(spanE, spanN), 6);
    ctx.lineWidth = 1;
    ctx.font = `11px ${th.mono}`;
    for (let e = Math.ceil((cE - w / 2 / scale) / gstep) * gstep; X(e) < w; e += gstep) {
      ctx.strokeStyle = Math.abs(e) < 1e-6 ? th.axis : th.grid;
      ctx.beginPath(); ctx.moveTo(Math.round(X(e)) + 0.5, 0); ctx.lineTo(Math.round(X(e)) + 0.5, h); ctx.stroke();
    }
    for (let n = Math.ceil((cN - h / 2 / scale) / gstep) * gstep; Y(n) > 0; n += gstep) {
      ctx.strokeStyle = Math.abs(n) < 1e-6 ? th.axis : th.grid;
      ctx.beginPath(); ctx.moveTo(0, Math.round(Y(n)) + 0.5); ctx.lineTo(w, Math.round(Y(n)) + 0.5); ctx.stroke();
    }
    // Rosa de viento y barra de escala en las esquinas más alejadas de la trayectoria
    const trackPx = tru.filter((_, i) => i % 5 === 0).map((p) => [X(p[1]), Y(p[0])])
      .concat([[X(0), Y(0)], [X(pred[1]), Y(pred[0])]]);
    const corners = [[64, 54], [w - 64, 54], [64, h - 42], [w - 64, h - 42]]
      .map((c) => ({ c, d: Math.min(...trackPx.map((p) => Math.hypot(p[0] - c[0], p[1] - c[1]))) }))
      .sort((a, b) => b.d - a.d);
    {
      const [ax, ay] = corners[0].c, len = 20;
      const dx = Math.sin(to) * len, dy = -Math.cos(to) * len;
      ctx.strokeStyle = th.ink2; ctx.fillStyle = th.ink2; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(ax - dx, ay - dy); ctx.lineTo(ax + dx, ay + dy); ctx.stroke();
      const ang = Math.atan2(dy, dx);
      ctx.beginPath();
      ctx.moveTo(ax + dx, ay + dy);
      ctx.lineTo(ax + dx - 7 * Math.cos(ang - 0.45), ay + dy - 7 * Math.sin(ang - 0.45));
      ctx.lineTo(ax + dx - 7 * Math.cos(ang + 0.45), ay + dy - 7 * Math.sin(ang + 0.45));
      ctx.closePath(); ctx.fill();
      ctx.textAlign = 'center'; ctx.textBaseline = 'top';
      ctx.fillText(`viento ${es(cfg.windMs, 0)} m/s · N ↑`, ax, ay + len + 6);
    }
    {
      const [cx, cy] = corners[1].c, bar = gstep * scale;
      ctx.strokeStyle = th.ink2; ctx.fillStyle = th.ink2; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(cx - bar / 2, cy + 8); ctx.lineTo(cx + bar / 2, cy + 8); ctx.stroke();
      ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
      ctx.fillText(`${gstep} m`, cx, cy + 4);
    }
    // Trayectoria real
    ctx.setLineDash([5, 4]); ctx.strokeStyle = th.ref; ctx.lineWidth = 1.5;
    ctx.beginPath();
    tru.forEach((p, i) => (i ? ctx.lineTo(X(p[1]), Y(p[0])) : ctx.moveTo(X(p[1]), Y(p[0]))));
    ctx.stroke(); ctx.setLineDash([]);
    // Trayectoria GPS recibida (solo con fix)
    ctx.strokeStyle = th.s1; ctx.lineWidth = 2; ctx.lineJoin = 'round';
    ctx.beginPath();
    let pen = false;
    for (const g of gps) {
      if (!g.fix) { pen = false; continue; }
      if (!pen) { ctx.moveTo(X(g.ne[1]), Y(g.ne[0])); pen = true; } else ctx.lineTo(X(g.ne[1]), Y(g.ne[0]));
    }
    ctx.stroke();
    // Posiciones congeladas durante No-Fix
    ctx.lineWidth = 2;
    for (const g of gps) if (!g.fix) {
      ctx.strokeStyle = th.s2; ctx.beginPath(); ctx.arc(X(g.ne[1]), Y(g.ne[0]), 5, 0, 2 * Math.PI); ctx.stroke();
    }
    // Rampa
    ctx.fillStyle = th.ink;
    ctx.beginPath(); ctx.moveTo(X(0), Y(0) - 7); ctx.lineTo(X(0) + 6, Y(0) + 5); ctx.lineTo(X(0) - 6, Y(0) + 5); ctx.closePath(); ctx.fill();
    // Posición actual o de aterrizaje
    const last = tru[tru.length - 1];
    if (last) {
      const lx = X(last[1]), ly = Y(last[0]);
      if (sim.tLand !== null) {
        ctx.strokeStyle = th.ink; ctx.lineWidth = 2.5;
        ctx.beginPath(); ctx.moveTo(lx - 6, ly - 6); ctx.lineTo(lx + 6, ly + 6); ctx.moveTo(lx + 6, ly - 6); ctx.lineTo(lx - 6, ly + 6); ctx.stroke();
      } else {
        ctx.fillStyle = th.accent; ctx.strokeStyle = th.panel; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(lx, ly, 5, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
      }
    }
    const capt = $('#map-caption');
    if (capt) {
      const d = Math.hypot(last ? last[0] : 0, last ? last[1] : 0);
      const brg = last ? ((Math.atan2(last[1], last[0]) / DEG) + 360) % 360 : 0;
      capt.textContent = `${sim.tLand !== null ? 'Aterrizaje' : 'Posición actual'} a ${Math.round(d)} m de la rampa, rumbo ${Math.round(brg)}°`;
    }
  }

  /* ─── Animación del vuelo: vista lateral en la dirección del viento ─── */
  function downwindAxis(cfg) {
    const to = (cfg.windFromDeg + 180) * DEG;
    return [Math.cos(to), Math.sin(to)];
  }

  function truthAt(sim, t) {
    const tr = sim.truth;
    if (!tr.length) return null;
    let lo = 0, hi = tr.length - 1;
    if (t >= tr[hi].t) return tr[hi];
    while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (tr[mid].t <= t) lo = mid; else hi = mid; }
    return tr[lo];
  }

  /* El cuerpo del cohete baja con su propio paracaídas a ~15 m/s.
     Es una trayectoria ilustrativa: no forma parte del modelo físico. */
  function rocketPose(sim, u) {
    if (sim.tApogee === null) return { x: 0, h: sim.h, landed: false, chute: 0, after: false };
    const tf = 1.2, vr = 15, tau = sim.t - sim.tApogee;
    const hDrop = 0.5 * G * tf * tf;
    const tLandR = tf + (sim.hApogee - hDrop) / vr;
    const tt = Math.min(tau, tLandR);
    const h = tt < tf ? sim.hApogee - 0.5 * G * tt * tt : sim.hApogee - hDrop - vr * (tt - tf);
    const ref = truthAt(sim, sim.tApogee + tt);
    const x = ref ? 0.9 * (ref.canN * u[0] + ref.canE * u[1]) : 0;
    return { x, h: Math.max(0, h), landed: tau >= tLandR, chute: clamp((tau - 0.6) / 0.8, 0, 1), after: true };
  }

  /* Cohete con la tobera en (x, y); angle 0 = morro hacia arriba. */
  function drawRocket(ctx, th, x, y, angle, flame, k) {
    const L = 44 * k, W = 9 * k;
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(angle);
    if (flame > 0) {
      const f = (12 + 12 * flame) * k;
      ctx.fillStyle = th.warn;
      ctx.beginPath(); ctx.moveTo(-W / 2 + k, 0); ctx.lineTo(0, f); ctx.lineTo(W / 2 - k, 0); ctx.closePath(); ctx.fill();
      ctx.fillStyle = th.accent;
      ctx.beginPath(); ctx.moveTo(-W / 4, 0); ctx.lineTo(0, f * 0.6); ctx.lineTo(W / 4, 0); ctx.closePath(); ctx.fill();
    }
    ctx.fillStyle = th.ink2;
    ctx.beginPath(); ctx.moveTo(-W / 2, -2 * k); ctx.lineTo(-W / 2 - 6 * k, 2 * k); ctx.lineTo(-W / 2, -13 * k); ctx.closePath(); ctx.fill();
    ctx.beginPath(); ctx.moveTo(W / 2, -2 * k); ctx.lineTo(W / 2 + 6 * k, 2 * k); ctx.lineTo(W / 2, -13 * k); ctx.closePath(); ctx.fill();
    ctx.fillStyle = th.rocket; ctx.strokeStyle = th.ink2; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.rect(-W / 2, -L, W, L); ctx.fill(); ctx.stroke();
    ctx.fillStyle = th.accent;
    ctx.fillRect(-W / 2, -L + 7 * k, W, 3 * k);   // bahía de carga del CanSat
    ctx.fillStyle = th.rocket;
    ctx.beginPath();
    ctx.moveTo(-W / 2, -L);
    ctx.quadraticCurveTo(-W / 2, -L - 10 * k, 0, -L - 16 * k);
    ctx.quadraticCurveTo(W / 2, -L - 10 * k, W / 2, -L);
    ctx.closePath(); ctx.fill(); ctx.stroke();
    ctx.restore();
  }

  /* Cúpula de paracaídas en (cx, cy) girada `ang`, con los cordones hasta (0, len) en ese marco. */
  function drawCanopy(ctx, th, cx, cy, ang, rw, len, color, soft) {
    const rh = rw * 0.62;
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(ang);
    ctx.strokeStyle = th.ink2; ctx.lineWidth = 1;
    ctx.beginPath();
    for (const fx of [-1, -0.4, 0.4, 1]) { ctx.moveTo(fx * rw, 0); ctx.lineTo(0, len); }
    ctx.stroke();
    ctx.fillStyle = color;
    ctx.beginPath(); ctx.ellipse(0, 0, rw, rh, 0, Math.PI, 2 * Math.PI); ctx.closePath(); ctx.fill();
    if (soft) {
      ctx.fillStyle = soft;
      ctx.beginPath(); ctx.ellipse(0, 0, rw * 0.45, rh, 0, Math.PI, 2 * Math.PI); ctx.closePath(); ctx.fill();
      ctx.fillStyle = color;
      ctx.beginPath(); ctx.ellipse(0, 0, rw * 0.14, rh, 0, Math.PI, 2 * Math.PI); ctx.closePath(); ctx.fill();
    }
    ctx.restore();
  }

  /* Lata centrada en (x, y) girada `ang` (eje Z del cuerpo hacia abajo en ang = 0). */
  function drawCan(ctx, th, x, y, ang, k) {
    const H = 14 * k, W = 9 * k;
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(ang);
    ctx.strokeStyle = th.ink2; ctx.lineWidth = 1.2;
    ctx.beginPath(); ctx.moveTo(0, -H / 2); ctx.lineTo(0, -H / 2 - 5 * k); ctx.stroke();  // antena GPS arriba
    ctx.fillStyle = th.can;
    ctx.beginPath();
    if (ctx.roundRect) ctx.roundRect(-W / 2, -H / 2, W, H, 2 * k); else ctx.rect(-W / 2, -H / 2, W, H);
    ctx.fill(); ctx.stroke();
    ctx.fillStyle = th.accent;
    ctx.fillRect(-W / 2, -H * 0.12, W, H * 0.24);
    ctx.restore();
  }

  /* Ángulo de pantalla de un vector NED proyectado sobre el plano vertical a favor del viento. */
  function screenAngle(vec, u) {
    return Math.atan2(-(vec[0] * u[0] + vec[1] * u[1]), vec[2]);
  }

  /* Lata con su paracaídas: el cordaje sigue el ángulo del péndulo del modelo. */
  function drawCanSatAssembly(ctx, th, sim, u, x, y, k) {
    const canAng = screenAngle(qRot(sim.q, [0, 0, 1]), u);
    const H = 14 * k;
    const landed = sim.phase === 'ATERRIZADO';
    if (!landed && sim.phi > 0) {
      const rAng = screenAngle(sim.riserDir(), u);
      const ax = x + Math.sin(canAng) * (H / 2 + k), ay = y - Math.cos(canAng) * (H / 2 + k);  // anclaje en la tapa superior
      const len = 34 * k;
      const cx = ax + Math.sin(rAng) * len, cy = ay - Math.cos(rAng) * len;
      drawCanopy(ctx, th, cx, cy, rAng, (4 + 14 * sim.phi) * k, len, th.accent, th.accentSoft);
    }
    if (landed) {
      ctx.fillStyle = th.accent;
      ctx.globalAlpha = 0.85;
      ctx.beginPath(); ctx.ellipse(x + 24 * k, y + 2 * k, 15 * k, 3 * k, 0, 0, 2 * Math.PI); ctx.fill();
      ctx.globalAlpha = 1;
      ctx.strokeStyle = th.ink2; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(x + 4 * k, y); ctx.lineTo(x + 10 * k, y + 2 * k); ctx.stroke();
    }
    drawCan(ctx, th, x, y, canAng, k);
    if (landed && sim.buzzerOn) {
      const ph = ui.reducedMotion ? 0.5 : (performance.now() / 700) % 1;
      ctx.strokeStyle = th.ink2; ctx.lineWidth = 1.2;
      for (let i = 0; i < 2; i++) {
        const r = (10 + 8 * ((ph + i * 0.5) % 1)) * k;
        ctx.globalAlpha = 1 - ((ph + i * 0.5) % 1);
        ctx.beginPath(); ctx.arc(x, y, r, -Math.PI * 0.85, -Math.PI * 0.15); ctx.stroke();
      }
      ctx.globalAlpha = 1;
    }
  }

  function drawScene() {
    const canvas = $('#cv-scene');
    if (!canvas || ui.mode !== 'anim' || !ui.sim) return;
    const th = ui.theme, sim = ui.sim, cfg = sim.cfg;
    const { ctx, w, h } = setupCanvas(canvas);
    const u = downwindAxis(cfg);
    const xv = (n, e) => n * u[0] + e * u[1];
    const now = performance.now() / 1000;

    // Encuadre fijo para toda la misión: altura y distancia con la misma escala
    const apo = Math.max(cfg.apogeeM, sim.hApogee || 0);
    let drift = cfg.windMs * 1.6 * sim.derived.tDescent;
    for (const r of sim.truth) drift = Math.max(drift, xv(r.canN, r.canE));
    drift = Math.max(drift, 200);
    const padL = 58, padR = 14, padT = 12, padB = 30;
    const plotW = w - padL - padR, plotH = h - padT - padB;
    const xMin = -0.12 * drift - 40;
    const s = Math.min(plotW / (drift * 1.12 + 80), plotH / (apo * 1.1));
    const xMax = xMin + plotW / s;
    const X = (x) => padL + (x - xMin) * s;
    const Y = (hh) => h - padB - hh * s;
    const gy = Y(0);

    // Cielo, colinas y suelo
    const sky = ctx.createLinearGradient(0, 0, 0, gy);
    sky.addColorStop(0, th.skyTop); sky.addColorStop(1, th.skyBot);
    ctx.fillStyle = sky; ctx.fillRect(0, 0, w, gy);
    ctx.fillStyle = th.hill;
    ctx.beginPath(); ctx.moveTo(0, gy);
    for (let px = 0; px <= w + 8; px += 8) ctx.lineTo(px, gy - 9 - 6 * Math.sin(px / 61) - 4 * Math.sin(px / 23 + 1.3));
    ctx.lineTo(w, gy); ctx.closePath(); ctx.fill();
    ctx.fillStyle = th.ground; ctx.fillRect(0, gy, w, h - gy);

    // Regla de altura y distancia sobre el suelo
    const hTop = (gy - padT) / s;
    const hs = niceStep(hTop, 5);
    ctx.font = `11px ${th.mono}`;
    ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
    for (let v = 0; v <= hTop; v += hs) {
      const yy = Math.round(Y(v)) + 0.5;
      ctx.strokeStyle = th.skyGrid; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(padL, yy); ctx.lineTo(w - padR, yy); ctx.stroke();
      ctx.fillStyle = th.ink2; ctx.fillText(`${v} m`, padL - 6, yy);
    }
    const xs = niceStep(xMax - xMin, Math.max(3, Math.floor(plotW / 110)));
    ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    for (let v = Math.ceil(xMin / xs) * xs; v <= xMax; v += xs) {
      const xx = Math.round(X(v)) + 0.5;
      ctx.strokeStyle = th.ink2; ctx.beginPath(); ctx.moveTo(xx, gy); ctx.lineTo(xx, gy + 4); ctx.stroke();
      ctx.fillStyle = th.ink2; ctx.fillText(`${v} m`, xx, gy + 8);
    }

    // Viento: trazos que avanzan con la velocidad del viento a cada altura
    ctx.strokeStyle = th.streak; ctx.lineWidth = 1.5;
    for (const frac of [0.12, 0.36, 0.6, 0.84]) {
      const hh = apo * frac;
      const ws = cfg.windMs * Math.min(Math.pow(Math.max(hh, 2) / 10, 1 / 7), 2);
      if (ws < 0.2) continue;
      const spacing = 150, len = 8 + ws * 2.5;
      const off = ui.reducedMotion ? 40 * frac : (now * ws * 9 + frac * 97) % spacing;
      ctx.beginPath();
      for (let px = padL - spacing + off; px < w; px += spacing) { ctx.moveTo(px, Y(hh)); ctx.lineTo(px + len, Y(hh)); }
      ctx.stroke();
    }
    ctx.fillStyle = th.ink2; ctx.textAlign = 'left'; ctx.textBaseline = 'top';
    ctx.font = `500 11px ${th.body}`;
    ctx.fillText(`Viento → ${es(cfg.windMs, 0)} m/s a 10 m, desde ${Math.round(cfg.windFromDeg)}°`, padL + 6, padT + 2);

    // Rampa
    ctx.strokeStyle = th.ink2; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(X(0) - 8, gy); ctx.lineTo(X(0) - 8, gy - 66); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(X(0) - 18, gy); ctx.lineTo(X(0) - 8, gy - 14); ctx.lineTo(X(0) + 2, gy); ctx.stroke();

    // Estela de humo del motor (solo mientras quema) y trayectoria del ascenso
    const hBurn = 0.5 * sim.aBoost * MODEL.tBurn * MODEL.tBurn;
    if (sim.phase !== 'PAD') {
      const top = sim.tApogee === null ? sim.h : sim.hApogee;
      const fade = sim.t < sim.tBurnout ? 1 : clamp(1 - (sim.t - sim.tBurnout) / 40, 0.2, 1);
      ctx.globalAlpha = 0.5 * fade;
      ctx.strokeStyle = th.smoke; ctx.lineWidth = 7; ctx.lineCap = 'round';
      ctx.beginPath(); ctx.moveTo(X(0), gy); ctx.lineTo(X(0), Y(Math.min(top, hBurn))); ctx.stroke();
      ctx.globalAlpha = 1;
      ctx.setLineDash([2, 4]); ctx.strokeStyle = th.ref; ctx.lineWidth = 1.2; ctx.lineCap = 'butt';
      ctx.beginPath(); ctx.moveTo(X(0), gy); ctx.lineTo(X(0), Y(top)); ctx.stroke();
      ctx.setLineDash([]);
    }

    // Trayectoria de la lata y paquetes que no llegaron a tierra
    if (sim.tApogee !== null) {
      ctx.setLineDash([4, 4]); ctx.strokeStyle = th.s1; ctx.lineWidth = 1.5;
      ctx.beginPath();
      let first = true;
      for (const r of sim.truth) {
        if (r.t < sim.tApogee) continue;
        const px = X(xv(r.canN, r.canE)), py = Y(r.h);
        if (first) { ctx.moveTo(px, py); first = false; } else ctx.lineTo(px, py);
      }
      ctx.lineTo(X(xv(sim.canN, sim.canE)), Y(sim.h));
      ctx.stroke(); ctx.setLineDash([]);
    }
    for (let i = 0; i < ui.rxIdx; i++) {
      const e = sim.rx[i];
      if (e.status === 'ok') continue;
      const r = sim.truth[e.k - 1];
      if (!r) continue;
      ctx.fillStyle = e.status === 'corrupt' ? th.warn : th.bad;
      ctx.beginPath(); ctx.arc(X(xv(r.canN, r.canE)), Y(r.h), 2.5, 0, 2 * Math.PI); ctx.fill();
    }

    // Cohete
    const rp = rocketPose(sim, u);
    const burning = sim.phase === 'ASCENSO' && sim.t - sim.tLaunch <= MODEL.tBurn;
    const flame = burning ? (ui.reducedMotion ? 0.6 : 0.55 + 0.45 * Math.sin(now * 47)) : 0;
    if (!rp.after) {
      drawRocket(ctx, th, X(0), Y(rp.h), 0, flame, 1);
    } else if (rp.landed) {
      drawRocket(ctx, th, X(rp.x) - 22, gy - 5, Math.PI / 2, 0, 1);
    } else {
      const rx = X(rp.x), ry = Y(rp.h);
      if (rp.chute > 0) drawCanopy(ctx, th, rx, ry - 34, 0, 4 + 8 * rp.chute, 28, th.ref, null);
      drawRocket(ctx, th, rx - 20, ry + 2, Math.PI * 0.42, 0, 0.9);
    }
    if (rp.after && !rp.landed) {
      ctx.fillStyle = th.ink2; ctx.font = `11px ${th.body}`; ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
      ctx.fillText('cohete', X(rp.x) + 44, Y(rp.h) - 8);
    }

    // CanSat
    const canX = X(xv(sim.canN, sim.canE));
    const canY = sim.phase === 'ATERRIZADO' ? gy - 5 : Y(sim.h);
    if (sim.tApogee !== null) {
      drawCanSatAssembly(ctx, th, sim, u, canX, canY, 1);
      ctx.fillStyle = th.ink; ctx.font = `600 11px ${th.body}`; ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
      ctx.fillText('CanSat', canX + 12, canY + (sim.phase === 'ATERRIZADO' ? -16 : 0));
    }

    // Marca de altura actual en la regla
    const curH = sim.h;
    const curY = sim.tApogee === null ? Y(curH) : canY;
    {
      const label = `${Math.round(curH)} m`;
      ctx.font = `500 11px ${th.mono}`;
      const tw = ctx.measureText(label).width + 10;
      ctx.setLineDash([3, 3]); ctx.strokeStyle = th.accent; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(padL, curY); ctx.lineTo(sim.tApogee === null ? X(0) - 12 : canX - 10, curY); ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = th.accent;
      const bx = padL - tw - 2, by = clamp(curY - 9, 2, h - 20);
      if (ctx.roundRect) { ctx.beginPath(); ctx.roundRect(bx, by, tw, 18, 4); ctx.fill(); } else ctx.fillRect(bx, by, tw, 18);
      ctx.fillStyle = th.accentInk; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(label, bx + tw / 2, by + 9);
    }

    // Cuenta atrás en rampa
    if (sim.phase === 'PAD') {
      ctx.fillStyle = th.ink; ctx.font = `600 13px ${th.mono}`; ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
      ctx.fillText(`T−${es(sim.tLaunch - sim.t, 1)} s`, X(0) + 14, gy - 40);
    }

    // Detalle ampliado de la lata (escritorio)
    if (w >= 560) {
      const bw = 168, bh = 200, bx = w - padR - bw, by = padT + 22;
      ctx.fillStyle = th.panel; ctx.globalAlpha = 0.92;
      if (ctx.roundRect) { ctx.beginPath(); ctx.roundRect(bx, by, bw, bh, 10); ctx.fill(); } else ctx.fillRect(bx, by, bw, bh);
      ctx.globalAlpha = 1;
      ctx.strokeStyle = th.line; ctx.lineWidth = 1;
      if (ctx.roundRect) { ctx.beginPath(); ctx.roundRect(bx + 0.5, by + 0.5, bw - 1, bh - 1, 10); ctx.stroke(); }
      ctx.fillStyle = th.ink2; ctx.font = `600 10px ${th.body}`; ctx.textAlign = 'left'; ctx.textBaseline = 'top';
      ctx.fillText('DETALLE DE LA LATA', bx + 10, by + 9);
      const cx = bx + bw / 2;
      ctx.save();
      ctx.beginPath(); ctx.rect(bx, by + 22, bw, bh - 44); ctx.clip();
      let caption;
      if (sim.tApogee === null) {
        drawRocket(ctx, th, cx, by + bh - 34, 0, flame, 2);
        caption = 'Va dentro del cohete';
      } else if (sim.phase === 'ATERRIZADO') {
        ctx.fillStyle = th.ground; ctx.fillRect(bx, by + bh - 62, bw, 40);
        drawCanSatAssembly(ctx, th, sim, u, cx - 14, by + bh - 72, 2.2);
        caption = sim.buzzerOn ? 'En el suelo · zumbador' : 'En el suelo';
      } else {
        const ay = by + bh - 62;
        ctx.setLineDash([2, 3]); ctx.strokeStyle = th.axis;
        ctx.beginPath(); ctx.moveTo(cx, by + 24); ctx.lineTo(cx, by + bh - 24); ctx.stroke();
        ctx.setLineDash([]);
        drawCanSatAssembly(ctx, th, sim, u, cx, ay, 1.8);
        caption = sim.phi <= 0 ? 'Caída libre' : `Inclinación ${es(sim.tiltDeg(), 1)}°`;
      }
      ctx.restore();
      ctx.fillStyle = th.ink; ctx.font = `500 11px ${th.mono}`; ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
      ctx.fillText(caption, cx, by + bh - 8);
    }
  }

  function renderAnimData(force) {
    if (ui.mode !== 'anim' || !ui.sim) return;
    const nowMs = performance.now();
    if (!force && nowMs - ui.lastAnimData < 100) return;
    ui.lastAnimData = nowMs;
    const sim = ui.sim, gs = ui.gs, r = gs.last;
    const set = (id, txt) => { const el = document.getElementById(id); if (el) el.textContent = txt; };
    const tRel = sim.t - sim.tLaunch;
    set('a-tplus', `${tRel < 0 ? 'T−' : 'T+'}${es(Math.abs(tRel), 1)} s`);
    const pill = $('#a-phase');
    pill.textContent = PHASES[sim.phase];
    pill.dataset.phase = sim.phase;
    set('a-h', `${es(sim.h, 1)} m`);
    set('a-v', Math.abs(sim.v) < 0.05 ? '0,0 m/s' : `${es(Math.abs(sim.v), 1)} m/s ${sim.v > 0 ? 'bajando' : 'subiendo'}`);
    set('a-g', `${es(Math.hypot(sim.aW[0], sim.aW[1], sim.aW[2] - G) / G, 2)} g`);
    const inside = sim.phase === 'PAD' || sim.phase === 'ASCENSO';
    set('a-tilt', inside ? 'dentro del cohete' : `${es(sim.tiltDeg(), 1)}°`);
    set('a-chute', sim.tApogee === null ? 'plegado' : sim.phase === 'ATERRIZADO' ? 'en el suelo'
      : sim.phi <= 0 ? 'desplegándose' : sim.phi < 1 ? `inflándose (${Math.round(sim.phi * 100)} %)` : 'abierto');
    set('a-dist', `${Math.round(Math.hypot(sim.canN, sim.canE))} m`);
    set('a-pkt', r ? `#${r.PACKET_COUNT}` : '—');
    set('a-pkt-sub', `${gs.stats.ok} recibidos · ${gs.stats.lost} perdidos · ${gs.stats.corrupt} corruptos`);
    set('a-alt', r ? `${es(r.ALTITUDE, 1)} m` : '—');
    set('a-pres', r ? `${es(r.PRESSURE, 2)} hPa` : '—');
    set('a-temp', r ? `${es(r.TEMP, 1)} °C` : '—');
    set('a-az', r ? `${es(r.AZ, 2)} m/s²` : '—');
    set('a-gz', r ? `${es(r.GZ, 1)} °/s` : '—');
    set('a-bat', r ? `${es(r.V_BAT, 3)} V · ${es(r.I_DRAW, 0)} mA` : '—');
    set('a-gps', r ? `${r.FIX_FLAG ? 'Fix' : 'Sin fix'} · ${r.SATS} sat.` : '—');
    $('#a-gps').dataset.state = r && r.FIX_FLAG === 0 ? 'warn' : 'ok';
    const raw = gs.rawLines.length ? gs.rawLines[gs.rawLines.length - 1].replace(/\n$/, '') : '—';
    set('a-frame', raw);
    if (ui.animEvents !== sim.events.length) {
      ui.animEvents = sim.events.length;
      const ol = $('#a-events');
      ol.textContent = '';
      for (const e of sim.events) {
        const li = document.createElement('li');
        const tt = document.createElement('b');
        const dt = e.t - sim.tLaunch;
        tt.textContent = `${dt < 0 ? 'T−' : 'T+'}${es(Math.abs(dt), 1)} s`;
        const lab = document.createElement('span');
        lab.textContent = e.label;
        li.append(tt, lab);
        ol.append(li);
      }
    }
  }

  function setMode(mode, fromUser) {
    ui.mode = mode === 'anim' ? 'anim' : 'panel';
    $('#anim').hidden = ui.mode !== 'anim';
    $('.kpis').hidden = ui.mode === 'anim';
    $$('[data-mode]').forEach((x) => x.setAttribute('aria-pressed', x.dataset.mode === ui.mode ? 'true' : 'false'));
    try { localStorage.setItem('cansat-vista', ui.mode); } catch (err) { /* sin almacenamiento: solo no se recuerda la vista */ }
    if (ui.mode === 'anim') {
      ui.animEvents = -1;
      if (fromUser && ui.sim.done && !ui.playing) { newRun(); renderAll(true); play(); }
      else { drawScene(); renderAnimData(true); }
    }
    updateButtons();
  }

  /* ─── Paneles de texto ─── */
  function renderDerived() {
    const cfg = normalizeConfig(readConfig());
    const d = derive(cfg);
    const set = (id, txt) => { const el = document.getElementById(id); if (el) el.textContent = txt; };
    set('d-area', `${es(d.A, 4)} m²`);
    set('d-cda', `${es(d.cdA, 4)} m²`);
    set('d-rho', `${es(d.rhoTop, 3)} / ${es(d.rhoGround, 3)} kg/m³`);
    set('d-vt', `${es(d.vtTop, 2)} → ${es(d.vtGround, 2)} m/s`);
    set('d-exp', es(d.baroExp, 4));
    set('d-tdesc', `${es(d.tDescent, 0)} s`);
    set('d-swing', `${es(d.fSwing, 2)} Hz`);
    set('d-link', `${d.frameBytes} B · ${Math.round(d.occupancy * 100)} %`);
    set('d-boost', `${es(d.aBoost / G, 1)} g durante ${es(MODEL.tBurn, 1)} s`);
    const vtOk = d.vtTop >= 8 && d.vtTop <= 11 && d.vtGround >= 8 && d.vtGround <= 11;
    const vtEl = document.getElementById('d-vt');
    if (vtEl) vtEl.dataset.state = vtOk ? 'ok' : 'fail';
    const linkEl = document.getElementById('d-link');
    if (linkEl) linkEl.dataset.state = d.occupancy >= 1 ? 'fail' : d.occupancy > 0.7 ? 'warn' : 'ok';
    $$('[data-live]').forEach((el) => {
      const k = el.dataset.live;
      if (k === 'vtTop') el.textContent = es(d.vtTop, 1);
      if (k === 'vtGround') el.textContent = es(d.vtGround, 1);
      if (k === 'fSwing') el.textContent = es(d.fSwing, 2);
      if (k === 'tDesc') el.textContent = es(d.tDescent, 0);
    });
    const pending = ui.sim && JSON.stringify(cfg) !== JSON.stringify(ui.sim.cfg);
    const note = $('#pending-note');
    if (note) note.hidden = !pending;
  }

  function readConfig() {
    const form = $('#cfg');
    const o = {};
    for (const el of form.elements) {
      if (!el.name) continue;
      o[el.name] = el.type === 'checkbox' ? el.checked : el.value;
    }
    return o;
  }

  function renderKpis() {
    const sim = ui.sim, gs = ui.gs, r = gs.last;
    const set = (id, txt) => { const el = document.getElementById(id); if (el) el.textContent = txt; };
    set('k-phase', PHASES[sim.phase]);
    $('#k-phase').dataset.phase = sim.phase;
    set('k-time', `${es(sim.t, 1)} s`);
    set('k-ts', `TIMESTAMP ${MODEL.bootMs + Math.round(sim.t * 1000)} ms`);
    set('k-alt', r ? `${es(r.ALTITUDE, 1)} m` : '—');
    set('k-alt-sub', r ? `modelo ${es(sim.h, 1)} m` : '');
    set('k-vel', r ? `${es(r.vBaro, 1)} m/s` : '—');
    set('k-vel-sub', `modelo ${es(sim.v, 1)} m/s`);
    set('k-pres', r ? `${es(r.PRESSURE, 2)} hPa` : '—');
    set('k-temp', r ? `${es(r.TEMP, 1)} °C` : '—');
    set('k-temp-sub', r ? `CPU ${es(r.TEMP_CPU, 1)} °C` : '');
    const sent = ui.rxIdx;
    const lossPct = sent ? (100 * gs.stats.missing) / Math.max(1, (r ? r.PACKET_COUNT : sent)) : 0;
    set('k-pkt', `${gs.stats.ok} / ${r ? r.PACKET_COUNT : 0}`);
    set('k-pkt-sub', `${gs.stats.lost} perdidos · ${gs.stats.corrupt} corruptos · ${es(lossPct, 1)} %`);
    set('k-gps', r ? (r.FIX_FLAG ? `Fix · ${r.SATS} sat.` : `Sin fix · ${r.SATS} sat.`) : '—');
    $('#k-gps').dataset.state = r && r.FIX_FLAG === 0 ? 'warn' : 'ok';
    set('k-gps-sub', r && r.LAT !== null ? `${es(r.LAT, 5)}, ${es(r.LON, 5)}` : '');
    set('k-bat', r ? `${es(r.V_BAT, 3)} V` : '—');
    set('k-bat-sub', r ? `${es(r.I_DRAW, 1)} mA` : '');
    const prog = $('#progress');
    if (prog) prog.style.setProperty('--p', `${Math.min(100, (100 * sim.t) / Math.max(sim.derived.tEnd, sim.t))}%`);
  }

  function renderValidation() {
    const { criteria, info } = validate(ui.sim, ui.gs);
    const STATUS = { ok: 'Cumple', fail: 'No cumple', pending: 'Pendiente', warn: 'Revisar', info: 'Dato' };
    const fill = (list, items) => {
      list.textContent = '';
      for (const c of items) {
        const li = document.createElement('li');
        li.dataset.state = c.status;
        const chip = document.createElement('span');
        chip.className = 'chip';
        chip.textContent = STATUS[c.status];
        const body = document.createElement('div');
        const t = document.createElement('p'); t.className = 'crit-title'; t.textContent = c.title;
        const v = document.createElement('p'); v.className = 'crit-value'; v.textContent = c.value;
        body.append(t, v);
        li.append(chip, body);
        list.append(li);
      }
    };
    fill($('#val-list'), criteria);
    fill($('#val-info'), info);
    const passed = criteria.filter((c) => c.status === 'ok').length;
    const failed = criteria.filter((c) => c.status === 'fail').length;
    const sum = $('#val-summary');
    sum.textContent = failed ? `${failed} de 5 criterios no se cumplen` : passed === 5 ? 'Los 5 criterios se cumplen' : `${passed} de 5 criterios confirmados; el resto espera al aterrizaje`;
    sum.dataset.state = failed ? 'fail' : passed === 5 ? 'ok' : 'pending';
  }

  function renderLog() {
    const pre = $('#log');
    const frag = document.createDocumentFragment();
    const MAXL = 240;
    if (ui.logTab === 'rx') {
      const items = ui.gs.log.slice(-MAXL);
      for (const e of items) {
        const div = document.createElement('div');
        div.className = 'l-' + e.kind;
        div.textContent = e.text;
        if (e.note) { const n = document.createElement('span'); n.className = 'l-note'; n.textContent = '  ⟵ ' + e.note; div.append(n); }
        frag.append(div);
      }
    } else {
      const head = document.createElement('div'); head.className = 'l-head'; head.textContent = CSV_HEADER; frag.append(head);
      for (const r of ui.sim.sd.slice(-MAXL)) {
        const div = document.createElement('div');
        div.className = 'l-ok';
        div.textContent = r.line.replace(/\n$/, '');
        if (r.sdStall) { const n = document.createElement('span'); n.className = 'l-note'; n.textContent = '  ⟵ volcado lento de la SD'; div.append(n); }
        frag.append(div);
      }
    }
    pre.textContent = '';
    pre.append(frag);
    pre.scrollTop = pre.scrollHeight;
    const count = $('#log-count');
    if (count) count.textContent = ui.logTab === 'rx' ? `${ui.gs.rawLines.length} líneas recibidas` : `${ui.sim.sd.length} tramas en la MicroSD`;
  }

  function renderSample() {
    const el = $('#sample-line');
    if (!el || !ui.sim) return;
    const r = ui.sim.sd.find((x) => ui.sim.tOpen !== null && x.t > ui.sim.tOpen + 20) || ui.sim.sd[ui.sim.sd.length - 1];
    el.textContent = r.line.replace(/\n$/, '');
    const bytes = $('#sample-bytes');
    if (bytes) bytes.textContent = `${r.bytes} bytes con el \\n`;
  }

  function drawAll(force) {
    drawScene();   // la animación se pinta en cada fotograma; las gráficas, como mucho cada 60 ms
    const now = performance.now();
    if (!force && now - ui.lastChartDraw < 60) return;
    ui.lastChartDraw = now;
    for (const c of ui.charts) c.draw();
    drawTimeline();
    drawMap();
  }

  function renderAll(force) {
    renderKpis();
    drawAll(force);
    if (ui.dirtyLog) { renderLog(); ui.dirtyLog = false; }
    renderValidation();
    renderAnimData(force);
    updateButtons();
  }

  function pumpRx(all) {
    const rx = ui.sim.rx;
    while (ui.rxIdx < rx.length && (all || rx[ui.rxIdx].tRx <= ui.sim.t + 1e-9)) {
      const e = rx[ui.rxIdx++];
      if (e.status === 'ok' || e.status === 'corrupt') ui.gs.ingest(e.line);
      ui.dirtyLog = true;
    }
  }

  function newRun() {
    cancelAnimationFrame(ui.raf);
    const cfg = normalizeConfig(readConfig());
    ui.sim = new CanSatSim(cfg);
    ui.gs = new GroundStation(cfg.teamId, cfg.rateHz, cfg.elevM);
    ui.rxIdx = 0;
    ui.playing = false; ui.paused = false;
    ui.dirtyLog = true;
    renderDerived();
  }

  function finishInstant() {
    cancelAnimationFrame(ui.raf);
    ui.sim.runToEnd();
    pumpRx(true);
    ui.playing = false; ui.paused = false;
    ui.dirtyLog = true;
    renderAll(true);
    renderSample();
  }

  function tick(now) {
    if (!ui.playing || ui.paused) return;
    const dtReal = Math.min(0.25, (now - ui.lastNow) / 1000);
    ui.lastNow = now;
    ui.clock += dtReal * ui.speed;
    ui.sim.advanceTo(ui.clock);
    pumpRx(ui.sim.done);
    if (ui.sim.done) { ui.playing = false; renderAll(true); renderSample(); return; }
    renderAll(false);
    ui.raf = requestAnimationFrame(tick);
  }

  function play() {
    ui.playing = true; ui.paused = false;
    ui.clock = ui.sim.t;
    ui.lastNow = performance.now();
    ui.raf = requestAnimationFrame(tick);
    updateButtons();
  }

  function updateButtons() {
    const pause = $('#btn-pause');
    pause.disabled = !ui.playing;
    pause.textContent = ui.paused ? 'Continuar' : 'Pausa';
    pause.setAttribute('aria-pressed', ui.paused ? 'true' : 'false');
    $('#live-dot').hidden = !(ui.playing && !ui.paused);
    const ap = $('#anim-play');
    ap.hidden = ui.playing && !ui.paused;
    ap.textContent = ui.paused ? 'Continuar' : 'Ver el vuelo';
  }

  async function copyText(text, btn) {
    const done = (msg) => { const old = btn.dataset.label || btn.textContent; btn.dataset.label = old; btn.textContent = msg; setTimeout(() => { btn.textContent = old; }, 1600); };
    try {
      await navigator.clipboard.writeText(text);
      done('Copiado');
    } catch (err) {
      const ta = $('#copy-fallback');
      ta.hidden = false;
      ta.value = text;
      ta.focus();
      ta.select();
      done('Selecciónalo y copia');
    }
  }

  function initUI() {
    readTheme();
    ui.charts = $$('figure[data-chart]').map((f) => new LineChart(f));
    const form = $('#cfg');
    form.addEventListener('submit', (e) => e.preventDefault());
    form.addEventListener('input', renderDerived);
    form.addEventListener('change', renderDerived);

    $('#btn-launch').addEventListener('click', () => { newRun(); renderAll(true); play(); });
    $('#btn-pause').addEventListener('click', () => {
      if (!ui.playing) return;
      ui.paused = !ui.paused;
      if (!ui.paused) play(); else updateButtons();
    });
    $('#btn-instant').addEventListener('click', () => {
      if (!ui.sim || ui.sim.done || !ui.playing) newRun();
      finishInstant();
    });
    $('#speed').addEventListener('change', (e) => { ui.speed = Number(e.target.value) || 1; });
    ui.speed = Number($('#speed').value) || 2;
    $$('[data-mode]').forEach((b) => b.addEventListener('click', () => setMode(b.dataset.mode, true)));
    $('#anim-play').addEventListener('click', () => {
      if (ui.playing && ui.paused) play();
      else { newRun(); renderAll(true); play(); }
    });

    $$('[data-view]').forEach((b) => b.addEventListener('click', () => {
      ui.view = b.dataset.view;
      $$('[data-view]').forEach((x) => x.setAttribute('aria-pressed', x === b ? 'true' : 'false'));
      drawAll(true);
    }));
    $$('[data-tab]').forEach((b) => b.addEventListener('click', () => {
      ui.logTab = b.dataset.tab;
      $$('[data-tab]').forEach((x) => x.setAttribute('aria-selected', x === b ? 'true' : 'false'));
      renderLog();
    }));
    $('#copy-sd').addEventListener('click', (e) => copyText(ui.sim.sdCsv(), e.currentTarget));
    $('#copy-rx').addEventListener('click', (e) => copyText(ui.gs.rawLines.join(''), e.currentTarget));

    const redraw = () => { readTheme(); drawAll(true); };
    if (window.ResizeObserver) {
      let pending = false;
      const ro = new ResizeObserver(() => { if (pending) return; pending = true; requestAnimationFrame(() => { pending = false; drawAll(true); }); });
      $$('canvas').forEach((c) => ro.observe(c));
    } else window.addEventListener('resize', () => drawAll(true));
    const mq = window.matchMedia('(prefers-color-scheme: dark)');
    if (mq.addEventListener) mq.addEventListener('change', redraw);
    new MutationObserver(redraw).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme', 'class'] });
    if (document.fonts && document.fonts.ready) document.fonts.ready.then(() => drawAll(true));

    // Estado inicial: una misión completa con los supuestos por defecto
    newRun();
    finishInstant();
    let saved = null;
    try { saved = localStorage.getItem('cansat-vista'); } catch (err) { /* sin almacenamiento */ }
    if (saved === 'anim') setMode('anim', false);
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', initUI);
  else initUI();
})(typeof window !== 'undefined' ? window : globalThis);
