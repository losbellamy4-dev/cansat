# CanSat · Simulador de descenso

Banco de pruebas del software del CanSat (Desafío CanSat ESA, fase regional de Andalucía) antes de montar el hardware.

## Uso

Abre `simulador/index.html` en el navegador. No necesita servidor ni conexión: si no hay internet, solo cambian las fuentes.

| Archivo | Contenido |
|---|---|
| `simulador/index.html` | Estructura de la página: supuestos, controles, gráficas, validación y registro de tramas |
| `simulador/estilos.css` | Estilos con tema claro y oscuro |
| `simulador/simulacion.js` | Motor físico, sensores, trama CSV, canal de radio, parser de tierra, validación e interfaz |

## Trama

```
TEAM_ID,PACKET_COUNT,TIMESTAMP,TEMP,PRESSURE,ALTITUDE,AX,AY,AZ,GX,GY,GZ,MX,MY,MZ,V_BAT,I_DRAW,TEMP_CPU,LAT,LON,GPS_ALT,SATS,FIX_FLAG\n
```

Un campo sin dato se envía como `-999` sin romper la trama.

## Tests en Node

El núcleo no depende del DOM:

```js
const S = require('./simulador/simulacion.js');
const sim = new S.CanSatSim({ seed: 42, rateHz: 5 }).runToEnd();
const gs = new S.GroundStation(sim.cfg.teamId, sim.cfg.rateHz, sim.cfg.elevM);
for (const e of sim.rx) if (e.status !== 'lost' && e.status !== 'overflow') gs.ingest(e.line);
console.log(S.validate(sim, gs).criteria);
```

Con la misma semilla y los mismos parámetros, `sim.sdCsv()` es idéntico byte a byte.
