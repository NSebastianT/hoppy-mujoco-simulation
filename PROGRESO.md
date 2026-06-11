# Bitacora del proyecto HOPPY (MuJoCo)

Documento de coordinacion del equipo. Resume de donde partimos, que ya esta
hecho, que falta y el plan por fases, mapeado a la rubrica. Se actualiza cada
vez que avanzamos para tener claro que decir en la presentacion y que sigue.

Ultima actualizacion: 11 de junio 2026.

## Punto de partida (lo que habia hoy)

- `main` tenia la simulacion del salto con el controlador hibrido funcionando
  sobre un modelo simplificado de capsulas (`models/hoppy.xml`), con 3 DoF
  (pitch, cadera, rodilla).
- El CAD oficial estaba colocado a mano encima del modelo y no ensamblaba bien.
- En el camino se habian perdido cosas de la rubrica (resorte de rodilla, config
  de solver recomendada) y los valores de actuador no estaban justificados.

## Estado por fase (rubrica, 100 puntos)

Leyenda: [x] hecho, [~] parcial, [ ] pendiente.

### Fase 1 - Modelo mecanico (20 pts) -- completa
- [x] 4 DoF: yaw + pitch pasivos, cadera + rodilla activos.
- [x] Solver recomendado: RK4, Newton, timestep 1 ms (1 kHz), iterations 50,
      tolerance 1e-8.
- [x] Resorte paralelo de rodilla (stiffness 2.0, springref -0.7).
- [x] Armature = N^2*Ir en cadera y rodilla, con valores del motor real.
- [x] Damping equivalente back-EMF = (kv*kt/Rw)*N^2.
- [x] Contrapeso en el extremo opuesto del gantry.

### Fase 2 - Restricciones del actuador (10 pts) -- en progreso
- [x] Saturacion de torque del motor aplicada en el modelo y el controlador:
      ±12.2 N·m (cadera) y ±13 N·m (rodilla), del pico de 30 A del driver
      (0.405 N·m/A a la salida). Antes estaba en ±35, irreal.
- [ ] Simulaciones comparativas mostrando el efecto de armature, damping,
      resorte y saturacion.

### Fase 3 - Contacto pie-suelo (15 pts) -- pendiente
- [ ] Ajuste de solref, solimp y friccion para contacto duro (minimizar rebotes,
      penetracion y deslizamiento).
- [~] Hay deteccion de contacto pie-suelo; falta justificar el criterio de
      touchdown/liftoff.

### Fase 4 - Maquina de estados y control hibrido (40 pts) -- parcial
- [x] Bucle a 1 kHz y maquina FLIGHT/STANCE basada en contacto.
- [x] Control cartesiano en vuelo (Jacobiano transpuesto, PD).
- [x] Control de fuerza en apoyo (GRF via Jacobiano transpuesto).
- [ ] Subir la energia del salto. Diagnostico: subir la fuerza de empuje NO
      sube el salto (el pitch del boom se topa en ~0.054 rad, el cuerpo sube
      ~4 cm con cualquier fuerza). El problema es que la pierna se extiende y
      pierde contacto antes de transferir impulso al boom -> rebotes rapidos y
      bajos. Hay que rediseniar el control de stance: mantener contacto mientras
      se acelera el cuerpo, con perfil de fuerza Bezier sobre ~0.15 s y timing
      de la maquina de estados acorde.
- [ ] Perfil de fuerza justificado (Bezier) y transicion suave alpha.

### Fase 5 - Sensores y procesamiento (15 pts) -- parcial
- [x] Estimacion de velocidad por derivada filtrada (no se usa qvel directo).
- [ ] Emulacion de encoder (28 CPR) y juego completo de graficas con analisis.

## Datos del motor (para el reporte)

goBilda 5202 Series Yellow Jacket, relacion 26.9:1 (parte 5202-2402-0027), base
RS-555, driver Pololu VNH5019 a 12 V (BOM del equipo).

- Velocidad libre 223 RPM, corriente libre 0.25 A.
- Par de stall 3.73 N·m, corriente de stall 9.2 A.
- Rw = V/Istall = 12/9.2 = 1.30 ohm.
- kt = kv = 0.0186 (a partir de la velocidad libre).
- Ir ~ 1e-5 kg·m^2 (estimado para RS-555; sustituir si conseguimos el dato).
- N_cadera = 26.9, N_rodilla = 28.8.
- armature = N^2*Ir -> 0.0072 (cadera), 0.0083 (rodilla).
- damping = (kv*kt/Rw)*N^2 -> 0.192 (cadera), 0.220 (rodilla).
- Torque por amp de salida = 3.73/9.2 = 0.405 N·m/A; a 30 A pico ~12-13 N·m.

## Modelos y como correr

Usar siempre el entorno `.venv-py312` (tiene mujoco, imageio y trimesh).

- `models/hoppy.xml`: modelo fisico (capsulas) + controlador. Es el que simula.
- `models/hoppy_cad_view.xml`: CAD oficial fiel armado del URDF. Solo visual.

Comandos:

    .venv-py312/bin/python src/render_simulation.py     # salto -> results/renders/hopping.mp4
    .venv-py312/bin/python src/render_cad_view.py       # CAD   -> results/renders/cad_view.mp4
    .venv-py312/bin/python src/run_logged_simulation.py # log y graficas en results/

El visor interactivo (view_hoppy, hybrid_controller_test) abre ventana GLFW y
crashea en Linux con driver Intel Xe; en Windows/Mac si funciona. Por eso en
Linux usamos los renders offscreen, que generan un mp4.

## Ramas

- `develop`: integracion. `main`: entrega.
- `rubric-physics`: fases de fisica (rama actual de trabajo).
- `cad-in-action`: animacion cinematica del CAD (preview, sin merge todavia).

## Decisiones abiertas

- Agregar el yaw pasivo ahora (la rubrica lo pide) o dejarlo para el final.
- Conseguir el datasheet del RS-555 para usar Ir exacto (hoy es estimado).
- Como presentar el CAD con fisica real para la entrega final.
