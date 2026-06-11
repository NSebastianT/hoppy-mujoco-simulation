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

### Fase 2 - Restricciones del actuador (10 pts) -- completa
- [x] Saturacion de torque del motor aplicada en el modelo y el controlador:
      ±12.2 N·m (cadera) y ±13 N·m (rodilla), del pico de 30 A del driver
      (0.405 N·m/A a la salida). Antes estaba en ±35, irreal.
- [x] Simulaciones comparativas (tools/compare_actuator.py -> grafica
      results/plots/cad_comparison_height.png). Corre el mismo salto toggleando
      cada efecto y mide el pico de altura del cuerpo:
        baseline (todo realista) ....... 0.393 m
        sin armature ................... 0.590 m  (sin inercia de rotor sube)
        sin damping .................... 0.489 m  (sin perdidas back-EMF sube)
        sin resorte de rodilla ......... 0.440 m  (efecto leve)
        sin saturacion (+-35 Nm) ....... 0.618 m  (con mas torque sube)
      Analisis: cada efecto realista (armature, damping, saturacion) BAJA el
      salto hacia lo fisico; el baseline es el mas bajo porque tiene todos los
      limitantes activos. Es la validacion del modelo de actuador.

### Fase 3 - Contacto pie-suelo (15 pts) -- completa
- [x] Contacto duro configurado y justificado (foot + floor):
        solref = "0.01 1"  -> constante de tiempo 10 ms, razon de amortiguamiento
                              1 (criticamente amortiguado): contacto rapido y sin
                              rebote artificial.
        solimp = "0.95 0.99 0.001" -> impedancia alta (rigido), poca compliance.
        friction = "1.5 0.02 0.001" -> friccion tangencial alta -> minimo
                              deslizamiento no deseado.
- [x] Calidad de contacto medida (6 s de salto): penetracion maxima de la esfera
      de colision ~18 mm (es invisible; la malla visible de la pierna se queda
      >0.11 m sobre el piso, no se ve hundida); deslizamiento del pie en contacto
      bajo (~0.18 m/s medio, casi todo es el avance tangencial intencional de las
      vueltas, no slip); fuerza normal suave (pico ~256 N) sin multi-rebote por
      aterrizaje. Endurecer mas solref/solimp no reduce la penetracion (es la
      esfera chica en el impacto, no compliance).
- [x] Criterio touchdown/liftoff: touchdown = par de geoms pie(o capsula de
      pierna)-piso en contacto via los contactos de MuJoCo, con guarda de tiempo
      minimo de vuelo (MIN_FLIGHT) para evitar chattering; liftoff = se pierde el
      contacto pasado MIN_STANCE, o tope MAX_STANCE. Robusto a rebotes de contacto
      por las guardas de tiempo minimo.

### Fase 4 - Maquina de estados y control hibrido (40 pts) -- parcial
- [x] Bucle a 1 kHz y maquina FLIGHT/STANCE basada en contacto.
- [x] Control cartesiano en vuelo (Jacobiano transpuesto, PD).
- [x] Control de fuerza en apoyo (GRF via Jacobiano transpuesto), con componente horizontal tangencial para avance alrededor del pilar.
- [ ] Subir la energia del salto. Diagnostico: subir la fuerza de empuje NO
      sube el salto (el pitch del boom se topa en ~0.054 rad, el cuerpo sube
      ~4 cm con cualquier fuerza). El problema es que la pierna se extiende y
      pierde contacto antes de transferir impulso al boom -> rebotes rapidos y
      bajos. Hay que rediseniar el control de stance: mantener contacto mientras
      se acelera el cuerpo, con perfil de fuerza Bezier sobre ~0.15 s y timing
      de la maquina de estados acorde.
- [x] Perfil de fuerza Bezier en stance, transicion suave alpha de 10 ms y PD de pierna mas fuerte en el CAD.

### Fase 5 - Sensores y procesamiento (15 pts) -- completa
- [x] Emulacion de encoder goBilda 5202: joint3/joint4 cuantizados a
      751.8 cuentas/rev en el eje de salida.
- [x] Estimacion de velocidad por derivada numerica filtrada; el PD del CAD ya
      no usa `qvel` directo para el feedback de velocidad.
- [x] Logging del CAD en `results/logs/cad_states.csv` y graficas regenerables
      en `results/plots/` con posiciones, velocidades, contacto, torques y
      estado hibrido.

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
- `models/hoppy_cad_physics.xml`: CAD oficial con FISICA real (colision de pie,
  actuadores, dinamica de joints). Es el objetivo para la presentacion: el
  modelo real saltando, no las cajitas. Ya cae y contacta el piso bien; falta
  portar el controlador y afinar el salto.

## Plan para el objetivo final (CAD con fisica saltando)

El profe quiere ver el modelo real saltando. Pasos:
1. [x] Modelo CAD con fisica (`hoppy_cad_physics.xml`): colision de pie/piso,
       motores en cadera/rodilla, armature/damping/resorte. Cae y apoya estable.
2. [x] Prueba de caida (contacto valido, sin explotar).
3. [x] Controlador hibrido portado (`src/cad_hop_controller.py`): el CAD SALTA
       con fisica (pie sube ~0.2 m, boom pitchea, estable). Esta es la version
       que se puede mostrar al profe: el modelo real saltando.
4. [x] Afinar el salto del CAD. El contrapeso visible se elimino y se plego
       en el inercial de Link2 (masa 3.87654 kg, COM x=0.06075 m), dejando el
       bias de joint2 alrededor de -1.2 N*m sin bloques flotantes. El
       controlador CAD usa perfil de fuerza Bezier de 0.15 s, alpha de 10 ms y
       PD fuerte de pierna. Tambien se alineo la colision del pie con el
       extremo visual de Link4 y se agrego una capsula delgada de pierna baja.
       El stance incluye warmup de 1.0 s para evitar el lanzon inicial. En 6 s
       reporta 11 vuelos reales (>0.10 s), startup_qvel_max ~4.71 rad/s,
       first_hop_peak ~0.384 m, steady_peak ~0.330 m, yaw_progress ~2.36 rad,
       monotonicidad de yaw 1.000, mesh_min_z ~0.112 m,
       estable y dentro de torque. El pie sigue siendo el contacto dominante
       frente a la capsula de pierna.
5. [x] Sensores y graficas (Fase 5): el controlador CAD usa velocidad estimada
       desde encoder cuantizado; `src/run_cad_logged.py` genera el CSV y las
       graficas de la rubrica en `results/`.
6. [ ] Contacto duro afinado (Fase 3).
7. [x] Render final del CAD saltando con fisica en `results/renders/cad_hopping.mp4` (no versionado).

Comandos:

    .venv-py312/bin/python src/cad_hop_controller.py    # CAD con fisica saltando -> results/renders/cad_hopping.mp4
    .venv-py312/bin/python src/render_simulation.py     # cajitas saltando -> results/renders/hopping.mp4
    .venv-py312/bin/python src/render_cad_view.py       # CAD quieto (giro) -> results/renders/cad_view.mp4
    .venv-py312/bin/python src/run_logged_simulation.py # log y graficas en results/
    .venv-py312/bin/python src/run_cad_logged.py        # CSV y graficas del CAD -> results/logs y results/plots

Demo EN VIVO del CAD saltando (ventana interactiva de MuJoCo):

    # Windows / macOS:
    .venv-py312/bin/python src/view_cad_hop.py

    # Linux (hay que forzar la NVIDIA; con Intel Xe el visor crashea):
    __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia .venv-py312/bin/python src/view_cad_hop.py

El visor interactivo abre ventana GLFW. En Linux con el GL de Intel Xe crashea,
por eso en Linux se fuerza la GPU NVIDIA (PRIME offload) con las variables de
arriba; ya probado y funciona. Para grabar mp4 se usan los renders offscreen.

## Ramas

- `develop`: integracion. `main`: entrega.
- `rubric-physics`: fases de fisica (rama actual de trabajo).
- `cad-in-action`: animacion cinematica del CAD (preview, sin merge todavia).

## Limitaciones conocidas

- Vueltas: el robot gira ~media vuelta (~180 grados, visible en el demo) y luego
  se estanca. La rotacion continua no se logra con tuning simple: la fuerza
  tangencial se acopla con la orientacion del plano de la pierna y se anula a la
  media vuelta. Rotacion continua = trabajo futuro de control (control de
  velocidad de yaw, no solo una fuerza tangencial). La camara del render/visor
  se subio (elevation -28) para que la pierna se vea girando y no se esconda
  tras la base.

## Decisiones abiertas

- Conseguir el datasheet del RS-555 para usar Ir exacto (hoy es estimado).
- Rediseno de control para vueltas continuas (si se quiere para una v2).
