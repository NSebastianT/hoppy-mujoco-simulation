# HOPPY MuJoCo Simulation

Proyecto de simulación del robot HOPPY en MuJoCo.

## Objetivo

Simular una versión simplificada del robot HOPPY usando MJCF/XML y Python. El modelo incluye:

- Gantry restringido con movimiento pasivo en pitch.
- Pierna con dos grados de libertad activos: hip y knee.
- Contrapeso en el extremo opuesto del gantry.
- Masa adicional en la zona de la cadera para representar motor/carcasa.
- Contacto pie-suelo.
- Detección de touchdown y lift-off.
- Máquina de estados FLIGHT/STANCE.
- Control híbrido basado en posturas articulares.
- Saturación de torque.
- Estimación filtrada de velocidades.
- Generación de logs y gráficas.
- Scripts de diagnóstico para revisar parámetros físicos del modelo.

## Nota sobre el modelo

Este repositorio no usa un modelo MuJoCo oficial de HOPPY ya existente. El archivo `models/hoppy.xml` es una aproximación simplificada construida en MJCF con base en la arquitectura general de HOPPY: gantry, contrapeso, pierna de dos articulaciones, contacto con el suelo y control híbrido.

El modelo actual no pretende representar con exactitud todas las piezas CAD, tornillos, perfiles metálicos, mallas STL o componentes reales del kit físico. Su propósito es capturar una estructura dinámica simplificada que permita simular contacto, fases de vuelo/apoyo, control y análisis de resultados.

En la versión actual se priorizó una simulación funcional y estable sobre una representación CAD-realista. El gantry se encuentra restringido para evitar que el sistema se descontrole fuera del plano principal de movimiento.

## Instalación

Crear ambiente Conda:

```bat
conda create -n hoppy_mujoco python=3.11 -y
conda activate hoppy_mujoco
pip install -r requirements.txt
```

## Scripts principales

Verificar instalación de MuJoCo:

```bat
python src\smoke_test.py
```

Ver el modelo HOPPY:

```bat
python src\view_hoppy.py
```

Probar contacto pie-suelo:

```bat
python src\test_contact.py
```

Probar máquina de estados:

```bat
python src\state_machine_test.py
```

Probar controlador híbrido con viewer:

```bat
python src\hybrid_controller_test.py
```

Generar logs y gráficas:

```bat
python src\run_logged_simulation.py
```

## Scripts de diagnóstico

Estos scripts se usaron para entender y ajustar la simulación antes de llegar a la versión actual:

```bat
python src\check_pose_grid.py
python src\passive_dynamics_test.py
python src\sweep_initial_velocity.py
python src\sweep_counterweight.py
python src\sweep_closed_loop_counterweight.py
python src\sweep_gantry_damping.py
```

Sirven para revisar condiciones iniciales, dinámica pasiva, masa del contrapeso, respuesta del gantry, contacto con el suelo y comportamiento del sistema con diferentes parámetros.

## Resultados

Las gráficas se guardan en:

```text
results/plots/
```

El log principal se guarda en:

```text
results/logs/hybrid_log.csv
```

Gráficas generadas actualmente:

```text
foot_world_z.png
foot_relative_position.png
foot_vertical_velocity.png
hip_height.png
gantry_pitch.png
gantry_pitch_velocity.png
hybrid_state.png
joint_positions.png
normal_force.png
torques.png
```

## Estado actual

El proyecto actualmente logra:

- Cargar el modelo HOPPY simplificado.
- Simular una pierna robótica con gantry, contrapeso, hip, knee y pie.
- Mantener el sistema contenido para evitar rotaciones o caídas fuera del plano principal.
- Detectar contacto pie-suelo.
- Cambiar entre FLIGHT y STANCE.
- Aplicar un controlador híbrido por posturas articulares.
- Producir oscilaciones verticales pequeñas y periódicas.
- Generar gráficas de altura de pie, altura de cadera, posición articular, velocidad estimada, torque, fuerza normal y estado híbrido.
- Guardar scripts de diagnóstico para justificar decisiones de modelado.

## Limitaciones actuales

El modelo actual es funcional, pero todavía tiene limitaciones importantes:

- No usa todavía STL/CAD real del HOPPY físico.
- No representa todas las masas, inercias, offsets ni componentes reales del sistema.
- El gantry está restringido para mejorar estabilidad.
- El comportamiento actual corresponde a hopping vertical pequeño, no a salto grande ni locomoción hacia adelante.
- El controlador es una aproximación educativa basada en posturas articulares, no una reproducción exacta del controlador oficial de HOPPY.
- El contacto en MuJoCo se maneja mediante contacto físico, no mediante un modelo matemático con restricción perfecta del pie durante stance.

## Pendientes

- Preguntar al profesor si se espera usar STL/URDF/CAD real o si basta un MJCF simplificado.
- Confirmar si el gantry puede estar restringido a un plano.
- Confirmar si el objetivo es salto vertical o locomoción hacia adelante.
- Mejorar el modelo físico usando parámetros más fieles de HOPPY si es necesario.
- Documentar comparaciones con/sin armature, damping, spring y torque saturation.
- Preparar una explicación técnica breve del modelo, controlador, resultados y limitaciones.