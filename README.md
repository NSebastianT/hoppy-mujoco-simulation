# HOPPY MuJoCo Simulation

Proyecto de simulación del robot HOPPY en MuJoCo.

## Objetivo

Simular una versión simplificada del robot HOPPY usando MJCF/XML y Python. El modelo incluye:

- Gantry con dos grados de libertad pasivos.
- Pierna con dos grados de libertad activos: hip y knee.
- Contrapeso en el extremo opuesto del gantry.
- Contacto pie-suelo.
- Detección de touchdown y lift-off.
- Máquina de estados FLIGHT/STANCE.
- Control híbrido inicial.
- Saturación de torque.
- Estimación filtrada de velocidades.
- Generación de logs y gráficas.

## Nota sobre el modelo

Este repositorio no usa un modelo MuJoCo oficial de HOPPY ya existente. El archivo `models/hoppy.xml` es una aproximación simplificada construida en MJCF con base en la arquitectura documentada de HOPPY: gantry pasivo, contrapeso, pierna de dos articulaciones, contacto con el suelo y control híbrido.

El modelo actual no pretende representar con exactitud todas las piezas CAD, tornillos, perfiles metálicos o componentes reales del kit físico. Su propósito es capturar la estructura dinámica principal necesaria para la simulación y el análisis del controlador.

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

## Resultados

Las gráficas se guardan en:

```text
results/plots/
```

El log principal se guarda en:

```text
results/logs/hybrid_log.csv
```

## Estado actual

El proyecto actualmente logra:

- Cargar el modelo HOPPY simplificado.
- Simular el robot en MuJoCo.
- Detectar contacto pie-suelo.
- Cambiar entre FLIGHT y STANCE.
- Aplicar un controlador híbrido inicial.
- Generar gráficas de posiciones, velocidades, torques, fuerza normal y estado híbrido.

Pendiente:

- Mejorar el modelo físico usando parámetros más fieles de HOPPY.
- Agregar gráficas globales como `gantry_pitch`, `gantry_velocity` y `foot_world_z`.
- Ajustar el controlador para intentar más de un ciclo de salto.
- Documentar comparaciones con/sin armature, damping, spring y torque saturation.