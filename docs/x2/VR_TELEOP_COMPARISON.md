# Comparación y pruebas de teleoperación VR del X2

Se comparó el `main` de Zachary-CHAN (`2feea27`) con nuestro fork en `8050ee7`. Ambos compartían la ruta v4 de tres puntos para brazos: mink resuelve hombro y codo y el controlador de muñeca aplica la orientación. Esa ruta usa una base sujeta. El modo neuronal `VR_3PT` de NVIDIA usa un encoder TELEOP y combina los objetivos del visor y los dos mandos con una referencia de locomoción. Los launchers X2 todavía no exponían ese modo ni una opción para el DLS de HandUMI.

## Modos añadidos

| Comando | Comportamiento |
| --- | --- |
| `./gear_sonic/scripts/sim_onnx_planner.sh --three-point-vr` | SONIC neuronal, PICO y dos mandos, con planner para las piernas. Añadir `--input-source quest3` para usar Quest 3 vía WebXR. |
| `./gear_sonic/scripts/sim_onnx_planner.sh --ik` | DLS de HandUMI para los brazos del X2, con PICO y dos mandos. |
| `./gear_sonic/scripts/run_pico_teleop_v4_sim.sh` | Referencia v4 original con mink. |

Los dos modos nuevos apuntan a simulación local y usan el visor y los mandos sin trackers de tobillo. El README principal quedó como una lista de comandos por modo.

- **SONIC.** Se añadió calibración de objetivos respecto a la pose medida del X2 y se corrigió la conversión de ejes PICO para conservar el giro de cabeza. El encoder recibe 267 valores y produce 64 tokens para el decoder. Si se pierde el tracking o la referencia, se suelta el control neuronal y se requiere reactivación manual. En PICO, `A+B+X+Y` calibra y activa o desactiva el control. El clic del stick izquierdo cambia entre TELEOP y planner. El gatillo izquierdo habilita la locomoción por sticks.
- **Modelos.** Los ONNX incluidos de 45.000 iteraciones exponen pose y SMPL. El modo neuronal nuevo usa el checkpoint público X2 de 14.000 iteraciones. Sus tres ONNX se exportan juntos y se comprueban contra PyTorch. Para prepararlos, ejecutar `./gear_sonic/scripts/sim_onnx_planner.sh --three-point-vr --prepare-models`.
- **HandUMI.** Se adaptó el DLS a la cinemática del X2: un paso por ciclo de 50 Hz, amortiguación dependiente de singularidad, regularización hacia reposo, orientación ponderada y límite de 1 rad/s por articulación. Cada grip habilita su brazo. La ruta reutiliza los límites, filtros y protecciones de alcance de v4. Si falta la tabla externa del SDK, MuJoCo usa los valores del reposo MC registrados en el repo.

## Pruebas sin visor

Pasaron 73 pruebas seleccionadas. Cubrieron, entre otros casos, equivalencia de observaciones, conversión PICO/Quest, pérdida y recuperación del tracking con ZMQ y ONNX, límites de DLS y regresión de v4. Las dos flags se comprobaron con `--dry-run`.

Se ejecutaron tres simulaciones locales en MuJoCo con entradas sintéticas. Cada video tiene 12 segundos, 24 fps y 288 cuadros verificados con ffprobe. La secuencia incluye reposo, alcance izquierdo, alcance derecho, alcance con ambas manos y vuelta al reposo. SONIC recibió además un giro de cabeza.

| Video | Base | Error medio muñeca izquierda / derecha |
| --- | --- | --- |
| `01_sonic_neuronal_tres_puntos.mp4` | Libre | 136,1 / 84,5 mm |
| `02_handumi_dls_ik.mp4` | Sujeta | 5,3 / 5,2 mm |
| `03_v4_original_mink.mp4` | Sujeta | 4,9 / 4,9 mm |

SONIC completó la secuencia sin caída, con una altura mínima de pelvis de 0,648 m y un seguimiento de manos impreciso. La postura inicial, la base y el punto de seguimiento difieren entre SONIC y los modos IK, por lo que los errores no constituyen una comparación controlada. Los videos, trayectorias, logs y el script de captura se guardaron localmente en `logs/pruebas_sin_visor_2026-11-23/` y en un ZIP del mismo nombre.

## Prueba interactiva y Codex Cloud

Para una prueba local con PICO se necesita XRoboToolkit PC Service, la app del visor enviando datos, y las dependencias del simulador. SONIC requiere además Docker/Compose y un modelo de planner X2. El lanzamiento completo del stack C++ quedó sin verificar por falta de acceso al socket Docker en la sesión local. También quedaron pendientes las pruebas con visor en vivo, la marcha con el planner y el robot físico.

Para Codex Cloud, la ruta inicial es preparar los ONNX y ejecutar pruebas sin visor. El entorno necesita una revisión con estos cambios, archivos LFS, dependencias y acceso a Hugging Face si no cuenta con los modelos. Para sesiones en vivo aún falta enlazar la entrada del visor local y la salida de imagen con la simulación cloud. También falta una ruta de replay común para comparar ambos modos con la misma grabación del PICO. Con 30 % de uso restante, la recomendación fue empezar por una comprobación corta de DLS y dejar la prueba completa de SONIC para cuando hubiera más margen.
