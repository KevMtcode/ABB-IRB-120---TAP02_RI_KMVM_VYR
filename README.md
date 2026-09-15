# ABB-IRB-120---TAP02_RI_KMVM_VYR
Célula de ensamble simulada en ROS2 + MoveIt2 + RViz2, donde el manipulador serial ABB IRB 120 realiza un ciclo completo de pick-and-place.

_________RESUMEN DE EJECUCIÓN_____________

PUNTO 4

4A – HOME → PRE-PICK

1. Abrir MoveIt/RViz:
rm -rf build/irb120_moveit_config install/irb120_moveit_config
source /opt/ros/jazzy/setup.bash
colcon build --packages-select irb120_moveit_config
source install/setup.bash
export LC_NUMERIC=C
ros2 launch irb120_moveit_config demo.launch.py

2. Publicar la PlanningScene con el obstáculo.

3. Configurar:
   Start State: HOME
   Goal State: PRE-PICK
   Planning Pipeline: OMPL

4. Probar los dos planners:
   - RRTConnectkConfigDefault
   - RRTstarkConfigDefault

5. Para cada planner:
   - Planear HOME → PRE-PICK.
   - Registrar el tiempo de planeación mostrado por MoveIt/RViz.
   - Ejecutar:
     python3 scripts/compare_4a.py
     para obtener la longitud de la trayectoria.
   - Comparar visualmente la suavidad de las trayectorias.

6. Elegir el planner con mejor desempeño.
   Este planner se reutiliza para el movimiento libre del punto 4C.


4B – PRE-PICK → PICK

1. Generar los perfiles cúbico y quíntico:
   python3 scripts/trayectorias_4bd.py

2. El programa genera:
   - Perfil cúbico.
   - Perfil quíntico.
   - 4 waypoints intermedios.
   - Gráficas de posición, velocidad y aceleración.
   - Archivos CSV para ida y vuelta.

3. Restricciones:
   Ida roja:
   PRE-PICK → PICK
   Vmax = 0.200 m/s
   Amax = 0.300 m/s²

   Vuelta azul:
   PICK → PRE-PICK
   Vmax = 0.100 m/s
   Amax = 0.020 m/s²

4. Con el robot en PRE-PICK, probar la ida cúbica:
   ros2 launch irb120_cartesian cartesian_path.launch.py \
   csv:="$(pwd)/scripts/waypoints/4B_cubico_ida.csv"

5. Con el robot nuevamente en PRE-PICK, probar la ida quíntica:
   ros2 launch irb120_cartesian cartesian_path.launch.py \
   csv:="$(pwd)/scripts/waypoints/4B_quintico_ida.csv"

6. Con el robot en PICK, probar el retorno cúbico:
   ros2 launch irb120_cartesian cartesian_path.launch.py \
   csv:="$(pwd)/scripts/waypoints/4B_cubico_vuelta.csv"

7. Con el robot nuevamente en PICK, probar el retorno quíntico:
   ros2 launch irb120_cartesian cartesian_path.launch.py \
   csv:="$(pwd)/scripts/waypoints/4B_quintico_vuelta.csv"

8. En cada prueba verificar:
   - Trayectoria completada = 100 %.
   - Tiempo del perfil = tiempo aplicado en MoveIt.
   - Aceleración máxima de cada articulación.
   - Movimiento recto en RViz.

9. Comparar las aceleraciones articulares y la suavidad.
   El perfil quíntico presentó el mejor desempeño, por lo que se selecciona para 4D.


4C – PICK → PRE-PLACE

1. Abrir MoveIt/RViz y mantener publicada la misma PlanningScene.

2. Llevar el robot a PICK.

3. Configurar:
   Start State: PICK
   Goal State: PRE-PLACE
   Planning Pipeline: OMPL

4. Seleccionar el planner ganador del punto 4A.

5. Planear PICK → PRE-PLACE.

6. Verificar que la trayectoria rodee los obstáculos y no presente colisiones.

7. Ejecutar la trayectoria para dejar el robot en PRE-PLACE.


4D – PRE-PLACE → PLACE

1. Ejecutar nuevamente:
   python3 scripts/trayectorias_4bd.py

2. Para 4D se utiliza únicamente el perfil quíntico, ya seleccionado como el mejor en 4B.

3. Restricciones:
   Ida roja:
   PRE-PLACE → PLACE
   Vmax = 0.200 m/s
   Amax = 0.300 m/s²

   Vuelta azul:
   PLACE → PRE-PLACE
   Vmax = 0.100 m/s
   Amax = 0.020 m/s²

4. Con el robot en PRE-PLACE ejecutar:
   ros2 launch irb120_cartesian cartesian_path.launch.py \
   csv:="$(pwd)/scripts/waypoints/4D_quintico_ida.csv"

5. Verificar:
   - Trayectoria completada = 100 %.
   - Tiempo del perfil = tiempo aplicado en MoveIt.
   - Movimiento recto PRE-PLACE → PLACE.
   - Aceleraciones articulares.

6. Para comprobar el retorno, colocar el robot en PLACE y ejecutar:
   ros2 launch irb120_cartesian cartesian_path.launch.py \
   csv:="$(pwd)/scripts/waypoints/4D_quintico_vuelta.csv"

7. Verificar nuevamente la trayectoria, tiempos y aceleraciones articulares.


8. Animar todo: ./scripts/ciclo_completo.sh
