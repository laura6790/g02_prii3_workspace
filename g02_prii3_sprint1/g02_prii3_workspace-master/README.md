
### PBI 1.2  - Workspace y Paquete ROS
- Workspace: g02_prii3.ws
- Paquete: g02_prii3_turtlesim
- Compilación: `colcon build --packages-select g02_prii3_turtlesim`

### PBI 1.3  - Controlador turtlesim
- Dibujo automático del número 2
- Servicios: stop_drawing, resume_drawing, reset_drawing
- Archivo launch único

### Uso:
```bash
# Clonar y compilar
git clone https://github.com/laura6790/g02_prii3_workspace.git
cd g02_prii3_workspace
colcon build --packages-select g02_prii3_turtlesim
source install/setup.bash

# Ejecutar
ros2 launch g02_prii3_turtlesim turtle_draw.launch.py

# Servicios
ros2 service call /stop_drawing std_srvs/srv/Empty
ros2 service call /reset_drawing std_srvs/srv/Empty
ros2 service call /resume_drawing std_srvs/srv/Empty

