#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import math

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        
        # Publisher para controlar la tortuga
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Servicios para controlar el dibujo
        self.stop_service = self.create_service(Empty, 'stop_drawing', self.stop_callback)
        self.resume_service = self.create_service(Empty, 'resume_drawing', self.resume_callback)
        self.reset_service = self.create_service(Empty, 'reset_drawing', self.reset_callback)
        
        # Variables de control
        self.is_drawing = True
        self.step = 0
        self.initial_x = 5.5  # Posición inicial centrada
        self.initial_y = 5.5
        
        # Secuencia de movimientos para dibujar el número 2
        self.movements = self.generate_number_2_movements()
        
        # Timer
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('=== Nodo de control de tortuga inicializado ===')
        self.get_logger().info('Servicios disponibles:')
        self.get_logger().info('  /stop_drawing - Detiene el dibujo')
        self.get_logger().info('  /resume_drawing - Reanuda el dibujo')
        self.get_logger().info('  /reset_drawing - REINICIA (limpia + posición inicial SIN dibujar)')
        
    def generate_number_2_movements(self):
        movements = []
        for _ in range(6):
            movements.append((0.0, 0.25))  # giro hacia la derecha
        for _ in range(7):
            movements.append((0.4, -0.55))  # curva suave hacia la derecha
        for _ in range(4):
            movements.append((0.5, 0.0))  # avance recto con orientación actual
        for _ in range(6):
            movements.append((0.0, 0.4))  # giro hacia la derecha
        for _ in range(5):
            movements.append((0.5, 0.0))  # recta final
        return movements
    
    
    def timer_callback(self):
        if not self.is_drawing or self.step >= len(self.movements):
            return
            
        linear, angular = self.movements[self.step]
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)
        
        self.get_logger().info(f'Paso {self.step+1}/{len(self.movements)}')
        self.step += 1
        
        if self.step >= len(self.movements):
            self.get_logger().info('¡Dibujo del número 2 completado!')
            
    def stop_callback(self, request, response):
        """Detiene el dibujo"""
        if self.is_drawing:
            self.is_drawing = False
            self.get_logger().info('Dibujo DETENIDO')
        else:
            self.get_logger().info('El dibujo ya estaba detenido')
        return response
        
    def resume_callback(self, request, response):
        """Reanuda el dibujo"""
        if not self.is_drawing:
            self.is_drawing = True
            self.get_logger().info('Dibujo REANUDADO')
        else:
            self.get_logger().info('El dibujo ya estaba en ejecución')
        return response
        
    def reset_callback(self, request, response):
        """REINICIA el dibujo - limpia y posición inicial SIN dibujar línea"""
        # 1. LEVANTAR EL LÁPIZ (desactivar dibujo)
        self.lift_pen()
        
        # 2. Resetear variables
        self.step = 0
        self.is_drawing = True
        
        # 3. Detener cualquier movimiento
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher_.publish(stop_msg)
        
        # 4. LIMPIAR LA PANTALLA
        self.clear_screen()
        
        # 5. Teletransportar a posición inicial (SIN dibujar)
        self.teleport_to_initial_position()
        
        # 6. BAJAR EL LÁPIZ (activar dibujo para el próximo movimiento)
        self.put_pen_down()
        
        self.get_logger().info('Dibujo REINICIADO - Posición inicial SIN línea')
        return response
    
    def lift_pen(self):
        """Levanta el lápiz para no dibujar"""
        try:
            from turtlesim.srv import SetPen
            client = self.create_client(SetPen, '/turtle1/set_pen')
            
            if client.wait_for_service(timeout_sec=1.0):
                request = SetPen.Request()
                request.off = 1  # 1 = lápiz levantado (no dibuja)
                future = client.call_async(request)
                self.get_logger().info('✏️ Lapiz LEVANTADO (no dibuja)')
            else:
                self.get_logger().warn('Servicio set_pen no disponible')
                
        except Exception as e:
            self.get_logger().error(f'Error al levantar lápiz: {e}')
    
    def put_pen_down(self):
        """Baja el lápiz para dibujar"""
        try:
            from turtlesim.srv import SetPen
            client = self.create_client(SetPen, '/turtle1/set_pen')
            
            if client.wait_for_service(timeout_sec=1.0):
                request = SetPen.Request()
                request.off = 0  # 0 = lápiz bajado (dibuja)
                request.r = 255  # Color rojo
                request.g = 0
                request.b = 0
                request.width = 3
                future = client.call_async(request)
                self.get_logger().info('Lápiz BAJADO (listo para dibujar)')
            else:
                self.get_logger().warn('Servicio set_pen no disponible')
                
        except Exception as e:
            self.get_logger().error(f'Error al bajar lápiz: {e}')
    
    def clear_screen(self):
        """Limpia la pantalla"""
        try:
            client = self.create_client(Empty, '/clear')
            
            if client.wait_for_service(timeout_sec=1.0):
                request = Empty.Request()
                future = client.call_async(request)
                self.get_logger().info('Pantalla LIMPIADA')
            else:
                self.get_logger().warn('Servicio de limpieza no disponible')
                
        except Exception as e:
            self.get_logger().error(f'Error al limpiar pantalla: {e}')
    
    def teleport_to_initial_position(self):
        """Teletransporta la tortuga a la posición inicial SIN dibujar"""
        try:
            from turtlesim.srv import TeleportAbsolute
            client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
            
            if client.wait_for_service(timeout_sec=1.0):
                request = TeleportAbsolute.Request()
                request.x = self.initial_x
                request.y = self.initial_y
                request.theta = 0.0
                
                future = client.call_async(request)
                self.get_logger().info(f'Teletransportando a posición inicial SIN dibujar')
            else:
                self.get_logger().warn('Servicio de teleport no disponible')
                
        except Exception as e:
            self.get_logger().error(f'Error al teletransportar: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
