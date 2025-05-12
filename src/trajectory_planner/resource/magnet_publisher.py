#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

class MagnetPublisher(Node):
    def __init__(self):
        super().__init__('magnet_publisher')
        
        # Publisher für normalen GPIO AN/AUS
        self.bool_publisher = self.create_publisher(Bool, '/magnet_control', 10)
        
        # Publisher für PWM-Steuerung (Duty Cycle 0-100%)
        self.pwm_publisher = self.create_publisher(Float32, '/magnet_pwm_control', 10)
        
        self.get_logger().info("Magnet Publisher gestartet! Warte auf Eingaben...")

    def publish_gpio(self, state: bool):
        """Veröffentlicht einen AN/AUS-Befehl für den Magneten."""
        msg = Bool()
        msg.data = state
        self.bool_publisher.publish(msg)
        self.get_logger().info(f"Gesendet: Magnet {'AN' if state else 'AUS'}")

    def publish_pwm(self, duty_cycle: float):
        """Veröffentlicht einen PWM-Wert für den Magneten (0-100%)."""
        duty_cycle = max(0.0, min(100.0, duty_cycle))  # Begrenzung auf gültige Werte
        msg = Float32()
        msg.data = duty_cycle
        self.pwm_publisher.publish(msg)
        self.get_logger().info(f"Gesendet: Magnet PWM auf {duty_cycle}% gesetzt")

def main(args=None):
    rclpy.init(args=args)
    node = MagnetPublisher()

    try:
        while rclpy.ok():
            mode = input("Modus wählen: (1) GPIO AN/AUS, (2) PWM: ")
            
            if mode == "1":
                value = input("Magnet AN (1) oder AUS (0): ")
                if value in ["0", "1"]:
                    node.publish_gpio(value == "1")
                else:
                    print("Ungültige Eingabe! Nur 0 oder 1.")

            elif mode == "2":
                value = input("PWM Duty Cycle (0-100%): ")
                try:
                    pwm_value = float(value)
                    node.publish_pwm(pwm_value)
                except ValueError:
                    print("Ungültige Eingabe! Bitte eine Zahl zwischen 0 und 100 eingeben.")

            else:
                print("Ungültige Auswahl! Bitte 1 oder 2 eingeben.")

    except KeyboardInterrupt:
        print("Beende Magnet Publisher...")
    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

