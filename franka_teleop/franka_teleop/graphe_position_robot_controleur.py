#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from tf2_msgs.msg import TFMessage
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import threading
import time

class PositionComparator(Node):
    def __init__(self):
        super().__init__('position_comparator')
        
        # Initialisation des variables pour stocker les données
        self.robot_positions = deque(maxlen=100000000)  # Limite à 1000 points
        self.controller_positions = deque(maxlen=100000000)
        self.timestamps_robot = deque(maxlen=100000000)
        self.timestamps_controller = deque(maxlen=100000000)
        
        # Variables pour synchronisation
        self.lock = threading.Lock()
        self.last_robot_pos = None
        self.last_controller_pos = None
        
        # Subscriptions
        self.position_effecteur_robot_sub = self.create_subscription(
            TFMessage,
            '/position_effecteur_robot',
            self.position_effecteur_robot_callback,
            10
        )
        
        self.position_controleur_sub = self.create_subscription(
            Float32MultiArray,
            '/valeur_effecteur',
            self.position_controleur_sub_callback,
            10
        )
        
        # Timer pour les graphiques (mise à jour toutes les 2 secondes)
        self.plot_timer = self.create_timer(2.0, self.update_plots)
        
        # Configuration matplotlib
        plt.ion()  # Mode interactif
        self.fig, self.axes = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.suptitle('Comparaison des positions - Robot vs Contrôleur Haptique')
        
        # Configuration des sous-graphiques
        self.axes[0, 0].set_title('Position X')
        self.axes[0, 0].set_xlabel('Temps (s)')
        self.axes[0, 0].set_ylabel('Position X (m)')
        self.axes[0, 0].grid(True)
        
        self.axes[0, 1].set_title('Position Y')
        self.axes[0, 1].set_xlabel('Temps (s)')
        self.axes[0, 1].set_ylabel('Position Y (m)')
        self.axes[0, 1].grid(True)
        
        self.axes[1, 0].set_title('Position Z')
        self.axes[1, 0].set_xlabel('Temps (s)')
        self.axes[1, 0].set_ylabel('Position Z (m)')
        self.axes[1, 0].grid(True)
        
        plt.tight_layout()
        
        self.get_logger().info('Comparateur de positions initialisé')
    
    def position_effecteur_robot_callback(self, msg):
        """Callback pour la position du robot (TFMessage)"""
        with self.lock:
            current_time = time.time()
            
            # Extraction de la position depuis le TFMessage
            # Supposons que la transformation contient la position du robot
            if msg.transforms:
                transform = msg.transforms[0]  # Premier transform
                position = transform.transform.translation
                
                robot_pos = [position.x, position.z, -position.y]
                self.robot_positions.append(robot_pos)
                self.timestamps_robot.append(current_time)
                self.last_robot_pos = robot_pos
                
                self.get_logger().debug(f'Position robot: {robot_pos}')
    
    def position_controleur_sub_callback(self, msg):
        """Callback pour la position du contrôleur haptique (Float32MultiArray)"""
        with self.lock:
            current_time = time.time()
            
            # Extraction des positions depuis Float32MultiArray
            if len(msg.data) >= 3:
                controller_pos = [msg.data[0], msg.data[1], msg.data[2]]
                self.controller_positions.append(controller_pos)
                self.timestamps_controller.append(current_time)
                self.last_controller_pos = controller_pos
                
                self.get_logger().debug(f'Position contrôleur: {controller_pos}')
    
    def update_plots(self):
        """Mise à jour des graphiques"""
        with self.lock:
            if not self.robot_positions or not self.controller_positions:
                return
            
            # Conversion en numpy arrays pour faciliter les calculs
            robot_data = np.array(list(self.robot_positions))
            controller_data = np.array(list(self.controller_positions))
            
            # Timestamps relatifs (en secondes depuis le début)
            if self.timestamps_robot and self.timestamps_controller:
                start_time = min(self.timestamps_robot[0], self.timestamps_controller[0])
                times_robot = np.array(self.timestamps_robot) - start_time
                times_controller = np.array(self.timestamps_controller) - start_time
            else:
                return
            
            # Effacement des graphiques précédents
            for ax in self.axes.flat:
                ax.clear()
            
            # Graphique X
            self.axes[0, 0].plot(times_robot, robot_data[:, 0], 'b-', label='Robot', linewidth=2)
            self.axes[0, 0].plot(times_controller, controller_data[:, 0], 'r-', label='Contrôleur', linewidth=2)
            self.axes[0, 0].set_title('Position X')
            self.axes[0, 0].set_xlabel('Temps (s)')
            self.axes[0, 0].set_ylabel('Position X (m)')
            self.axes[0, 0].legend()
            self.axes[0, 0].grid(True)
            
            # Graphique Y
            self.axes[0, 1].plot(times_robot, robot_data[:, 1], 'b-', label='Robot', linewidth=2)
            self.axes[0, 1].plot(times_controller, controller_data[:, 1], 'r-', label='Contrôleur', linewidth=2)
            self.axes[0, 1].set_title('Position Y')
            self.axes[0, 1].set_xlabel('Temps (s)')
            self.axes[0, 1].set_ylabel('Position Y (m)')
            self.axes[0, 1].legend()
            self.axes[0, 1].grid(True)
            
            # Graphique Z
            self.axes[1, 0].plot(times_robot, robot_data[:, 2], 'b-', label='Robot', linewidth=2)
            self.axes[1, 0].plot(times_controller, controller_data[:, 2], 'r-', label='Contrôleur', linewidth=2)
            self.axes[1, 0].set_title('Position Z')
            self.axes[1, 0].set_xlabel('Temps (s)')
            self.axes[1, 0].set_ylabel('Position Z (m)')
            self.axes[1, 0].legend()
            self.axes[1, 0].grid(True)
            
            plt.tight_layout()
            plt.draw()
            plt.pause(0.01)
    
    def save_plot(self, filename=None):
        """Sauvegarde le graphique actuel"""
        if filename is None:
            # Génération d'un nom de fichier avec timestamp
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f'position_comparison_{timestamp}.png'
        
        try:
            self.fig.savefig(filename, dpi=300, bbox_inches='tight')
            self.get_logger().info(f'Graphique sauvegardé: {filename}')
        except Exception as e:
            self.get_logger().error(f'Erreur lors de la sauvegarde: {e}')
    
    def save_data_to_csv(self, filename=None):
        """Sauvegarde les données dans un fichier CSV"""
        if filename is None:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f'position_data_{timestamp}.csv'
        
        try:
            import csv
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # En-têtes
                writer.writerow(['timestamp_robot', 'robot_x', 'robot_y', 'robot_z', 
                               'timestamp_controller', 'controller_x', 'controller_y', 'controller_z'])
                
                # Données
                for i in range(min(len(self.robot_positions), len(self.controller_positions))):
                    robot_pos = self.robot_positions[i]
                    controller_pos = self.controller_positions[i]
                    robot_time = self.timestamps_robot[i] if i < len(self.timestamps_robot) else 0
                    controller_time = self.timestamps_controller[i] if i < len(self.timestamps_controller) else 0
                    
                    writer.writerow([robot_time, robot_pos[0], robot_pos[1], robot_pos[2],
                                   controller_time, controller_pos[0], controller_pos[1], controller_pos[2]])
            
            self.get_logger().info(f'Données sauvegardées: {filename}')
        except Exception as e:
            self.get_logger().error(f'Erreur lors de la sauvegarde des données: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    position_comparator = PositionComparator()
    
    # Instructions pour l'utilisateur
    print("\n=== COMMANDES DISPONIBLES ===")
    print("Appuyez sur 's' + Entrée pour sauvegarder le graphique")
    print("Appuyez sur 'd' + Entrée pour sauvegarder les données CSV")
    print("Appuyez sur 'q' + Entrée pour quitter")
    print("==============================\n")
    
    # Thread pour gérer les entrées utilisateur
    def handle_input():
        while rclpy.ok():
            try:
                user_input = input().strip().lower()
                if user_input == 's':
                    position_comparator.save_plot()
                elif user_input == 'd':
                    position_comparator.save_data_to_csv()
                elif user_input == 'q':
                    rclpy.shutdown()
                    break
            except EOFError:
                break
            except KeyboardInterrupt:
                break
    
    # Lancement du thread pour les entrées utilisateur
    input_thread = threading.Thread(target=handle_input)
    input_thread.daemon = True
    input_thread.start()
    
    try:
        rclpy.spin(position_comparator)
    except KeyboardInterrupt:
        pass
    finally:
        position_comparator.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()