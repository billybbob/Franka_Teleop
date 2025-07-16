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

class VitesseComparator(Node):
    def __init__(self):
        super().__init__('vitesse_comparator')
        
        # Initialisation des variables pour stocker les données
        self.robot_positions = deque(maxlen=1000000)  # Limite raisonnable
        self.controller_vitesses = deque(maxlen=1000000)  # Corrigé: nom cohérent
        self.robot_vitesses = deque(maxlen=1000000)  # Ajouté: variable manquante
        self.timestamps_robot = deque(maxlen=1000000)
        self.timestamps_controller = deque(maxlen=1000000)
        
        # Variables pour synchronisation
        self.lock = threading.Lock()
        self.last_robot_pos = None
        self.last_controller_vitesse = None
        
        # Variables pour calcul de vitesse avec filtrage
        self.last_time = time.time()
        self.previous_robot_pos = None
        
        # Filtrage des vitesses (moyenne mobile)
        self.velocity_filter_size = 3
        self.velocity_history_x = deque(maxlen=self.velocity_filter_size)
        self.velocity_history_y = deque(maxlen=self.velocity_filter_size)
        self.velocity_history_z = deque(maxlen=self.velocity_filter_size)
        
        # Subscriptions
        self.position_effecteur_robot_sub = self.create_subscription(
            TFMessage,
            '/position_effecteur_robot',
            self.position_effecteur_robot_callback,
            10
        )
        
        self.vitesse_controleur_sub = self.create_subscription(
            Float32MultiArray,
            '/cmd_vel',
            self.vitesse_controleur_callback,
            10
        )
        
        # Timer pour les graphiques (mise à jour toutes les 2 secondes)
        self.plot_timer = self.create_timer(2.0, self.update_plots)
        
        # Configuration matplotlib
        plt.ion()  # Mode interactif
        self.fig, self.axes = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.suptitle('Comparaison des vitesses - Robot vs Contrôleur Haptique')
        
        # Configuration des sous-graphiques
        self.axes[0, 0].set_title('Vitesse X')
        self.axes[0, 0].set_xlabel('Temps (s)')
        self.axes[0, 0].set_ylabel('Vitesse X (m/s)')
        self.axes[0, 0].grid(True)
        
        self.axes[0, 1].set_title('Vitesse Y')
        self.axes[0, 1].set_xlabel('Temps (s)')
        self.axes[0, 1].set_ylabel('Vitesse Y (m/s)')
        self.axes[0, 1].grid(True)
        
        self.axes[1, 0].set_title('Vitesse Z')
        self.axes[1, 0].set_xlabel('Temps (s)')
        self.axes[1, 0].set_ylabel('Vitesse Z (m/s)')
        self.axes[1, 0].grid(True)
        
        # Suppression du 4ème graphique inutilisé
        self.axes[1, 1].axis('off')
        
        plt.tight_layout()
        
        self.get_logger().info('Comparateur de vitesse initialisé')
    
    def position_effecteur_robot_callback(self, msg):
        """Callback pour la position du robot (TFMessage)"""
        with self.lock:
            current_time = time.time()
            
            # Extraction de la position depuis le TFMessage
            if msg.transforms:
                transform = msg.transforms[0]  # Premier transform
                position = transform.transform.translation
                
                # Corrigé: coordonnées cohérentes
                robot_pos = [position.x, position.y, position.z]
                self.robot_positions.append(robot_pos)
                self.timestamps_robot.append(current_time)
                
                # Calcul de la vitesse basé sur la différence de position
                if self.previous_robot_pos is not None:
                    dt = current_time - self.last_time
                    
                    # Filtrage plus strict du dt pour éviter les valeurs aberrantes
                    if dt < 0.001:  # Moins de 1ms
                        dt = 0.001
                    elif dt > 1.0:
                        dt = 1.0
                    
                    # Calcul des vitesses
                    vitesse_x = (robot_pos[0] - self.previous_robot_pos[0]) / dt
                    vitesse_y = (robot_pos[1] - self.previous_robot_pos[1]) / dt
                    vitesse_z = (robot_pos[2] - self.previous_robot_pos[2]) / dt
                    
                    # Filtrage des vitesses aberrantes (seuil réaliste pour un robot)
                    max_velocity = 0.5  # 5 m/s max (très généreux pour un robot manipulateur)
                    if (abs(vitesse_x) > max_velocity or 
                        abs(vitesse_y) > max_velocity or 
                        abs(vitesse_z) > max_velocity):
                        self.get_logger().warning(f'Vitesse aberrante détectée: [{vitesse_x:.2f}, {vitesse_y:.2f}, {vitesse_z:.2f}] m/s')
                        return
                    
                    # Filtrage par moyenne mobile pour lisser les données
                    self.velocity_history_x.append(vitesse_x)
                    self.velocity_history_y.append(vitesse_y)
                    self.velocity_history_z.append(vitesse_z)
                    
                    # Calcul de la moyenne mobile
                    filtered_vx = sum(self.velocity_history_x) / len(self.velocity_history_x)
                    filtered_vy = sum(self.velocity_history_y) / len(self.velocity_history_y)
                    filtered_vz = sum(self.velocity_history_z) / len(self.velocity_history_z)
                    
                    robot_vitesse = [filtered_vx, filtered_vy, filtered_vz]
                    self.robot_vitesses.append(robot_vitesse)
                    
                    self.get_logger().debug(f'Vitesse robot: {robot_vitesse}')
                
                # Mise à jour des variables de référence
                self.previous_robot_pos = robot_pos
                self.last_robot_pos = robot_pos
                self.last_time = current_time
                
                self.get_logger().debug(f'Position robot: {robot_pos}')
    
    def vitesse_controleur_callback(self, msg):
        """Callback pour la vitesse du contrôleur haptique (Float32MultiArray)"""
        with self.lock:
            current_time = time.time()
            
            # Extraction des vitesses depuis Float32MultiArray
            if len(msg.data) >= 3:
                controller_vitesse = [msg.data[0], -msg.data[1], -msg.data[2]]
                self.controller_vitesses.append(controller_vitesse)
                self.timestamps_controller.append(current_time)
                self.last_controller_vitesse = controller_vitesse
                
                self.get_logger().debug(f'Vitesse contrôleur: {controller_vitesse}')
    
    def update_plots(self):
        """Mise à jour des graphiques"""
        with self.lock:
            if not self.robot_vitesses or not self.controller_vitesses:
                self.get_logger().debug("Données insuffisantes pour tracer les graphiques")
                return
            
            # Conversion en numpy arrays pour faciliter les calculs
            robot_data = np.array(list(self.robot_vitesses))
            controller_data = np.array(list(self.controller_vitesses))
            
            # Timestamps relatifs (en secondes depuis le début)
            if self.timestamps_robot and self.timestamps_controller:
                start_time = min(self.timestamps_robot[0], self.timestamps_controller[0])
                # Conversion en liste pour permettre le slicing
                robot_times_list = list(self.timestamps_robot)
                controller_times_list = list(self.timestamps_controller)
                
                # Prendre les derniers timestamps correspondant aux vitesses
                if len(robot_times_list) >= len(self.robot_vitesses):
                    times_robot = np.array(robot_times_list[-len(self.robot_vitesses):]) - start_time
                else:
                    times_robot = np.array(robot_times_list) - start_time
                
                times_controller = np.array(controller_times_list) - start_time
            else:
                return
            
            # Effacement des graphiques précédents
            for i in range(2):
                for j in range(2):
                    if i == 1 and j == 1:  # Skip le 4ème graphique
                        continue
                    self.axes[i, j].clear()
            
            # Graphique X
            self.axes[0, 0].plot(times_robot, robot_data[:, 0], 'b-', label='Robot', linewidth=2)
            self.axes[0, 0].plot(times_controller, controller_data[:, 0], 'r-', label='Contrôleur', linewidth=2)
            self.axes[0, 0].set_title('Vitesse X')
            self.axes[0, 0].set_xlabel('Temps (s)')
            self.axes[0, 0].set_ylabel('Vitesse X (m/s)')
            self.axes[0, 0].legend()
            self.axes[0, 0].grid(True)
            
            # Graphique Y
            self.axes[0, 1].plot(times_robot, robot_data[:, 1], 'b-', label='Robot', linewidth=2)
            self.axes[0, 1].plot(times_controller, controller_data[:, 1], 'r-', label='Contrôleur', linewidth=2)
            self.axes[0, 1].set_title('Vitesse Y')
            self.axes[0, 1].set_xlabel('Temps (s)')
            self.axes[0, 1].set_ylabel('Vitesse Y (m/s)')
            self.axes[0, 1].legend()
            self.axes[0, 1].grid(True)
            
            # Graphique Z
            self.axes[1, 0].plot(times_robot, robot_data[:, 2], 'b-', label='Robot', linewidth=2)
            self.axes[1, 0].plot(times_controller, controller_data[:, 2], 'r-', label='Contrôleur', linewidth=2)
            self.axes[1, 0].set_title('Vitesse Z')
            self.axes[1, 0].set_xlabel('Temps (s)')
            self.axes[1, 0].set_ylabel('Vitesse Z (m/s)')
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
            filename = f'vitesse_comparison_{timestamp}.png'
        
        try:
            self.fig.savefig(filename, dpi=300, bbox_inches='tight')
            self.get_logger().info(f'Graphique sauvegardé: {filename}')
        except Exception as e:
            self.get_logger().error(f'Erreur lors de la sauvegarde: {e}')
    
    def save_data_to_csv(self, filename=None):
        """Sauvegarde les données dans un fichier CSV"""
        if filename is None:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f'vitesse_data_{timestamp}.csv'
        
        try:
            import csv
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # En-têtes
                writer.writerow(['timestamp_robot', 'robot_vx', 'robot_vy', 'robot_vz', 
                               'timestamp_controller', 'controller_vx', 'controller_vy', 'controller_vz'])
                
                # Données
                min_len = min(len(self.robot_vitesses), len(self.controller_vitesses))
                robot_vitesses_list = list(self.robot_vitesses)
                controller_vitesses_list = list(self.controller_vitesses)
                robot_times_list = list(self.timestamps_robot)
                controller_times_list = list(self.timestamps_controller)
                
                for i in range(min_len):
                    robot_vitesse = robot_vitesses_list[i]
                    controller_vitesse = controller_vitesses_list[i]
                    
                    # Prendre les timestamps correspondants
                    if len(robot_times_list) >= len(robot_vitesses_list):
                        robot_time = robot_times_list[-(len(robot_vitesses_list)-i)]
                    else:
                        robot_time = robot_times_list[i] if i < len(robot_times_list) else 0
                        
                    controller_time = controller_times_list[i] if i < len(controller_times_list) else 0
                    
                    writer.writerow([robot_time, robot_vitesse[0], robot_vitesse[1], robot_vitesse[2],
                                   controller_time, controller_vitesse[0], controller_vitesse[1], controller_vitesse[2]])
            
            self.get_logger().info(f'Données sauvegardées: {filename}')
        except Exception as e:
            self.get_logger().error(f'Erreur lors de la sauvegarde des données: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    vitesse_comparator = VitesseComparator()
    
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
                    vitesse_comparator.save_plot()
                elif user_input == 'd':
                    vitesse_comparator.save_data_to_csv()
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
        rclpy.spin(vitesse_comparator)
    except KeyboardInterrupt:
        pass
    finally:
        vitesse_comparator.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()