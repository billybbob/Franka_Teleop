---

# Project Overview

Ce projet est basé sur le dépôt Git : (https://github.com/frankarobotics/franka_ros2), où vous pourrez trouver des informations complémentaires.

Il a été modifié pour implémenter un système complet de téléopération pour le robot Franka FR3 en utilisant un contrôleur haptique **Haption Desktop 6D**. Ce système permet de contrôler le robot à distance avec un retour de force haptique, offrant ainsi une interface intuitive pour la manipulation robotique avancée. Il intègre également une simulation sous **Gazebo Fortress** afin de faciliter la prise en main du contrôleur.

---

## Prérequis

## Installation

1. **Installer l’environnement de développement ROS 2**

   ***franka\_ros2*** repose sur ***ROS 2 Humble***.

   Pour configurer votre environnement ROS 2, suivez les instructions officielles d’installation de la version ***Humble***, disponibles [**ici**](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

   Le guide propose deux options principales d’installation : **Desktop** et **Bare Bones**.

   #### Choisissez **une** des options suivantes :

   * **Installation "Desktop" de ROS 2** (`ros-humble-desktop`)
     Inclut une installation complète avec les outils graphiques et les paquets de visualisation (ex : Rviz et Gazebo).
     **Recommandé** pour les utilisateurs ayant besoin de simulation ou de visualisation.

   * **Installation "ROS-Base" (Bare Bones)** (`ros-humble-ros-base`)
     Une installation minimale qui inclut uniquement les bibliothèques de base de ROS 2.
     Adaptée aux environnements avec ressources limitées ou aux systèmes sans interface graphique.

   ```bash
   # remplacez <YOUR CHOICE> par ros-humble-desktop ou ros-humble-ros-base
   sudo apt install <YOUR CHOICE>
   ```

   ---

   Installez également les outils de développement :

   ```bash
   sudo apt install ros-dev-tools
   ```

   L’installation de la version **Desktop** ou **Bare Bones** devrait automatiquement sourcer l’environnement ROS 2, mais dans certains cas vous devrez le faire manuellement :

   ```bash
   source /opt/ros/humble/setup.sh
   ```

2. **Créer un espace de travail ROS 2 :**

   ```bash
   mkdir -p ~/franka_ros2_ws/src
   cd ~/franka_ros2_ws  # pas dans le dossier src
   ```

3. **Cloner les dépôts :**

   ```bash
   git clone https://github.com/billybbob/Franka_Teleop.git src
   ```

4. **Installer les dépendances :**

   ```bash
   vcs import src < src/franka.repos --recursive --skip-existing
   ```

5. **Détecter et installer les dépendances du projet :**

   ```bash
   rosdep install --from-paths src --ignore-src --rosdistro humble -y
   ```

6. **Prise en compte de l'API Raptor :**

  '''bash
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/franka_ros2_ws/src/haption_raptor_api/Dependencies/RaptorAPI/bin/Linux/glibc-2.31
  '''

  Une fois cela fait vous allez pouvoir construire le dossier lié à cette API :

  '''bash
  colcon build --packages-select raptor_api_interfaces
  '''

  Placer le fichier *start_RaptorAPIWrapper.sh* en dehors du dossier source. Vous pouvez maintenant lancer RaptorAPIWrapper :

  '''bash
  source install/local_setup.bash
  chmod +x start_RaptorAPIWrapper.sh
  sudo ./start_RaptorAPIWrapper.sh
  '''

  Vous devriez avoir une erreur, vous disant que des fichier sont manquants dans /install/... Pour cela :

  '''bash
  sudo cp -R /src/Haption/lib /install/haption_raptor_api
  '''

  Vous pouvez maintenant relancer l'API, si cela fonctionne supprimer *haption_raptor_api* et *raptor_api_interfaces* du dossier source, ainsi que start_RaptorAPIWrapper.sh

7. **Compiler le projet :**

   ```bash
   # utilisez l’option --symlink-install pour réduire l’usage disque et faciliter le développement
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

8. **Ajuster l’environnement :**

   ```bash
   # Adapter l’environnement pour reconnaître les paquets et dépendances de votre workspace ROS 2 fraîchement compilé
   source install/setup.sh
   ```

---
## Utilisation

1. **Calibration du contrôleur**

   Ouvrir une première fenêtre, la sourcer, et utiliser :
  '''bash
  ros2 run test_cartesien TestCalibration /etc/Haption/desktop_6D_n76.param "channel=SimpleChannelUDP:localip=0.0.0.0:localport=12120:remoteip=192.168.100.53:remoteport=5000"
  '''
  
2.  **Lancement de la simulation**

  Dans une seconde fenêtre, la sourcer également, et utiliser :
  '''bash
  ros2 launch franka_gazebo_bringup gazebo_joint_velocity_position.launch.py load_gripper:=true franka_hand:='franka_hand' robot_ip:=dont-care use_fake_hardware:=true
  '''

3. **Lancement du contrôleur**

  De retour dans la première fenêtre faire :
  '''bash
  ros2 run test_cartesien TestPoseCartesian
  '''

4. **Lancement du pilotage**

  Enfin dans une troisième (et quatrième) fenêtre.
  Pour lancer le mode vitesse, ouvrir une nouvelle fenêtre, sourcé, et lancé :
   '''bash
  ros2 run test_cartesien value_to_speed
  '''

  Pour lancer le mode position, ouvrir une nouvelle fenêtre, sourcé, et lancé :
  '''bash
  ros2 run test_cartesien offset_position
  '''

---

## Intégration ROS 2 pour les robots de recherche Franka Robotics

[![CI](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml)

Consultez la documentation [Franka Control Interface (FCI)][fci-docs] pour plus d’informations.

---

## Licence

Tous les paquets du dépôt `franka_ros2` sont sous licence [Apache 2.0][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs

---
