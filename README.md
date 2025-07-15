---
# Project Overview

Ce projet est basé sur le dépôt Git https://github.com/frankarobotics/franka_ros2 , où vous pourrez trouver des informations complémentaires.

Il a été modifié pour implémenter un système complet de téléopération pour le robot Franka FR3 en utilisant un contrôleur haptique **Haption Desktop 6D**. Ce système permet de contrôler le robot à distance avec un retour de force haptique, offrant ainsi une interface intuitive pour la manipulation robotique avancée. Il intègre également une simulation sous **Gazebo Fortress** afin de faciliter la prise en main du contrôleur.

---

#### Table des Matières
- [Installation](#Installation)
- [Utilisation avec la simulation](#utilisation-avec-la-simulation)
- [Utilisation avec un robot](#utilisation-avec-un-robot)
- [Erreur possible de rencontrer](#erreur-possible-de-rencontrer)
- [Conseil d'utilisation](#conseil-dutilisation)
  

## Prérequis

## Installation

1. **Installer l’environnement de développement ROS 2**

   _**franka_ros2**_ repose sur _**ROS 2 Humble**_.

   Pour configurer votre environnement ROS 2, suivez les instructions officielles d’installation de la version _**Humble**_, disponibles [**ici**](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

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
   mkdir -p ~/franka_teleop_ws/src
   cd ~/franka_teleop_ws  # pas dans le dossier src
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
   rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys="ros_gz_example_description numpy"
   ```

6. **Prise en compte de l'API Raptor :**

   Si vous avez accès au répertoire Haption alors faites les étapes indiquées sur ce git, si vous avez accès au dossier faites les étapes indiquées dans ce derniers. Sinon passer à la suite mais vous ne pourrez pas téléopérer le robot ni la simulation.

7. **Compiler le projet :**

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

8. **Ajuster l’environnement :**

   ```bash
   source install/setup.sh
   ```

---

## Utilisation avec la simulation

1. **Calibration du contrôleur**

   Ouvrir une première fenêtre, la sourcer, et utiliser :

   ```bash
   ros2 run test_cartesien TestCalibration /etc/Haption/desktop_6D_n76.param "channel=SimpleChannelUDP:localip=0.0.0.0:localport=12120:remoteip=192.168.100.53:remoteport=5000"
   ```

2. **Lancement de la simulation**

   Dans une seconde fenêtre, la sourcer également, et utiliser :

   ```bash
   ros2 launch franka_gazebo_bringup gazebo_joint_velocity_position.launch.py load_gripper:=true franka_hand:='franka_hand' robot_ip:=dont-care use_fake_hardware:=true
   ```

3. **Lancement du contrôleur**

   De retour dans la première fenêtre, faire :

   ```bash
   ros2 run test_cartesien TestPoseCartesian
   ```

---

## Utilisation avec un robot

1. **Modification de programme**

   Dû au fait que le robot crée des topics avec des noms différents de ceux de la simulation, il faut modifier certains codes. Ainsi pour mgd.py, franka_ig_solver.py, force_position.py et force_vitesse.py il faut changer l'abonnement au topic "/joint_states" par "/NS_1/joint_states". De plus pour switch_mode.py il faut aussi faire un changement, passer de "joint_velocity_example_controller.py" à "cartesian_velocity_example_controller.py"

2. **Démarrage du robot**

   Aller sur internet et dans la barre de recherche mettre l'adresse IP du robot, soit ici : 192.168.100.XXX
   Puis désactiver les freins, activer le FCI et se mettre en mode programme. Alors la lumière du robot devrait être en vert.

3. **Calibration du contrôleur**

   Ouvrir une première fenêtre, la sourcer, et utiliser :

   ```bash
   ros2 run test_cartesien TestCalibration /etc/Haption/desktop_6D_n76.param "channel=SimpleChannelUDP:localip=0.0.0.0:localport=12120:remoteip=192.168.100.53:remoteport=5000"
   ```

4. **Lancement du robot**

   Dans une seconde fenêtre, la sourcer également, et utiliser :

   ```bash
   ros2 launch franka_bringup example.launch.py ros2 controller_name:=joint_position_example_controller robot_config_file:=/utilisateur/franka_ros2_ws/src/franka_bringup/config/custom_franka.config.yaml
   ```

5. **Lancement du contrôleur**

   De retour dans la première fenêtre, faire :

   ```bash
   ros2 run test_cartesien TestPoseCartesian
   ```

---

## Conseil d'utilisation

Pour les premiers essais il est conseillé d’utiliser la simulation, cependant si l’ordinateur est peu performant cette dernière peut ne pas être fluide. De plus pour les débuts il peut être envisageable de ne pas activé le retour de force lorsque vous êtes en mode position afin de comprendre comment se comporte le robot. Cependant avec le mode vitesse il est conseillé d’utiliser toujours le retour de force afin de savoir à n’importe quel instant où est la zone morte.
Par la suite s’il y a volonté de rester en mode position et que le coefficient d’homothétie n’est pas suffisamment grand pour faire les mouvements souhaitez, vous pouvez utiliser l’homme-mort. Lorsque vous le touchez déplacer vous à l’opposer de là où vous souhaitez aller, puis une fois en limite du contrôleur relâchez l’homme-mort et faite votre mouvement. Attention cela va changer de place le repère du contrôleur, c’est pourquoi lorsque vous faites cela il est impératif d’avoir le bouton d’arrêt d’urgence à côté. De plus avec cette utilisation il est compliqué de passer par la suite le mode vitesse, puisque vous avez déplacé la zone morte en bougeant le repère.

---

## Erreur possible de rencontrer

Lors de l'utilisation du contrôleur haptique il est possible d'avoir une "erreur 2", alors la communication avec ce dernier se coupe. Cela est dû au fait que la communication avec celui-ci est dépassé les 300ms. Malheureusement à cause du watchdogs implémenté en dur il n'y a pas de solution pour ce problème.

Lors de l'utilisation avec le robot vous pourrez avoir une erreur de discontinuité de vitesse et/ou d'accélération ainsi qu'une violation des limites, cela peut arriver si vous essayer d'aller derrière le robot. Malheureusement là aussi il n'y a pas de solution.

---

## Intégration ROS 2 pour les robots de recherche Franka Robotics

[![CI](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml)

Consultez la documentation [Franka Control Interface (FCI)][fci-docs] pour plus d’informations.

---

## Licence

Tous les paquets du dépôt `Franka_Teleop` sont sous licence [Apache 2.0][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs
