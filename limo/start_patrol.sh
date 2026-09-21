#!/bin/bash

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

cleanup() {
    echo -e "\n[!] Arrêt d'urgence de la patrouille..."
    kill 0
    exit
}
trap cleanup EXIT INT

echo "============================================="
echo " CARTES DISPONIBLES DANS LA MÉMOIRE DU ROBOT :"
ls /home/agilex/*.yaml
echo "============================================="
echo "Attention: Place bien le robot sur le 'Point Zéro' de cette carte !"
read -p "Écris le nom de la carte (SANS le .yaml, ex: carte_couloir3) : " NOM_CARTE

echo -e "\n=== ETAPE 1 : Allumage du corps ==="
ros2 launch limo_bringup limo_start.launch.py &
sleep 5

echo "=== ETAPE 1.2 : Publication robot_description pour rviz2 ==="
# Script Python dédié : publie /robot_description avec QoS TRANSIENT_LOCAL
# sans émettre aucun TF — le TF tree reste uniquement celui de limo_bringup
python3 ~/mes_scripts_ia/pub_robot_description.py &

echo "=== ETAPE 1.3 : Publication des joints pour rviz2 ==="
ros2 run joint_state_publisher joint_state_publisher --ros-args -p robot_description:="$(cat /home/agilex/limo_propre.urdf)" &
sleep 2

echo "=== ETAPE 1.5 : Allumage des Yeux (Caméra Orbbec) ==="
ros2 launch orbbec_camera dabai.launch.py &
sleep 5

NAV2_PARAMS="/home/agilex/limo_ros2_ws/install/limo_bringup/share/limo_bringup/param/nav2.yaml"
echo "=== ETAPE 3 : IA Nav2 (carte: ${NOM_CARTE}, params: nav2.yaml LIMO) ==="
ros2 launch nav2_bringup bringup_launch.py \
    map:=/home/agilex/${NOM_CARTE}.yaml \
    params_file:=${NAV2_PARAMS} \
    use_composition:=False &
sleep 8

echo "=== ETAPE 4 : Point de départ Zéro ==="
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" &
sleep 5

echo "=== ETAPE 5 : Lancement du script IA ==="
python3 ~/mes_scripts_ia/patrouille.py ${NOM_CARTE}

wait
