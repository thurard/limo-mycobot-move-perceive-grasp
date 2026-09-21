#!/bin/bash

# Variables réseau (pour pouvoir piloter le robot depuis ton PC)
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

cleanup() {
    echo -e "\n[!] Arrêt de la cartographie..."
    kill 0
    exit
}
trap cleanup EXIT INT

echo "=== ETAPE 1 : Allumage du robot ==="
ros2 launch limo_bringup limo_start.launch.py &
sleep 5

echo "=== ETAPE 2 : Lancement de Cartographer ==="
ros2 launch limo_bringup cartographer.launch.py

wait
