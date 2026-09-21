#!/usr/bin/env python3
# ============================================================
# detect_3d.py — Detection d'objets + position 3D.
#
# Combine YOLO (quoi + ou dans l'image) et la profondeur de la
# camera (distance) pour calculer la position 3D reelle de chaque
# objet detecte, dans le repere de la camera. C'est la brique de
# base pour une future saisie par le bras.
#
# Le robot est suppose A L'ARRET face a l'objet.
#
# Usage : python3 detect_3d.py
#   (camera lancee : ros2 launch orbbec_camera dabai.launch.py)
# ============================================================

import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

# --- Parametres camera (releves via camera_info) ---
FX = 490.7
FY = 490.7
CX = 324.4
CY = 216.4

# Dimensions des deux flux (couleur et profondeur different en hauteur !)
COULEUR_W, COULEUR_H = 640, 480
DEPTH_W, DEPTH_H = 640, 400

# YOLO
MODELE_YOLO = "yolov8n.pt"
SEUIL_YOLO = 0.40


class Detect3D(Node):
    def __init__(self):
        super().__init__('detect_3d')
        self.bridge = CvBridge()
        self.img_couleur = None
        self.img_depth = None

        self.create_subscription(Image, '/camera/color/image_raw',
                                 self._cb_couleur, 10)
        self.create_subscription(Image, '/camera/depth/image_raw',
                                 self._cb_depth, 10)

        self.get_logger().info("Chargement YOLO...")
        self.yolo = YOLO(MODELE_YOLO)
        self.get_logger().info("YOLO pret.")

    def _cb_couleur(self, msg):
        self.img_couleur = msg

    def _cb_depth(self, msg):
        self.img_depth = msg

    def _profondeur_a(self, depth_array, u_couleur, v_couleur):
        """Renvoie la profondeur (en metres) au pixel couleur (u,v).

        L'image de profondeur (640x400) n'a pas la meme hauteur que l'image
        couleur (640x480) : on met le pixel a l'echelle. On prend la mediane
        d'une petite fenetre autour du point pour eviter les trous (valeurs 0).
        """
        # Mise a l'echelle couleur -> profondeur
        u_d = int(u_couleur * DEPTH_W / COULEUR_W)
        v_d = int(v_couleur * DEPTH_H / COULEUR_H)

        # Fenetre 5x5 autour du pixel, en restant dans les bornes
        r = 2
        u0, u1 = max(0, u_d - r), min(DEPTH_W, u_d + r + 1)
        v0, v1 = max(0, v_d - r), min(DEPTH_H, v_d + r + 1)
        fenetre = depth_array[v0:v1, u0:u1].astype(np.float32)

        # Ignorer les 0 (pas de mesure) et convertir mm -> m
        valeurs = fenetre[fenetre > 0]
        if valeurs.size == 0:
            return None
        return float(np.median(valeurs)) / 1000.0

    def analyser(self):
        """Fait une detection + calcul 3D sur la derniere image recue."""
        if self.img_couleur is None or self.img_depth is None:
            self.get_logger().warn("Images pas encore recues.")
            return

        cv_couleur = self.bridge.imgmsg_to_cv2(self.img_couleur, 'bgr8')
        # Profondeur 16UC1 -> tableau numpy uint16 (en millimetres)
        depth = self.bridge.imgmsg_to_cv2(self.img_depth, '16UC1')

        resultats = self.yolo(cv_couleur, conf=SEUIL_YOLO, verbose=False)
        r = resultats[0]
        noms = r.names

        if r.boxes is None or len(r.boxes) == 0:
            print("  Aucun objet detecte.")
            return

        print(f"\n  {len(r.boxes)} objet(s) detecte(s) :")
        for box in r.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            nom = noms[cls_id]

            # Centre de la boite (en pixels image couleur)
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)

            # Profondeur -> position 3D
            z = self._profondeur_a(depth, u, v)
            if z is None:
                print(f"    - {nom} ({conf*100:.0f}%) : profondeur indispo "
                      f"(trou dans la depth a ce pixel).")
                continue

            # Projection pixel + profondeur -> coordonnees 3D (repere camera)
            # X : droite(+), Y : bas(+), Z : profondeur(+)
            X = (u - CX) * z / FX
            Y = (v - CY) * z / FY
            Z = z
            print(f"    - {nom} ({conf*100:.0f}%) : "
                  f"position (X={X:+.2f}m, Y={Y:+.2f}m, Z={Z:.2f}m)")
            print(f"        -> {Z:.2f}m devant, "
                  f"{abs(X):.2f}m {'a droite' if X>0 else 'a gauche'}, "
                  f"{abs(Y):.2f}m {'plus bas' if Y>0 else 'plus haut'} que la camera")


def main():
    rclpy.init()
    node = Detect3D()

    print("Attente des images camera...")
    t0 = time.time()
    while (time.time() - t0) < 10.0:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.img_couleur is not None and node.img_depth is not None:
            break

    print("\n=== ANALYSE ===")
    node.analyser()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
