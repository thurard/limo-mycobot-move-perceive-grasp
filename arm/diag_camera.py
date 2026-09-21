#!/usr/bin/env python3
# ============================================================
# diag_camera.py — Diagnostic de la camera pour la detection 3D.
#
# Verifie les dimensions et l'alignement entre l'image couleur
# et le nuage de points, pour savoir comment construire la
# detection 3D (YOLO + profondeur).
#
# Usage : python3 diag_camera.py
#   (la camera doit etre lancee : ros2 launch orbbec_camera dabai.launch.py)
# ============================================================

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo


class DiagCamera(Node):
    def __init__(self):
        super().__init__('diag_camera')
        self.img = None
        self.cloud = None
        self.info = None

        self.create_subscription(Image, '/camera/color/image_raw',
                                 self._cb_img, 10)
        self.create_subscription(PointCloud2, '/camera/depth/points',
                                 self._cb_cloud, 10)
        self.create_subscription(CameraInfo, '/camera/color/camera_info',
                                 self._cb_info, 10)

    def _cb_img(self, msg):
        self.img = msg

    def _cb_cloud(self, msg):
        self.cloud = msg

    def _cb_info(self, msg):
        self.info = msg


def main():
    rclpy.init()
    node = DiagCamera()

    print("Attente des messages camera (max 10s)...")
    import time
    t0 = time.time()
    while (time.time() - t0) < 10.0:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.img is not None and node.cloud is not None:
            break

    print("\n=== IMAGE COULEUR ===")
    if node.img is not None:
        print(f"  width  : {node.img.width}")
        print(f"  height : {node.img.height}")
        print(f"  encoding : {node.img.encoding}")
    else:
        print("  [!] Aucune image couleur recue.")

    print("\n=== NUAGE DE POINTS ===")
    if node.cloud is not None:
        print(f"  width  : {node.cloud.width}")
        print(f"  height : {node.cloud.height}")
        organise = node.cloud.height > 1
        print(f"  organise (height>1) : {organise}")
        print(f"  point_step : {node.cloud.point_step} octets/point")
        print(f"  champs : {[f.name for f in node.cloud.fields]}")
        if organise:
            print("  -> ORGANISE : le pixel (u,v) de YOLO correspond au")
            print("     point (u,v) du nuage. Detection 3D directe possible.")
        else:
            print("  -> NON organise : il faudra projeter via camera_info.")
    else:
        print("  [!] Aucun nuage de points recu.")

    print("\n=== CAMERA_INFO (couleur) ===")
    if node.info is not None:
        print(f"  width x height : {node.info.width} x {node.info.height}")
        # Matrice intrinseque K : [fx 0 cx ; 0 fy cy ; 0 0 1]
        k = node.info.k
        print(f"  fx={k[0]:.1f}  fy={k[4]:.1f}  cx={k[2]:.1f}  cy={k[5]:.1f}")
    else:
        print("  [!] camera_info non recu (pas bloquant si nuage organise).")

    print("\n=== CONCLUSION ===")
    if node.cloud is not None and node.cloud.height > 1:
        print("  Nuage organise -> on utilise /camera/depth/points directement.")
    elif node.info is not None:
        print("  Nuage non organise mais camera_info dispo -> projection possible.")
    else:
        print("  Infos insuffisantes, relance avec la camera bien demarree.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
