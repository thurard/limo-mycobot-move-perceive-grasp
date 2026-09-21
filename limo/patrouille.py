import sys
import time
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

# --- Ajouts pour la fonction photo ---
import tf2_ros
from cv_bridge import CvBridge
import cv2

# --- Ajout pour la détection d'objets YOLO ---
from ultralytics import YOLO

# Modèle YOLO (nano, léger, adapté au Jetson) et seuil de confiance.
# Le modèle est chargé UNE fois au démarrage (voir __init__), pour être
# réutilisable en temps réel par la suite.
# Distance au point a laquelle on memorise la "vue d'ensemble" (en m).
# Le robot photographie de loin (objet entier visible, identifiable par YOLO)
# au lieu d'etre colle a l'obstacle. Photo utilisee seulement si le point echoue.
DIST_PHOTO = 1.5

MODELE_YOLO = "yolov8n.pt"
SEUIL_YOLO = 0.40

# --- Parametres camera pour la position 3D (releves via camera_info) ---
FX_CAM = 490.7
FY_CAM = 490.7
CX_CAM = 324.4
CY_CAM = 216.4
# L'image couleur (640x480) et la profondeur (640x400) different en hauteur :
# il faut mettre les pixels a l'echelle entre les deux.
COULEUR_W, COULEUR_H = 640, 480
DEPTH_W, DEPTH_H = 640, 400


# Dossier où sont sauvegardées les photos des points bloqués
DOSSIER_PHOTOS = os.path.expanduser("~/photos_patrouille")

# --- Timeout de navigation DYNAMIQUE ---
# Le délai accordé à Nav2 pour atteindre un point dépend de la distance à
# parcourir : court pour un point proche (le robot renonce vite plutôt que de
# s'enfoncer dans un recoin), long pour un point lointain (il a le temps d'y
# aller). Formule : timeout = (distance / VITESSE_EST) * MARGE + MINI.
VITESSE_EST = 0.3    # vitesse moyenne effective estimée du robot (m/s)
MARGE_TIMEOUT = 2.0  # facteur de marge (détours, rotations, ralentissements)
TIMEOUT_MINI = 10.0  # plancher : temps min même pour un point très proche (s)
TIMEOUT_MAXI = 90.0  # plafond de sécurité (s), pour ne jamais attendre trop

# Pause laissée à Nav2 pour se "nettoyer" après une annulation de goal, AVANT
# d'envoyer le goal suivant. Sans cette pause, le bt_navigator rejette le goal
# suivant avec "send_goal failed" (status=6) car ses serveurs d'action ne sont
# pas encore prêts à ré-accepter — ce qui faisait échouer en chaîne tous les
# points après un premier abandon (effet "légume").
PAUSE_RECUP_NAV2 = 4.0


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Abonnement à l'image couleur (on garde la dernière reçue) ---
        self.derniere_image = None
        self.derniere_depth = None
        self.bridge = CvBridge()
        self.create_subscription(
            Image, '/camera/color/image_raw', self._cb_image, 10)
        # Abonnement à la profondeur (pour la position 3D des objets)
        self.create_subscription(
            Image, '/camera/depth/image_raw', self._cb_depth, 10)

        # --- TF listener pour lire la position du robot (map -> base_link) ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Sous-dossier daté pour CETTE patrouille : chaque session a son propre
        # dossier (photos + rapport), ce qui permet de ne rapatrier que la
        # dernière et garde le robot bien rangé.
        os.makedirs(DOSSIER_PHOTOS, exist_ok=True)
        session = time.strftime("%Y%m%d_%H%M%S")
        self.dossier_session = os.path.join(DOSSIER_PHOTOS, session)
        os.makedirs(self.dossier_session, exist_ok=True)

        # Buffers de la "vue d'ensemble" memorisee pendant l'approche
        self.buffer_photo = None
        self.buffer_depth = None

        # Liste des photos prises pendant la patrouille (pour le rapport)
        self.photos_prises = []

        # Chargement du modèle YOLO UNE SEULE FOIS au démarrage. Il reste en
        # mémoire pour toute la patrouille (rapide à chaque photo, et prêt pour
        # un usage temps réel futur). Le modèle se télécharge automatiquement au
        # tout premier lancement (~6 Mo).
        self.get_logger().info("Chargement du modele YOLO...")
        try:
            self.yolo = YOLO(MODELE_YOLO)
            self.get_logger().info("Modele YOLO charge.")
        except Exception as e:
            self.get_logger().error(f"Echec chargement YOLO : {e}")
            self.yolo = None

    def _cb_image(self, msg):
        self.derniere_image = msg

    def _cb_depth(self, msg):
        self.derniere_depth = msg

    def _position_3d(self, depth_array, x1, y1, x2, y2):
        """Position 3D (X,Y,Z en m, repere camera) de l'objet delimite par la
        boite (x1,y1,x2,y2) en pixels couleur. La profondeur Z est la MEDIANE
        sur toute la boite (en ignorant les trous), bien plus robuste que la
        seule valeur au centre (qui peut tomber sur un trou ou l'objet derriere).
        Renvoie None si aucune profondeur valide dans la boite.
        """
        if depth_array is None:
            return None
        import numpy as np
        # Centre de la boite (pour la position laterale X,Y)
        u = (x1 + x2) // 2
        v = (y1 + y2) // 2
        # Mise a l'echelle couleur (640x480) -> profondeur (640x400)
        sx = DEPTH_W / COULEUR_W
        sy = DEPTH_H / COULEUR_H
        # Boite dans le repere profondeur, on la retrecit un peu (60% centrale)
        # pour eviter les bords de l'objet qui captent le fond derriere.
        bw = (x2 - x1) * 0.3
        bh = (y2 - y1) * 0.3
        dx1 = int(max(0, (u - bw) * sx))
        dx2 = int(min(DEPTH_W, (u + bw) * sx))
        dy1 = int(max(0, (v - bh) * sy))
        dy2 = int(min(DEPTH_H, (v + bh) * sy))
        if dx2 <= dx1 or dy2 <= dy1:
            return None
        zone = depth_array[dy1:dy2, dx1:dx2].astype(np.float32)
        valeurs = zone[zone > 0]
        if valeurs.size == 0:
            return None
        z = float(np.median(valeurs)) / 1000.0  # mm -> m
        X = (u - CX_CAM) * z / FX_CAM
        Y = (v - CY_CAM) * z / FY_CAM
        return (X, Y, z)

    def arret_urgence(self):
        # Publie le stop plusieurs fois sur ~0.5 s pour couvrir un éventuel
        # dernier cmd_vel du controller_server (qui tourne à 10 Hz).
        stop = Twist()
        for _ in range(10):
            self.cmd_vel_pub.publish(stop)
            time.sleep(0.05)
        self.get_logger().warn("ARRET D'URGENCE envoye.")

    def annuler_goal(self, goal_handle):
        """Annule proprement le goal en cours et ATTEND la confirmation."""
        self.get_logger().warn("Annulation du goal en cours...")
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)

        if not cancel_future.done():
            self.get_logger().error("Requete d'annulation sans reponse (timeout 5s).")
            return False

        cancel_response = cancel_future.result()
        if cancel_response is not None and len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Annulation acceptee par Nav2.")
            return True

        self.get_logger().warn("Nav2 n'a annule aucun goal (deja termine ?).")
        return False

    # ------------------------------------------------------------------
    # Orientation vers un point bloque + capture photo
    # ------------------------------------------------------------------
    def _lire_pose_robot(self, silencieux=False):
        """Renvoie (x, y, yaw) du robot dans 'map', ou None si TF indispo.

        IMPORTANT : lookup_transform est appele avec timeout=0 (NON BLOQUANT).
        Un timeout non nul bloque le thread courant... qui est justement celui
        qui doit faire spin() pour que le TransformListener recoive les TF !
        -> deadlock : le lookup attend des TF qui n'arriveront jamais, d'ou les
        erreurs "TF indisponible" / "extrapolation into the past" alors que le
        TF existe bel et bien (verifie via tf2_echo).
        Ici on lit simplement la derniere TF deja recue par les spin_once du
        reste du code.

        silencieux=True : n'affiche pas d'erreur (appels en boucle).
        """
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except Exception as e:
            if not silencieux:
                self.get_logger().error(f"TF map->base_link indisponible : {e}")
            return None

        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (x, y, yaw)

    def _tourner_de(self, delta_angle, vitesse=0.5):
        """Tourne sur place d'un angle delta_angle (rad), en boucle ouverte."""
        delta_angle = math.atan2(math.sin(delta_angle), math.cos(delta_angle))
        if abs(delta_angle) < 0.05:
            return

        sens = 1.0 if delta_angle > 0 else -1.0
        duree = abs(delta_angle) / vitesse

        tw = Twist()
        tw.angular.z = sens * vitesse

        t0 = time.time()
        while (time.time() - t0) < duree:
            self.cmd_vel_pub.publish(tw)
            time.sleep(0.05)

        self.cmd_vel_pub.publish(Twist())
        time.sleep(0.2)

    def pivoter_pour_degager(self, angle=math.pi, vitesse=0.4):
        """Rotation lente sur place après la photo, pour se dégager d'un
        obstacle proche SANS recul aveugle. La rotation sur place ne déplace
        pas le centre du robot (zéro risque de heurt), et pendant qu'il tourne
        le lidar/caméra rafraîchissent le costmap. Nav2 reprend ensuite la main
        avec un costmap à jour et planifie la sortie vers une zone dégagée.

        angle : amplitude de rotation (par défaut un demi-tour, pi rad).
        vitesse : lente (0.4 rad/s) pour laisser le costmap suivre.
        """
        duree = abs(angle) / vitesse
        tw = Twist()
        tw.angular.z = vitesse

        self.get_logger().info(
            "Rotation lente pour se degager (costmap se rafraichit)...")
        t0 = time.time()
        while (time.time() - t0) < duree:
            self.cmd_vel_pub.publish(tw)
            time.sleep(0.05)

        self.cmd_vel_pub.publish(Twist())
        time.sleep(0.3)

    def photographier_point(self, x_cible, y_cible, num_point):
        """Sauvegarde une photo du point inatteignable.

        Priorite a la VUE D'ENSEMBLE memorisee pendant l'approche (a DIST_PHOTO
        du point) : l'objet y est entier et identifiable, et le robot etait deja
        oriente vers le point. Pas de rotation -> pas d'erreur d'angle.

        Repli (si le robot n'est jamais arrive assez pres pour memoriser) :
        ancien comportement = rotation de visee + capture sur place.
        """
        self.get_logger().warn(
            f"Point {num_point} inatteignable -> photo.")

        if self.buffer_photo is not None:
            # --- Cas nominal : on utilise la vue d'ensemble memorisee ---
            self.get_logger().info(
                f"Utilisation de la vue d'ensemble memorisee "
                f"(prise a ~{DIST_PHOTO}m du point).")
            img_msg = self.buffer_photo
            depth_msg = self.buffer_depth
        else:
            # --- Repli : rotation de visee + capture sur place ---
            self.get_logger().warn(
                "Pas de vue d'ensemble memorisee -> rotation + photo sur place.")
            time.sleep(1.0)  # laisser l'arret d'urgence se terminer

            pose = self._lire_pose_robot()
            if pose is None:
                self.get_logger().error("Photo impossible : position inconnue.")
                return False
            x_r, y_r, yaw_r = pose
            angle_vers_cible = math.atan2(y_cible - y_r, x_cible - x_r)
            delta = angle_vers_cible - yaw_r
            delta = math.atan2(math.sin(delta), math.cos(delta))
            self.get_logger().info(
                f"Rotation pour viser le point {num_point} "
                f"(delta={math.degrees(delta):.0f} deg).")
            self._tourner_de(delta)

            self.cmd_vel_pub.publish(Twist())
            time.sleep(1.2)

            self.derniere_image = None
            self.derniere_depth = None
            t0 = time.time()
            while (time.time() - t0) < 3.0:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.derniere_image is not None and self.derniere_depth is not None:
                    break
            img_msg = self.derniere_image
            depth_msg = self.derniere_depth

        if img_msg is None:
            self.get_logger().error("Photo impossible : aucune image disponible.")
            return False

        try:
            cv_img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Echec conversion image : {e}")
            return False

        horodatage = time.strftime("%Y%m%d_%H%M%S")
        date_lisible = time.strftime("%d/%m/%Y %H:%M:%S")
        nom_court = f"point_{num_point}_bloque_{horodatage}.jpg"
        nom_fichier = os.path.join(self.dossier_session, nom_court)

        # --- Détection YOLO sur l'image capturée ---
        # On fait tourner le modèle (chargé au démarrage) sur l'image, on
        # sauvegarde la version ANNOTEE (boîtes + noms) et on récupère la liste
        # des objets détectés pour le rapport.
        objets_detectes = []
        if self.yolo is not None:
            try:
                import numpy as np
                # Convertir la profondeur associee a CETTE image (buffer ou
                # capture de repli) en tableau. Elle est synchronisee avec
                # l'image couleur utilisee -> distances coherentes.
                depth_array = None
                if depth_msg is not None:
                    depth_array = self.bridge.imgmsg_to_cv2(
                        depth_msg, '16UC1')

                resultats = self.yolo(cv_img, conf=SEUIL_YOLO, verbose=False)
                r = resultats[0]
                noms = r.names

                # On dessine nous-mêmes les annotations (pour ajouter la
                # distance), en partant d'une copie de l'image couleur.
                img_a_sauver = cv_img.copy()

                if r.boxes is not None:
                    for box in r.boxes:
                        cls_id = int(box.cls[0])
                        conf = float(box.conf[0])
                        nom = noms[cls_id]
                        x1, y1, x2, y2 = [int(c) for c in box.xyxy[0].tolist()]
                        u = (x1 + x2) // 2
                        v = (y1 + y2) // 2

                        # Position 3D via la profondeur (mediane sur la boite)
                        pos = self._position_3d(depth_array, x1, y1, x2, y2)
                        if pos is not None:
                            X, Y, Z = pos
                            label = f"{nom} {conf*100:.0f}% {Z:.2f}m"
                            objets_detectes.append(
                                f"{nom} ({conf*100:.0f}%) "
                                f"[X={X:+.2f} Y={Y:+.2f} Z={Z:.2f}]")
                        else:
                            label = f"{nom} {conf*100:.0f}% (dist?)"
                            objets_detectes.append(
                                f"{nom} ({conf*100:.0f}%) [profondeur indispo]")

                        # Dessin : rectangle + label
                        cv2.rectangle(img_a_sauver, (x1, y1), (x2, y2),
                                      (0, 200, 0), 2)
                        cv2.putText(img_a_sauver, label, (x1, max(0, y1 - 8)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                    (0, 200, 0), 2)

                if objets_detectes:
                    self.get_logger().info(
                        f"YOLO 3D : {', '.join(objets_detectes)}")
                else:
                    self.get_logger().info("YOLO : aucun objet detecte.")
            except Exception as e:
                self.get_logger().error(f"Echec detection YOLO 3D : {e}")
                img_a_sauver = cv_img  # en cas d'échec, image originale
        else:
            img_a_sauver = cv_img  # YOLO non chargé -> image brute

        cv2.imwrite(nom_fichier, img_a_sauver)
        self.get_logger().info(f">>> Photo (annotee) enregistree : {nom_fichier}")

        # Enregistrer les infos pour le rapport de fin de patrouille
        self.photos_prises.append({
            "point": num_point,
            "date": date_lisible,
            "fichier": nom_court,
            "objets": ", ".join(objets_detectes) if objets_detectes else "aucun",
        })

        # Rotation lente pour se dégager de la plaque avant de repartir : le
        # robot, arrêté tout près de l'obstacle (dans son inflation), ne trouve
        # aucune trajectoire de sortie. En pivotant sur place (sans recul
        # aveugle), le costmap se rafraîchit et Nav2 retrouve une issue.
        self.pivoter_pour_degager()
        return True

    def _calculer_timeout(self, x_cible, y_cible):
        """Timeout de navigation proportionnel à la distance robot->cible.

        timeout = (distance / VITESSE_EST) * MARGE_TIMEOUT, borné entre
        TIMEOUT_MINI et TIMEOUT_MAXI. Si la position du robot est inconnue
        (TF indispo), on retombe sur une valeur médiane sûre.
        """
        pose = self._lire_pose_robot()
        if pose is None:
            self.get_logger().warn(
                "Position inconnue pour le calcul du timeout -> valeur par defaut.")
            return 30.0
        x_r, y_r, _ = pose
        distance = math.sqrt((x_cible - x_r) ** 2 + (y_cible - y_r) ** 2)
        timeout = (distance / VITESSE_EST) * MARGE_TIMEOUT
        # Bornage entre plancher et plafond
        timeout = max(TIMEOUT_MINI, min(timeout, TIMEOUT_MAXI))
        return timeout

    def generer_rapport(self):
        """Génère un rapport CSV + HTML des photos prises pendant la patrouille.
        Le CSV contient les données (point, date, fichier), le HTML affiche les
        photos avec leurs infos, ouvrable d'un double-clic dans un navigateur.
        """
        if not self.photos_prises:
            self.get_logger().info("Aucune photo prise : pas de rapport a generer.")
            return

        horodatage = time.strftime("%Y%m%d_%H%M%S")

        # --- CSV ---
        chemin_csv = os.path.join(self.dossier_session, f"rapport_{horodatage}.csv")
        try:
            with open(chemin_csv, "w", encoding="utf-8") as f:
                f.write("point,date,objets,fichier\n")
                for p in self.photos_prises:
                    # Guillemets autour des objets (peuvent contenir des virgules)
                    f.write(f"{p['point']},{p['date']},\"{p.get('objets','')}\",{p['fichier']}\n")
            self.get_logger().info(f">>> Rapport CSV : {chemin_csv}")
        except Exception as e:
            self.get_logger().error(f"Echec ecriture CSV : {e}")

        # --- HTML ---
        chemin_html = os.path.join(self.dossier_session, f"rapport_{horodatage}.html")
        try:
            date_rapport = time.strftime("%d/%m/%Y a %H:%M:%S")
            cartes = ""
            for p in self.photos_prises:
                cartes += f"""
    <div class="carte">
      <img src="{p['fichier']}" alt="Point {p['point']}">
      <div class="infos">
        <span class="point">Point {p['point']}</span>
        <span class="date">{p['date']}</span>
      </div>
      <div class="objets">Objets : {p.get('objets','aucun')}</div>
    </div>"""

            html = f"""<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="utf-8">
<title>Rapport de patrouille - {date_rapport}</title>
<style>
  body {{ font-family: sans-serif; background: #f0f0f0; margin: 20px; }}
  h1 {{ color: #004887; }}
  .resume {{ color: #555; margin-bottom: 20px; }}
  .grille {{ display: flex; flex-wrap: wrap; gap: 16px; }}
  .carte {{ background: #fff; border-radius: 8px; box-shadow: 0 2px 6px rgba(0,0,0,0.15);
           overflow: hidden; width: 320px; }}
  .carte img {{ width: 100%; height: 240px; object-fit: cover; display: block; }}
  .infos {{ padding: 10px; display: flex; justify-content: space-between; align-items: center; }}
  .point {{ font-weight: bold; color: #004887; }}
  .date {{ color: #777; font-size: 0.9em; }}
  .objets {{ padding: 0 10px 10px; color: #333; font-size: 0.85em; }}
</style>
</head>
<body>
  <h1>Rapport de patrouille</h1>
  <div class="resume">
    Genere le {date_rapport} &mdash; {len(self.photos_prises)} point(s) inatteignable(s) photographie(s).
  </div>
  <div class="grille">{cartes}
  </div>
</body>
</html>"""
            with open(chemin_html, "w", encoding="utf-8") as f:
                f.write(html)
            self.get_logger().info(f">>> Rapport HTML : {chemin_html}")
        except Exception as e:
            self.get_logger().error(f"Echec ecriture HTML : {e}")

    def go_to_pose(self, x, y, z_orient, w_orient):
        self.get_logger().info("Attente de Nav2...")

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("Nav2 injoignable apres 15s — arret d'urgence.")
            self.arret_urgence()
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        norm = math.sqrt(float(z_orient) ** 2 + float(w_orient) ** 2)
        if norm == 0.0:
            z_n, w_n = 0.0, 1.0
        else:
            z_n, w_n = float(z_orient) / norm, float(w_orient) / norm

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = z_n
        goal_msg.pose.pose.orientation.w = w_n

        self.get_logger().info(f"Envoi vers coordonnee : X={x}, Y={y}")

        # Timeout DYNAMIQUE : calculé selon la distance robot->cible. Un point
        # proche a un petit timeout (renonce vite, ne s'enfonce pas dans un
        # recoin) ; un point lointain a un grand timeout (le temps d'y aller).
        timeout_nav = self._calculer_timeout(x, y)
        self.get_logger().info(f"Timeout navigation calcule : {timeout_nav:.0f}s")

        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)

        if not send_goal_future.done():
            self.get_logger().error("Timeout envoi goal — Nav2 ne repond plus.")
            self.arret_urgence()
            return False

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Cible refusee par Nav2 (obstacle ou Nav2 pas pret).")
            return False

        self.get_logger().info("Cible acceptee ! Le robot est en route...")

        # --- Surveillance pendant la navigation ---
        # On ne peut pas utiliser spin_until_future_complete (bloquant) car on
        # veut surveiller la distance au point pendant le trajet. On boucle donc
        # a la main : quand le robot passe sous DIST_PHOTO du point, on MEMORISE
        # l'image courante (buffer). Elle ne sera sauvegardee que si le point
        # finit par echouer -> photo prise a bonne distance (vue d'ensemble),
        # et deja orientee vers le point (le robot y fonce) : pas de rotation
        # de visee necessaire, donc pas d'erreur d'angle.
        self.buffer_photo = None
        self.buffer_depth = None
        get_result_future = goal_handle.get_result_async()

        t_debut = time.time()
        while not get_result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

            # Timeout de navigation atteint ?
            if (time.time() - t_debut) > timeout_nav:
                break

            # Capture du buffer quand on approche du point (une seule fois)
            if self.buffer_photo is None:
                # silencieux : appele en boucle, inutile de spammer les erreurs
                pose = self._lire_pose_robot(silencieux=True)
                if pose is not None:
                    x_r, y_r, _ = pose
                    dist = math.sqrt((x - x_r) ** 2 + (y - y_r) ** 2)
                    if dist <= DIST_PHOTO and self.derniere_image is not None:
                        self.buffer_photo = self.derniere_image
                        self.buffer_depth = self.derniere_depth
                        self.get_logger().info(
                            f"Vue d'ensemble memorisee a {dist:.2f}m du point "
                            f"(utilisee seulement si le point echoue).")

        if not get_result_future.done():
            # Point non atteint dans le delai imparti : on considere le point
            # inatteignable. On ANNULE le goal pour que le robot cesse de
            # s'acharner sur l'obstacle, puis on renvoie False -> la photo sera
            # declenchee par la boucle principale.
            self.get_logger().warn(
                f"Point non atteint en {timeout_nav:.0f}s "
                f"-> considere inatteignable, annulation.")
            self.annuler_goal(goal_handle)
            self.arret_urgence()
            return False

        result = get_result_future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(">>> ARRIVE A DESTINATION ! <<<")
            return True
        else:
            self.get_logger().error(f"Echec navigation (status={result.status}).")
            return False


def main(args=None):
    rclpy.init(args=args)

    nom_carte = sys.argv[1] if len(sys.argv) > 1 else "carte_couloir3"

    catalogue_cibles = {
        "carte_couloir3": [
            (2.675, -4.035, -0.465, 0.885),
            (4.352, -3.051, 0.804, 0.594),
            (1.937, 1.838, 0.826, 0.564),
            (0.0, 0.0, 0.0, 1.0),
        ],
        "map_salle": [
            (2.244, 0.103, 0.016, 1.000),
            (2.263, -1.737, -0.713, 0.701),
            (1.586, 2.168, -0.682, 0.731),
            (0.327, 0.436, 0.999, 0.045),
            (-1.384, 1.385, -0.601, 0.799),
            (-0.132, -2.170, 0.866, 0.499),
            (0.0, 0.0, 0.0, 1.0),
        ],
        "map_train": [
            (-0.040, 0.000, -0.001, 1.000),
            (0.756, -0.391, -0.654, 0.757),
            (1.363, 0.274, -0.713, 0.701),
            (-0.124, -0.771, -0.422, 0.906),
            (0.0, 0.0, 0.0, 1.0),
        ],
        "map_carre": [
            (0.680, 0.149, -0.475, 0.880),
            (0.231, -0.311, 1.000, -0.018),
            (-1.455, 0.257, 0.633, 0.774),
            (-1.441, 1.172, -0.044, 0.999),
            (0.079, 0.591, 0.979, 0.203),
            (0.0, 0.0, 0.0, 1.0),
        ],
        "map_bras": [
            (2.044, 0.200, 0.056, 0.998),
            (2.384, -1.274, -0.717, 0.697),
            (3.270, 1.392, 0.992, -0.126),
            (-0.584, -1.398, 0.960, -0.282),
            (-1.483, 0.594, -0.248, 0.969),
            (-0.101, -0.058, 0.029, 1.000),
        ],
        # Carte pour la video de demonstration de la patrouille.
        "map_ulte": [
            (1.824, -2.052, -0.704, 0.711),   # Point 1
            (2.567, 0.664, 0.107, 0.994),     # Point 2
            (-1.623, -2.422, 0.532, 0.847),   # Point 3
            (-1.761, -0.139, -0.501, 0.866),  # Point 4
            (0.0, 0.0, 0.0, 1.0),             # retour a l'origine
        ],
    }

    if nom_carte not in catalogue_cibles:
        print(f"[ERREUR] Aucune cible definie pour la carte '{nom_carte}' !")
        rclpy.shutdown()
        return

    patrol_node = PatrolNode()
    cibles = catalogue_cibles[nom_carte]

    print(f"\n>>> Chargement du parcours pour : {nom_carte} <<<")

    # ATTENTE ACTIVE DU TF : on ne demarre pas la patrouille tant que la
    # transformation map->base_link n'est pas disponible. Sinon le script
    # demande la position du robot avant qu'AMCL n'ait fini de s'initialiser
    # -> "TF map->base_link indisponible" en rafale, timeout par defaut,
    # et pas de vue d'ensemble memorisee.
    print("[INFO] Attente de la localisation (TF map->base_link)...")
    t0 = time.time()
    tf_pret = False
    while (time.time() - t0) < 60.0:
        rclpy.spin_once(patrol_node, timeout_sec=0.2)
        if patrol_node._lire_pose_robot(silencieux=True) is not None:
            tf_pret = True
            break

    if tf_pret:
        delai = time.time() - t0
        print(f"[INFO] Localisation OK apres {delai:.1f}s. "
              f"Demarrage de la patrouille !")
    else:
        print("[ATTENTION] TF map->base_link toujours indisponible apres 60s.")
        print("            AMCL ne se localise pas — verifie l'initialpose.")
        print("            La patrouille demarre quand meme (mode degrade).")

    loop_num = 1
    echecs_consecutifs = 0
    SEUIL_ALERTE = 3

    try:
        while rclpy.ok():
            print(f"\n=====================================")
            print(f"--- CYCLE DE PATROUILLE N° {loop_num} ---")
            print(f"=====================================")

            for i, cible in enumerate(cibles):
                is_last = (i == len(cibles) - 1)

                if patrol_node.go_to_pose(*cible):
                    echecs_consecutifs = 0
                    if is_last:
                        print("Retour a l'origine valide. Fin du cycle.")
                        time.sleep(3)
                    else:
                        print(f"[BRAS myCobot] Action sur le Point {i+1}...")
                        time.sleep(2)
                else:
                    echecs_consecutifs += 1
                    nom_etape = "Retour base" if is_last else f"Cible {i+1}"
                    print(f"[SECURITE] {nom_etape} echouee "
                          f"(echec {echecs_consecutifs} d'affilee).")

                    # Point inatteignable -> photo (sauf retour base).
                    # Le robot ne force pas le passage : il documente a distance.
                    if not is_last:
                        x_c, y_c = cible[0], cible[1]
                        patrol_node.photographier_point(x_c, y_c, i + 1)

                    # PAUSE DE RECUPERATION : laisser Nav2 se nettoyer apres
                    # l'annulation avant d'envoyer le goal suivant. Evite le
                    # "send_goal failed" (status=6) qui faisait echouer en
                    # chaine tous les points suivants.
                    print(f"[INFO] Recuperation Nav2 ({PAUSE_RECUP_NAV2:.0f}s)...")
                    time.sleep(PAUSE_RECUP_NAV2)

                    if echecs_consecutifs >= SEUIL_ALERTE:
                        print("\n" + "!" * 45)
                        print(f"[ALERTE] {echecs_consecutifs} echecs consecutifs !")
                        print("Robot probablement bloque ou localisation perdue.")
                        print("!" * 45 + "\n")

                    time.sleep(3)

            loop_num += 1

    except KeyboardInterrupt:
        print("\n[INTERRUPTION] Arret demande — securisation du robot...")
        patrol_node.arret_urgence()

    finally:
        # Générer le rapport des photos avant de fermer (même après Ctrl-C)
        try:
            patrol_node.generer_rapport()
        except Exception as e:
            print(f"[WARN] Rapport non genere : {e}")
        patrol_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
