#!/usr/bin/env python3
# ============================================================
# calib_axes.py - Determine la correspondance axes bras <-> image
#
# A executer SUR LE PI DU BRAS :
#     python3 ~/calib_axes.py
#
# Deplace le bras d'une petite distance connue sur X (avant) puis
# sur Y (lateral), et mesure comment l'objet se deplace dans
# l'image (dx, dy). En deduit quel axe bras agit sur quel axe
# image, et avec quel signe/gain. A reporter dans guidage_visuel.py.
#
# La balle orange doit etre posee, stable, et VISIBLE par la camera
# pendant toute la manoeuvre (le bras ne bouge que de +/- 15 mm).
# ============================================================

import time
import cv2
import numpy as np
from pymycobot import MyCobot

PORT = '/dev/ttyAMA0'
BAUD = 1000000
CAMERA = 0
LARGEUR, HAUTEUR = 640, 480

# Seuils balle orange
BAS = np.array([2, 90, 90])
HAUT = np.array([16, 255, 255])
AIRE_MINI = 800

DEPLACEMENT_MM = 15.0     # petit pas de test
VITESSE = 20


def connecter():
    mc = MyCobot(PORT, BAUD)
    time.sleep(2)
    return mc


def lire_coords(mc):
    """get_coords fiable : reessaie si liste vide (probleme de timing)."""
    for _ in range(5):
        c = mc.get_coords()
        if c:
            return c
        time.sleep(0.4)
    return None


def detecter(cap):
    """Renvoie (dx, dy) de la balle, ou None. Vide le tampon avant lecture."""
    for _ in range(5):
        cap.grab()
    ok, img = cap.read()
    if not ok:
        return None
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    masque = cv2.inRange(hsv, BAS, HAUT)
    noyau = np.ones((5, 5), np.uint8)
    masque = cv2.morphologyEx(masque, cv2.MORPH_OPEN, noyau)
    masque = cv2.morphologyEx(masque, cv2.MORPH_CLOSE, noyau)
    contours, _ = cv2.findContours(masque, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < AIRE_MINI:
        return None
    x, y, w, h = cv2.boundingRect(c)
    return (x + w // 2 - LARGEUR // 2, y + h // 2 - HAUTEUR // 2)


def mesure_stable(cap, n=3):
    """Moyenne de n detections pour reduire le bruit."""
    vals = []
    for _ in range(n):
        d = detecter(cap)
        if d is not None:
            vals.append(d)
        time.sleep(0.3)
    if not vals:
        return None
    return (sum(v[0] for v in vals) / len(vals),
            sum(v[1] for v in vals) / len(vals))


def bouger(mc, axe, delta):
    """Deplace le bras de delta mm sur l'axe 0=X(avant) ou 1=Y(lateral)."""
    c = lire_coords(mc)
    if c is None:
        print("  [!] get_coords echoue, mouvement annule.")
        return False
    c[axe] += delta
    mc.send_coords(c, VITESSE, 1)
    time.sleep(3)
    return True


def main():
    print("Connexion au bras...")
    mc = connecter()
    cap = cv2.VideoCapture(CAMERA)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, LARGEUR)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HAUTEUR)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    time.sleep(1)

    ref = mesure_stable(cap)
    if ref is None:
        print("Balle non detectee au depart. Repositionne et relance.")
        return
    print(f"Position initiale image : dx={ref[0]:.0f} dy={ref[1]:.0f}")

    # --- Test axe X (avant) ---
    print(f"\n[Test X] avance de {DEPLACEMENT_MM:.0f} mm...")
    if not bouger(mc, 0, DEPLACEMENT_MM):
        return
    apres_x = mesure_stable(cap)
    bouger(mc, 0, -DEPLACEMENT_MM)   # retour
    if apres_x is None:
        print("  Balle perdue apres deplacement X.")
        return
    d_dx_x = apres_x[0] - ref[0]
    d_dy_x = apres_x[1] - ref[1]
    print(f"  -> dx varie de {d_dx_x:+.0f}, dy varie de {d_dy_x:+.0f}")

    # --- Test axe Y (lateral) ---
    print(f"\n[Test Y] deplacement lateral de {DEPLACEMENT_MM:.0f} mm...")
    if not bouger(mc, 1, DEPLACEMENT_MM):
        return
    apres_y = mesure_stable(cap)
    bouger(mc, 1, -DEPLACEMENT_MM)   # retour
    if apres_y is None:
        print("  Balle perdue apres deplacement Y.")
        return
    d_dx_y = apres_y[0] - ref[0]
    d_dy_y = apres_y[1] - ref[1]
    print(f"  -> dx varie de {d_dx_y:+.0f}, dy varie de {d_dy_y:+.0f}")

    # --- Conclusion ---
    print("\n===== RESULTAT =====")
    print(f"Avancer X de +{DEPLACEMENT_MM:.0f}mm : dx {d_dx_x:+.0f}, dy {d_dy_x:+.0f}")
    print(f"Bouger  Y de +{DEPLACEMENT_MM:.0f}mm : dx {d_dx_y:+.0f}, dy {d_dy_y:+.0f}")
    print("\nInterpretation :")
    # Quel axe image bouge le plus pour chaque mouvement bras ?
    if abs(d_dx_x) > abs(d_dy_x):
        print(f"  X (avant) agit surtout sur dx  (gain {d_dx_x/DEPLACEMENT_MM:+.1f} px/mm)")
    else:
        print(f"  X (avant) agit surtout sur dy  (gain {d_dy_x/DEPLACEMENT_MM:+.1f} px/mm)")
    if abs(d_dx_y) > abs(d_dy_y):
        print(f"  Y (lateral) agit surtout sur dx  (gain {d_dx_y/DEPLACEMENT_MM:+.1f} px/mm)")
    else:
        print(f"  Y (lateral) agit surtout sur dy  (gain {d_dy_y/DEPLACEMENT_MM:+.1f} px/mm)")
    print("\nEnvoie ces lignes pour que je fige la conversion.")

    cap.release()


if __name__ == '__main__':
    main()
