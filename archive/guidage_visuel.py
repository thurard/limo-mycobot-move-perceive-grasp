#!/usr/bin/env python3
# ============================================================
# guidage_visuel.py - Saisie guidee par la camera du bras
#
# A executer SUR LE PI DU BRAS :  python3 ~/guidage_visuel.py
#
# METHODE (celle qui a fonctionne : depart AU SOL)
# Le bras part d'une position de depart proche du sol, penchee sur la
# zone de la balle (PAS en hauteur - la version en hauteur cassait tout).
# La camera voit la balle, qui peut varier de quelques cm selon le
# stationnement du robot. L'asservissement corrige cet ecart en bougeant
# le bras jusqu'a amener la balle a la cible image, PUIS ferme la pince.
#
# Point clef : on reste dans la zone basse ou le bras a de la marge ;
# les corrections sont locales, le bras ne tente pas une grande extension
# qui saturerait une articulation.
# ============================================================

import time
import cv2
import numpy as np
from pymycobot import MyCobot

PORT = '/dev/ttyAMA0'
BAUD = 1000000
CAMERA = 0
LARGEUR, HAUTEUR = 640, 480

# --- Detection couleur (calibree) ---
ORANGE_BAS = np.array([0, 40, 97]);    ORANGE_HAUT = np.array([14, 130, 220])
ORANGE_BAS_2 = np.array([170, 40, 97]); ORANGE_HAUT_2 = np.array([179, 130, 220])
AIRE_MINI = 200

# --- POSITION DE DEPART : celle qui marchait, penchee sur la balle au SOL.
# Pince OUVERTE en arrivant (pour ne pas frapper la balle).
# Angles releves avec la pince en position de saisie au sol.
POSE_DEPART = [20.03, -129.37, -21.88, 82.52, -9.93, -34.36]

# --- Cible image : ou la balle apparait quand la pince est dessus,
# vue depuis POSE_DEPART. Releve stable pince-sur-balle : dx~-183 dy~-51.
CIBLE_DX = -183
CIBLE_DY = -51
TOLERANCE = 20

# --- Asservissement (corrige la variation de position de la balle) ---
PAS_SONDE_MM = 15.0
PAS_MAX_MM = 12.0
GAIN_ASSERV = 0.7
ITER_MAX = 12
SEUIL_BOUGE_PX = 5

# --- Saisie : une fois centre, on descend un peu et on ferme ---
DESCENTE_FERMETURE_MM = 15.0   # petite descente pour enserrer sans taper
REMONTEE_MM = 90.0
V_ANGLES = 25
V_DEPLACEMENT = 18
V_PINCE = 50


def connecter():
    mc = MyCobot(PORT, BAUD)
    time.sleep(2); mc.power_on(); time.sleep(2)
    return mc


def lire_coords(mc, essais=8):
    for _ in range(essais):
        c = mc.get_coords()
        if c and len(c) == 6:
            return c
        time.sleep(0.4)
    return None


def detecter(cap):
    for _ in range(5):
        cap.grab()
    ok, img = cap.read()
    if not ok:
        return None
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.bitwise_or(cv2.inRange(hsv, ORANGE_BAS, ORANGE_HAUT),
                       cv2.inRange(hsv, ORANGE_BAS_2, ORANGE_HAUT_2))
    noyau = np.ones((5, 5), np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, noyau)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, noyau)
    c, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not c:
        return None
    g = max(c, key=cv2.contourArea)
    if cv2.contourArea(g) < AIRE_MINI:
        return None
    x, y, w, h = cv2.boundingRect(g)
    return (x + w // 2 - LARGEUR // 2, y + h // 2 - HAUTEUR // 2)


def detecter_stable(cap, n=6):
    xs, ys = [], []
    for _ in range(n + 5):
        d = detecter(cap)
        if d is not None:
            xs.append(d[0]); ys.append(d[1])
        if len(xs) >= n:
            break
        time.sleep(0.12)
    if len(xs) < 3:
        return None
    return (float(np.median(xs)), float(np.median(ys)))


def bouger(mc, coords, pause=2.5):
    mc.send_coords(list(coords), V_DEPLACEMENT, 1)
    time.sleep(pause)


def main():
    print("Connexion au bras...")
    mc = connecter()

    # Pince OUVERTE avant d'aller en position (pour ne pas frapper la balle)
    mc.set_gripper_state(0, V_PINCE)
    time.sleep(2)

    # Aller a la position de depart (penchee sur la balle, au sol)
    print("Position de depart (au sol, penchee sur la balle)...")
    mc.send_angles(POSE_DEPART, V_ANGLES)
    time.sleep(6)

    cap = cv2.VideoCapture(CAMERA)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, LARGEUR)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HAUTEUR)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    time.sleep(1)

    d0 = detecter_stable(cap)
    if d0 is None:
        print("Balle non vue depuis le depart. Balle en place ?")
        cap.release(); return
    ex0, ey0 = d0[0]-CIBLE_DX, d0[1]-CIBLE_DY
    print(f"Balle vue dx={d0[0]:.0f} dy={d0[1]:.0f} "
          f"(cible {CIBLE_DX},{CIBLE_DY}, ecart {ex0:+.0f},{ey0:+.0f})")

    # Asservissement : amener la balle a la cible
    if abs(ex0) > TOLERANCE or abs(ey0) > TOLERANCE:
        print("\n--- Centrage sur la balle ---")
        coords = lire_coords(mc)
        if coords is None:
            print("Coords illisibles."); cap.release(); return

        # Sondage local
        base = detecter_stable(cap)
        c = list(coords); c[0] += PAS_SONDE_MM
        bouger(mc, c); apx = detecter_stable(cap); bouger(mc, coords)
        c = list(coords); c[1] += PAS_SONDE_MM
        bouger(mc, c); apy = detecter_stable(cap); bouger(mc, coords)
        if None in (base, apx, apy):
            print("Balle perdue au sondage."); cap.release(); return
        Jxx=(apx[0]-base[0])/PAS_SONDE_MM; Jyx=(apx[1]-base[1])/PAS_SONDE_MM
        Jxy=(apy[0]-base[0])/PAS_SONDE_MM; Jyy=(apy[1]-base[1])/PAS_SONDE_MM
        det=Jxx*Jyy-Jxy*Jyx
        print(f"  jacobienne det={det:.2f}")
        if abs(det) < 1e-3:
            print("  degeneree."); cap.release(); return
        Jinv=np.linalg.inv(np.array([[Jxx,Jxy],[Jyx,Jyy]]))

        coords = lire_coords(mc)
        for i in range(ITER_MAX):
            d = detecter_stable(cap)
            if d is None:
                print(f"  [{i+1}] perdue."); break
            ex, ey = d[0]-CIBLE_DX, d[1]-CIBLE_DY
            print(f"  [{i+1}] dx={d[0]:+.0f} dy={d[1]:+.0f} err=({ex:+.0f},{ey:+.0f})")
            if abs(ex) <= TOLERANCE and abs(ey) <= TOLERANCE:
                print("  -> centre sur la balle."); break
            dv = Jinv.dot([-ex,-ey])*GAIN_ASSERV
            dbx,dby = dv[0],dv[1]
            mnorm=max(abs(dbx),abs(dby),1e-6)
            if mnorm>PAS_MAX_MM: dbx*=PAS_MAX_MM/mnorm; dby*=PAS_MAX_MM/mnorm
            avant=d
            coords[0]+=dbx; coords[1]+=dby
            bouger(mc, coords)
            ap=detecter_stable(cap)
            if ap and abs(ap[0]-avant[0])+abs(ap[1]-avant[1])<SEUIL_BOUGE_PX:
                print("  mouvement sans effet - stop.")
                break
    else:
        print("Balle deja bien placee.")

    # Saisie : petite descente pour enserrer, puis fermer
    print("\n--- Saisie ---")
    coords = lire_coords(mc)
    if coords is None:
        print("Coords illisibles avant saisie."); cap.release(); return
    coords[2] -= DESCENTE_FERMETURE_MM
    bouger(mc, coords, pause=3)
    mc.set_gripper_state(1, V_PINCE)   # fermer
    time.sleep(2)
    coords[2] += REMONTEE_MM
    bouger(mc, coords, pause=4)
    print("Sequence terminee - balle saisie.")
    cap.release()


if __name__ == '__main__':
    main()
