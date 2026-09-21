#!/usr/bin/env python3
# ============================================================
# guidage_angles.py - Saisie guidee par la camera, ASSERVISSEMENT EN ANGLES
#
# A executer SUR LE PI DU BRAS :  python3 ~/guidage_angles.py
#
# POURQUOI EN ANGLES (et pas en coordonnees)
# L'asservissement cartesien (send_coords) echouait : sondage degenere
# (det~0), saturation d'une articulation, refus silencieux. On commande
# donc DIRECTEMENT les articulations qui deplacent la pince au-dessus de
# la zone :
#     J1 (rotation base)  -> deplace la pince lateralement (gauche/droite)
#     J2 (epaule)         -> deplace la pince en avant/arriere
# send_angles bouge franchement (effet image mesurable) et ne sature
# jamais en silence comme la cinematique inverse.
#
# On MESURE l'effet de J1 et J2 sur l'image (sondage), on inverse, on
# asservit. Puis on descend (J2/coude) et on ferme.
# ============================================================

import time
import cv2
import numpy as np
from pymycobot import MyCobot

PORT = '/dev/ttyAMA0'
BAUD = 1000000
CAMERA = 0
LARGEUR, HAUTEUR = 640, 480

ORANGE_BAS = np.array([0, 40, 97]);    ORANGE_HAUT = np.array([14, 130, 220])
ORANGE_BAS_2 = np.array([170, 40, 97]); ORANGE_HAUT_2 = np.array([179, 130, 220])
AIRE_MINI = 200

# --- Position de depart : HAUTE (vue d'ensemble large), voit bien la balle.
# Position de travail actuelle. Le "trop haut a la descente" se regle par
# la descente vers un Z ABSOLU (voir plus bas), pas en changeant la position.
POSE_DEPART = [21.26, -62.05, -85.25, 55.45, 0.26, -30.23]

# --- Cible image : ou la balle doit apparaitre pour etre sous la pince.
CIBLE_DX = -183
CIBLE_DY = -51
TOLERANCE = 12          # px : resserre pour un centrage plus fin (etait 25).
                        # Plus bas = pince mieux placee avant la descente.

# --- Sondage/asservissement EN ANGLES ---
SONDE_J1_DEG = 6.0        # pas de test sur la base
SONDE_J2_DEG = 6.0        # pas de test sur l'epaule
PAS_MAX_DEG = 5.0         # correction max par iteration (securite)
GAIN_ASSERV = 0.6
ITER_MAX = 15
SEUIL_BOUGE_PX = 5
# Compensation poignet : mesure = J2 +20 deg fait basculer la pince (RY) de
# ~9 deg. Donc ~0.45 deg de bascule par deg de J2. On corrige J4 du meme
# montant (signe a verifier au test) pour garder la pince orientee vers le bas.
COMP_POIGNET = -0.45

# --- Descente de saisie : vers un Z ABSOLU (niveau de la balle au sol) ---
# Le bras descend jusqu'a cette hauteur quelle que soit sa hauteur de
# centrage. La balle au sol est vers Z=-40 ; on vise un peu au-dessus pour
# enserrer sans taper (centre de balle ~20mm du sol). A ajuster au test.
Z_SAISIE_ABS = -25.0
# Descente par paliers : on descend de PALIER_MM a chaque etape, en
# re-centrant la balle entre chaque. Empeche la derive d'une longue
# descente aveugle de s'accumuler.
PALIER_MM = 60.0    # pas de descente (etait 40) : moins de paliers = plus
                    # rapide. Le re-centrage a chaque palier rattrape la derive.
V_ANGLES = 70       # vitesse mouvements angles : rapide (le bras encaisse bien)
V_PINCE = 50
# Valeur de fermeture pour TENIR la balle sans surcharger le servo.
# Mesure : la pince bute sur la balle vers 53 ; on ferme a 50, elle tient
# sans forcer. NE PAS utiliser set_gripper_state(1) (ferme a fond -> 255,
# servo en protection, pince bloquee jusqu'a un power_off/power_on).
VALEUR_FERMETURE = 50


def connecter():
    mc = MyCobot(PORT, BAUD)
    time.sleep(2)
    # Power cycle : reset les servos, notamment la pince si elle etait bloquee
    # (etat 255) d'une execution precedente. Evite de devoir redemarrer le bras.
    mc.power_off()
    time.sleep(2)
    mc.power_on()
    time.sleep(2)
    return mc


def lire_angles(mc, essais=8):
    for _ in range(essais):
        a = mc.get_angles()
        if a and len(a) == 6:
            return a
        time.sleep(0.4)
    return None


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
    # 6 images (etait 9) : compromis stabilite/vitesse. La detection est franche
    # (balle sans motifs), 6 suffisent pour une mediane stable.
    xs, ys = [], []
    for _ in range(n + 4):
        d = detecter(cap)
        if d is not None:
            xs.append(d[0]); ys.append(d[1])
        if len(xs) >= n:
            break
        time.sleep(0.06)
    if len(xs) < 3:
        return None
    return (float(np.median(xs)), float(np.median(ys)))


def envoyer_angles(mc, angles, pause=1.0):
    mc.send_angles([float(a) for a in angles], V_ANGLES)
    time.sleep(pause)


def saisir_avec_vision(mc):
    """Saisie guidee par la camera. mc doit etre deja connecte (le serveur
    reutilise sa connexion). Renvoie True si la balle a ete saisie, False sinon.
    Lancee seule (main), elle connecte elle-meme le bras."""
    # RESET PINCE au debut de CHAQUE saisie : le serveur reste en ecoute en
    # permanence, donc si la pince s'est bloquee (etat 255) lors d'une saisie
    # precedente, elle le reste. Un power_off/power_on la debloque a coup sur.
    # Sans ca, une pince bloquee reste bloquee pour toutes les saisies suivantes.
    print("Reset pince (power cycle)...")
    mc.power_off()
    time.sleep(1.5)
    mc.power_on()
    time.sleep(1.5)

    mc.set_gripper_state(0, V_PINCE)   # pince ouverte
    time.sleep(1)

    print("Position de depart...")
    envoyer_angles(mc, POSE_DEPART, pause=3.5)

    # Re-ouvrir la pince APRES la mise en position (au cas ou la 1ere commande
    # serait passee avant que les moteurs/pince soient prets).
    mc.set_gripper_state(0, V_PINCE)
    time.sleep(1)
    angles = lire_angles(mc)
    if angles is None:
        print("Angles illisibles."); return False

    cap = cv2.VideoCapture(CAMERA)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, LARGEUR)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HAUTEUR)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    time.sleep(1)

    d0 = detecter_stable(cap)
    if d0 is None:
        print("Balle non vue au depart."); cap.release(); return False
    print(f"Balle vue dx={d0[0]:.0f} dy={d0[1]:.0f} (cible {CIBLE_DX},{CIBLE_DY})")

    # --- Sondage EN ANGLES : effet de J1 et J2 sur l'image ---
    print("\n--- Sondage (J1 base, J2 epaule) ---")
    base = detecter_stable(cap)
    a = list(angles); a[0] += SONDE_J1_DEG
    envoyer_angles(mc, a); apj1 = detecter_stable(cap); envoyer_angles(mc, angles)
    a = list(angles); a[1] += SONDE_J2_DEG
    envoyer_angles(mc, a); apj2 = detecter_stable(cap); envoyer_angles(mc, angles)
    if None in (base, apj1, apj2):
        print("Balle perdue au sondage."); cap.release(); return False

    # Jacobienne : d(image) = J . d(angles J1,J2)
    Jx1=(apj1[0]-base[0])/SONDE_J1_DEG; Jy1=(apj1[1]-base[1])/SONDE_J1_DEG
    Jx2=(apj2[0]-base[0])/SONDE_J2_DEG; Jy2=(apj2[1]-base[1])/SONDE_J2_DEG
    J=np.array([[Jx1,Jx2],[Jy1,Jy2]])
    det=Jx1*Jy2-Jx2*Jy1
    print(f"  J={J.tolist()}  det={det:.2f}")
    print(f"  (J1: {Jx1:+.1f},{Jy1:+.1f} px/deg | J2: {Jx2:+.1f},{Jy2:+.1f} px/deg)")
    if abs(det) < 1e-2:
        print("  Sondage inefficace (J1/J2 ne bougent pas assez l'image)."); 
        cap.release(); return False
    Jinv=np.linalg.inv(J)

    # --- Asservissement EN ANGLES (fonction de centrage reutilisable) ---
    def centrer(cap, angles, Jinv, label=""):
        """Centre la balle sur la cible image en bougeant J1/J2 (+ comp J4).
        Renvoie (angles, centre_ok)."""
        centre = False
        for i in range(ITER_MAX):
            d = detecter_stable(cap)
            if d is None:
                print(f"  {label}[{i+1}] perdue."); break
            ex, ey = d[0]-CIBLE_DX, d[1]-CIBLE_DY
            print(f"  {label}[{i+1}] dx={d[0]:+.0f} dy={d[1]:+.0f} "
                  f"err=({ex:+.0f},{ey:+.0f})")
            if abs(ex) <= TOLERANCE and abs(ey) <= TOLERANCE:
                print(f"  {label}-> centre."); centre = True; break
            dang = Jinv.dot([-ex,-ey])*GAIN_ASSERV
            d1, d2 = dang[0], dang[1]
            mnorm=max(abs(d1),abs(d2),1e-6)
            if mnorm>PAS_MAX_DEG: d1*=PAS_MAX_DEG/mnorm; d2*=PAS_MAX_DEG/mnorm
            avant=d
            angles[0]+=d1
            angles[1]+=d2
            angles[3] += COMP_POIGNET * d2   # garde la pince piquee vers le bas
            envoyer_angles(mc, angles)
            # (Optimisation : plus de 2e detection "anti-blocage" par iteration.
            #  Elle DOUBLAIT le cout en detections. La prochaine iteration
            #  detecte de toute facon ; si le bras est bloque, err ne bougera
            #  pas et on le verra. On gagne ~40% du temps de centrage.)
        return angles, centre

    print("\n--- Centrage initial ---")
    angles = lire_angles(mc)
    angles, centre = centrer(cap, angles, Jinv)
    if not centre:
        print("Centrage imparfait - on continue quand meme.")

    # --- Saisie : DESCENTE PAR PALIERS avec RE-CENTRAGE ---
    # Au lieu d'une longue descente aveugle (qui derive de plusieurs cm), on
    # descend par petites etapes en Z, et a CHAQUE palier on re-centre la
    # balle (la camera etant sur la pince, elle la revoit). La derive ne peut
    # plus s'accumuler : la pince reste au-dessus de la balle jusqu'en bas.
    print("\n--- Descente par paliers (re-centrage a chaque etape) ---")
    coords = None
    for _ in range(8):
        coords = mc.get_coords()
        if coords and len(coords) == 6:
            break
        time.sleep(0.4)
    if coords is None:
        print("Coords illisibles avant descente."); cap.release(); return False

    z = coords[2]
    while z > Z_SAISIE_ABS + PALIER_MM:
        z_cible = max(Z_SAISIE_ABS, z - PALIER_MM)
        coords = lire_coords(mc)
        coords[2] = z_cible
        print(f"  palier : Z {z:.0f} -> {z_cible:.0f}")
        mc.send_coords([float(v) for v in coords], 45, 1)
        time.sleep(1.0)
        # Re-centrer a ce palier tant que la balle est visible
        d = detecter_stable(cap)
        if d is not None:
            angles = lire_angles(mc)
            angles, _ = centrer(cap, angles, Jinv, label="re-centre ")
        else:
            print("  balle non vue a ce palier - on continue la descente.")
        z_lu = lire_coords(mc)
        z = z_lu[2] if z_lu else z_cible

    # Dernier segment court jusqu'au niveau de saisie
    coords = lire_coords(mc)
    coords[2] = Z_SAISIE_ABS
    print(f"  descente finale vers Z={Z_SAISIE_ABS:.0f}")
    mc.send_coords([float(v) for v in coords], 40, 1)
    time.sleep(1.2)
    # Fermeture a VALEUR (50) et non a fond : la pince s'arrete sur la balle
    # (~53) sans forcer. Fermer a fond (set_gripper_state(1)) surcharge le
    # servo qui passe en protection (valeur 255, pince bloquee jusqu'a reset).
    mc.set_gripper_value(VALEUR_FERMETURE, V_PINCE)
    time.sleep(2)
    # Verifie que la pince a bien repondu. Valeur 255 ou -1 = bloquee/erreur :
    # on tente un reset (power cycle) puis on referme une fois.
    try:
        v = mc.get_gripper_value()
    except Exception:
        v = 255
    if v is None or v >= 250 or v < 0:
        print(f"  Pince ne repond pas (valeur={v}) - reset et 2e tentative...")
        mc.power_off(); time.sleep(2); mc.power_on(); time.sleep(2)
        mc.set_gripper_value(VALEUR_FERMETURE, V_PINCE)
        time.sleep(2)
        try:
            v = mc.get_gripper_value()
        except Exception:
            v = 255
        print(f"  valeur pince apres 2e tentative : {v}")
    z_depart = 120
    coords = lire_coords(mc)
    if coords:
        coords[2] = z_depart
        mc.send_coords([float(v) for v in coords], 20, 1)
        time.sleep(4)
    print("Sequence terminee.")
    cap.release()
    return True


def main():
    """Lancement seul : connecte le bras puis lance la saisie guidee."""
    print("Connexion au bras...")
    mc = connecter()
    ok = saisir_avec_vision(mc)
    print("Resultat saisie :", "OK" if ok else "ECHEC")


if __name__ == '__main__':
    main()
