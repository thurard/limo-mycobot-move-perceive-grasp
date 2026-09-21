#!/usr/bin/env python3
# ============================================================
# calib_couleur.py - Calibration AUTOMATIQUE de la couleur de la balle
#
# A executer SUR LE PI DU BRAS :
#     python3 ~/calib_couleur.py
#
# Ne demande PAS de viser un point. Le script cherche lui-meme la
# balle : il balaie les teintes orange/rose, garde la tache la plus
# ronde et pleine (une balle = un disque compact), puis mesure
# finement sa couleur et en deduit des seuils HSV robustes.
#
# Sortie : les lignes ORANGE_BAS / ORANGE_HAUT a coller dans
# guidage_visuel.py et detect_couleur.py.
#
# Prend plusieurs images et ne garde que les detections stables,
# pour que la calibration ne depende pas d'une image bruitee.
# ============================================================

import cv2
import numpy as np
import time

CAMERA = 0
LARGEUR, HAUTEUR = 640, 480

# Plage LARGE de recherche. IMPORTANT : la balance des blancs de la camera
# rend la balle ORANGE en ROSE/SAUMON. On cherche donc large, du rose-magenta
# (H haut, ~160-179) au orange (H bas, 0-25), en passant par le rouge (0).
# On gere le fait que le rouge/rose est a cheval sur 0 et 180.
# Deux sous-plages combinees.
H_ROSE = (160, 179)           # rose/magenta (cote haut du cercle)
H_ORANGE = (0, 25)            # rouge -> orange (cote bas)
S_MINI_RECHERCHE = 50         # la balle rose pale est peu saturee de loin
V_MINI_RECHERCHE = 60

N_IMAGES = 8                  # images analysees pour la stabilite
AIRE_MINI = 200               # une balle fait au moins ca, meme de loin


def capturer(cap):
    for _ in range(4):
        cap.grab()
    ok, img = cap.read()
    return img if ok else None


def trouver_balle(img):
    """Cherche la tache coloree la plus 'ronde et pleine' dans la plage large.
    Renvoie (masque_balle, (cx,cy,rayon)) ou None."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Deux sous-plages (rose cote haut + orange cote bas) combinees,
    # car la couleur percue est a cheval sur 0/180.
    m_rose = cv2.inRange(hsv,
                         np.array([H_ROSE[0], S_MINI_RECHERCHE, V_MINI_RECHERCHE]),
                         np.array([H_ROSE[1], 255, 255]))
    m_orange = cv2.inRange(hsv,
                           np.array([H_ORANGE[0], S_MINI_RECHERCHE, V_MINI_RECHERCHE]),
                           np.array([H_ORANGE[1], 255, 255]))
    masque_couleur = cv2.bitwise_or(m_rose, m_orange)
    noyau = np.ones((5, 5), np.uint8)
    masque_couleur = cv2.morphologyEx(masque_couleur, cv2.MORPH_OPEN, noyau)
    # Version 'pleine' pour LOCALISER la balle (bouche les motifs noirs).
    masque_plein = cv2.morphologyEx(masque_couleur, cv2.MORPH_CLOSE, noyau)
    gros_noyau = np.ones((15, 15), np.uint8)
    masque_plein = cv2.morphologyEx(masque_plein, cv2.MORPH_CLOSE, gros_noyau)

    contours, _ = cv2.findContours(masque_plein, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    meilleur = None
    meilleur_score = 0
    for c in contours:
        aire = cv2.contourArea(c)
        if aire < AIRE_MINI:
            continue
        # Circularite : 4*pi*aire / perimetre^2  (1.0 = cercle parfait).
        # On tolere assez bas (0.5) car les motifs noirs deforment un peu.
        perim = cv2.arcLength(c, True)
        if perim == 0:
            continue
        circ = 4 * np.pi * aire / (perim * perim)
        if circ < 0.5:          # rejette les taches franchement non-rondes
            continue
        # Score : privilegie une tache ronde ET de bonne taille
        score = circ * aire
        if score > meilleur_score:
            meilleur_score = score
            (x, y), r = cv2.minEnclosingCircle(c)
            meilleur = (c, int(x), int(y), int(r), circ, aire)

    if meilleur is None:
        return None, None
    c, cx, cy, r, circ, aire = meilleur
    # Disque plein de la balle (sert a la localiser)
    disque = np.zeros(masque_plein.shape, np.uint8)
    cv2.drawContours(disque, [c], -1, 255, -1)
    # IMPORTANT : pour MESURER la couleur, on ne garde que les pixels
    # reellement colores DANS le disque (masque de couleur initial), pas
    # les motifs noirs. Sinon la mesure melange rose et noir et les seuils
    # deviennent absurdement larges.
    masque_couleur_balle = cv2.bitwise_and(masque_couleur, disque)
    return masque_couleur_balle, (cx, cy, r, circ, aire)


def main():
    cap = cv2.VideoCapture(CAMERA)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, LARGEUR)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HAUTEUR)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    time.sleep(1)

    print("Recherche de la balle sur plusieurs images...")
    echantillons = []      # pixels HSV de la balle, accumules
    trouvees = 0
    derniere_img = None
    derniere_info = None

    for k in range(N_IMAGES):
        img = capturer(cap)
        if img is None:
            continue
        masque_balle, info = trouver_balle(img)
        if masque_balle is None:
            print(f"  image {k+1}: balle non trouvee")
            time.sleep(0.2)
            continue
        cx, cy, r, circ, aire = info
        print(f"  image {k+1}: balle a ({cx},{cy}) r={r} "
              f"circ={circ:.2f} aire={aire:.0f}")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        pixels = hsv[masque_balle > 0]
        if len(pixels) > 0:
            echantillons.append(pixels)
            trouvees += 1
            derniere_img = img
            derniere_info = info
        time.sleep(0.2)

    cap.release()

    if trouvees < 3:
        print(f"\nBalle trouvee sur seulement {trouvees} image(s) - trop peu.")
        print("Verifie qu'elle est bien dans le champ et eclairee.")
        if derniere_img is not None:
            cv2.imwrite('/home/er/calib_couleur.jpg', derniere_img)
            print("Image sauvee : ~/calib_couleur.jpg")
        return

    # Tous les pixels balle accumules
    tous = np.vstack(echantillons)

    # Bornes robustes : on ignore les 10% extremes (reflets, bords).
    def bornes(x, bas=10, haut=90):
        return int(np.percentile(x, bas)), int(np.percentile(x, haut))

    # La teinte rose est A CHEVAL sur 0/180 : les pixels sont soit vers 0-10,
    # soit vers 170-179. Une simple mediane serait fausse. On detecte le cas.
    H = tous[:, 0]
    proche_180 = np.mean(H > 90) > 0.5    # majorite des pixels cote haut ?

    s_bas, s_haut = bornes(tous[:, 1])
    v_bas, v_haut = bornes(tous[:, 2])
    # Marges : S/V larges vers le bas pour tolerer la balle delavee de loin
    s_bas = max(40, s_bas - 40)
    v_bas = max(40, v_bas - 40)

    print("\n===== SEUILS CALIBRES =====")
    print(f"Mediane balle : H={np.median(H):.0f} "
          f"S={np.median(tous[:,1]):.0f} V={np.median(tous[:,2]):.0f}")

    if proche_180:
        # Balle cote rose/magenta : plage H haute, plus repli sur 0 si besoin
        h_pixels = H[H > 90]
        h_bas = max(150, int(np.percentile(h_pixels, 5)) - 5)
        h_haut = 179
        print(f"\n# Balle percue ROSE/MAGENTA (teinte a cheval sur 180)")
        print(f"ORANGE_BAS  = np.array([{h_bas}, {s_bas}, {v_bas}])")
        print(f"ORANGE_HAUT = np.array([{h_haut}, {s_haut}, {v_haut}])")
        print(f"# 2e plage pour le repli cote 0 :")
        print(f"ORANGE_BAS_2  = np.array([0, {s_bas}, {v_bas}])")
        print(f"ORANGE_HAUT_2 = np.array([10, {s_haut}, {v_haut}])")
    else:
        h_bas, h_haut = bornes(H)
        h_bas = max(0, h_bas - 5); h_haut = min(179, h_haut + 5)
        print(f"\nORANGE_BAS  = np.array([{h_bas}, {s_bas}, {v_bas}])")
        print(f"ORANGE_HAUT = np.array([{h_haut}, {s_haut}, {v_haut}])")
    print(f"AIRE_MINI = 200")
    print("\n-> Colle ces lignes dans le detecteur")

    # Image de controle : balle entouree + masque applique
    if derniere_img is not None:
        cx, cy, r, circ, aire = derniere_info
        vis = derniere_img.copy()
        cv2.circle(vis, (cx, cy), r, (0, 255, 0), 2)
        cv2.circle(vis, (cx, cy), 3, (0, 0, 255), -1)
        # Verifie les seuils calibres sur cette image
        hsv = cv2.cvtColor(derniere_img, cv2.COLOR_BGR2HSV)
        m = cv2.inRange(hsv, np.array([h_bas, s_bas, v_bas]),
                        np.array([h_haut, s_haut, v_haut]))
        vis[m > 0] = (0.5 * vis[m > 0] + np.array([0, 128, 0])).astype(np.uint8)
        cv2.imwrite('/home/er/calib_couleur.jpg', vis)
        print("\nImage de controle : ~/calib_couleur.jpg")
        print("  (cercle vert = balle trouvee, teinte verte = seuils calibres)")


if __name__ == '__main__':
    main()
