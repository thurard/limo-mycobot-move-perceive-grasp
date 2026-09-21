#!/usr/bin/env python3
# ============================================================
# detect_couleur.py - Detection d'un objet colore (camera du bras)
#
# A executer SUR LE PI DU BRAS :
#     python3 ~/detect_couleur.py
#
# Capture une image, isole les zones de la couleur cible (rose par
# defaut), trouve le plus gros amas et indique sa position dans
# l'image. Sauvegarde une image annotee pour verification.
#
# But : verifier que la detection couleur est fiable, et mesurer
# ou apparait l'objet quand la pince est bien placee (= consigne
# de centrage pour un futur asservissement).
# ============================================================

import cv2
import numpy as np

CAMERA = 0
LARGEUR, HAUTEUR = 640, 480

# --- Plage de couleur cible en HSV ---
# Calee sur balle orange fluo : H median ~8 (plage 2-9), S~160, V~210.
# Marge autour pour tolerer les zones plus claires/sombres de la sphere.
# Balle rose pale sans motifs : teinte a cheval sur 0/180, deux sous-plages.
ROSE_BAS_1 = np.array([0, 40, 97])
ROSE_HAUT_1 = np.array([14, 130, 220])
ROSE_BAS_2 = np.array([170, 40, 97])
ROSE_HAUT_2 = np.array([179, 130, 220])

AIRE_MINI = 200


def capturer():
    cap = cv2.VideoCapture(CAMERA)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, LARGEUR)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HAUTEUR)
    import time
    time.sleep(1)              # laisser la camera se stabiliser
    ok, img = cap.read()
    cap.release()
    if not ok:
        raise RuntimeError("Echec de capture camera")
    return img


def masque_couleur(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, ROSE_BAS_1, ROSE_HAUT_1)
    m2 = cv2.inRange(hsv, ROSE_BAS_2, ROSE_HAUT_2)
    masque = cv2.bitwise_or(m1, m2)
    # Nettoyage : supprime le bruit, bouche les trous
    noyau = np.ones((5, 5), np.uint8)
    masque = cv2.morphologyEx(masque, cv2.MORPH_OPEN, noyau)
    masque = cv2.morphologyEx(masque, cv2.MORPH_CLOSE, noyau)
    return masque


def main():
    img = capturer()
    masque = masque_couleur(img)

    contours, _ = cv2.findContours(masque, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        print("Aucune zone rose detectee.")
        cv2.imwrite('/home/er/detect_couleur.jpg', img)
        return

    # Plus gros contour = l'objet
    c = max(contours, key=cv2.contourArea)
    aire = cv2.contourArea(c)
    if aire < AIRE_MINI:
        print(f"Zone rose trop petite (aire={aire:.0f}) - probablement du bruit.")
        cv2.imwrite('/home/er/detect_couleur.jpg', img)
        return

    x, y, w, h = cv2.boundingRect(c)
    cx, cy = x + w // 2, y + h // 2

    # Position par rapport au CENTRE de l'image (ce qui sert au centrage)
    centre_img_x = LARGEUR // 2
    centre_img_y = HAUTEUR // 2
    ecart_x = cx - centre_img_x     # >0 : objet a droite dans l'image
    ecart_y = cy - centre_img_y     # >0 : objet en bas dans l'image

    print(f"Objet rose detecte :")
    print(f"  centre dans l'image : ({cx}, {cy})   aire : {aire:.0f} px")
    print(f"  ecart au centre     : dx={ecart_x:+d}  dy={ecart_y:+d}")
    print(f"  -> objet {'a droite' if ecart_x>0 else 'a gauche'}, "
          f"{'en bas' if ecart_y>0 else 'en haut'} de l'image")

    # Image annotee
    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
    cv2.drawMarker(img, (centre_img_x, centre_img_y), (255, 0, 0),
                   cv2.MARKER_CROSS, 20, 2)
    cv2.imwrite('/home/er/detect_couleur.jpg', img)
    print("  image annotee : ~/detect_couleur.jpg")


if __name__ == '__main__':
    main()
