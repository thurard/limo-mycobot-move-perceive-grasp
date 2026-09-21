#!/usr/bin/env python3
# ============================================================
# detect_photo.py — Détection d'objets YOLO sur une image.
#
# Prend une image, détecte les objets courants (personne, chaise,
# sac, bouteille... 80 classes du modèle COCO), et sauvegarde une
# version annotée (boîtes + noms + score de confiance).
#
# Usage :
#   python3 detect_photo.py <chemin_image>
#   python3 detect_photo.py ~/photos_patrouille/20260706_175655/point_3_bloque_XXX.jpg
#
# La sortie est sauvée à côté de l'image, suffixée "_detecte.jpg".
# ============================================================

import sys
import os
import time

from ultralytics import YOLO

# Modèle léger (nano) : rapide, adapté au Jetson. Se télécharge tout seul
# au premier lancement (~6 Mo) puis est réutilisé.
MODELE = "yolov8n.pt"

# Seuil de confiance : on n'affiche que les détections sûres à >= 40%.
SEUIL_CONFIANCE = 0.40


def main():
    if len(sys.argv) < 2:
        print("Usage : python3 detect_photo.py <chemin_image>")
        sys.exit(1)

    chemin_image = os.path.expanduser(sys.argv[1])
    if not os.path.isfile(chemin_image):
        print(f"[ERREUR] Image introuvable : {chemin_image}")
        sys.exit(1)

    print(f"Chargement du modele {MODELE}...")
    t0 = time.time()
    model = YOLO(MODELE)
    print(f"  Modele charge en {time.time()-t0:.1f}s")

    print(f"Detection sur : {chemin_image}")
    t0 = time.time()
    # conf = seuil de confiance ; verbose=False pour un log propre
    resultats = model(chemin_image, conf=SEUIL_CONFIANCE, verbose=False)
    print(f"  Detection faite en {time.time()-t0:.2f}s")

    r = resultats[0]

    # --- Résumé texte des objets détectés ---
    noms = r.names  # dictionnaire id -> nom de classe
    detections = r.boxes
    if detections is None or len(detections) == 0:
        print("  Aucun objet detecte (au-dessus du seuil).")
    else:
        print(f"  {len(detections)} objet(s) detecte(s) :")
        # Compter par type
        compte = {}
        for box in detections:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            nom = noms[cls_id]
            compte[nom] = compte.get(nom, 0) + 1
            print(f"    - {nom} (confiance {conf*100:.0f}%)")
        print("  Resume :", ", ".join(f"{n}x {o}" for o, n in compte.items()))

    # --- Sauvegarde de l'image annotée ---
    base, ext = os.path.splitext(chemin_image)
    chemin_sortie = f"{base}_detecte{ext}"
    # r.plot() renvoie l'image annotée (numpy BGR) ; on la sauve avec OpenCV
    import cv2
    img_annotee = r.plot()
    cv2.imwrite(chemin_sortie, img_annotee)
    print(f">>> Image annotee sauvee : {chemin_sortie}")


if __name__ == "__main__":
    main()
