#!/usr/bin/env python3
# ============================================================
# saisie.py - Sequence pick-and-place pour myCobot 280 Pi
#
# A executer SUR LE PI DU BRAS (er@192.168.1.100) :
#     python3 ~/saisie.py
#
# La position de saisie est un PARAMETRE : c'est ce qui permettra
# plus tard au LIMO d'envoyer les coordonnees d'un objet detecte.
# ============================================================

import time
from pymycobot import MyCobot

# --- Connexion au bras ---
PORT = '/dev/ttyAMA0'
BAUD = 1000000
DELAI_INIT = 2.0          # le bras a besoin d'un temps d'init avant de repondre

# --- Positions de reference (relevees par apprentissage) ---
# Position de saisie mesuree en placant le bras a la main autour de l'objet.
SAISIE_DEFAUT = [244.6, 48.3, 52.1, -87.7, -42.6, -85.4]

# Hauteur (mm) au-dessus de l'objet pour la phase d'approche.
# On descend ensuite verticalement : plus propre qu'une arrivee de biais.
HAUTEUR_APPROCHE = 120.0

# Position repliee (en ANGLES articulaires, pas en coordonnees)
REPOS = [0, 120, -150, 30, 0, 0]

# --- Vitesses (0-100) ---
V_DEPLACEMENT = 25        # trajets a vide
V_DESCENTE = 20           # plus lent pour la descente sur l'objet
V_PINCE = 50
V_ANGLES = 30

# --- Temporisations ---
T_MOUVEMENT = 5.0         # attente apres un send_coords
T_PINCE = 2.0             # attente apres une commande de pince


def connecter():
    """Ouvre la liaison serie avec le bras et attend son initialisation."""
    mc = MyCobot(PORT, BAUD)
    time.sleep(DELAI_INIT)
    return mc


def position_approche(coords_objet):
    """Renvoie la position d'approche : meme point, mais plus haut.
    On ne modifie que Z, l'orientation de la pince reste identique."""
    approche = list(coords_objet)
    approche[2] += HAUTEUR_APPROCHE
    return approche


def aller_a(mc, coords, vitesse, libelle=""):
    """Deplacement en coordonnees cartesiennes, en ligne droite (mode 1)."""
    if libelle:
        print(f"  -> {libelle}")
    mc.send_coords(coords, vitesse, 1)
    time.sleep(T_MOUVEMENT)


def ouvrir_pince(mc):
    mc.set_gripper_state(0, V_PINCE)
    time.sleep(T_PINCE)


def fermer_pince(mc):
    mc.set_gripper_state(1, V_PINCE)
    time.sleep(T_PINCE)


def ranger(mc):
    """Replie le bras en position de rangement (commande en angles)."""
    print("Rangement du bras...")
    mc.send_angles(REPOS, V_ANGLES)
    time.sleep(T_MOUVEMENT)


def saisir(mc, coords_objet):
    """Saisit un objet situe aux coordonnees donnees.

    Sequence : approche au-dessus -> pince ouverte -> descente verticale
    -> fermeture -> remontee. Le bras repart avec l'objet en pince.
    """
    approche = position_approche(coords_objet)

    print("SAISIE")
    aller_a(mc, approche, V_DEPLACEMENT, "approche au-dessus de l'objet")
    ouvrir_pince(mc)
    aller_a(mc, coords_objet, V_DESCENTE, "descente sur l'objet")
    fermer_pince(mc)
    aller_a(mc, approche, V_DEPLACEMENT, "remontee avec l'objet")


def deposer(mc, coords_depot):
    """Depose l'objet tenu en pince aux coordonnees donnees."""
    approche = position_approche(coords_depot)

    print("DEPOSE")
    aller_a(mc, approche, V_DEPLACEMENT, "approche du point de depot")
    aller_a(mc, coords_depot, V_DESCENTE, "descente pour poser")
    ouvrir_pince(mc)
    aller_a(mc, approche, V_DEPLACEMENT, "remontee a vide")


def main():
    mc = connecter()
    print("Bras connecte. Angles actuels :", mc.get_angles())

    try:
        ranger(mc)

        # Pour l'instant on saisit et on repose au meme endroit.
        # Plus tard, coords_objet viendra de la perception du LIMO.
        saisir(mc, SAISIE_DEFAUT)
        deposer(mc, SAISIE_DEFAUT)

        ranger(mc)
        print("Sequence terminee.")

    except KeyboardInterrupt:
        print("\nInterruption - arret du bras.")
        mc.stop()


if __name__ == '__main__':
    main()
