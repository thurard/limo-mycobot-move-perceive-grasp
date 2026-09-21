#!/usr/bin/env python3
# ============================================================
# deposer.py - Depose la balle a une position FIXE de depot
#
# A executer SUR LE PI DU BRAS :  python3 ~/deposer.py
#
# A lancer APRES une saisie reussie (le bras tient la balle, pince fermee).
# Pas de vision ici : le point de depot est connu et fixe.
#
# Sequence :
#   1. Monter au-dessus du point de depot (approche par le haut, pour ne
#      pas trainer la balle sur la surface).
#   2. Descendre VERTICALEMENT jusqu'a la surface de depot (meme facon
#      qu'a la montee : seul Z change).
#   3. Ouvrir la pince -> pose la balle sur la surface plane.
#   4. Remonter, pince vide.
#
# Concu pour etre importable dans la patrouille :
#     from deposer import deposer
#     deposer(mc)                 # utilise le point de depot par defaut
#     deposer(mc, coords=[...])   # ou un autre point
# ============================================================

import time
from pymycobot import MyCobot

PORT = '/dev/ttyAMA0'
BAUD = 1000000

# --- Point de depot FIXE (coords releves : pince a la surface de depot) ---
COORDS_DEPOT = [244.2, 25.3, -30.1, -171.5, -4.99, -38.6]

# Hauteur de l'approche au-dessus du depot (pour arriver par le haut)
HAUTEUR_APPROCHE_MM = 90.0
# Hauteur de retrait apres avoir lache la balle
RETRAIT_MM = 100.0

V_DEPLACEMENT = 20
V_DESCENTE = 15
V_PINCE = 50


def connecter():
    mc = MyCobot(PORT, BAUD)
    time.sleep(2)
    # Power cycle : reset les servos (debloque la pince si etat 255).
    mc.power_off()
    time.sleep(2)
    mc.power_on()
    time.sleep(2)
    return mc


def lire_coords(mc, essais=8):
    for _ in range(essais):
        c = mc.get_coords()
        if c and len(c) == 6:
            return c
        time.sleep(0.4)
    return None


def deposer(mc, coords=None):
    """Depose la balle au point de depot. mc doit tenir la balle (pince
    fermee) au moment de l'appel."""
    if coords is None:
        coords = list(COORDS_DEPOT)

    # 1. Approche : au-dessus du point de depot (meme X,Y, Z rehausse)
    approche = list(coords)
    approche[2] = coords[2] + HAUTEUR_APPROCHE_MM
    print(f"1. Approche au-dessus du depot (Z={approche[2]:.0f})")
    mc.send_coords([float(v) for v in approche], V_DEPLACEMENT, 1)
    time.sleep(4)

    # 2. Descente verticale jusqu'a la surface de depot (seul Z change)
    print(f"2. Descente verticale jusqu'a la surface (Z={coords[2]:.0f})")
    pose = lire_coords(mc)
    if pose is None:
        pose = list(approche)
    pose[2] = coords[2]
    mc.send_coords([float(v) for v in pose], V_DESCENTE, 1)
    time.sleep(4)

    # 3. Ouvrir la pince -> pose la balle sur la surface
    print("3. Ouverture pince (depose la balle)")
    mc.set_gripper_state(0, V_PINCE)
    time.sleep(2)

    # 4. Remonter, pince vide
    print("4. Retrait vers le haut")
    pose = lire_coords(mc)
    if pose is None:
        pose = list(coords)
    pose[2] = coords[2] + RETRAIT_MM
    mc.send_coords([float(v) for v in pose], V_DEPLACEMENT, 1)
    time.sleep(4)
    print("Depot termine.")


def main():
    print("Connexion au bras...")
    mc = connecter()
    print("Depot de la balle au point fixe...")
    deposer(mc)


if __name__ == '__main__':
    main()
