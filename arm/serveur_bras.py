#!/usr/bin/env python3
# ============================================================
# serveur_bras.py - Serveur de commande du myCobot
#
# A executer SUR LE PI DU BRAS :
#     python3 ~/serveur_bras.py
#
# Reste en ecoute et execute les ordres recus sur le reseau.
# Necessite saisie.py, guidage_angles.py et deposer.py dans le meme dossier.
#
# Protocole : une requete = une ligne JSON, une reponse = une ligne JSON.
#   {"action": "etat"}
#   {"action": "ranger"}
#   {"action": "saisir_vision"}                # <-- saisie GUIDEE PAR CAMERA
#   {"action": "deposer_fixe"}                 # <-- depot au point fixe memorise
#   {"action": "saisir",  "coords": [6 val]}   # saisie par coords (sans vision)
#   {"action": "deposer", "coords": [6 val]}   # depot par coords
# ============================================================

import json
import socket
import traceback

import saisie
import guidage_angles          # saisie guidee par la camera
import deposer as depot_mod    # depot au point fixe

HOTE = '0.0.0.0'    # ecoute sur toutes les interfaces (WiFi ET Ethernet)
PORT = 5555


def traiter(mc, requete):
    """Execute une requete et renvoie le dictionnaire de reponse."""
    action = requete.get('action')

    if action == 'etat':
        return {'ok': True,
                'angles': mc.get_angles(),
                'coords': mc.get_coords()}

    if action == 'ranger':
        saisie.ranger(mc)
        return {'ok': True, 'message': 'bras range'}

    # --- Saisie GUIDEE PAR CAMERA (rattrape l'imprecision de stationnement) ---
    if action == 'saisir_vision':
        ok = guidage_angles.saisir_avec_vision(mc)
        return {'ok': bool(ok),
                'message': 'balle saisie' if ok else 'saisie echouee',
                'coords_atteintes': mc.get_coords()}

    # --- Depot au point FIXE memorise ---
    if action == 'deposer_fixe':
        depot_mod.deposer(mc)
        return {'ok': True, 'message': 'balle deposee',
                'coords_atteintes': mc.get_coords()}

    # --- Saisie / depot par COORDONNEES (sans vision) ---
    if action in ('saisir', 'deposer'):
        coords = requete.get('coords')
        if not coords or len(coords) != 6:
            return {'ok': False,
                    'erreur': 'coords doit contenir 6 valeurs [x,y,z,rx,ry,rz]'}
        if action == 'saisir':
            saisie.saisir(mc, coords)
        else:
            saisie.deposer(mc, coords)
        return {'ok': True, 'message': f'{action} termine',
                'coords_atteintes': mc.get_coords()}

    return {'ok': False, 'erreur': f"action inconnue : {action}"}


def main():
    print("Connexion au bras...")
    mc = saisie.connecter()
    print("Bras pret. Angles :", mc.get_angles())

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOTE, PORT))
    srv.listen(1)
    print(f"En ecoute sur le port {PORT}. Ctrl+C pour arreter.")

    try:
        while True:
            client, adresse = srv.accept()
            print(f"\n--- Connexion de {adresse[0]} ---")
            try:
                client.settimeout(180.0)   # saisie vision peut prendre ~1-2 min
                ligne = client.makefile('r').readline()
                if not ligne:
                    client.close()
                    continue
                requete = json.loads(ligne)
                print("Requete :", requete)
                reponse = traiter(mc, requete)
            except Exception as e:
                traceback.print_exc()
                reponse = {'ok': False, 'erreur': str(e)}

            print("Reponse :", reponse)
            try:
                client.sendall((json.dumps(reponse) + "\n").encode())
            except Exception:
                pass
            client.close()

    except KeyboardInterrupt:
        print("\nArret du serveur.")
    finally:
        srv.close()


if __name__ == '__main__':
    main()
