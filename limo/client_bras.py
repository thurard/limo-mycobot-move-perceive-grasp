#!/usr/bin/env python3
# ============================================================
# client_bras.py - Commande le bras a distance
#
# A executer SUR LE PC ou SUR LE LIMO :
#     python3 client_bras.py etat
#     python3 client_bras.py ranger
#     python3 client_bras.py saisir 244.6 48.3 52.1 -87.7 -42.6 -85.4
#     python3 client_bras.py deposer 244.6 48.3 52.1 -87.7 -42.6 -85.4
#
# Utilisable aussi comme module depuis patrouille.py :
#     from client_bras import commander
#     commander({'action': 'saisir', 'coords': [...]})
# ============================================================

import json
import socket
import sys

# Adresse du Pi du bras sur le reseau iPhone.
# (Acces de secours en Ethernet : 192.168.1.100)
IP_BRAS = '172.20.10.12'
PORT = 5555
TIMEOUT = 240.0     # la saisie vision (paliers + re-centrages) peut prendre
                    # ~1-2 min ; on laisse large pour ne pas abandonner pendant
                    # que le bras travaille encore.


def commander(requete, ip=IP_BRAS, port=PORT, timeout=TIMEOUT):
    """Envoie une requete au bras et renvoie sa reponse (dict).
    Leve une exception si le bras est injoignable."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect((ip, port))
        s.sendall((json.dumps(requete) + "\n").encode())
        ligne = s.makefile('r').readline()
        return json.loads(ligne) if ligne else {'ok': False,
                                                'erreur': 'pas de reponse'}
    finally:
        s.close()


def main():
    if len(sys.argv) < 2:
        print(__doc__ or "Usage : client_bras.py <action> [coords...]")
        sys.exit(1)

    action = sys.argv[1]
    requete = {'action': action}

    if action in ('saisir', 'deposer'):
        if len(sys.argv) != 8:
            print(f"Usage : client_bras.py {action} x y z rx ry rz")
            sys.exit(1)
        requete['coords'] = [float(v) for v in sys.argv[2:8]]

    try:
        reponse = commander(requete)
    except Exception as e:
        print(f"[ERREUR] Bras injoignable sur {IP_BRAS}:{PORT} -> {e}")
        print("  Verifie que serveur_bras.py tourne sur le Pi,")
        print("  et que l'adresse IP_BRAS est toujours la bonne.")
        sys.exit(1)

    print(json.dumps(reponse, indent=2))
    sys.exit(0 if reponse.get('ok') else 1)


if __name__ == '__main__':
    main()
