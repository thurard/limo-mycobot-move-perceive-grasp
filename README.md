# limo-mycobot-move-perceive-grasp

Chaîne robotique **« se déplacer – percevoir – saisir »** développée pendant mon
stage au **Department of Production and Robotics Engineering, KMUTNB (Bangkok)**,
de mai à août 2026.

Le système combine un robot mobile **AgileX LIMO** (navigation autonome +
perception) et un bras **myCobot 280 Pi** (saisie d'objet), reliés par un pont
réseau. L'objectif : que le LIMO patrouille une zone cartographiée, détecte des
objets, et déclenche la saisie d'une cible par le bras.

> **Contexte.** Ce dépôt rassemble le code de stage tel qu'il a tourné sur le
> matériel réel. Ce n'est pas un paquet ROS2 packagé (`colcon`), mais un
> ensemble de scripts lancés à la main sur chaque machine. Les scripts sont
> abondamment commentés (en français) pour rester lisibles et réutilisables.

---

## Architecture matérielle

```
   ┌──────────────────────────┐         réseau Wi-Fi (hotspot iPhone)        ┌───────────────────────────┐
   │        AgileX LIMO        │   ─────────────  socket TCP  ─────────────►  │   Raspberry Pi du myCobot  │
   │  (ROS2 Foxy)              │            JSON / port 5555                   │   (myCobot 280 Pi)         │
   │                           │                                              │                            │
   │  • Nav2 (AMCL, DWB)       │                                              │  • pymycobot               │
   │  • Cartographer (SLAM)    │                                              │  • OpenCV (caméra pince)   │
   │  • caméra Orbbec RGB-D    │                                              │  • asservissement visuel   │
   │  • YOLOv8n (détection)    │                                              │    en angles               │
   │                           │                                              │                            │
   │  client_bras.py  ────────────────► serveur_bras.py  (écoute port 5555)  ◄──────────────────────────┘
   └──────────────────────────┘
```

- **LIMO** : navigation, cartographie, perception globale (YOLO + profondeur).
- **Pi du bras** : reçoit des ordres du LIMO/PC via un serveur socket et exécute
  la saisie avec sa propre caméra embarquée dans la pince.
- **Pont réseau** : `client_bras.py` (côté LIMO/PC) ↔ `serveur_bras.py` (côté Pi),
  messages JSON sur le port **5555**.

---

## Résultat technique clé : asservissement visuel *en angles*

Le point dur du stage a été la saisie fiable de la cible. La commande
**cartésienne** du myCobot (`send_coords`) sature silencieusement près des limites
articulaires : le bras « accepte » la consigne sans l'atteindre, et se retrouve
dans une pose dégénérée. La solution retenue est un **asservissement en angles**
(`guidage_angles.py`) :

- on corrige directement **J1 (base)** et **J2 (épaule)** à partir de l'écart
  pixel entre la cible détectée et une consigne image (`CIBLE_DX`, `CIBLE_DY`),
  via un jacobien empirique ;
- **descente par paliers** (`PALIER_MM`) avec **re-centrage** à chaque palier ;
- **protection de la pince** : fermeture à `VALEUR_FERMETURE = 50` (et non 255,
  qui force et endommage le servo), avec cycle d'alimentation de réinitialisation.

C'est cette approche en angles qui a rendu la saisie robuste et répétable.

---

## Arborescence

```
limo-mycobot-move-perceive-grasp/
├── limo/                     # code exécuté SUR le LIMO
│   ├── patrouille.py          # nœud de patrouille : navigation Nav2 + YOLO + rapport
│   ├── client_bras.py         # envoie les ordres au bras (socket, port 5555)
│   ├── limo_start.launch.py   # TF statiques (laser, caméra, IMU) + description robot
│   ├── pub_robot_description.py
│   ├── start_patrol.sh        # script de lancement patrouille
│   └── start_mapping.sh       # script de lancement cartographie (Cartographer)
│
├── arm/                      # code exécuté SUR le Raspberry Pi du myCobot
│   ├── serveur_bras.py        # serveur socket : reçoit les actions et les exécute
│   ├── guidage_angles.py      # ★ asservissement visuel EN ANGLES (saisie fiable)
│   ├── saisie.py              # poses de saisie/dépôt/repos par coordonnées
│   ├── deposer.py             # dépôt à une position fixe
│   ├── detect_couleur.py      # détection de l'objet coloré (caméra pince, HSV)
│   ├── detect_3d.py           # YOLO + profondeur → position 3D (repère caméra)
│   ├── detect_photo.py        # détection sur photo (hors-ligne)
│   ├── calib_couleur.py       # calibration des plages HSV
│   ├── calib_axes.py          # calibration des axes image ↔ mouvement bras
│   └── diag_camera.py         # diagnostic caméra (dimensions, alignement, nuage)
│
├── config/                  # fichiers de configuration ROS2 / Nav2 / SLAM
│   ├── nav2_src.yaml          # config Nav2 « source » (inflation_radius 0.45)
│   ├── nav2_install.yaml      # config Nav2 « installée » (footprint scale 10.0,
│   │                          #   caméra en obstacle_layer via pointcloud)
│   └── limo_lds_2d.lua        # config Cartographer (tracking_frame=base_link, IMU)
│
├── maps/                    # carte utilisée par la patrouille
│   ├── map_bras.pgm
│   └── map_bras.yaml
│
└── archive/                 # versions obsolètes conservées pour l'historique
    └── guidage_visuel.py      # ancien asservissement CARTÉSIEN (abandonné :
                               #   saturation silencieuse de send_coords)
```

---

## Lancement (ordre réel utilisé pendant le stage)

Les scripts tournent sur **deux machines** ; l'ordre ci-dessous est celui
effectivement suivi lors des démonstrations.

**1. Sur le Raspberry Pi du bras** — démarrer le serveur :
```bash
python3 ~/serveur_bras.py
```
Le serveur écoute le port 5555 et attend les ordres.

**2. Sur le LIMO** — lancer la caméra, la localisation (Nav2/AMCL) puis la
patrouille :
```bash
./start_patrol.sh      # ou lancement manuel de patrouille.py
```
Le LIMO enchaîne les points de la carte, mémorise une photo de chaque point à
~1,5 m, et génère un rapport CSV + HTML des détections.

**3. Déclenchement de la saisie** — depuis le LIMO (ou le PC), quand le robot est
à poste face à la cible :
```bash
python3 client_bras.py saisir_vision   # le bras cherche la balle et la saisit
python3 client_bras.py deposer_fixe    # le bras dépose à la position fixe
```

### Intégration patrouille ↔ bras

L'orchestration entre la patrouille et le bras était **semi-automatique**, et non
entièrement autonome de bout en bout :

- `patrouille.py` gère seul la navigation et la perception.
- Au point où le bras doit agir, `patrouille.py` contient **un marqueur**
  (`print(...)` + `time.sleep(2)`) : **il n'appelle pas encore automatiquement**
  `client_bras.py`.
- Concrètement, le `serveur_bras.py` était démarré *avant* la patrouille, puis les
  ordres `saisir_vision` / `deposer_fixe` étaient **envoyés manuellement** pendant
  que le LIMO patrouillait de façon autonome.

Le pont réseau et la saisie sont donc pleinement fonctionnels et testés ; il reste
à remplacer le marqueur de `patrouille.py` par un vrai appel à `commander(...)`
(déjà importable depuis `client_bras.py`) pour fermer la boucle automatiquement.

---

## Actions du serveur du bras

`serveur_bras.py` accepte les actions JSON suivantes (via `client_bras.py`) :

| Action           | Effet                                                        |
|------------------|-------------------------------------------------------------|
| `etat`           | renvoie l'état courant du bras                               |
| `ranger`         | replie le bras en pose de repos                              |
| `saisir_vision`  | saisie autonome par asservissement visuel en angles         |
| `deposer_fixe`   | dépôt à la position fixe prédéfinie                          |
| `saisir` (coords)| saisie à des coordonnées données                            |
| `deposer` (coords)| dépôt à des coordonnées données                            |

---

## Environnement

- **ROS2 Foxy**, Nav2 (AMCL, DWB, costmaps), Cartographer (SLAM), CycloneDDS.
- **Perception** : YOLOv8n (`yolov8n.pt`, classes COCO), OpenCV, caméra Orbbec RGB-D.
- **Bras** : `pymycobot`, myCobot 280 Pi.
- Les poids YOLO (`*.pt`) ne sont **pas** versionnés (voir `.gitignore`) : les
  télécharger séparément.

---

## Auteur

**Tom Hurard** — étudiant ingénieur SeaTech (Toulon), parcours SYSMER.
Stage encadré par **Amornphun Phunopas** (KMUTNB) et **Cédric Anthierens** (SeaTech).
