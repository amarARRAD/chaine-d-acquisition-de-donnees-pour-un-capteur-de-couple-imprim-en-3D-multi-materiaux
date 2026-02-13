# Chaîne d’acquisition pour capteur de couple capacitif imprimé en 3D (ESP32-S3 + FDC1004)

Ce dépôt regroupe le travail réalisé dans le cadre du Master 2 EEEA (Systèmes embarqués) en collaboration avec l’Inria.  
Le projet vise à rendre autonome la mesure de couple d’un capteur capacitif imprimé en 3D multi-matériaux, destiné à une intégration dans un exosquelette.

## Principe
Le capteur comporte 4 structures capacitives. Lorsqu’un couple est appliqué, la déformation mécanique provoque des variations de capacité. Ces 4 mesures sont combinées pour obtenir une grandeur différentielle, puis un angle et enfin un couple estimé.

## Architecture matérielle
- **FDC1004** : convertisseur capacité-numérique (4 canaux) avec **blindage actif** (/Blinddd) pour limiter les capacités parasites.
- **ESP32-S3-WROOM-1** : récupération des mesures via **I2C**, traitement embarqué et communication.
- **PCB circulaire 2 couches** conçu pour être monté directement sur le capteur (plan de masse + plan de blindage).
- Interfaces prévues : **UART** (programmation / debug) et **CAN** (prévu pour intégration future exosquelette).

## Logiciel embarqué
Le traitement initialement réalisé sous MATLAB a été traduit en **C++ (ESP-IDF)** sous forme de librairie :
- driver I2C FDC1004 (lecture des 4 capacités, conversion en pF)
- calcul de la capacité différentielle (`Ctot`)
- estimation d’angle et de couple (modèle statique avec hystérésis + terme dynamique)

## Contenu du dépôt
- Fichiers **KiCad** (schéma + routage + fabrication)
- Code **C++ / ESP-IDF** (librairie capteur + exemples)
- Rapport (LaTeX) et annexes (datasheets / documents)

## Perspectives
- réintégration d’une **Flash externe**
- ajout de **filtrage numérique**
- ajout de capteurs complémentaires (température, IMU) pour améliorer la robustesse en conditions réelles
