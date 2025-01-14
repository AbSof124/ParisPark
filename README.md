# ParisPark
Projet de Simulation de Parking Intelligent

---

# Projet de Simulation de Parking Intelligent

Ce projet a été développé en 2019 dans le cadre d'un post-doctorat. Il combine **Python**, **MATLAB**, et **SUMO** pour simuler des scénarios d'attribution de parkings intelligents dans un environnement urbain.

## Description du Projet

Le projet vise à optimiser l'attribution de parkings pour minimiser la distance parcourue par les véhicules. Il intègre des modules pour la communication entre MATLAB et SUMO ainsi que des configurations spécifiques pour les scénarios de simulation.

## Prérequis

Pour exécuter ce projet, vous aurez besoin de :
- **MATLAB 2019** ou une version ultérieure.
- **SUMO** (Simulation of Urban Mobility) correctement installé.
- Les fichiers de configuration pour SUMO (.net, .rou, .cfg) sont inclus dans le dossier du projet.

## Structure du Projet

- **Fichiers MATLAB** :
  - `parkingessie.m` : Script principal pour lancer une simulation spécifique à l'ESIEE.
  - `parkingparis.m` : Script principal pour un scénario spécifique à Paris.
  - Autres fichiers `.m` : Fonctions auxiliaires utilisées par les scripts principaux.

- **Modules Python** :
  - `addstoptoroute.py` : Script Python pour ajouter des arrêts aux itinéraires SUMO.
  - `generateparkinglots.py` : Génère les parkings pour le scénario simulé.

- **Fichiers SUMO** :
  - `.net` : Fichier de configuration du réseau SUMO.
  - `.rou` : Fichier définissant les itinéraires des véhicules.
  - `.cfg` : Fichier de configuration principale pour SUMO.

## Instructions d'Exécution

1. **Préparation de l'environnement** :
   - Assurez-vous que MATLAB 2019 et SUMO sont installés sur votre système.
   - Configurez correctement SUMO pour qu'il soit accessible depuis les scripts.

2. **Lancer une simulation** :
   - Pour le scénario ESIEE :
     ```bash
     matlab -r "run('parkingessie.m')"
     ```
   - Pour le scénario Paris :
     ```bash
     matlab -r "run('parkingparis.m')"
     ```

3. **Interaction avec SUMO** :
   - Les scripts Python (`addstoptoroute.py` et `generateparkinglots.py`) sont automatiquement utilisés pour relier MATLAB avec SUMO pendant les simulations.

## Notes

- **MATLAB** est utilisé pour la logique principale et la génération des scénarios.
- **SUMO** est utilisé pour la simulation de trafic et de mouvement des véhicules.
- Si nécessaire, adaptez les chemins des fichiers SUMO dans les scripts pour correspondre à votre environnement local.

## Auteurs

Ce projet a été développé par **Sofiène Abidi** dans le cadre d'un post-doctorat.

---
