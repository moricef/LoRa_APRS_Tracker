# LLM Task Tracker

Suivi des demandes faites au LLM pour vérifier qu'elles ont été correctement accomplies.
Mis à jour à chaque session. Le LLM DOIT consulter ce fichier en début de session
et mettre à jour l'état des tâches en cours.

## Légende

- [ ] A faire
- [~] En cours / partiellement fait
- [x] Fait et validé sur device
- [!] Régression ou problème détecté

## Session 2026-03-22 — branche feature/rewrite-nav-render

### Rendu NAV

- [x] Réécriture renderSingleFeature() — transposition fidèle IceNav (3 fonctions + dispatch)
- [x] Suppression proj16X/proj16Y (code mort)
- [x] Semantic culling différencié : polygons < 3x3px, lines < 2x2px
- [x] Pool cap 16384 → 20000
- [x] Suppression multiplicateur widthF *= 1.05f (inutile)
- [x] Single-pass dispatch (suppression phase COUNT redondante)
- [x] Réduction polices labels VLW : 0.65/0.8/1.0x (était 0.8/1.0/1.2x)
- [x] Compensation drawWideLine : widthF - 0.5f (compense +0.5 interne LovyanGFX)
- [x] Suppression guard PSRAM 400KB fixe → pattern IceNav alloc/evict/skip
- [x] Fix spam refresh timer (applyRenderedViewport en boucle offset 0,0)

### Zoom buttons

- [x] Queue zoom pendant rendu (pendingZoom flag, consommé dans applyRenderedViewport)

### GPS filter

- [x] Seuil jitter trop agressif pour marche : 8km/h→4, 25m→10m

### A faire

- [x] Pool NAV switchable raster↔NAV : 5×512KB slots, fallback ps_malloc, cleanup map exit. Gemini 3.1 Pro + Claude Opus review/fix.
- [x] Filtre GPS : anti-spike lastValidTime + lissage marche alpha≤0.5 + DP epsilon 0.0001
- [x] Régénérer tuiles NAV avec widths ajustées après calibration drawWideLine
- [ ] Logs série entrelacés Core0/Core1 (cosmétique, pas bloquant)

### IceNav devel_new_render — à transposer (commits 0117f125 + e743b4ae)

- [ ] LOD adaptatif lignes : Manhattan distance, 3px Z15+, 2px Z13+
- [ ] Fast-path drawLine pour lignes fines (évite drawWideLine overhead)
- [ ] Casing culling sélectif : pas de casing si priority < 13
- [ ] Pre-calcul attributs hors boucle dans renderNavLineString
- [ ] Cache clearing explicite au changement de zoom

### Mémoire / Coexistence
- [x] Fix BLE crash on init/advertising (Malloc failed)
- [x] Déplacer FreeRTOS task stacks (Render/Preload) en PSRAM 
- [x] Déplacer objets C++ LGFX_Sprite en PSRAM (placement new)
- [x] Mettre les buffers FreeRTOS Queues de la map en PSRAM
- [x] Réduire Arduino loopTask stack size à 6144
- [x] Support seamless hot-switching WiFi ↔ Map ↔ BLE sans reboot
