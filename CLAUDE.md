# Claude Code Context

## Project

LoRa APRS Tracker - Fork with LVGL touchscreen UI for T-Deck Plus

## Versioning

**Format:** vMAJOR.MINOR.PATCH (semver)

- MAJOR: Breaking changes
- MINOR: New features
- PATCH: Bug fixes

Current: **v1.4.0**

See `VERSIONING.md` for full details and history.

## Key Files

- `src/lvgl_ui.cpp` - LVGL initialization + navigation (~730 lines)
- `src/ui_dashboard.cpp` - Dashboard screen
- `src/ui_messaging.cpp` - Messages, conversations, contacts, compose, frames, stats
- `src/ui_settings.cpp` - Settings screens (callsign, display, sound, wifi, speed)
- `src/ui_popups.cpp` - Popups TX/RX/notifications
- `src/ui_map_manager.cpp` - Map functionality
- `src/msg_utils.cpp` - Message handling
- `docs/index.html` - Web flasher (update version here)
- `docs/firmware/` - Compiled binaries for web flasher

## Rôles

- **Claude** : Écriture et modification du code uniquement. **NE JAMAIS compiler**.
- **Utilisateur** : Compilation et upload. C'est l'utilisateur qui compile, pas Claude.
  - Compilation : `~/.platformio/penv/bin/pio run -e ttgo_t_deck_plus_433`
  - Upload : `~/.platformio/penv/bin/pio run -e ttgo_t_deck_plus_433 -t upload`

## Méthodologie debug

Quand un bug visuel ou comportemental est signalé :

1. **Diagnostiquer AVANT de coder.** Émettre une hypothèse claire. Proposer un test minimal pour la valider (désactiver une feature, ajouter un log, simplifier le rendu). Ne pas coder de fix tant que la cause n'est pas isolée.
2. **Maximum 2 tentatives de fix.** Si le 2e fix ne résout pas le problème, s'arrêter. Repenser le diagnostic depuis zéro. Ne pas empiler des workarounds.
3. **Ne pas committer du code de debug ou des tentatives non validées.** Ne committer que des fixes confirmés par l'utilisateur après test sur le matériel.
4. **Lire et comprendre les primitives avant de les utiliser.** Exemple : avant d'utiliser `drawWideLine`, comprendre exactement ce qu'elle dessine (capsule ? rectangle ? anti-aliasing ?). Ne pas deviner.
5. **Lire le code source des primitives LovyanGFX avant de les utiliser.** Certaines fonctions (`drawWideLine`, `draw_wedgeline`) modifient l'état global (`clipRect`) en interne. Toujours vérifier les effets de bord dans `.pio/libdeps/.../LovyanGFX/src/lgfx/v1/LGFXBase.cpp`.
6. **Ne jamais plafonner silencieusement un conteneur.** Si un vecteur utilise `PSRAMAllocator`, le laisser grandir. Un OOM PSRAM est préférable à un rendu tronqué sans erreur visible.
7. **Analyser les propositions externes (Gemini, ChatGPT, etc.) avec esprit critique.** Ne pas les appliquer aveuglément. Identifier ce qui est correct, ce qui est faux, et ce qui manque.
8. **Ne jamais refactorer un chemin de rendu sans porter TOUTE la logique.** Si du code est déplacé d'un endroit à un autre (ex: labels séparés dans un pass textRefs), vérifier que toutes les fonctionnalités (font VLW, scaling, collision, etc.) sont reprises. Le dead code laissé derrière est un bug silencieux.
9. **Road casing abandonné.** Les primitives LovyanGFX (drawWideLine, fillCircle+drawLine) ne permettent pas un casing propre sur ESP32. Ne pas retenter — utiliser drawWideLine simple comme IceNav-v3.

## Versioning dates

- `src/LoRa_APRS_Tracker.cpp` → `versionDate` : date du firmware original CA2RXU. **NE PAS MODIFIER.**
- `include/ui_common.h` → `UI_VERSION` et `UI_VERSION_DATE` : version et date de l'UI LVGL du fork. Mettre à jour ici.

## Invariants critiques (NE PAS MODIFIER)

- **Labels texte NAV** : les GEOM_TEXT sont rendus dans le pass `textRefs` (après toute la géométrie), PAS dans le `case 4` des globalLayers. La font VLW Unicode DOIT être settée avant ce pass (`map.setFont(&vlwFont)` si `vlwFontLoaded`). Le `case 4` dans le switch des globalLayers est du dead code — ne pas y mettre de logique de rendu.
- **VLW PointerWrapper** : `vlwFontWrapper` DOIT être `static` car `VLWfont::loadFont()` stocke un pointeur `_fontData` vers le wrapper pour lire les bitmaps des glyphes à la demande. Un wrapper stack-local = dangling pointer = crash ou `|` à la place des accents.
- **Building outlines** : seuls les polygones avec `fp[4] & 0x80` (hasOutline) reçoivent un outline. Ne pas appliquer d'outline à tous les polygones.

## Workflow

1. Code changes → compile → test
2. Commit with convention: `feat:`, `fix:`, `refactor:`
3. Push to `fork` remote (not `origin`)
4. Update `docs/firmware/*.bin` and `docs/index.html` version

## Remotes

- `origin` = upstream (richonguzman/LoRa_APRS_Tracker)
- `fork` = user fork (moricef/LoRa_APRS_Tracker) ← push here

## LVGL Fonts

Polices Montserrat disponibles (à activer dans lv_conf.h si nécessaire) :

| Macro | Taille | Usage dans code |
|-------|--------|-----------------|
| LV_FONT_MONTSERRAT_12 | 12px | `&lv_font_montserrat_12` |
| LV_FONT_MONTSERRAT_14 | 14px | `&lv_font_montserrat_14` |
| LV_FONT_MONTSERRAT_16 | 16px | `&lv_font_montserrat_16` |
| LV_FONT_MONTSERRAT_18 | 18px | `&lv_font_montserrat_18` |
| LV_FONT_MONTSERRAT_20 | 20px | `&lv_font_montserrat_20` |
| ... | ... | ... |
| LV_FONT_MONTSERRAT_48 | 48px | `&lv_font_montserrat_48` |

**Note:** Seules les polices activées dans `lv_conf.h` sont disponibles à la compilation.

## Refactoring (Completed)

### Modularisation lvgl_ui.cpp ✓

| Module | Lignes | Contenu |
|--------|--------|---------|
| `ui_dashboard.cpp` | 476 | Écran principal |
| `ui_messaging.cpp` | 1676 | Messages, conversations, contacts, compose, frames, stats |
| `ui_popups.cpp` | 505 | Popups TX/RX/notifications |
| `ui_settings.cpp` | 1822 | Écrans de configuration |
| `ui_map_manager.cpp` | 1739 | Carte |
| `lvgl_ui.cpp` | 730 | Init LVGL + navigation |

**Total : ~7300 lignes** réparties en 6 modules.

## Branche courante : feature/station-gps-traces

### Session 2026-02-04 : Stats persistence + NAV parser fixes + BLE crash fix

**Commit `da841b8`** — Persistance des stats sur SD + fix parser NAV + rendering
- Sauvegarde/chargement `LinkStats` + `StationStats` (max 20) dans `/LoRa_Tracker/stats.json`
- Throttle 5 min, éviction oldest station quand plein
- **Fix NAV parser** : `ringCount` lu comme `uint16` (était `uint8`, ne matchait pas le générateur Python)
- **Fix ringEnds offset** : `+2` au lieu de `+1` dans les deux fonctions de rendu
- **Rendering fixes** : `drawLine` pour width ≤ 2 (évite round caps de `drawWideLine`)
- Police map labels : `FreeSans9pt7b` (~13px) au lieu du bitmap 6×8

**Commit `7b2f597`** — Fix crash BLE eco mode + display eco
- Déplacement de `BLEDevice::init()` hors du callback LVGL (flag différé + traitement dans main loop)
- Fix crash quand Settings > BLE avec display eco activé (stack overflow LVGL)

**Commit `25e122e`** — Fix SD logger + stats JSON + police 7pt
- **Fix boucle infinie** : `rotateLogs()` appelait `log()` qui rappelait `rotateLogs()` → stack overflow
- Augmentation buffer JSON stats : 2048 → 4096 octets (fix `NoMemory`)
- Police custom `FreeSans7pt7b` (~10px) générée avec Adafruit fontconvert pour labels carte plus petits

### Bugs connus

**Sprite viewport non persistant → fragmentation PSRAM** (non critique, workaround en place)
- Le sprite viewport (600×520, ~624KB) est libéré/recréé à chaque cycle carte
- Après plusieurs cycles, `createSprite()` peut échouer (fragmentation PSRAM)
- Pattern Jordi (IceNav-v3) : allouer une fois, jamais libérer

### Principes de conception mémoire (à respecter impérativement)

Tirés de l'analyse du code IceNav-v3 de Jordi :

1. **Sprites persistants** — allouer une seule fois, ne jamais libérer/recréer. Évite la fragmentation PSRAM.
2. **Tout en PSRAM** — les gros buffers (sprites, canvas, cache tuiles) en PSRAM. La DRAM (~53KB libre au boot) est réservée à l'OS, WiFi, et petites structures.
3. **Allocation/libération dans le même scope** — les buffers temporaires (données NAV) doivent être alloués et libérés dans la même fonction.
4. **Pas d'objets LVGL pour les éléments de carte** — dessin direct sur sprite/canvas (`fillCircle`, `pushImage`, `lv_canvas_draw_img`, etc.).
5. **Culling** — bounds check avant chaque dessin pour ne pas rendre hors écran.

### LVGL API — champs vérifiés

**`lv_draw_label_dsc_t`** (fichier `.pio/libdeps/ttgo_t_deck_plus_433/lvgl/src/draw/lv_draw_label.h`) :
- Champs existants : `font`, `color`, `opa`, `sel_start`, `sel_end`, `sel_color`, `sel_bg_color`, `line_space`, `letter_space`, `ofs_x`, `ofs_y`, `bidi_dir`, `align`, `flag`, `decor`, `blend_mode`
- **N'EXISTE PAS** : `bk_color`, `bk_opa` — ne jamais utiliser ces champs

Pour un fond derrière du texte sur canvas : utiliser `lv_canvas_draw_rect()` puis `lv_canvas_draw_text()` par-dessus.

### Logs mémoire

- `src/LoRa_APRS_Tracker.cpp` : log mémoire toutes les 10 secondes (DRAM, PSRAM, largest DRAM block)
- `src/ui_dashboard.cpp` : log avant entrée dans la carte
- `src/ui_map_manager.cpp` : log après sortie de la carte

### Branche feature/core0-async-rendering

Travail préservé sur le rendu NAV asynchrone Core 0 (commit `7548079`). Voir `DEBUG_ASYNC_NAV.md` à la racine pour le plan de debug. Ne pas fusionner — contient des bugs non résolus (tuiles NAV queueées mais non traitées par Core 0).
