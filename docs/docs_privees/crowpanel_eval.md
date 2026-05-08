# Évaluation Code pour Variante Crowpanel Advance 3.5

**Date :** 2026-04-07  
**Source :** Branche `devel-features` (T-Deck Plus)  
**Cible :** Crowpanel Advance 3.5 (ESP32-S3, SPI ILI9488, 480×320)

---

## CORRECTION IMPORTANTE

❌ **Erreur dans l'évaluation précédente :** J'ai mentionné TFT_eSPI. **Ce n'est pas utilisé.**

✅ **Réalité :** Le code utilise déjà **LovyanGFX** (voir `src/display.cpp` ligne 1 : `#include "LGFX_TDeck.h"`). TFT_eSPI n'existe nulle part dans ce projet.

Le Crowpanel aussi utilise **LovyanGFX** (voir repo async `include/LGFX_CrowPanel_35.h`).

---

## 1. COMPARAISON T-DECK PLUS vs CROWPANEL ADVANCE 3.5

### Similitudes (✅ Facilité migration)
| Aspect | T-Deck Plus | Crowpanel | Statut |
|--------|-------------|-----------|--------|
| **MCU** | ESP32-S3 | ESP32-S3 | ✅ Identique |
| **Affichage API** | LovyanGFX | LovyanGFX | ✅ MÊME BIBLIOTHÈQUE |
| **Display Driver** | ST7789 | ILI9488 | ✅ Les deux supportés par LovyanGFX |
| **Display Interface** | SPI | SPI | ✅ Identique |
| **Touch** | FT6236 (I2C) | GT911 (I2C) | ✅ Même protocole I2C |
| **LoRa** | SX1262 (SPI) | SX1262 (SPI) | ✅ Identique |
| **GPS** | NEO-M9N (UART) | Externe (UART) | ✅ Compatible |
| **SD Card** | SD (SPI) | SD (SPI) | ✅ Même FS |
| **LittleFS** | Oui | Oui | ✅ Compatible |
| **PSRAM** | 8MB | Oui | ✅ Probable |

### Différences (⚠️ Points adaptation)
| Aspect | T-Deck Plus | Crowpanel | Impact |
|--------|-------------|-----------|--------|
| **Résolution écran** | 320×240 | 480×320 | 🟡 MOYEN (LVGL scaling) |
| **Pins SPI écran** | SCLK=40, MOSI=41, DC=? | SCLK=42, MOSI=39, DC=41 | 🟢 Juste remplacer pins |
| **Pin backlight** | 42 | 38 | 🟢 Juste remplacer |
| **Touch IC** | FT6236 (0x38) | GT911 (0x5D) | 🟡 Changer I2C addr + classe |
| **Touch pins** | I2C SDA/SCL | GPIO 15/16 | 🟢 À configurer |
| **GPIO Disponibles** | Limités (joystick) | Plus libres | 🟢 Bonus |
| **Joystick** | Oui (5 directions) | Non | 🟡 À désactiver |
| **Audio (I2S)** | Oui (speaker/mic) | Pas mentionné | 🟡 À valider |

---

## 2. ARCHITECTURE AFFICHAGE (SIMPLE)

### Code actuel (devel-features)

```cpp
// src/display.cpp ligne 1-2
#ifdef HAS_TFT
    #include "LGFX_TDeck.h"
    LGFX_TDeck tft;
    LGFX_Sprite sprite(&tft);
#endif
```

### Approche de migration (très simple)

```cpp
// src/display.cpp
#ifdef HAS_TFT
    #ifdef TTGO_T_DECK_PLUS
        #include "LGFX_TDeck.h"
        LGFX_TDeck tft;
    #elif defined(CROWPANEL_ADVANCE_35)
        #include "LGFX_CrowPanel_35.h"
        LGFX_CrowPanel_35 tft;
    #endif
    LGFX_Sprite sprite(&tft);
#endif
```

**C'est tout.** LovyanGFX gère automatiquement les différences d'écran une fois que la classe LGFX_* est correctement configurée.

### Où ajouter LGFX_CrowPanel_35.h

Copier du repo async vers devel-features :

```
async/include/LGFX_CrowPanel_35.h → devel-features/include/LGFX_CrowPanel_35.h
```

---

## 3. SCALING LVGL (SEUL VRAI PROBLÈME)

### T-Deck Plus

```
LV_HOR_RES_MAX=320
LV_VER_RES_MAX=240
```

### Crowpanel

```
LV_HOR_RES_MAX=480
LV_VER_RES_MAX=320
```

**Impact :** L'UI LVGL s'adapte automatiquement, mais :
- Les éléments seront plus grands/espacés (plus de pixels)
- Les fonts Montserrat seront les mêmes size physiques mais différentes résolutions
- Les layouts des boutons/menus devront peut-être être repensés pour la nouvelle forme (480 large au lieu de 320)

**Effort :** 🟡 MOYEN — Il faudra tester visuellement et ajuster les positions si nécessaire.

---

## 4. TOUCHER (MOYEN)

### Code actuel : FT6236

Le code utilise probablement `TouchLib` ou une classe wrapper. À chercher dans :
- `src/touch_utils.cpp`
- `src/touch.cpp`

Cherchons :
```bash
grep -r "FT6236\|TouchLib" src/ include/
```

**À faire :**
1. Créer une classe wrapper pour les deux contrôleurs (FT6236 vs GT911)
2. Ou utiliser compilation conditionnelle :

```cpp
#ifdef CROWPANEL_ADVANCE_35
    // Utiliser GT911
#else
    // Utiliser FT6236
#endif
```

---

## 5. JOYSTICK (TRÈS FAIBLE)

### T-Deck Plus

Code dans `src/joystick_utils.cpp` avec `#ifdef HAS_JOYSTICK`.

### Crowpanel

Pas de joystick → juste commenter :

```cpp
// #define HAS_JOYSTICK  (dans board_pinout.h)
```

L'UI tactile fonctionne déjà sans joystick.

---

## 6. AUDIO I2S (À VALIDER)

Vérifier si Crowpanel a speaker/mic. Si non :

```cpp
// #define HAS_I2S  (dans board_pinout.h)
```

---

## 7. CHECKLIST ADAPTATION

### CRITICAL

- [ ] Copier `LGFX_CrowPanel_35.h` du repo async
- [ ] Adapter `src/display.cpp` (ajout `#elif CROWPANEL_ADVANCE_35`)
- [ ] Adapter `platformio.ini` (résolutions LVGL 480×320)
- [ ] Adapter `board_pinout.h` (pins SPI, touch I2C, désactiver joystick)
- [ ] Test compilation

### MEDIUM

- [ ] Adapter touch controller (FT6236 → GT911)
- [ ] Test UI scaling (éléments bien positionnés ?)
- [ ] Test tactile (calibration 4 coins)

### LOW

- [ ] Vérifier audio présent/absent
- [ ] Optimiser positions UI si nécessaire

---

## 8. EFFORT D'ADAPTATION GLOBAL

| Phase | Complexité | Temps |
|-------|-----------|-------|
| Setup variant | 🟢 Faible | 15 min |
| Display config | 🟢 Faible | 15 min |
| Touch adaptation | 🟡 Moyen | 45 min |
| UI scaling test | 🟡 Moyen | 1h |
| **TOTAL** | | **~2–2.5h** |

**C'est BEAUCOUP plus simple que l'évaluation précédente.**

Raison : **LovyanGFX gère déjà tout.** Il suffit juste de changer la classe LGFX_* et les pins.

---

## 9. FICHIERS À MODIFIER

```
Copier :
  async/include/LGFX_CrowPanel_35.h → include/LGFX_CrowPanel_35.h

Créer :
  variants/crowpanel_advance_35/
  ├── board_pinout.h        (copier du async, adapter pins)
  └── platformio.ini         (copier du async, adapter flags)

Modifier :
  src/display.cpp           (ajout #elif CROWPANEL_ADVANCE_35)
  src/touch_utils.cpp       (support GT911 si nécessaire)
```

---

## 10. PLAN MIGRATION (RÉALISTE)

### Étape 1 : Setup (15 min)
- [ ] Copier `LGFX_CrowPanel_35.h`
- [ ] Copier variant Crowpanel (board_pinout.h, platformio.ini)
- [ ] Adapter pins si nécessaire (vérifier avec schéma)

### Étape 2 : Display (15 min)
- [ ] Modifier `src/display.cpp` (ajout branche CROWPANEL)
- [ ] Build test

### Étape 3 : Touch (45 min)
- [ ] Vérifier ou adapter touch driver (FT6236 vs GT911)
- [ ] Test tactile

### Étape 4 : Test UI (1h)
- [ ] Boot
- [ ] Navigation menus
- [ ] Vérifier positions éléments
- [ ] Ajuster si nécessaire

---

## 11. CONCLUSION

**Niveau de complexité réel : 🟢 FAIBLE–MOYEN (2–2.5h)**

**NOT a rewrite.** Juste :
1. Changer la classe LGFX_* (déjà écrite dans le repo async)
2. Adapter 3-4 pins
3. Changer I2C address du touch (0x38 → 0x5D)
4. Test UI scaling

**Avantages :**
- ✅ Code LovyanGFX identique (API universelle)
- ✅ Variant déjà existe dans async (copy-paste)
- ✅ Tout le code LoRa/GPS/SD est 100% réutilisable
- ✅ Écran SPI → pas de problème coexistence bus

**Recommandation :** Démarrer dès réception Crowpanel. ~2.5h pour avoir UI fonctionnelle.

---

**Rapport corrigé :** 2026-04-07 10:15
