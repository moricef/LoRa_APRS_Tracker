# State Blackboard — pattern pour rendu carte asynchrone

## Contexte

Branche `feature/core0-async-rendering` (commit `7548079`) — rendu NAV async Core 0 via queue de commandes. Gelée, non fusionnée, bugs connus (tuiles queuées mais non traitées).

**Cause architecturale probable du blocage :** envoi de commandes FIFO au Core 0 → engorgement quand le producteur va plus vite que le consommateur.

## Pattern recommandé pour la reprise

Remplacer la queue de commandes par un **State Blackboard** — une structure d'état partagée que le thread de rendu lit en boucle.

### Principe

Au lieu d'envoyer des commandes (« dessine cette tuile », « ajoute ce label »), on a une seule **structure d'état partagée** décrivant ce qu'il faut afficher maintenant :

```cpp
struct MapRenderState {
    double lat, lon;
    uint8_t zoom;
    bool nav_active;
    int centerTileX, centerTileY;
    uint32_t version;  // incrémenté à chaque update
};
```

### Producteur (thread principal — GPS/UI/LVGL)

```cpp
xSemaphoreTake(mapStateMutex, portMAX_DELAY);
mapState.lat = newLat;
mapState.lon = newLon;
mapState.zoom = newZoom;
mapState.version++;
xSemaphoreGive(mapStateMutex);
```

Coût : quelques µs sous mutex. N'attend jamais le renderer.

### Consommateur (Core 0 — rendu)

```cpp
void renderTask(void* pv) {
    MapRenderState local;
    uint32_t lastVersion = 0;
    while (true) {
        // Copie courte sous mutex
        xSemaphoreTake(mapStateMutex, portMAX_DELAY);
        local = mapState;
        xSemaphoreGive(mapStateMutex);

        // Skip si rien n'a changé
        if (local.version == lastVersion) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        lastVersion = local.version;

        // Rendu long (200-500ms), PAS sous mutex
        renderMapToSprite(local);

        // Signal LVGL : nouvelle frame prête
        spriteReadyFlag = true;
    }
}
```

### Côté LVGL (thread UI)

Un `lv_img` pointe sur le sprite PSRAM statique. Quand `spriteReadyFlag` est vrai, LVGL invalide l'objet image et redessine. Le rendu de la carte est **totalement indépendant** du cycle LVGL.

## Différence clé avec une queue de commandes

| Queue FIFO | Blackboard |
|-----------|------------|
| Producteur > consommateur → engorgement | Producteur écrase l'état, pas d'accumulation |
| Chaque commande doit être traitée | Le renderer saute les frames obsolètes |
| Ordre strict maintenu | Toujours dernier état connu affiché |
| Mémoire croît avec retard | Mémoire constante |

**Comportement attendu :** si l'utilisateur bouge le zoom 5 fois pendant que le Core 0 dessine, la 6e frame dessinera directement l'état final. Les 5 intermédiaires sont droppées (frame dropping) — exactement comme un moteur de jeu qui baisse son FPS sous charge plutôt que de freezer.

Structurellement impossible à engorger.

## Quand implémenter ce pattern

**Prérequis avant refonte :** mesurer le blocking time du rendu synchrone actuel dans un cas chargé (Z11, beaucoup de bâtiments).

- **Si blocking time < ~400ms** : le rendu synchrone tient. Ne pas sur-optimiser (KISS).
- **Si blocking time ≥ 400ms** : risque réel pour le temps-réel radio/GPS :
  - GPS 9600 baud → buffer UART 512B = ~530ms de tolérance avant overflow
  - LoRa RX : le SX1262 garde le paquet en FIFO jusqu'à lecture, mais un 2e paquet arrivant avant lecture = collision FIFO = paquet perdu
  - Risque avéré sur cartes denses → async devient nécessaire, pas cosmétique

## Cible prioritaire

Le lag au zoom est constaté sur T-Deck Plus, quasi invisible sur CrowPanel.
→ Si implémenté, c'est avant tout un fix pour T-Deck, pas pour CrowPanel.

## Références

- Branche gelée : `feature/core0-async-rendering`, commit `7548079`
- `DEBUG_ASYNC_NAV.md` à la racine — plan de debug de l'ancienne tentative
- Pattern analogue : moteurs de jeu vidéo (double buffering avec frame dropping)
