# Trace parasite sur carte — analyse en cours

## Symptômes observés

La trace GPS propre (violet `0x9933FF`) présente occasionnellement une **ligne droite parasite** qui relie deux points distants, en plus de la trace correcte suivant le trajet réel.

Caractéristiques :
- **N'apparaît pas au départ** de la session — apparaît pendant le mouvement
- Peut relier 2 points quelconques distants de **plusieurs km ou centaines de mètres**
- Peut relier le **point de départ au point d'arrivée** directement
- **Asymétrie aller/retour** : sur un A/R, peut apparaître à l'aller mais pas au retour (ou inversement)
- Les points ajoutés au fur et à mesure suivent correctement la trace et la position

## Hypothèse DeepSeek (écartée)

DeepSeek proposait que la ligne parasite soit la jonction entre :
- Les points SD chargés au boot (session précédente)
- Les points RAM récents (session courante)

→ Une ligne droite entre le dernier point SD et le premier point RAM.

**Fix proposé par DeepSeek :** supprimer tous les `trace_*.bin` dans `TraceSD::init()` au boot.

### Pourquoi cette hypothèse ne tient pas

Si c'était la jonction SD→RAM, la ligne parasite serait :
- Présente **dès l'ouverture de la carte** (pas pendant le mouvement)
- Toujours au **même endroit** (la jonction unique)
- Indépendante du mouvement

Or les symptômes réels sont inverses : apparition pendant le mouvement, lieu variable, asymétrie.

## Vérification des commits historiques

L'utilisateur se souvenait d'un travail précédent sur l'effacement des fichiers `.bin` au boot. Vérification :

- `67f6785c` — *feat: persist own trace to SD and render from SD+RAM viewport*
  - Crée `trace_sd.h/cpp`, introduit la persistance SD binaire
  - **Aucune logique de suppression au boot**
- `64e94959` — *feat: PSRAM cache for GPS traces*
  - Cache PSRAM, `loadFromSD()` au boot, croissance dynamique
  - **Aucune logique de suppression au boot**

→ Le souvenir de l'utilisateur ne correspond à aucun commit. Soit discussion non implémentée, soit commit stash/abandonné.

## Nouvelle hypothèse (à valider par observation)

**Cause probable : `readViewport()` filtre les points SD par bounding box lat/lon.**

Code concerné : `src/map/trace_sd.cpp:141-154`

```cpp
int readViewport(float minLat, float maxLat, float minLon, float maxLon,
                 TraceRecord* outBuf, int maxPoints) {
    for (int i = 0; i < cacheCount && count < maxPoints; ++i) {
        if (cache[i].lat >= minLat && cache[i].lat <= maxLat &&
            cache[i].lon >= minLon && cache[i].lon <= maxLon) {
            outBuf[count++] = cache[i];
        }
    }
}
```

Si des points du cache sortent du viewport visible, ils sont **exclus** de la liste retournée, créant un **trou** dans la séquence chronologique.

`lv_canvas_draw_line()` dessine ensuite une polyligne continue passant par tous les points restants → **ligne droite entre deux points non adjacents chronologiquement** autour du trou.

### Exemple

Trace chronologique : A → B → C → D → E

Si B et C sont hors viewport (par exemple, boucle qui sort du cadre) :
- `readViewport()` retourne : [A, D, E]
- `draw_own_trace()` dessine : A—D—E (ligne droite A→D parasite)

### Explication des symptômes

- **Pendant le mouvement** : le viewport suit l'utilisateur, donc le bounding box bouge. Certains points anciens entrent/sortent du viewport → les trous se forment et disparaissent dynamiquement.
- **Asymétrie A/R** : à l'aller, certains points d'une boucle ou d'un détour sont hors viewport → trou. Au retour, d'autres points sont concernés → trou différent ou absent.
- **Distance variable** : plus il y a de points consécutifs hors viewport, plus la ligne droite est longue.

## Test à réaliser (à la demande de l'utilisateur — observations complémentaires)

L'utilisateur veut **observer d'abord** avant de coder un fix.

### Observations à faire

1. La ligne parasite est-elle corrélée à des zooms particuliers ? (Au zoom faible, le viewport couvre plus → moins de points exclus)
2. Apparaît-elle après un détour/boucle ? (Points hors trajet principal)
3. Disparaît-elle en zoomant plus large ? (Si oui → confirme hypothèse viewport)
4. Les "2 points" reliés par la ligne parasite sont-ils toujours à la **limite** du viewport visible ?

### Fix potentiel (si hypothèse confirmée)

**Option A — ne pas filtrer par viewport** : retourner tous les points du cache depuis `readViewport()`. LVGL fera le clipping au dessin. Coût : plus de points projetés via `latLonToPixel()`.

**Option B — marge généreuse** : élargir la bounding box de X° en lat/lon pour inclure les points proches mais hors du cadre strict.

**Option C — dessin par segments** : dans `draw_own_trace()`, détecter les "gaps" chronologiques (écart temps ou distance entre points consécutifs > seuil) et casser la polyligne en plusieurs `lv_canvas_draw_line()` séparés.

## État actuel du code

- `src/map/trace_sd.cpp:141-154` — `readViewport()` filtre par bounding box
- `src/map/map_render.cpp:384-468` — `draw_own_trace()` :
  - Collecte SD points via `readViewport()` dans les bounds du sprite
  - Collecte RAM points (tous, sans filtrage)
  - Ajoute position courante
  - Dessine une seule polyligne via `lv_canvas_draw_line()`
- Aucune suppression de fichiers au boot dans `init()`
- `TRACE_RENDER_MAX = 1024` points max combinés

## Prochaines étapes

1. L'utilisateur refait des observations précises sur le terrain
2. Retour avec données (zoom, pattern de trajet, etc.)
3. Validation ou réfutation de l'hypothèse viewport
4. Implémentation du fix correspondant
