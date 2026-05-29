# 24-bit colour and the deadlock saga

## How 24-bit colour was achieved

ESPHome's stock LVGL component was hard-coded to RGB565 throughout. Reaching 24-bit needed coordinated patches across four components (`display`, `lvgl`, `mipi_dsi`, `image`), all kept under `components/` as local overrides so they transplant cleanly into an upstream PR.

The end-to-end data flow:

```
LVGL (LV_COLOR_DEPTH=32, ARGB8888)
   │  4 bytes/pixel [B, G, R, A] on little-endian
   ▼
PPA SRM rotate (ARGB8888 in/out)
   │  4 bytes/pixel
   ▼
mipi_dsi::draw_pixels_at  (bitness = COLOR_BITNESS_8888)
   │  strip alpha → 3 bytes/pixel [R, G, B]
   ▼
esp_lcd_panel_draw_bitmap → RGB888 DPI framebuffer
   ▼
EK79007 panel @ 1024×600 over MIPI-DSI
```

### The patches

**1. New `ColorBitness` enum value** ([components/display/display_color_utils.h](components/display/display_color_utils.h))

The cleanest way to signal "this source buffer is 4 bytes/pixel" to *any* display driver was a new enum value alongside the existing `COLOR_BITNESS_888 / 565 / 332`:

```cpp
enum ColorBitness : uint8_t {
  COLOR_BITNESS_888 = 0,
  COLOR_BITNESS_565 = 1,
  COLOR_BITNESS_332 = 2,
  COLOR_BITNESS_8888 = 3,   // 4 bytes [B, G, R, A]; alpha byte ignored
};
```

`ColorUtil::to_color` got a fast path that ignores the alpha byte before the generic first/second/third-bits decoder runs.

**2. Base-class `draw_pixels_at` 4-byte path** ([components/display/display.cpp](components/display/display.cpp))

Just a new arm in the existing switch reading `src[0..2]` as B/G/R and packing into the same `color_value` format the rest of the function expects. This is the safety net for any non-mipi_dsi display driver that LVGL might hand 32-bit buffers to.

**3. LVGL schema** ([components/lvgl/__init__.py](components/lvgl/__init__.py))

One character:

```py
cv.Optional(CONF_COLOR_DEPTH, default=16): cv.one_of(16, 32),
```

Default stays at 16 for backward compat; existing builds are unaffected.

**4. LVGL's runtime bitness** ([components/lvgl/lvgl_esphome.h](components/lvgl/lvgl_esphome.h))

`LV_BITNESS` is the constant LVGL hands to the display via `draw_pixels_at`. At depth 32 it used to map (incorrectly) to `COLOR_BITNESS_888`. Fixed to `COLOR_BITNESS_8888`.

**5. LVGL's flush format** ([components/lvgl/lvgl_esphome.cpp](components/lvgl/lvgl_esphome.cpp))

This was the bug that caused the first round of pure-noise displays. The flush format was hard-coded to RGB565:

```cpp
lv_display_set_color_format(this->disp_, LV_COLOR_FORMAT_RGB565);
```

The obvious fix was a conditional on `LV_COLOR_DEPTH`. The *non*-obvious fix was discovering that **`LV_COLOR_FORMAT_XRGB8888` does not work** on this build:

```cpp
// lv_conf.h (auto-emitted by ESPHome):
#define LV_DRAW_SW_SUPPORT_ARGB8888 1   // enabled
#define LV_DRAW_SW_SUPPORT_XRGB8888 0   // disabled!
```

LVGL's SW renderer has no XRGB8888 path compiled in, so the buffer stayed uninitialised → the display showed whatever random bytes PSRAM happened to contain (purple noise after a fresh boot, leftover RGB565 data otherwise). Switching to `LV_COLOR_FORMAT_ARGB8888` was the unlock — identical memory layout `[B, G, R, A]`, only the meaning of the 4th byte differs, and our strip path ignores it either way.

**6. mipi_dsi strip path** ([components/mipi_dsi/mipi_dsi.cpp](components/mipi_dsi/mipi_dsi.cpp))

When `bitness == COLOR_BITNESS_8888 && color_depth_ == COLOR_BITNESS_888`, walk the source `w × h × 4` buffer into a reusable PSRAM staging buffer at `w × h × 3` (alpha byte dropped, RGB byte order obeys the panel's `color_mode_`), then one bulk `esp_lcd_panel_draw_bitmap`. The staging buffer is allocated via `heap_caps_malloc(..., MALLOC_CAP_SPIRAM)` and grown on demand so we don't burn DRAM on the ~1.8MB worst case.

**7. Image format mapping** ([components/image/image.cpp](components/image/image.cpp))

`IMAGE_TYPE_RGB` image *data* on disk is always packed RGB888 (or ARGB8888 with alpha) regardless of `LV_COLOR_DEPTH`. The original upstream code had inverted `#if LV_COLOR_DEPTH == 32` guards using LVGL-v8 names that aren't even defined at depth 32. My first patch fixed the LVGL-v8 names but kept the depth conditional and mapped no-alpha RGB to `LV_COLOR_FORMAT_XRGB8888` — which both lied about the stride (3 bytes data, 4 declared) and hit the same disabled-XRGB blender path. Final fix: drop the `#if` entirely, always map RGB→RGB888 and RGB-with-alpha→ARGB8888.

---

## How the deadlock was resolved

The recurring crash signature was diagnostic gold: WDT fires at exactly 30s, both cores at `esp_cpu_wait_for_intr` (idle), `loopTask did not reset the watchdog`. That's never a busy loop — it's `loopTask` blocked on a primitive with `portMAX_DELAY`.

This took six iterations to nail down.

### Iteration 1 — per-row strip blocked the loop task

First strip implementation submitted **one DMA per row** of the source area, each followed by `xSemaphoreTake(io_lock_, portMAX_DELAY)` waiting on the DPI panel's `on_color_trans_done` callback. Easy deadlock:

```cpp
err = esp_lcd_panel_draw_bitmap(...);   // returns ESP_ERR_INVALID_STATE under burst
xSemaphoreTake(io_lock_, portMAX_DELAY); // ← waits for a callback that never fires
```

Fix: single bulk DMA + check `err == ESP_OK` before the take.

### Iteration 2 — binary semaphore desync

Even after the err-check, the WDT kept firing. Tracing through the IDF code revealed the binary `io_lock_` semaphore desyncs under bursty calls:

1. Submit A returns `ESP_OK`. We wait, callback fires, we wake. ✓
2. Submit B returns `ESP_ERR_INVALID_STATE` because A's DMA is still in flight. We log + bail.
3. A's DMA finishes later, callback fires, `io_lock_` count: 0→1.
4. Submit C succeeds. New DMA queued.
5. `xSemaphoreTake(io_lock_, portMAX_DELAY)` returns **immediately** consuming the stale give from step 3 — even though C's DMA isn't done. Buffer corruption, and eventually a real desync where the count is permanently 0 and no future give lands → WDT.

Fix: drain-stale-then-bounded-wait pattern:

```cpp
while (xSemaphoreTake(io_lock_, 0) == pdTRUE) { /* discard */ }
err = esp_lcd_panel_draw_bitmap(...);
if (err != ESP_OK) return;
if (xSemaphoreTake(io_lock_, pdMS_TO_TICKS(100)) != pdTRUE) {
  ESP_LOGW(...);  // never wait forever, never trip WDT from here
}
```

Both the drain (guarantees the semaphore is 0 before submit, so the only give from here on is *our* DMA's) and the 100ms ceiling matter.

### Iteration 3 — PPA's `BLOCKING` mode was the *real* hang

After the strip path was bounded, the WDT *still* fired with the same signature. Reading the IDF source for `ppa_do_scale_rotate_mirror` showed why: in `PPA_TRANS_MODE_BLOCKING` it does an unbounded `xSemaphoreTake(..., portMAX_DELAY)` on an internal semaphore. Under DMA2D contention between PPA and the DPI panel, that wait could effectively become unbounded.

Fix in [components/lvgl/lvgl_esphome.cpp](components/lvgl/lvgl_esphome.cpp): converted PPA to `PPA_TRANS_MODE_NON_BLOCKING` + per-client `on_trans_done` callback that gives a dedicated `ppa_done_sem_` from the ISR, plus the same drain-stale-then-bounded-wait pattern wrapping `ppa_do_scale_rotate_mirror`. After this, the crashes stopped.

### Iteration 4 — IDF transaction-slot queue exhaustion

The new non-blocking PPA worked, but produced spammy warnings:

```
E (...) ppa_srm: exceed maximum pending transactions for the client, consider increase max_pending_trans_num
[W][lvgl]: PPA rotation submit failed: ESP_FAIL
```

PPA's client config had `max_pending_trans_num = 1`. Under rapid LVGL flushes, the IDF transaction queue would momentarily be empty between the previous transaction's ISR recycle and the next submit — basically a TOCTOU window where the slot hasn't been returned yet. Bumped to 4 to give it headroom.

### Iteration 5 — saturation backoff for sustained burst load

Even at 4 slots, sustained rapid drags eventually starved the queue (4 consecutive PPA completion timeouts → all 4 slots stuck in flight → ESP_FAIL on every subsequent submit). The root cause is **hardware contention**: PPA and the DPI panel both use DMA2D. The DPI panel's async fbcpy hogs the bus during heavy LVGL rendering and PPA completions back up.

Final defence: explicit saturation backoff with rate-limited logging. On *any* PPA failure (timeout or `ESP_FAIL`), set a `ppa_skip_until_ms_` deadline 500ms in the future. While saturated, `ppa_rotate_` returns `false` immediately and `draw_buffer_` uses its existing CPU-rotation fallback. PPA's in-flight stuck transactions drain naturally during the cooldown; the next call after `ppa_skip_until_ms_` retries PPA. Warning is throttled to once per second.

Per-call PPA timeout also tightened from 100ms → 50ms, since a healthy PPA SRM finishes in single-digit ms; anything over 50ms is saturation, not slowness.

### Iteration 6 — in-flight gate eliminates the IDF error at its source

Iterations 4–5 reduced the `exceed maximum pending transactions` noise but never killed it: the saturation backoff only kicks in *after* a failed submit, and that failed submit is exactly what makes IDF log the error. `ppa_do_scale_rotate_mirror` emits its own `ESP_LOG_ERROR` line whenever its slot queue is empty at submit time — we can't suppress it once we've called the function. So under sustained contention the system would still oscillate (retry every 500ms → IDF logs error → `ESP_FAIL` → cooldown), spamming the log on every recovery attempt even though the CPU-rotation fallback kept the UI alive.

The fix is to **never submit when the queue is full**, by tracking outstanding transactions ourselves. The correctness hinges on IDF's ISR ordering in [esp_driver_ppa/src/ppa_core.c](https://github.com/espressif/esp-idf) `ppa_transaction_done_cb`:

```c
need_yield |= ppa_recycle_transaction(client, trans_elm);  // slot returned to queue FIRST
...
need_yield |= done_cb(client, &edata, trans_elm_user_data); // OUR callback fires AFTER
```

IDF returns the slot to its `trans_elm_ptr_queue` *before* invoking our `on_trans_done` callback. So if we:

- increment `ppa_inflight_` (a `std::atomic<uint8_t>`) right after a successful submit, and
- decrement it from our completion ISR (`ppa_trans_done_cb`, which now receives the component via `srm_config.user_data` instead of the bare semaphore),

then `ppa_inflight_` is **always ≥ the number of slots IDF still has checked out** (our decrement runs strictly after IDF's recycle). Gating submission on `ppa_inflight_ < PPA_MAX_PENDING_TRANS` therefore guarantees a free slot exists at submit time — `xQueueReceive` inside `ppa_do_scale_rotate_mirror` can't fail, so the error log can't fire. When the gate trips we silently fall back to CPU rotation; no failed submit, no spam.

```cpp
// ppa_rotate_, after the cooldown check:
if (this->ppa_inflight_.load(std::memory_order_relaxed) >= PPA_MAX_PENDING_TRANS)
  return false;                                  // queue full → CPU fallback, no IDF call
...
ppa_do_scale_rotate_mirror(...);                 // guaranteed a slot
this->ppa_inflight_.fetch_add(1, ...);           // count it until the ISR decrements
```

This also makes **recovery cleaner and automatic**. Previously the system re-attempted PPA on a fixed 500ms timer and hoped a slot was free (often it wasn't → more spam). Now re-acceleration is gated on real slot availability: as stalled transactions complete, their ISR decrements `ppa_inflight_`, and the moment it drops below the limit the next flush re-engages PPA. Full hardware rotation resumes as soon as the count reaches 0. There's no permanent-leak risk — every `ESP_OK` submit is a queued transaction that *will* complete and fire the ISR decrement, so the counter always drains.

`max_pending_trans_num` (and the gate threshold) live in one shared `PPA_MAX_PENDING_TRANS = 4` constant. The cooldown/timeout defences from iteration 5 are retained as a second layer for the genuine-stall case.

---

## Final pipeline characteristics

| | RGB565 / depth=16 (previous) | ARGB8888 / depth=32 (now) |
|---|---|---|
| LVGL framebuffer | 2 bytes/pixel | 4 bytes/pixel |
| DPI framebuffer | RGB565 | RGB888 |
| Rotation | PPA (RGB565 SRM) | PPA (ARGB8888 SRM) with SW-rotation fallback |
| Conversion to DPI | none — direct blit | per-frame alpha-strip in `mipi_dsi::draw_pixels_at` |
| Effective colour depth | 5-6-5 (banding visible on gradients) | 8-8-8 |
| Worst-case loop-task block | unbounded (`portMAX_DELAY`) | 50ms (PPA wait) + 100ms (strip wait) per flush, never both unbounded |
| Behaviour under DMA2D contention | n/a — light enough that contention didn't matter | in-flight gate skips PPA while its slots are busy (no IDF error), graceful CPU-rotation fallback, auto re-acceleration as slots drain |

The whole thing is structured so that each `components/<name>/` directory is a drop-in copy of upstream `esphome/components/<name>/` with a tightly-scoped diff — a single squash-rebase moves all of this into an esphome fork ready for PR.