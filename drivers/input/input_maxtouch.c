#define DT_DRV_COMPAT microchip_maxtouch

#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/logging/log.h>

#include "input_maxtouch.h"

LOG_MODULE_REGISTER(maxtouch, CONFIG_INPUT_LOG_LEVEL);

static int mxt_seq_read(const struct device *dev, const uint16_t addr, void *buf,
                        const uint8_t len) {
    const struct mxt_config *config = dev->config;

    const uint16_t addr_lsb = sys_cpu_to_le16(addr);

    return i2c_write_read_dt(&config->bus, &addr_lsb, sizeof(addr_lsb), buf, len);
}

static int mxt_seq_write(const struct device *dev, const uint16_t addr, void *buf,
                         const uint8_t len) {
    const struct mxt_config *config = dev->config;
    struct i2c_msg msg[2];

    const uint16_t addr_lsb = sys_cpu_to_le16(addr);
    msg[0].buf = (uint8_t *)&addr_lsb;
    msg[0].len = 2U;
    msg[0].flags = I2C_MSG_WRITE;

    msg[1].buf = (uint8_t *)buf;
    msg[1].len = len;
    msg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

    return i2c_transfer_dt(&config->bus, msg, 2);
}

static inline bool is_t100_report(const struct device *dev, int report_id) {
    const struct mxt_config *config = dev->config;
    struct mxt_data *data = dev->data;

    return (report_id >= data->t100_first_report_id + 2 &&
            report_id < data->t100_first_report_id + 2 + config->max_touch_points);
}

// --- Gesten / Mausemulation --------------------------------------------------------------
// Einheiten: ~49 Counts pro mm (2048 Counts ueber 42 mm, siehe Kconfig.shield)
#define MXT_TAP_MAX_MS 250      // Tap: DOWN..UP kuerzer als das
#define MXT_TAP2_MAX_MS 400     // Zwei-Finger-Tap: Finger landen/heben nicht gleichzeitig
#define MXT_TAP2_IGNORE_MOVE_MS 250 // so kurze Zwei-Finger-Tips zaehlen trotz Schwerpunktsprung
#define MXT_TAP_MAX_MOVE 80     // Tap: Bewegung kleiner als das (~1.6 mm)
#define MXT_TAP2_MAX_MOVE 160   // Zwei-Finger-Tap: Schwerpunkt springt beim Aufsetzen staerker
// Zwei nah beieinander liegende Finger verschmelzen im Chip zu einem Touch. Erkennung ueber
// die Kontaktflaeche: ab 24 immer, ab 16 nur wenn sie gegenueber dem Aufsetzen um die Haelfte
// gewachsen ist (ein einzelner Finger waechst beim festeren Druck deutlich weniger).
#define MXT_MERGED_AREA 24
#define MXT_MERGED_AREA_MIN 16
#define MXT_MERGED_GROWTH_NUM 3 // Faktor 3/2
// Naehert sich ein zweiter Finger, verschiebt der Chip den Schwerpunkt des einen gemeldeten
// Touches sprunghaft, bevor er ihn als eigenen Finger meldet. Die Traces (siehe
// den Traces) zeigten ~22 ms vor dem zweiten Kontakt eine einzelne Messung mit 117 bis
// 239 Counts, waehrend echte Bewegung in denselben Aufzeichnungen nie ueber ~43 Counts
// hinausgeht. Eine feste Grenze von 150 liess die Haelfte dieser Spruenge durch, deshalb
// wird stattdessen die Geschwindigkeit begrenzt: 2048 Counts entsprechen 42 mm, also
// 48.8 Counts/mm, und 27 Counts/ms sind rund 550 mm/s -- schneller wischt kein Finger.
#define MXT_JUMP_SPEED 27       // Counts pro ms (~550 mm/s)
#define MXT_JUMP_MIN 60         // Untergrenze bei sehr kurzem Messabstand
#define MXT_JUMP_LIMIT 150      // Obergrenze bei langem Messabstand
#define MXT_FAST_MOVE 20        // ab dieser Schrittweite keine Abhebe-Pufferung (Sprungquelle)
#define MXT_REPORT_INTERVAL_MS 8 // Cursor-Takt: 125 Hz, so viel traegt die BLE-Split-Strecke
#define MXT_CURSOR_WAIT_MS 150  // Cursor startet nach dieser Zeit ...
#define MXT_CURSOR_START_MOVE 48 // ... oder nach ~1 mm Weg; Bewegung davor wird verworfen (QMK)
#define MXT_SCROLL_DIV 120      // Counts pro Scroll-Schritt (~2.4 mm Fingerweg)
#define MXT_SCROLL_START_MOVE 40 // Scroll startet nach ~0.8 mm ...
#define MXT_SCROLL_START_MS 300  // ... die innerhalb dieser Zeit zusammenkommen muessen
// Abhebe-Erkennung: nur bei deutlichem Amplitudeneinbruch und nur kurz, sonst wird die
// zurueckgehaltene Bewegung als Sprung nachgeliefert (Log: bis zu 211 Counts am Stueck).
#define MXT_LIFT_DROP_PCT 80    // Amplitude unter 80 % des Mittels = moegliches Abheben
#define MXT_LIFT_AREA_PCT 75    // ... oder Flaeche unter 75 % der Flaeche beim Aufsetzen
#define MXT_LIFT_MAX_SAMPLES 5  // danach normal weiterbewegen (max. ~5 Messungen Verzug)
// Wischgesten. Nur echte Maustasten (BTN_0..BTN_4) verwenden: Tastenverhalten ueber
// zip_button_behaviors auszuloesen liess bei einem per Split angebundenen Trackpad beide
// Haelften abstuerzen.
#define MXT_SWIPE_DIST 300       // ~6 mm Mindestweg
#define MXT_SWIPE_MAX_MS 700
// Zwei-Finger-Wischer quer = Maustaste 4/5 (Safari: Seite zurueck/vor)
#define MXT_BTN_PAGE_BACK INPUT_BTN_3
#define MXT_BTN_PAGE_FORWARD INPUT_BTN_4
#define MXT_FLICK_DIST 600       // ~12 mm Mindestweg fuer den Seitenwechsel
#define MXT_FLICK_MAX_MS 400
#define MXT_CLICK_RELEASE_MS 200 // Taste nach Tap so lange halten: neuer Finger in dieser Zeit = Drag

// Momentum: nach dem Abheben laeuft das Scrollen mit abnehmender Geschwindigkeit aus
#define MXT_MOMENTUM_TICK_MS 25
#define MXT_MOMENTUM_DECAY_NUM 15   // pro Tick x 15/16 (~halbiert nach ca. 0.27 s)
#define MXT_MOMENTUM_DECAY_DEN 16
#define MXT_MOMENTUM_MIN_VEL 300    // Counts/s: darunter stoppen
#define MXT_MOMENTUM_START_VEL 600  // Counts/s: ab dieser Abhebegeschwindigkeit auslaufen lassen
#define MXT_MOMENTUM_MAX_MS 3000

static inline int16_t mxt_abs16(int16_t v) { return v < 0 ? -v : v; }
static inline int32_t mxt_abs32(int32_t v) { return v < 0 ? -v : v; }

static void mxt_emit_scroll(const struct device *dev, int16_t dx, int16_t dy) {
    struct mxt_data *data = dev->data;
    data->scroll_acc_y += dy;
    data->scroll_acc_x += dx;
    int16_t vs = data->scroll_acc_y / MXT_SCROLL_DIV;
    int16_t hs = data->scroll_acc_x / MXT_SCROLL_DIV;
    if (vs != 0) {
        data->scroll_acc_y -= vs * MXT_SCROLL_DIV;
        // vertikale Richtung invertiert (Inhalt folgt den Fingern)
        input_report_rel(dev, INPUT_REL_WHEEL, -vs, hs == 0, K_NO_WAIT);
    }
    if (hs != 0) {
        data->scroll_acc_x -= hs * MXT_SCROLL_DIV;
        input_report_rel(dev, INPUT_REL_HWHEEL, hs, true, K_NO_WAIT);
    }
}

static void mxt_momentum_stop(struct mxt_data *data) {
    if (data->momentum_active) {
        data->momentum_active = false;
        k_work_cancel_delayable(&data->momentum_work);
        LOG_INF("gesture: momentum stop");
    }
    data->scroll_vel_x = data->scroll_vel_y = 0;
}

static void mxt_momentum_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct mxt_data *data = CONTAINER_OF(dwork, struct mxt_data, momentum_work);
    if (!data->momentum_active) {
        return;
    }
    int16_t dx = (int16_t)(data->scroll_vel_x * MXT_MOMENTUM_TICK_MS / 1000);
    int16_t dy = (int16_t)(data->scroll_vel_y * MXT_MOMENTUM_TICK_MS / 1000);
    mxt_emit_scroll(data->dev, dx, dy);
    data->scroll_vel_x = data->scroll_vel_x * MXT_MOMENTUM_DECAY_NUM / MXT_MOMENTUM_DECAY_DEN;
    data->scroll_vel_y = data->scroll_vel_y * MXT_MOMENTUM_DECAY_NUM / MXT_MOMENTUM_DECAY_DEN;
    if (mxt_abs32(data->scroll_vel_x) + mxt_abs32(data->scroll_vel_y) < MXT_MOMENTUM_MIN_VEL) {
        data->momentum_active = false;
        LOG_INF("gesture: momentum end");
        return;
    }
    k_work_schedule(dwork, K_MSEC(MXT_MOMENTUM_TICK_MS));
}

static void mxt_button_release(struct mxt_data *data) {
    if (data->button_held) {
        data->button_held = false;
        data->dragging = false;
        input_report_key(data->dev, data->click_button, 0, true, K_NO_WAIT);
        LOG_INF("gesture: release");
    }
}

static void mxt_click_release_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct mxt_data *data = CONTAINER_OF(dwork, struct mxt_data, click_release_work);
    mxt_button_release(data);
}

static void mxt_click(const struct device *dev, uint16_t code) {
    struct mxt_data *data = dev->data;
    LOG_INF("gesture: button 0x%x", code);
    data->click_button = code;
    data->button_held = true;
    input_report_key(dev, code, 1, true, K_NO_WAIT);
    k_work_schedule(&data->click_release_work, K_MSEC(MXT_CLICK_RELEASE_MS));
}

// Der Chip misst im Free-Run und liefert bis zu 300 Positionen/s. Jede davon wird zu zwei
// input-Events, die der Split-Peripheral einzeln per GATT-Notification an die linke Haelfte
// schickt -- mehr als die BLE-Strecke traegt. Sobald die Verbindung nach einer Pause ein
// langsameres Connection-Interval bekommt, staut sich die Queue: der Zeiger wird langsam und
// springt. Deshalb wird die Bewegung aufsummiert und nur mit MXT_REPORT_INTERVAL_MS
// abgeschickt; es geht kein Weg verloren, nur die Paketrate sinkt.
static void mxt_flush_cursor(const struct device *dev, bool force) {
    struct mxt_data *data = dev->data;
    uint32_t now = k_uptime_get_32();
    if (!force && (now - data->last_report_ms) < MXT_REPORT_INTERVAL_MS) {
        return;
    }
    data->last_report_ms = now;
    // Bewegung wird eine Taktstufe zurueckgehalten: naehert sich ein zweiter Finger, laesst
    // sich die inzwischen gesammelte Schwerpunktverschiebung noch verwerfen, statt sie als
    // Sprung abzuschicken.
    int16_t dx = data->hold_dx, dy = data->hold_dy;
    data->hold_dx = data->pend_dx;
    data->hold_dy = data->pend_dy;
    data->pend_dx = data->pend_dy = 0;
    if (force) {
        dx += data->hold_dx;
        dy += data->hold_dy;
        data->hold_dx = data->hold_dy = 0;
    }
    if (dx == 0 && dy == 0) {
        return;
    }
    input_report_rel(dev, INPUT_REL_X, dx, false, K_NO_WAIT);
    input_report_rel(dev, INPUT_REL_Y, dy, true, K_NO_WAIT);
}

// Ein zweiter Finger ist aufgetaucht: der Chip hat den Schwerpunkt des einen gemeldeten
// Touches schon Richtung des neuen Fingers gezogen, bevor er ihn als eigenen Kontakt
// gemeldet hat. Die noch nicht abgeschickte Bewegung ist genau dieser Ruck -> wegwerfen.
static void mxt_drop_cursor(const struct device *dev) {
    struct mxt_data *data = dev->data;
    if (data->pend_dx || data->pend_dy || data->hold_dx || data->hold_dy) {
        LOG_INF("gesture: cursor drop at scroll start dx=%d dy=%d",
                data->pend_dx + data->hold_dx, data->pend_dy + data->hold_dy);
    }
    data->pend_dx = data->pend_dy = 0;
    data->hold_dx = data->hold_dy = 0;
}

static void mxt_process_touch(const struct device *dev, uint8_t idx, enum t100_touch_event ev,
                              uint16_t x_pos, uint16_t y_pos, uint8_t ampl, uint8_t area) {
    struct mxt_data *data = dev->data;
    const struct mxt_config *config = dev->config;
    if (idx >= MXT_MAX_FINGERS || !data->ready) {
        return;
    }
    bool merged = area >= MXT_MERGED_AREA;
    {
        struct mxt_finger *mf = &data->fingers[idx];
        if (!merged && area >= MXT_MERGED_AREA_MIN && mf->down_area &&
            area * 2 >= mf->down_area * MXT_MERGED_GROWTH_NUM) {
            merged = true;
        }
    }
    struct mxt_finger *f = &data->fingers[idx];
    int16_t x = (int16_t)x_pos, y = (int16_t)y_pos;
    uint32_t now = k_uptime_get_32();

    if (ev == MOVE && !f->active) {
        ev = DOWN; // DOWN verpasst (z.B. waehrend Init): ab hier tracken
    }

    switch (ev) {
    case DOWN: {
        mxt_momentum_stop(data); // neue Beruehrung stoppt das Auslaufen sofort
        bool first = (data->active_mask == 0);
        f->active = true;
        f->x = f->down_x = x;
        f->y = f->down_y = y;
        f->last_ms = now;
        f->jump_skip = 0;
        f->merged = merged;
        f->down_area = area;
        data->skip_delta = true; // Position des fuehrenden Fingers springt beim Aufsetzen
        if (!first) {
            mxt_drop_cursor(dev); // zweiter Finger: den Anlauf-Ruck nicht abschicken
            // Ab hier ist es eine Scroll-Geste: Anlauf neu messen, Akku leeren
            data->scroll_started = false;
            data->scroll_start_dx = data->scroll_start_dy = 0;
            data->scroll_start_ms = now;
            data->scroll_acc_x = data->scroll_acc_y = 0;
        } else {
            data->pend_dx = data->pend_dy = 0;
            data->hold_dx = data->hold_dy = 0;
        }
        f->ampl_sum = 0;
        f->ampl_cnt = 0;
        f->ampl_avg = 0;
        f->lift_buffering = false;
        f->lift_samples = 0;
        f->buf_x = f->buf_y = 0;
        data->active_mask |= BIT(idx);
        if (first) {
            data->gesture_start_ms = now;
            data->gesture_max_fingers = 0;
            data->gesture_moved = false;
            data->scroll_acc_x = data->scroll_acc_y = 0;
            data->scroll_last_ms = now;
            data->scroll_started = false;
            data->scroll_start_dx = data->scroll_start_dy = 0;
            data->scroll_start_ms = now;
            data->gesture_dx = data->gesture_dy = 0;
            data->swipe_fired = false;
            data->cursor_started = false;
            if (data->button_held && data->click_button == INPUT_BTN_0) {
                // Tap-and-Drag: Finger kam zurueck, solange die Taste noch gehalten wird
                k_work_cancel_delayable(&data->click_release_work);
                data->dragging = true;
                LOG_INF("gesture: drag start");
            }
        }
        uint8_t n = __builtin_popcount(data->active_mask) + (merged ? 1 : 0);
        // Waehrend eines Drags bleibt es eine Ein-Finger-Geste: geht der Platz aus, legt man
        // einen zweiten Finger auf und zieht damit weiter, statt zu scrollen. Die Zahl wird
        // dafuer auf 1 gedeckelt -- nicht die Zuweisung unterdrueckt, denn der Drag beginnt
        // in genau diesem DOWN, und ohne die 1 bliebe die Geste bei 0 und damit ohne Zweig.
        if (data->dragging && n > 1) {
            n = 1;
        }
        if (n > data->gesture_max_fingers) {
            data->gesture_max_fingers = n;
        }
        break;
    }
    case MOVE: {
        int16_t dx = x - f->x, dy = y - f->y;
        f->x = x;
        f->y = y;
        bool merge_changed = (merged != f->merged);
        f->merged = merged;
        if (merged && data->gesture_max_fingers < 2 && !data->dragging) {
            data->gesture_max_fingers = 2; // verschmolzene Finger: Scroll-Geste, kein Cursor
            mxt_drop_cursor(dev);
        }
        uint32_t jump_dt = now - f->last_ms;
        f->last_ms = now;
        int32_t jump_limit = (int32_t)MXT_JUMP_SPEED * (jump_dt == 0 ? 1 : jump_dt);
        if (jump_limit < MXT_JUMP_MIN) {
            jump_limit = MXT_JUMP_MIN;
        } else if (jump_limit > MXT_JUMP_LIMIT) {
            jump_limit = MXT_JUMP_LIMIT;
        }
        bool jumped = mxt_abs16(dx) > jump_limit || mxt_abs16(dy) > jump_limit;
        if (merge_changed || data->skip_delta || jumped || f->jump_skip) {
            // Position springt beim Trennen/Verschmelzen und bei jedem Fingerwechsel.
            // Der Ruck kommt laut den Traces als Paar: auf die grosse Messung folgt eine
            // halb so grosse in die Gegenrichtung, die unter der Grenze bliebe.
            dx = 0;
            dy = 0;
            data->skip_delta = false;
            f->jump_skip = jumped ? 1 : 0;
        }
        uint8_t n = __builtin_popcount(data->active_mask);
        uint8_t lowest = __builtin_ctz(data->active_mask);
        int16_t tap_move = data->gesture_max_fingers >= 2 ? MXT_TAP2_MAX_MOVE : MXT_TAP_MAX_MOVE;
        if (mxt_abs16(x - f->down_x) > tap_move || mxt_abs16(y - f->down_y) > tap_move) {
            data->gesture_moved = true;
        }
        if (data->gesture_max_fingers == 1 && idx == lowest) {
            // Reine Ein-Finger-Geste: Cursor. Beim Drag koennen mehrere Finger liegen --
            // den Zeiger fuehrt dann der mit dem niedrigsten Index, und hebt der ab, setzt
            // skip_delta die Uebernahme durch den naechsten sprungfrei fort.
            if (!data->cursor_started) {
                // Wie im QMK-Treiber: Bewegung am Anfang verwerfen (nicht sammeln), bis
                // Wartezeit oder Mindestweg erreicht sind. Kein Sprung beim Start.
                if (now - data->gesture_start_ms >= MXT_CURSOR_WAIT_MS ||
                    mxt_abs16(x - f->down_x) > MXT_CURSOR_START_MOVE ||
                    mxt_abs16(y - f->down_y) > MXT_CURSOR_START_MOVE) {
                    data->cursor_started = true;
                }
                break;
            }
            // Abhebe-Erkennung: Amplitude deutlich unter dem Mittel -> Bewegung zurueckhalten
            bool fast = (mxt_abs16(dx) + mxt_abs16(dy)) > MXT_FAST_MOVE;
            bool ampl_drop = !fast && f->ampl_avg && ampl * 100 < f->ampl_avg * MXT_LIFT_DROP_PCT;
            bool area_drop = !fast && f->down_area && area * 100 < f->down_area * MXT_LIFT_AREA_PCT;
            if (ampl_drop || area_drop) {
                if (!f->lift_buffering) {
                    f->lift_buffering = true;
                    f->lift_samples = 0;
                }
            }
            if (fast || (ampl >= f->ampl_avg && !area_drop) ||
                f->lift_samples >= MXT_LIFT_MAX_SAMPLES) {
                f->lift_buffering = false;
            }
            f->ampl_sum += ampl;
            f->ampl_cnt++;
            if ((!f->lift_buffering && f->ampl_cnt > 5) || f->ampl_cnt > 16) {
                f->ampl_avg = f->ampl_sum / f->ampl_cnt;
                f->ampl_sum = 0;
                f->ampl_cnt = 0;
            }
            if (f->lift_buffering) {
                f->lift_samples++;
                f->buf_x += dx;
                f->buf_y += dy;
                break;
            }
            if (f->buf_x != 0 || f->buf_y != 0) {
                LOG_INF("gesture: lift buffer released dx=%d dy=%d (ampl=%d avg=%d)", f->buf_x,
                        f->buf_y, ampl, f->ampl_avg);
            }
            dx += f->buf_x;
            dy += f->buf_y;
            f->buf_x = f->buf_y = 0;
            data->pend_dx += dx;
            data->pend_dy += dy;
            mxt_flush_cursor(dev, false);
        } else if (idx == lowest && data->gesture_max_fingers >= 3) {
            // Drei Finger: nur den Weg sammeln, Auswertung als Wischgeste beim Abheben
            data->gesture_dx += dx;
            data->gesture_dy += dy;
        } else if (idx == lowest && data->gesture_max_fingers <= 2 && (n >= 2 || merged)) {
            data->gesture_dx += dx;
            data->gesture_dy += dy;
            if (!data->scroll_started) {
                // Zwei Finger liegen, scrollen aber noch nicht. Ihr Schwerpunkt wandert dabei
                // langsam; wuerde das in den Scroll-Akku laufen, steht der beim Losscrollen
                // schon dicht an der Schrittschwelle und der erste Schritt faellt sofort und
                // an beliebiger Stelle heraus -- genau der Sprung beim Scroll-Beginn.
                // Erst ein Mindestweg innerhalb eines Zeitfensters startet das Scrollen.
                if (now - data->scroll_start_ms > MXT_SCROLL_START_MS) {
                    data->scroll_start_ms = now;
                    data->scroll_start_dx = data->scroll_start_dy = 0;
                }
                data->scroll_start_dx += dx;
                data->scroll_start_dy += dy;
                if (mxt_abs16(data->scroll_start_dx) <= MXT_SCROLL_START_MOVE &&
                    mxt_abs16(data->scroll_start_dy) <= MXT_SCROLL_START_MOVE) {
                    break;
                }
                data->scroll_started = true;
                // Nur den Weg seit dem Losscrollen uebernehmen, nicht die Drift davor.
                data->scroll_acc_x = data->scroll_start_dx;
                data->scroll_acc_y = data->scroll_start_dy;
                LOG_INF("gesture: scroll start dx=%d dy=%d", data->scroll_start_dx,
                        data->scroll_start_dy);
                data->scroll_last_ms = now;
                break;
            }
            // Zwei Finger (getrennt oder verschmolzen): Bewegung des ersten Fingers wird zu
            // Scroll-Schritten
            uint32_t dt = now - data->scroll_last_ms;
            data->scroll_last_ms = now;
            if (dt > 0 && dt < 200) {
                // Geschwindigkeit glaetten (Counts/s), Basis fuers Auslaufen nach dem Abheben
                int32_t vx = (int32_t)dx * 1000 / (int32_t)dt;
                int32_t vy = (int32_t)dy * 1000 / (int32_t)dt;
                data->scroll_vel_x = (data->scroll_vel_x * 3 + vx) / 4;
                data->scroll_vel_y = (data->scroll_vel_y * 3 + vy) / 4;
            }
            mxt_emit_scroll(dev, dx, dy);
        }
        break;
    }
    case UP: {
        mxt_flush_cursor(dev, true); // Restweg nicht bis zur naechsten Beruehrung liegen lassen
        if (!f->active) {
            break; // UP fuer einen Finger, der nicht (mehr) aktiv ist: keine zweite Gestenauswertung
        }
        f->active = false;
        data->active_mask &= ~BIT(idx);
        data->skip_delta = true; // ... und beim Abheben eines von mehreren Fingern
        if (data->active_mask == 0) {
            uint32_t dur = now - data->gesture_start_ms;
            // Alle Werte, an denen die Flick-Erkennung haengt, einmal pro Geste ausgeben:
            // damit laesst sich ablesen, welche der vier Bedingungen nicht erfuellt war.
            LOG_INF("gesture: lift fingers=%d dur=%u dx=%d dy=%d moved=%d swiped=%d drag=%d",
                    data->gesture_max_fingers, dur, data->gesture_dx, data->gesture_dy,
                    data->gesture_moved, data->swipe_fired, data->dragging);
            if (data->dragging) {
                bool second_tap = !data->gesture_moved && dur <= MXT_TAP_MAX_MS &&
                                  data->gesture_max_fingers == 1;
                LOG_INF("gesture: drag end (second_tap=%d)", second_tap);
                mxt_button_release(data);
                if (second_tap) {
                    mxt_click(dev, INPUT_BTN_0); // Doppelklick
                }
            } else if (data->gesture_max_fingers >= 3) {
                LOG_INF("gesture: 3-finger end dx=%d dy=%d dur=%u", data->gesture_dx,
                        data->gesture_dy, dur);
                if (!data->gesture_moved && dur <= MXT_TAP2_MAX_MS) {
                    mxt_click(dev, INPUT_BTN_2); // Drei-Finger-Tap = Mittelklick
                }
            } else if (data->gesture_max_fingers == 2 && !data->swipe_fired &&
                       dur <= MXT_FLICK_MAX_MS &&
                       mxt_abs16(data->gesture_dx) >= MXT_FLICK_DIST &&
                       mxt_abs16(data->gesture_dy) * 3 < mxt_abs16(data->gesture_dx)) {
                // Zwei Finger schnell quer: Seite zurueck/vor (wie am Mac)
                data->swipe_fired = true;
                LOG_INF("gesture: page %s (dx=%d dur=%u)",
                        data->gesture_dx > 0 ? "back" : "forward", data->gesture_dx, dur);
                mxt_click(dev, data->gesture_dx > 0 ? MXT_BTN_PAGE_BACK : MXT_BTN_PAGE_FORWARD);
            } else if (config->scroll_momentum && data->gesture_max_fingers >= 2 &&
                       data->gesture_moved &&
                       (mxt_abs32(data->scroll_vel_x) + mxt_abs32(data->scroll_vel_y)) >=
                           MXT_MOMENTUM_START_VEL &&
                       (now - data->scroll_last_ms) < 100) {
                // Scroll-Geste mit Schwung: auslaufen lassen
                data->momentum_active = true;
                LOG_INF("gesture: momentum start vx=%d vy=%d", data->scroll_vel_x,
                        data->scroll_vel_y);
                k_work_schedule(&data->momentum_work, K_MSEC(MXT_MOMENTUM_TICK_MS));
            } else {
                uint32_t max_ms = data->gesture_max_fingers >= 2 ? MXT_TAP2_MAX_MS : MXT_TAP_MAX_MS;
                LOG_INF("gesture: end fingers=%d moved=%d dur=%u", data->gesture_max_fingers,
                        data->gesture_moved, dur);
                bool two_finger_tap = data->gesture_max_fingers == 2 &&
                                      (!data->gesture_moved || dur <= MXT_TAP2_IGNORE_MOVE_MS);
                if (data->gesture_max_fingers == 1 && !data->gesture_moved && dur <= max_ms) {
                    mxt_click(dev, INPUT_BTN_0);
                } else if (two_finger_tap && dur <= max_ms) {
                    mxt_click(dev, INPUT_BTN_1);
                }
            }
        }
        break;
    }
    default:
        break;
    }
}

static void mxt_report_data(const struct device *dev) {
    const struct mxt_config *config = dev->config;
    struct mxt_data *data = dev->data;
    int ret;

    if (!data->t44_message_count_address) {
        return;
    }

    uint8_t msg_count = 0;
    ret = mxt_seq_read(dev, data->t44_message_count_address, &msg_count, 1);

    // Diagnose: Heartbeat ca. alle 2s (bei 8ms Polling), zeigt dass Timer + I2C laufen
    static uint32_t poll_count;
    if ((++poll_count % 250) == 1) {
        LOG_INF("poll #%u: T44 msg_count=%d (i2c ret=%d)", poll_count, msg_count, ret);
    }

    if (ret < 0) {
        LOG_ERR("Failed to read message count: %d", ret);
        return;
    }

    uint16_t pending_fingers = 0;
    bool last_touch_status = false;
    for (int i = 0; i < msg_count; i++) {
        struct mxt_message msg;

        ret = mxt_seq_read(dev, data->t5_message_processor_address, &msg, sizeof(msg));
        if (ret < 0) {
            LOG_ERR("Failed to read message: %d", ret);
            return;
        }

        // Diagnose: jede Message roh ausgeben (rid + Daten)
        LOG_DBG("msg rid=%d t100=%d data=%02x %02x %02x %02x %02x %02x", msg.report_id,
                is_t100_report(dev, msg.report_id), msg.data[0], msg.data[1], msg.data[2],
                msg.data[3], msg.data[4], msg.data[5]);

        if (is_t100_report(dev, msg.report_id)) {
            uint8_t finger_idx = msg.report_id - data->t100_first_report_id - 2;
            bool pending_for_finger = (pending_fingers & BIT(finger_idx)) != 0;

            enum t100_touch_event ev = msg.data[0] & 0xF;
            uint16_t x_pos = msg.data[1] + (msg.data[2] << 8);
            uint16_t y_pos = msg.data[3] + (msg.data[4] << 8);

            uint8_t ampl = msg.data[5];
            uint8_t area = msg.data[6];
            LOG_DBG("touch finger=%d ev=%d x=%d y=%d ampl=%d area=%d", finger_idx, ev, x_pos, y_pos,
                    ampl, area);
            // Alle Event-Typen an die Gesten: schnelle Tipps kommen als DOWNUP,
            // unterdrueckte Finger als SUP/DOWNSUP/UNSUPUP.
            switch (ev) {
            case DOWNUP:
                mxt_process_touch(dev, finger_idx, DOWN, x_pos, y_pos, ampl, area);
                mxt_process_touch(dev, finger_idx, UP, x_pos, y_pos, ampl, area);
                break;
            case UNSUPUP:
                // ausgeblendeter Finger wurde tatsaechlich abgehoben
                mxt_process_touch(dev, finger_idx, UP, x_pos, y_pos, ampl, area);
                break;
            case SUP:
            case DOWNSUP:
                // Finger liegt noch, wird aber ausgeblendet: Zustand beibehalten
                break;
            case UNSUP:
                mxt_process_touch(dev, finger_idx, MOVE, x_pos, y_pos, ampl, area);
                break;
            case DOWN:
            case MOVE:
            case UP:
                mxt_process_touch(dev, finger_idx, ev, x_pos, y_pos, ampl, area);
                break;
            default:
                break;
            }
            (void)pending_for_finger;
        } else if (msg.report_id == data->t6_command_processor_report_id) {
            // T6-Status wie im QMK-Treiber dekodieren: zeigt, wann der Chip von sich aus
            // kalibriert (CAL) und ob Signalfehler/Overflow auftreten.
            const uint8_t status = msg.data[0];
            LOG_INF("T6 status: RESET=%d OFL=%d SIGERR=%d CAL=%d CFGERR=%d COMSERR=%d",
                    (status & BIT(7)) ? 1 : 0, (status & BIT(6)) ? 1 : 0, (status & BIT(5)) ? 1 : 0,
                    (status & BIT(4)) ? 1 : 0, (status & BIT(3)) ? 1 : 0, (status & BIT(2)) ? 1 : 0);
        } else {
            LOG_HEXDUMP_DBG(msg.data, 5, "message data");
        }
    }

    (void)pending_fingers;
    (void)last_touch_status;

    return;
}

static void mxt_work_cb(struct k_work *work) {
    struct mxt_data *data = CONTAINER_OF(work, struct mxt_data, work);
    const struct mxt_config *config = data->dev->config;
    mxt_report_data(data->dev);
    if (data->irq_mode) {
        // Pegel-Interrupt: CHG bleibt low solange Nachrichten anstehen, deshalb erst nach
        // dem Leeren wieder scharf schalten.
        gpio_pin_interrupt_configure_dt(&config->chg, GPIO_INT_LEVEL_ACTIVE);
    }
}

static void mxt_gpio_cb(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct mxt_data *data = CONTAINER_OF(cb, struct mxt_data, gpio_cb);
    const struct mxt_config *config = data->dev->config;
    gpio_pin_interrupt_configure_dt(&config->chg, GPIO_INT_DISABLE);
    k_work_submit(&data->work);
}

static void mxt_poll_timer_cb(struct k_timer *timer)
{
    struct mxt_data *data = (struct mxt_data *)k_timer_user_data_get(timer);
    k_work_submit(&data->work);
}

static int mxt_load_object_table(const struct device *dev, struct mxt_information_block *info) {
    struct mxt_data *data = dev->data;
    int ret = 0;

    ret = mxt_seq_read(dev, MXT_REG_INFORMATION_BLOCK, info, sizeof(struct mxt_information_block));

    if (ret < 0) {
        LOG_ERR("Failed to load the info block: %d", ret);
        return ret;
    }

    LOG_HEXDUMP_DBG(info, sizeof(struct mxt_information_block), "info block");
    LOG_DBG("Found a maXTouch: family %d, variant %d, version %d. Matrix size: "
            "%d/%d and num of objects %d",
            info->family_id, info->variant_id, info->version, info->matrix_x_size,
            info->matrix_y_size, info->num_objects);

    data->matrix_x_size = info->matrix_x_size;
    data->matrix_y_size = info->matrix_y_size;

    uint8_t report_id = 1;
    uint16_t object_addr =
        sizeof(struct mxt_information_block); // Object table starts after the info block
    for (int i = 0; i < info->num_objects; i++) {
        struct mxt_object_table_element obj_table;

        ret = mxt_seq_read(dev, object_addr, &obj_table, sizeof(obj_table));
        if (ret < 0) {
            LOG_ERR("Failed to load object table %d: %d", i, ret);
            return ret;
        }

        uint16_t addr = sys_le16_to_cpu(obj_table.position);

        LOG_INF("obj T%-3d addr=0x%04x size=%-3d inst=%d rids=%d first_rid=%d", obj_table.type, addr,
                obj_table.size_minus_one + 1, obj_table.instances_minus_one + 1,
                obj_table.report_ids_per_instance, report_id);

        switch (obj_table.type) {
        case 2:
            data->t2_encryption_status_address = addr;
            break;
        case 5:
            data->t5_message_processor_address = addr;
            // We won't request a checksum, so subtract one
            data->t5_max_message_size = obj_table.size_minus_one - 1;
            break;
        case 6:
            data->t6_command_processor_address = addr;
            data->t6_command_processor_report_id = report_id;
            break;
        case 7:
            data->t7_powerconfig_address = addr;
            break;
        case 8:
            data->t8_acquisitionconfig_address = addr;
            break;
        case 25:
            data->t25_self_test_address = addr;
            data->t25_self_test_report_id = report_id;
            break;
        case 37:
            data->t37_diagnostic_debug_address = addr;
            break;
        case 42:
            data->t42_proci_touchsupression_address = addr;
            break;
        case 44:
            data->t44_message_count_address = addr;
            break;
        case 46:
            data->t46_cte_config_address = addr;
            break;
        case 47:
            data->t47_proci_stylus_address = addr;
            break;
        case 56:
            data->t56_proci_shieldless_address = addr;
            break;
        case 65:
            data->t65_proci_lensbending_address = addr;
            break;
        case 80:
            data->t80_proci_retransmissioncompensation_address = addr;
            break;
        case 100:
            data->t100_multiple_touch_touchscreen_address = addr;
            data->t100_first_report_id = report_id;
            break;
        }

        object_addr += sizeof(obj_table);
        report_id += obj_table.report_ids_per_instance * (obj_table.instances_minus_one + 1);
    }

    return 0;
};

static int mxt_load_config(const struct device *dev,
                           const struct mxt_information_block *information) {
    struct mxt_data *data = dev->data;
    const struct mxt_config *config = dev->config;
    int ret;

    if (data->t7_powerconfig_address) {
        struct mxt_gen_powerconfig_t7 t7_conf = {0};
        t7_conf.idleacqint = config->idle_acq_time;
        t7_conf.actacqint = config->active_acq_time;
        t7_conf.actv2idleto = config->active_to_idle_timeout;
        t7_conf.cfg = MXT_T7_CFG_ACTVPIPEEN | MXT_T7_CFG_IDLEPIPEEN; // Enable pipelining in both active and idle mode

        ret = mxt_seq_write(dev, data->t7_powerconfig_address, &t7_conf, sizeof(t7_conf));
        if (ret < 0) {
            LOG_ERR("Failed to set T7 config: %d", ret);
            return ret;
        }
    }

    if (data->t8_acquisitionconfig_address) {
        struct mxt_gen_acquisitionconfig_t8 t8_conf = {0};
        ret = mxt_seq_read(dev, data->t8_acquisitionconfig_address, &t8_conf, sizeof(t8_conf));
        LOG_HEXDUMP_INF(&t8_conf, sizeof(t8_conf), "T8 before");
        memset(&t8_conf, 0, sizeof(t8_conf));
        t8_conf.chrgtime = config->charge_time;
        // Exakt die Werte aus George Nortons QMK-Treiber (drivers/sensors/maxtouch.c,
        // Zweig multitouch_experiment): der Chip haelt seine Baseline selbst in Ordnung.
        // Drift-Kompensation bleibt aus (QMK nullt das Feld), stattdessen sorgen
        // Auto-Recal und Forced Calibration fuer die Erholung nach einer Fehlkalibrierung.
        t8_conf.tchdrift = 0;
        t8_conf.driftst = 0;
        t8_conf.tchautocal = 50;  // 10s: Recal, wenn ein Touch so lange ununterbrochen anliegt
        t8_conf.atchcalst = 0;    // Anti-Touch-Pruefung dauerhaft aktiv (QMK-Default)

        // Anti-Touch-Erkennung mit Forced Calibration: erkennt eine verdriftete Baseline
        // (negative Deltas) und kalibriert neu. War hier abgeschaltet, weil der Luftspalt
        // unter der losen Folie Recal-Loops ausgeloest hat -- die Folie ist inzwischen
        // flaechig verklebt, also QMK-Verhalten wieder herstellen.
        t8_conf.atchcalsthr = 50;
        t8_conf.atchfrccalthr = 50;
        t8_conf.atchfrccalratio = 25;
        t8_conf.measallow = config->allowed_measurement_types;

        ret = mxt_seq_write(dev, data->t8_acquisitionconfig_address, &t8_conf, sizeof(t8_conf));
        if (ret < 0) {
            LOG_ERR("Failed to set T8 config: %d", ret);
            return ret;
        }
        ret = mxt_seq_read(dev, data->t8_acquisitionconfig_address, &t8_conf, sizeof(t8_conf));
        LOG_HEXDUMP_INF(&t8_conf, sizeof(t8_conf), "T8 after");
    }

#ifdef MXT_ENABLE_STYLUS
    if (data->t42_proci_touchsupression_address) {
        struct mxt_proci_touchsupression_t42 t42_conf = {};

        t42_conf.ctrl = MXT_T42_CTRL_ENABLE | MXT_T42_CTRL_SHAPEEN;
        t42_conf.maxapprarea = 0;   // Default (0): suppress any touch that approaches >40 channels.
        t42_conf.maxtcharea = 0;    // Default (0): suppress any touch that covers >35 channels.
        t42_conf.maxnumtchs = 6;    // Suppress all touches if >6 are detected.
        t42_conf.supdist = 0;       // Default (0): Suppress all touches within 5 nodes of a suppressed large object detection.
        t42_conf.disthyst = 0;
        t42_conf.supstrength = 0;   // Default (0): suppression strength of 128.
        t42_conf.supextto = 0;      // Timeout to save power; set to 0 to disable.
        t42_conf.shapestrength = 0; // Default (0): shape suppression strength of 10, range [0, 31].
        t42_conf.maxscrnarea = 0;
        t42_conf.edgesupstrength = 0;
        t42_conf.cfg = 1;

        ret = mxt_seq_write(dev, data->t42_proci_touchsupression_address, &t42_conf, sizeof(t42_conf));
        if (ret < 0) {
            LOG_ERR("Failed to set T42 config: %d", ret);
            return ret;
        }
    }
#endif

    // Mutural Capacitive Touch Engine (CTE) configuration, currently we use all the default values but it feels like some of this stuff might be important.
    if (data->t46_cte_config_address) {
        struct mxt_spt_cteconfig_t46 t46_conf = {};
        t46_conf.idlesyncsperx = config->idle_syncs_per_x;      // ADC samples per X.
        t46_conf.activesyncsperx = config->active_syncs_per_x;  // ADC samples per X.
        t46_conf.inrushcfg = 0;                                 // Set Y-line inrush limit resistors.


        ret = mxt_seq_write(dev, data->t46_cte_config_address, &t46_conf, sizeof(t46_conf));
        if (ret < 0) {
            LOG_ERR("Failed to set T46 config: %d", ret);
            return ret;
        }
    }

#ifdef MXT_ENABLE_STYLUS
    if (data->t47_proci_stylus_address) {
        struct mxt_proci_stylus_t47 t47_conf = {};
        t47_conf.cfg = MXT_T47_CFG_SUPSTY;  // Supress stylus detections when normal touches are present.
        t47_conf.contmax = 80;              // The maximum contact diameter of the stylus in 0.1mm increments
        t47_conf.maxtcharea = 100;          // Maximum touch area a contact can have an still be considered a stylus
        t47_conf.stability = 30;            // Higher values prevent the stylus from dropping out when it gets small
        t47_conf.confthr = 6;               // Higher values increase the chances of correctly detecting as stylus, but introduce a delay
        t47_conf.amplthr = 60;              // Any touches smaller than this are classified as stylus touches
        t47_conf.supstyto = 5;              // Continue to suppress stylus touches until supstyto x 200ms after the last touch is removed.
        t47_conf.hoversup = 200;            // 255 Disables hover supression
        t47_conf.maxnumsty = 1;             // Only report a single stylus
        t47_conf.ctrl = 1;                  // Enable stylus detection

        ret = mxt_seq_write(dev, data->t47_proci_stylus_address, &t47_conf, sizeof(t47_conf));
        if (ret < 0) {
            LOG_ERR("Failed to set T46 config: %d", ret);
            return ret;
        }
    }
#endif

    if (config->shieldless_enable && data->t56_proci_shieldless_address) {
        // Nur die ersten 4 Bytes schreiben (Objekt ist auf diesem Chip 18 Bytes gross)
        uint8_t t56_conf[4] = {MXT_T56_CTRL_ENABLE, 0, 1 /* optint */, 10 /* inttime */};
        ret = mxt_seq_write(dev, data->t56_proci_shieldless_address, t56_conf, sizeof(t56_conf));
        if (ret < 0) {
            LOG_ERR("Failed to set T56 config: %d", ret);
            return ret;
        }
    }

    if (data->t80_proci_retransmissioncompensation_address) {
        struct mxt_proci_retransmissioncompensation_t80 t80_conf = {};
        t80_conf.ctrl = config->retransmission_compensation_disable == false;
        t80_conf.compgain = 5;
        t80_conf.targetdelta = 125;
        t80_conf.compthr = 60;

        ret = mxt_seq_write(dev, data->t80_proci_retransmissioncompensation_address, &t80_conf, sizeof(t80_conf));
        if (ret < 0) {
            LOG_ERR("Failed to set T80 config: %d", ret);
            return ret;
        }
    }

    if (data->t100_multiple_touch_touchscreen_address) {
        struct mxt_touch_multiscreen_t100 t100_conf = {0};

        ret = mxt_seq_read(dev, data->t100_multiple_touch_touchscreen_address, &t100_conf,
                           sizeof(t100_conf));
        if (ret < 0) {
            LOG_ERR("Failed to load the initial T100 config: %d", ret);
            return ret;
        }
        LOG_HEXDUMP_INF(&t100_conf, sizeof(t100_conf), "T100 before");

        t100_conf.ctrl =
            MXT_T100_CTRL_RPTEN | MXT_T100_CTRL_ENABLE | MXT_T100_CTRL_SCANEN;  // Enable the t100 object, and enable
                                                                                // message reporting for the t100 object.1`
                                                                                // and enable close scanning mode.
        uint8_t cfg1 = 0;

        if (config->repeat_each_cycle) {
            cfg1 |= MXT_T100_CFG_RPTEACHCYCLE;
        }

        if (config->swap_xy) {
            cfg1 |= MXT_T100_CFG_SWITCHXY;
        }

        if (config->invert_x) {
            cfg1 |= MXT_T100_CFG_INVERTX;
        }

        if (config->invert_y) {
            cfg1 |= MXT_T100_CFG_INVERTY;
        }

        t100_conf.cfg1 = cfg1; // Could also handle rotation, and axis inversion in hardware here

        t100_conf.scraux = 0x7;   // AUX data: Report the number of touch events, touch area, anti touch area
        t100_conf.tchaux = 0x02 | 0x04; // pro Touch Amplitude (data[5]) und Flaeche (data[6])
        t100_conf.tcheventcfg = 24;     // wie QMK-Treiber: Meldungen fuer ausgeblendete Finger aus
        t100_conf.amplcoeff = 16;       // Amplitude skalieren (QMK/Procyon)
        t100_conf.numtch = config->max_touch_points;  // The number of touch reports
                                                      // we want to receive (upto 10)
        // Tatsaechlich belegte Leitungen des Sensor-PCBs; der Info-Block liefert nur das
        // Maximum des Controllers (z.B. 14x24 beim mXT336UD, Procyon 42x50 nutzt 10x12).
        uint8_t x_lines = config->x_lines ? config->x_lines : information->matrix_x_size;
        uint8_t y_lines = config->y_lines ? config->y_lines : information->matrix_y_size;
        x_lines = MIN(x_lines, information->matrix_x_size);
        y_lines = MIN(y_lines, information->matrix_y_size);
        data->x_lines_used = x_lines;
        data->y_lines_used = y_lines;
        t100_conf.xorigin = 0;
        t100_conf.xsize = x_lines;
        t100_conf.yorigin = 0;
        t100_conf.ysize = y_lines;
        t100_conf.xpitch = config->x_pitch ? config->x_pitch : (config->sensor_width * 10 / x_lines);
        t100_conf.ypitch = config->y_pitch ? config->y_pitch : (config->sensor_height * 10 / y_lines);
        LOG_INF("T100 matrix %dx%d lines, pitch %d/%d (0.1mm)", x_lines, y_lines, t100_conf.xpitch,
                t100_conf.ypitch);
        t100_conf.xedgecfg = 9;
        t100_conf.xedgedist = 10;
        t100_conf.yedgecfg = 9;
        t100_conf.yedgedist = 10;
        t100_conf.gain = config->gain;  // Single transmit gain for mutual capacitance measurements
        t100_conf.dxgain = 0;           // Dual transmit gain for mutual capacitance
                                        // measurements (255 = auto calibrate)
        t100_conf.tchthr = config->touch_threshold;
        t100_conf.tchhyst = config->touch_hysteresis;
        t100_conf.intthr = config->internal_touch_threshold;
        t100_conf.intthryst = config->internal_touch_hysteresis;
        t100_conf.mrgthr = config->merge_threshold;   // kleiner = nahe Finger trennen leichter
        t100_conf.mrghyst = config->merge_hysteresis;
        t100_conf.mrgthradjstr = 20;
        t100_conf.movsmooth = config->move_smooth; // Glaettung bei langsamen Bewegungen
        t100_conf.movfilter = 0;        // The lower 4 bits are the speed response value, higher
                                        // values reduce lag, but also smoothing

        // These two fields implement a simple filter for reducing jitter, but large
        // values cause the pointer to stick in place before moving.
        t100_conf.movhysti = sys_cpu_to_le16(config->move_hyst_initial); // Initial movement hysteresis
        t100_conf.movhystn = sys_cpu_to_le16(config->move_hyst_next);    // Next movement hysteresis
        t100_conf.cfg2 = config->confthr;                                // Touch-Entprellung

        t100_conf.tchdiup = 4; // MXT_UP touch detection integration - the number of cycles before the sensor decides an MXT_UP event has occurred
        t100_conf.tchdidown = 2; // MXT_DOWN touch detection integration - the number of cycles before the sensor decides an MXT_DOWN event has occurred
        t100_conf.nexttchdi = 2;
        t100_conf.calcfg = 0;
        if (config->swap_xy) {
            t100_conf.xrange = sys_cpu_to_le16(CONFIG_ZMK_TRACKPAD_LOGICAL_Y-1);
            t100_conf.yrange = sys_cpu_to_le16(CONFIG_ZMK_TRACKPAD_LOGICAL_X-1);
        }
        else {
            t100_conf.xrange = sys_cpu_to_le16(CONFIG_ZMK_TRACKPAD_LOGICAL_X-1);
            t100_conf.yrange = sys_cpu_to_le16(CONFIG_ZMK_TRACKPAD_LOGICAL_Y-1);
        }
        ret = mxt_seq_write(dev, data->t100_multiple_touch_touchscreen_address, &t100_conf,
                            sizeof(t100_conf));
        if (ret < 0) {
            LOG_ERR("Failed to set T100 config: %d", ret);
            return ret;
        }
        ret = mxt_seq_read(dev, data->t100_multiple_touch_touchscreen_address, &t100_conf,
                           sizeof(t100_conf));
        LOG_HEXDUMP_INF(&t100_conf, sizeof(t100_conf), "T100 after");
    }

    // Kalibrierung anstossen: T6 CALIBRATE ist Offset 2 (Offset 3 waere REPORTALL)
    if (data->t6_command_processor_address) {
        uint8_t calibrate = 0x55;
        ret = mxt_seq_write(dev, data->t6_command_processor_address +
                            offsetof(struct mxt_gen_commandprocessor_t6, calibrate),
                            &calibrate, 1);
        LOG_INF("T6 CALIBRATE sent (ret=%d)", ret);
        k_sleep(K_MSEC(200));
    }

    return 0;
}

#define MXT_SAFETY_POLL_MS 500  // Sicherheitsnetz im Interrupt-Betrieb
#define MXT_INIT_RETRY_MS 500
#define MXT_INIT_FIRST_DELAY_MS 500
#define MXT_RECAL_DELAY_MS 3000

static int mxt_calibrate(const struct device *dev) {
    struct mxt_data *data = dev->data;
    uint8_t calibrate = 0x55;
    if (!data->t6_command_processor_address) {
        return -ENODEV;
    }
    return mxt_seq_write(dev, data->t6_command_processor_address +
                         offsetof(struct mxt_gen_commandprocessor_t6, calibrate),
                         &calibrate, 1);
}

#define MXT_DIAG_DELTAS 0x10
#define MXT_DIAG_REFS 0x11
#define MXT_DIAG_PAGE_UP 0x01
#define MXT_T37_PAGE_BYTES 128
#define MXT_DIAG_PERIOD_MS 3000
#define MXT_MAX_NODES (14 * 24)

static int16_t mxt_diag_nodes[MXT_MAX_NODES];

static int mxt_t37_read_page(const struct device *dev, uint8_t mode, uint8_t page, uint8_t *buf) {
    struct mxt_data *data = dev->data;
    for (int i = 0; i < 25; i++) {
        uint8_t hdr[2];
        int ret = mxt_seq_read(dev, data->t37_diagnostic_debug_address, hdr, 2);
        if (ret < 0) {
            return ret;
        }
        if (hdr[0] == mode && hdr[1] == page) {
            return mxt_seq_read(dev, data->t37_diagnostic_debug_address + 2, buf,
                                MXT_T37_PAGE_BYTES);
        }
        k_msleep(2);
    }
    return -ETIMEDOUT;
}

// T37: Rohdaten aller Nodes (int16, X-major, Stride = matrix_y_size des Chips)
static int mxt_diag_dump(const struct device *dev, uint8_t mode, const char *name, bool full) {
    struct mxt_data *data = dev->data;
    int ret;
    if (!data->t37_diagnostic_debug_address || !data->t6_command_processor_address) {
        return -ENODEV;
    }
    uint16_t total = data->matrix_x_size * data->matrix_y_size;
    if (total > MXT_MAX_NODES) {
        total = MXT_MAX_NODES;
    }
    uint8_t pages = (total * 2 + MXT_T37_PAGE_BYTES - 1) / MXT_T37_PAGE_BYTES;
    uint8_t cmd = mode;
    uint16_t t6_diag = data->t6_command_processor_address +
                       offsetof(struct mxt_gen_commandprocessor_t6, diagnostic);

    ret = mxt_seq_write(dev, t6_diag, &cmd, 1);
    if (ret < 0) {
        return ret;
    }
    for (uint8_t p = 0; p < pages; p++) {
        uint8_t buf[MXT_T37_PAGE_BYTES];
        ret = mxt_t37_read_page(dev, mode, p, buf);
        if (ret < 0) {
            LOG_WRN("T37 page %d read failed: %d", p, ret);
            return ret;
        }
        for (int i = 0; i < MXT_T37_PAGE_BYTES / 2; i++) {
            uint16_t idx = p * (MXT_T37_PAGE_BYTES / 2) + i;
            if (idx < total) {
                mxt_diag_nodes[idx] = (int16_t)(buf[2 * i] | (buf[2 * i + 1] << 8));
            }
        }
        if (p + 1 < pages) {
            cmd = MXT_DIAG_PAGE_UP;
            ret = mxt_seq_write(dev, t6_diag, &cmd, 1);
            if (ret < 0) {
                return ret;
            }
        }
    }

    uint8_t xs = full ? data->matrix_x_size : data->x_lines_used;
    uint8_t ys = full ? data->matrix_y_size : data->y_lines_used;
    LOG_INF("%s dump (%dx%d of %dx%d):", name, xs, ys, data->matrix_x_size, data->matrix_y_size);
    for (uint8_t x = 0; x < xs; x++) {
        const int16_t *r = &mxt_diag_nodes[x * data->matrix_y_size];
        // bis zu 12 Werte pro Zeile; Chip-Maximum 24 -> zwei Zeilen
        for (uint8_t y0 = 0; y0 < ys; y0 += 12) {
            int16_t v[12] = {0};
            for (uint8_t k = 0; k < 12 && (y0 + k) < ys; k++) {
                v[k] = r[y0 + k];
            }
            LOG_INF("X%02d Y%02d: %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d", x, y0, v[0],
                    v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9], v[10], v[11]);
        }
    }
    return 0;
}

static void mxt_diag_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct mxt_data *data = CONTAINER_OF(dwork, struct mxt_data, diag_work);
    mxt_diag_dump(data->dev, MXT_DIAG_DELTAS, "T37 deltas", false);
    k_work_schedule(dwork, K_MSEC(MXT_DIAG_PERIOD_MS));
}

// Zweite Kalibrierung, wenn Stromversorgung, USB und BLE nach dem Boot ruhig sind.
static void mxt_recal_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct mxt_data *data = CONTAINER_OF(dwork, struct mxt_data, recal_work);
    const struct mxt_config *config = data->dev->config;
    int ret = mxt_calibrate(data->dev);
    LOG_INF("delayed T6 CALIBRATE sent (ret=%d)", ret);
    if (config->diag_dump) {
        k_msleep(300);
        mxt_diag_dump(data->dev, MXT_DIAG_REFS, "T37 references", true);
        k_work_schedule(&data->diag_work, K_MSEC(MXT_DIAG_PERIOD_MS));
    }
}

// Der maXTouch haengt am geschalteten VCC des nice!nano (ext-power) und braucht nach
// Power-on einige hundert ms, bis er auf I2C antwortet. Deshalb wird die eigentliche
// Initialisierung verzoegert und bei Fehlern wiederholt, statt im Device-Init zu scheitern.
static void mxt_init_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct mxt_data *data = CONTAINER_OF(dwork, struct mxt_data, init_work);
    const struct device *dev = data->dev;
    const struct mxt_config *config = dev->config;
    int ret;

    data->init_attempts++;

    if (!i2c_is_ready_dt(&config->bus)) {
        LOG_ERR("i2c bus isn't ready (attempt %u)", data->init_attempts);
        k_work_schedule(dwork, K_MSEC(MXT_INIT_RETRY_MS));
        return;
    }

    struct mxt_information_block info = {0};
    ret = mxt_load_object_table(dev, &info);
    if (ret < 0 || info.num_objects == 0 || info.num_objects == 0xFF) {
        LOG_WRN("maxtouch not responding at 0x%02x (attempt %u, ret=%d, objects=%d), retry in %dms",
                config->bus.addr, data->init_attempts, ret, info.num_objects, MXT_INIT_RETRY_MS);
        k_work_schedule(dwork, K_MSEC(MXT_INIT_RETRY_MS));
        return;
    }

    LOG_INF("maxtouch found after %u attempt(s): family=%d variant=%d version=%d matrix=%dx%d objects=%d",
            data->init_attempts, info.family_id, info.variant_id, info.version, info.matrix_x_size,
            info.matrix_y_size, info.num_objects);
    LOG_INF("T5=0x%04x T6=0x%04x T44=0x%04x T100=0x%04x T100_first_rid=%d",
            data->t5_message_processor_address, data->t6_command_processor_address,
            data->t44_message_count_address, data->t100_multiple_touch_touchscreen_address,
            data->t100_first_report_id);

    ret = mxt_load_config(dev, &info);
    if (ret < 0) {
        LOG_ERR("Failed to load default config: %d, retry in %dms", ret, MXT_INIT_RETRY_MS);
        k_work_schedule(dwork, K_MSEC(MXT_INIT_RETRY_MS));
        return;
    }

    // Load any existing messages to clear them (ohne Gestenauswertung)
    mxt_report_data(dev);
    data->ready = true;

    if (config->chg.port && device_is_ready(config->chg.port)) {
        gpio_init_callback(&data->gpio_cb, mxt_gpio_cb, BIT(config->chg.pin));
        ret = gpio_add_callback(config->chg.port, &data->gpio_cb);
        if (ret == 0) {
            ret = gpio_pin_interrupt_configure_dt(&config->chg, GPIO_INT_LEVEL_ACTIVE);
        }
        if (ret == 0) {
            data->irq_mode = true;
        } else {
            LOG_WRN("CHG interrupt not usable (%d), falling back to polling", ret);
        }
    }

    if (data->irq_mode) {
        // Der Chip meldet sich ueber CHG. Der langsame Timer ist nur ein Sicherheitsnetz,
        // falls eine Flanke verloren geht; das spart gegenueber 8-ms-Polling viel Strom.
        k_timer_start(&data->poll_timer, K_MSEC(MXT_SAFETY_POLL_MS), K_MSEC(MXT_SAFETY_POLL_MS));
        LOG_INF("maxtouch config loaded, CHG interrupt mode (safety poll %dms)",
                MXT_SAFETY_POLL_MS);
    } else {
        k_timer_start(&data->poll_timer, K_MSEC(8), K_MSEC(8));
        LOG_INF("maxtouch config loaded, polling every 8ms");
    }
    k_work_schedule(&data->recal_work, K_MSEC(MXT_RECAL_DELAY_MS));
}

static int mxt_init(const struct device *dev) {
    struct mxt_data *data = dev->data;
    const struct mxt_config *config = dev->config;
    int ret;

    data->dev = dev;

    LOG_INF("maxtouch init: i2c addr 0x%02x, deferring probe by %dms", config->bus.addr,
            MXT_INIT_FIRST_DELAY_MS);

    gpio_pin_configure_dt(&config->chg, GPIO_INPUT);
    // Interrupt wird erst nach dem Konfigurieren des Chips scharf geschaltet
    ret = gpio_pin_interrupt_configure_dt(&config->chg, GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("Failed to configure interrupt for CHG pin %d", ret);
    }

    k_work_init(&data->work, mxt_work_cb);
    k_timer_init(&data->poll_timer, mxt_poll_timer_cb, NULL);
    k_timer_user_data_set(&data->poll_timer, data);

    k_work_init_delayable(&data->momentum_work, mxt_momentum_work_cb);
    k_work_init_delayable(&data->click_release_work, mxt_click_release_cb);
    k_work_init_delayable(&data->recal_work, mxt_recal_work_cb);
    k_work_init_delayable(&data->diag_work, mxt_diag_work_cb);
    k_work_init_delayable(&data->init_work, mxt_init_work_cb);
    k_work_schedule(&data->init_work, K_MSEC(MXT_INIT_FIRST_DELAY_MS));

    return 0;
}

#define MXT_INST(n)                                                                                     \
    static struct mxt_data mxt_data_##n;                                                                \
    static const struct mxt_config mxt_config_##n = {                                                   \
        .bus = I2C_DT_SPEC_INST_GET(n),                                                                 \
        .chg = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(n), chg_gpios, {}),                                      \
        .max_touch_points = DT_INST_PROP_OR(n, max_touch_points, 5),                                    \
        .idle_acq_time = DT_INST_PROP_OR(n, idle_acq_time_ms, 32),                                      \
        .active_acq_time = DT_INST_PROP_OR(n, active_acq_time_ms, 10),                                  \
        .active_to_idle_timeout = DT_INST_PROP_OR(n, active_to_idle_timeout_ms, 50),                    \
        .repeat_each_cycle = DT_INST_PROP(n, repeat_each_cycle),                                        \
        .swap_xy = DT_INST_PROP(n, swap_xy),                                                            \
        .invert_x = DT_INST_PROP(n, invert_x),                                                          \
        .invert_y = DT_INST_PROP(n, invert_y),                                                          \
        .sensor_width = DT_INST_PROP(n, sensor_width),                                                  \
        .sensor_height = DT_INST_PROP(n, sensor_height),                                                \
        .x_lines = DT_INST_PROP_OR(n, x_lines, 0),                                                  \
        .y_lines = DT_INST_PROP_OR(n, y_lines, 0),                                                  \
        .x_pitch = DT_INST_PROP_OR(n, x_pitch, 0),                                                  \
        .y_pitch = DT_INST_PROP_OR(n, y_pitch, 0),                                                  \
        .move_hyst_initial = DT_INST_PROP(n, move_hysteresis_initial),                              \
        .move_hyst_next = DT_INST_PROP(n, move_hysteresis_next),                                    \
        .confthr = DT_INST_PROP(n, confthr),                                                        \
        .move_smooth = DT_INST_PROP(n, move_smooth),                                                \
        .merge_threshold = DT_INST_PROP(n, merge_threshold),                                        \
        .merge_hysteresis = DT_INST_PROP(n, merge_hysteresis),                                      \
        .scroll_momentum = DT_INST_PROP(n, scroll_momentum),                                        \
        .shieldless_enable = DT_INST_PROP(n, shieldless_enable),                                    \
        .diag_dump = DT_INST_PROP(n, diag_dump),                                                    \
        .touch_threshold = DT_INST_PROP_OR(n, touch_threshold, 18),                                     \
        .touch_hysteresis = DT_INST_PROP_OR(n, touch_hysteresis, 8),                                    \
        .internal_touch_threshold = DT_INST_PROP_OR(n, internal_touch_threshold, 10),                   \
        .internal_touch_hysteresis = DT_INST_PROP_OR(n, internal_touch_hysteresis, 4),                  \
        .gain = DT_INST_PROP_OR(n, gain, 4),                                                            \
        .charge_time = DT_INST_PROP_OR(n, charge_time, 10),                                             \
        .allowed_measurement_types = DT_INST_PROP_OR(n, allowed_measurement_types, 3),                  \
        .active_syncs_per_x = DT_INST_PROP_OR(n, active_syncs_per_x, 20),                               \
        .idle_syncs_per_x = DT_INST_PROP_OR(n, idle_syncs_per_x, 20),                                   \
        .retransmission_compensation_disable = DT_INST_PROP(n, retransmission_compensation_disable),    \
    };                                                                                                  \
    DEVICE_DT_INST_DEFINE(n, mxt_init, NULL, &mxt_data_##n, &mxt_config_##n, POST_KERNEL,               \
                          CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MXT_INST)
