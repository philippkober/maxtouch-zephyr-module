#define DT_DRV_COMPAT microchip_maxtouch

#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <stddef.h>
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
        LOG_INF("msg rid=%d t100=%d data=%02x %02x %02x %02x %02x %02x", msg.report_id,
                is_t100_report(dev, msg.report_id), msg.data[0], msg.data[1], msg.data[2],
                msg.data[3], msg.data[4], msg.data[5]);

        if (is_t100_report(dev, msg.report_id)) {
            uint8_t finger_idx = msg.report_id - data->t100_first_report_id - 2;
            bool pending_for_finger = (pending_fingers & BIT(finger_idx)) != 0;

            enum t100_touch_event ev = msg.data[0] & 0xF;
            uint16_t x_pos = msg.data[1] + (msg.data[2] << 8);
            uint16_t y_pos = msg.data[3] + (msg.data[4] << 8);

            switch (ev) {
            case DOWN:
            case MOVE:
            case UP:
            case NO_EVENT:
                if (pending_for_finger) {
                    input_report_key(dev, INPUT_BTN_TOUCH, last_touch_status, true, K_FOREVER);
                    pending_fingers = 0;
                }
                WRITE_BIT(pending_fingers, finger_idx, 1);
                last_touch_status = (ev != UP);
                static int32_t last_x = -1, last_y = -1;
                LOG_INF("touch finger=%d ev=%d x=%d y=%d", finger_idx, ev, x_pos, y_pos);
                if (ev == DOWN) {
                    last_x = x_pos; last_y = y_pos;
                } else if (ev == MOVE && last_x >= 0) {
                    input_report_rel(dev, INPUT_REL_X, x_pos - last_x, false, K_FOREVER);
                    input_report_rel(dev, INPUT_REL_Y, y_pos - last_y, true, K_FOREVER);
                    last_x = x_pos; last_y = y_pos;
                } else if (ev == UP) {
                    last_x = -1; last_y = -1;
                }
                break;
            default:
                // All other events are ignored
                break;
            }
        } else {
            LOG_HEXDUMP_DBG(msg.data, 5, "message data");
        }
    }

    if (pending_fingers != 0) {
        input_report_key(dev, INPUT_BTN_TOUCH, last_touch_status, true, K_FOREVER);
    }

    return;
}

static void mxt_work_cb(struct k_work *work) {
    struct mxt_data *data = CONTAINER_OF(work, struct mxt_data, work);
    mxt_report_data(data->dev);
}

static void mxt_gpio_cb(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct mxt_data *data = CONTAINER_OF(cb, struct mxt_data, gpio_cb);
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
        t8_conf.chrgtime = config->charge_time;
        // Drift-Kompensation an, sonst bleibt eine Kalibrierung mit Finger in der Naehe
        // dauerhaft als Anti-Touch in der Baseline (Werte wie im funktionierenden XIAO-Test)
        t8_conf.tchdrift = 5;
        t8_conf.driftst = 20;
        t8_conf.tchautocal = 50; // Selbstheilung: Recal nach 10s Dauer-Touch (hat im Test geholfen)
        t8_conf.atchcalst = 5;

        // Antitouch detection - reject palms etc..
        t8_conf.atchcalsthr = 35;
        t8_conf.atchfrccalthr = 50;
        t8_conf.atchfrccalratio = 25;
        t8_conf.measallow = config->allowed_measurement_types;

        ret = mxt_seq_write(dev, data->t8_acquisitionconfig_address, &t8_conf, sizeof(t8_conf));
        if (ret < 0) {
            LOG_ERR("Failed to set T8 config: %d", ret);
            return ret;
        }
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

        t100_conf.scraux = 0x7;                       // AUX data: Report the number of touch events, touch area, anti touch area
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
        t100_conf.xpitch = (config->sensor_width * 10 / x_lines);   // Pitch between X-Lines (0.1mm * XPitch).
        t100_conf.ypitch = (config->sensor_height * 10 / y_lines);  // Pitch between Y-Lines (0.1mm * YPitch).
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
        t100_conf.mrgthr = 5;           // Merge threshold
        t100_conf.mrghyst = 10;         // Merge threshold hysteresis
        t100_conf.mrgthradjstr = 20;
        t100_conf.movsmooth = 0;        // The amount of smoothing applied to movements,
                                        // this tails off at higher speeds
        t100_conf.movfilter = 0;        // The lower 4 bits are the speed response value, higher
                                        // values reduce lag, but also smoothing

        // These two fields implement a simple filter for reducing jitter, but large
        // values cause the pointer to stick in place before moving.
        t100_conf.movhysti = 10; // Initial movement hysteresis
        t100_conf.movhystn = 4; // Next movement hysteresis

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

    // Load any existing messages to clear them
    mxt_report_data(dev);

    k_timer_start(&data->poll_timer, K_MSEC(8), K_MSEC(8));
    LOG_INF("maxtouch config loaded, polling every 8ms");
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
    ret = gpio_pin_interrupt_configure_dt(&config->chg, GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("Failed to configure interrupt for CHG pin %d", ret);
    }

    k_work_init(&data->work, mxt_work_cb);
    k_timer_init(&data->poll_timer, mxt_poll_timer_cb, NULL);
    k_timer_user_data_set(&data->poll_timer, data);

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
