#pragma once

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

#define MXT_MAX_FINGERS 5

struct mxt_finger {
    bool active;
    int16_t x, y;
    int16_t down_x, down_y;
    bool merged;        // Kontaktflaeche so gross, dass es zwei verschmolzene Finger sind
    uint8_t down_area;  // Flaeche beim Aufsetzen, Referenz fuer das Wachstum
    // Abhebe-Erkennung (wie im QMK-Treiber des Autors): faellt die Amplitude um >10 %,
    // wird die Bewegung zurueckgehalten und beim Abheben verworfen.
    int32_t ampl_sum;
    uint8_t ampl_cnt;
    uint8_t ampl_avg;
    bool lift_buffering;
    uint8_t lift_samples;
    int16_t buf_x, buf_y;
    uint32_t last_ms;   // Zeitpunkt der letzten Messung, Basis fuer die Sprunggrenze
    uint8_t jump_skip;  // Sprung erkannt: auch die naechste Messung verwerfen
};

struct mxt_data {
    const struct device *dev;
    // Gesten: Tap = Linksklick, 2-Finger-Tap = Rechtsklick, 2 Finger ziehen = Scrollen
    struct mxt_finger fingers[MXT_MAX_FINGERS];
    uint8_t active_mask;
    uint8_t gesture_max_fingers;
    uint32_t gesture_start_ms;
    bool gesture_moved;
    int16_t scroll_acc_x, scroll_acc_y;
    // Scroll laeuft erst nach einem Mindestweg an: zwei ruhende Finger driften langsam, und
    // diese Drift darf sich nicht im Scroll-Akku sammeln, bis sie beim Losscrollen als
    // fertiger Schritt herausfaellt.
    bool scroll_started;
    int16_t scroll_start_dx, scroll_start_dy;
    uint32_t scroll_start_ms;
    // Auslaufendes Scrollen (Momentum) nach dem Abheben
    int32_t scroll_vel_x, scroll_vel_y; // Counts pro Sekunde, geglaettet
    uint32_t scroll_last_ms;
    struct k_work_delayable momentum_work;
    bool momentum_active;
    int16_t gesture_dx, gesture_dy; // Gesamtweg des fuehrenden Fingers (fuer Wischgesten)
    bool skip_delta;   // naechste Messung verwerfen (Position springt beim Fingerwechsel)
    bool swipe_fired;  // Wischgeste in dieser Beruehrung bereits ausgeloest
    bool cursor_started; // Cursor laeuft erst nach Wartezeit/Mindestweg, Bewegung davor wird verworfen
    // Cursor-Takt: der Chip liefert im Free-Run bis zu 300 Messungen/s, die BLE-Split-Strecke
    // traegt das nicht. Bewegung wird aufsummiert und mit festem Takt abgeschickt.
    int16_t pend_dx, pend_dy;
    // Eine Stufe Verzoegerung: was hier liegt, wurde noch nicht abgeschickt und kann
    // verworfen werden, wenn sich nachtraeglich ein zweiter Finger zeigt.
    int16_t hold_dx, hold_dy;
    uint32_t last_report_ms;
    struct k_work_delayable click_release_work;
    uint16_t click_button;
    bool irq_mode;      // CHG-Interrupt statt Dauer-Polling
    bool ready;         // Chip konfiguriert, alte Meldungen verworfen: erst dann Gesten auswerten
    bool button_held;   // Taste gedrueckt (Tap-Release ausstehend oder Drag laeuft)
    bool dragging;      // Tap-and-Drag aktiv
    struct gpio_callback gpio_cb;
    struct k_work work;
    struct k_work_delayable init_work;
    struct k_work_delayable recal_work;
    struct k_work_delayable diag_work;
    uint8_t matrix_x_size;
    uint8_t matrix_y_size;
    uint8_t x_lines_used;
    uint8_t y_lines_used;
    struct k_timer poll_timer;
    uint32_t init_attempts;

    uint16_t t2_encryption_status_address;
    uint16_t t5_message_processor_address;
    uint16_t t5_max_message_size;
    uint16_t t6_command_processor_address;
    uint16_t t7_powerconfig_address;
    uint16_t t8_acquisitionconfig_address;
    uint16_t t25_self_test_address;
    uint16_t t37_diagnostic_debug_address;
    uint16_t t42_proci_touchsupression_address;
    uint16_t t44_message_count_address;
    uint16_t t46_cte_config_address;
    uint16_t t47_proci_stylus_address;
    uint16_t t56_proci_shieldless_address;
    uint16_t t65_proci_lensbending_address;
    uint16_t t80_proci_retransmissioncompensation_address;
    uint16_t t100_multiple_touch_touchscreen_address;

    uint16_t t6_command_processor_report_id;
    uint16_t t25_self_test_report_id;
    uint16_t t100_first_report_id;
};

struct mxt_config {
    const struct i2c_dt_spec bus;
    const struct gpio_dt_spec chg;
    const uint8_t idle_acq_time;
    const uint8_t active_acq_time;
    const uint8_t active_to_idle_timeout;
    const uint8_t max_touch_points;
    const bool swap_xy;
    const bool invert_x;
    const bool invert_y;
    const bool repeat_each_cycle;
    const uint16_t sensor_width;
    const uint16_t sensor_height;
    const uint8_t x_lines;
    const uint8_t y_lines;
    const uint8_t x_pitch;
    const uint8_t y_pitch;
    const uint8_t move_hyst_initial;
    const uint8_t move_hyst_next;
    const uint8_t confthr;
    const uint8_t move_smooth;
    const uint8_t merge_threshold;
    const uint8_t merge_hysteresis;
    const bool scroll_momentum;
    const bool shieldless_enable;
    const bool diag_dump;
    const uint8_t touch_threshold;
    const uint8_t touch_hysteresis;
    const uint8_t internal_touch_threshold;
    const uint8_t internal_touch_hysteresis;
    const uint8_t gain;
    const uint8_t charge_time;
    const uint8_t allowed_measurement_types;
    const uint8_t active_syncs_per_x;
    const uint8_t idle_syncs_per_x;
    const bool retransmission_compensation_disable;
};

#define MXT_REG_INFORMATION_BLOCK (0)

struct mxt_object_table_element {
    uint8_t type;
    uint16_t position;
    uint8_t size_minus_one;
    uint8_t instances_minus_one;
    uint8_t report_ids_per_instance;
} __packed;

struct mxt_information_block {
    uint8_t family_id;
    uint8_t variant_id;
    uint8_t version;
    uint8_t build;
    uint8_t matrix_x_size;
    uint8_t matrix_y_size;
    uint8_t num_objects;
} __packed;

struct mxt_message {
    uint8_t report_id;
    uint8_t data[8]; // status, x(2), y(2), aux: ampl, area
} __packed;

struct mxt_message_count {
    uint8_t count;
} __packd;

struct mxt_gen_encryptionstatus_t2 {
    uint16_t status;
    uint8_t payloadcrc[3];
    uint8_t enccustcrc[3];
    uint8_t error;
} __packed;

struct mxt_gen_commandprocessor_t6 {
    uint8_t reset;
    uint8_t backupnv;
    uint8_t calibrate;
    uint8_t reportall;
    uint8_t debugctrl;
    uint8_t diagnostic;
    uint8_t debugctrl2;
} __packed;

#define MXT_T6_DIAGNOSTIC_PAGE_UP                               0x01
#define MXT_T6_DIAGNOSTIC_PAGE_DOWN                             0x02
#define MXT_T6_DIAGNOSTIC_MUTUAL_CAPACITANCE_DELTA_MODE         0x10
#define MXT_T6_DIAGNOSTIC_MUTUAL_CAPACITANCE_REFERENCE_MODE     0x11
#define MXT_T6_DIAGNOSTIC_MUTUAL_CAPACITANCE_DC_DATA_MODE       0x38
#define MXT_T6_DIAGNOSTIC_DEVICE_INFORMATION_MODE               0x80
#define MXT_T6_DIAGNOSTIC_SELF_CAPACITANCE_SIGNAL_MODE          0xF5
#define MXT_T6_DIAGNOSTIC_SELF_CAPACITANCE_DELTA_MODE           0xF7
#define MXT_T6_DIAGNOSTIC_SELF_CAPACITANCE_REFERENCE_MODE       0xF8
#define MXT_T6_DIAGNOSTIC_KEY_DELTA_MODE                        0x17
#define MXT_T6_DIAGNOSTIC_KEY_REFERENCE_MODE                    0x18
#define MXT_T6_DIAGNOSTIC_KEY_SIGNAL_MODE                       0x19
#define MXT_T6_DIAGNOSTIC_CALIBRATION_RECOVERY_TUNING_DATA_MODE 0x33
#define MXT_T6_DIAGNOSTIC_PIN_FAULT_SELF_TEST_MODE              0x35
#define MXT_T6_DIAGNOSTIC_TOUCH_STATUS_DATA_MODE                0x36
#define MXT_T6_DIAGNOSTIC_PRODUCT_DATA_STORE_MODE               0x81
#define MXT_T6_DIAGNOSTIC_TOUCHSCREEN_MODE                      0xF4

struct mxt_gen_powerconfig_t7 {
    uint8_t idleacqint;
    uint8_t actacqint;
    uint8_t actv2idleto;
    uint8_t cfg;
    uint8_t cfg2;
    uint8_t idleacqintfine;
    uint8_t actvaqintfine;
} __packed;

#define MXT_T7_CFG_INITACTV             BIT(7)
#define MXT_T7_CFG_OVFRPTSUP            BIT(6)
#define MXT_T7_CFG_ACTV2IDLETOMSB_SHIFT 2
#define MXT_T7_CFG_ACTV2IDLETOMSB_MASK  0x3C
#define MXT_T7_CFG_ACTVPIPEEN           BIT(1)
#define MXT_T7_CFG_IDLEPIPEEN           BIT(0)

struct mxt_gen_acquisitionconfig_t8 {
    uint8_t chrgtime;
    uint8_t reserved;
    uint8_t tchdrift;
    uint8_t driftst;
    uint8_t tchautocal;
    uint8_t sync;
    uint8_t atchcalst;
    uint8_t atchcalsthr;
    uint8_t atchfrccalthr;
    uint8_t atchfrccalratio;
    uint8_t measallow;
    uint8_t reserved2[3];
    uint8_t cfg;
} __packed;

struct mxt_spt_selftest_t25 {
    uint8_t ctrl;
    uint8_t cmd;
    uint8_t upsiglim_lsb;
    uint8_t upsiglim_msb;
    uint8_t losiglim_lsb;
    uint8_t losiglim_msb;
    uint8_t pindwellus;
    uint8_t sigrangelim_lsb;
    uint8_t sigrangelim_msb;
    uint8_t pinthr;
    uint8_t pertstinterval;
    uint8_t pertstholdoff;
    uint8_t pertstrptfactor;
    uint8_t pertstrtpwidth;
    uint8_t pertstcfg;
    uint8_t sesiglimits[3];
} __packed;

#define MXT_T25_TEST_FINISHED       0x00
#define MXT_T25_TEST_POWER          0x01
#define MXT_T25_TEST_PIN_FAULT      0x12
#define MXT_T25_TEST_SIGNAL_LIMIT   0x17
#define MXT_T25_TEST_ALL            0xFE
#define MXT_T25_TEST_INVALID        0xFD
#define MXT_T25_TEST_PASSED         0xFE

struct mxt_proci_touchsupression_t42 {
    uint8_t ctrl;
    uint8_t reserved;
    uint8_t maxapprarea;
    uint8_t maxtcharea;
    uint8_t supstrength;
    uint8_t supextto;
    uint8_t maxnumtchs;
    uint8_t shapestrength;
    uint8_t supdist;
    uint8_t disthyst;
    uint8_t maxscrnarea;
    uint8_t cfg;
    uint8_t reserved2;
    uint8_t edgesupstrength;
} __packed;

#define MXT_T42_CTRL_ENABLE         0x01
#define MXT_T42_CTRL_SHAPEEN        0x04
#define MXT_T42_CTRL_DISLOBJ        0x08
#define MXT_T42_CTRL_DISTLOCK       0x10
#define MXT_T42_CTRL_SUPDISTEN      0x20
#define MXT_T42_CTRL_EDGESUP        0x40
#define MXT_T42_CFG_RELAXCLOSEUP    0x01
#define MXT_T42_CFG_RELAXDIAGSUP    0x02
#define MXT_T42_CFG_SUPTCHRPTEN     0x04

struct mxt_spt_cteconfig_t46 {
    uint8_t reserved[2];
    uint8_t idlesyncsperx;
    uint8_t activesyncsperx;
    uint8_t adcspersync;
    uint8_t piusesperadc;
    uint8_t xslew;
    uint16_t syncdelay;
    uint8_t xvoltage;
    uint8_t reserved2;
    uint8_t inrushcfg;
    uint8_t reserved3[6];
    uint8_t cfg;
} __packed;


#define MXT_T46_CFG_SYNCLOSSMODE 0x02
#define MXT_T46_CFG_DELTASHIFT   0x04

struct mxt_proci_stylus_t47 {
    uint8_t ctrl;
    uint8_t reserved;
    uint8_t contmax;
    uint8_t stability;
    uint8_t maxtcharea;
    uint8_t amplthr;
    uint8_t styshape;
    uint8_t hoversup;
    uint8_t confthr;
    uint8_t syncsperx;
    uint8_t xposadj;
    uint8_t yposadj;
    uint8_t cfg;
    uint8_t reserved2[7];
    uint8_t supstyto;
    uint8_t maxnumsty;
    uint8_t xedgectrl;
    uint8_t xedgedist;
    uint8_t yedgectrl;
    uint8_t yedgedist;
    uint8_t supto;
    uint8_t supclassmode;
    uint8_t dxxedgectrl;
    uint8_t dxxedgedist;
    uint8_t xedgectrlhi;
    uint8_t xedgedisthi;
    uint8_t dxxedgectrlhi;
    uint8_t dxxedgedisthi;
    uint8_t yedgectrlhi;
    uint8_t yedgedisthi;
    uint8_t cfg2;
    uint8_t movfilter;
    uint8_t movsmooth;
    uint8_t movpred;
    uint8_t satbxlo;
    uint8_t satbxhi;
    uint8_t satbylo;
    uint8_t satbyhi;
    uint8_t satbdxxlo;
    uint8_t satbdxxhi;
    uint8_t movhistcfg;
} __packed;

#define MXT_T47_CFG_SUPSTY  0x20

struct mxt_proci_shieldless_t56 {
    uint8_t ctrl;
    uint8_t reserved;
    uint8_t optint;
    uint8_t inttime;
    uint8_t intdelay[41];
} __packed;

#define MXT_T56_CTRL_ENABLE 0x01
#define MXT_T56_CTRL_RPTEN  0x02

struct mxt_proci_lensbending_t65 {
    uint8_t ctrl;
    uint8_t gradthr;
    uint8_t ylonoisemul_lsb;
    uint8_t ylonoisemul_msb;
    uint8_t ylonoisediv_lsb;
    uint8_t ylonoisediv_msb;
    uint8_t yhinoisemul_lsb;
    uint8_t yhinoisemul_msb;
    uint8_t yhinoisediv_lsb;
    uint8_t yhinoisediv_msb;
    uint8_t lpfiltcoef;
    uint8_t forcescale_lsb;
    uint8_t forcescale_msb;
    uint8_t forcethr;
    uint8_t forcethrhyst;
    uint8_t forcedi;
    uint8_t forcehyst;
    uint8_t atchratio;
    uint8_t reserved[2];
    uint8_t exfrcthr;
    uint8_t exfrcthrhyst;
    uint8_t exfrcto;
} __packed;

#define MXT_T65_CTRL_ENABLE             0x01
#define MXT_T65_CTRL_RPTEN              0x02
#define MXT_T65_CTRL_DISPRESS           0x08
#define MXT_T65_CTRL_DISRELEASE         0x10
#define MXT_T65_CTRL_DISHIST            0x80
#define MXT_T65_ATCHRATIO_ATCHRATIO     0x01
#define MXT_T65_ATCHRATIO_DSRATIO       0x10

struct mxt_proci_retransmissioncompensation_t80 {
    uint8_t ctrl;
    uint8_t compgain;
    uint8_t targetdelta;
    uint8_t compthr;
    uint8_t atchthr;
    uint8_t moistcfg;
    uint8_t reserved;
    uint8_t moistthr;
    uint8_t moistinvtchthr;
    uint8_t moistcfg2;
    uint8_t compstrthr;
    uint8_t compcfg;
    uint8_t moistvldthrsf;
    uint8_t moistcfg3;
    unsigned char moistdegthr;
} __packed;

struct mxt_touch_multiscreen_t100 {
    uint8_t ctrl;
    uint8_t cfg1;
    uint8_t scraux;
    uint8_t tchaux;
    uint8_t tcheventcfg;
    uint8_t akscfg;
    uint8_t numtch;
    uint8_t xycfg;
    uint8_t xorigin;
    uint8_t xsize;
    uint8_t xpitch;
    uint8_t xlocip;
    uint8_t xhiclip;
    uint16_t xrange;
    uint8_t xedgecfg;
    uint8_t xedgedist;
    uint8_t dxxedgecfg;
    uint8_t dxxedgedist;
    uint8_t yorigin;
    uint8_t ysize;
    uint8_t ypitch;
    uint8_t ylocip;
    uint8_t yhiclip;
    uint16_t yrange;
    uint8_t yedgecfg;
    uint8_t yedgedist;
    uint8_t gain;
    uint8_t dxgain;
    uint8_t tchthr;
    uint8_t tchhyst;
    uint8_t intthr;
    uint8_t noisesf;
    uint8_t cutoffthr;
    uint8_t mrgthr;
    uint8_t mrgthradjstr;
    uint8_t mrghyst;
    uint8_t dxthrsf;
    uint8_t tchdidown;
    uint8_t tchdiup;
    uint8_t nexttchdi;
    uint8_t calcfg;
    uint8_t jumplimit;
    uint8_t movfilter;
    uint8_t movsmooth;
    uint8_t movpred;
    uint16_t movhysti;
    uint16_t movhystn;
    uint8_t amplhyst;
    uint8_t scrareahyst;
    uint8_t intthryst;
    uint8_t xedgecfghi;
    uint8_t xedgedisthi;
    uint8_t dxxedgecfghi;
    uint8_t dxxedgedisthi;
    uint8_t yedgecfghi;
    uint8_t yedgedisthi;
    uint8_t cfg2;
    uint8_t movhystcfg;
    uint8_t amplcoeff;
    uint8_t amploffset;
    uint8_t jumplimitmov;
    uint16_t jlmmovthr;
    uint8_t jlmmovintthr;
} __packed;

#define MXT_T100_CTRL_SCANEN      BIT(7)
#define MXT_T100_CTRL_DISSCRMSG0  BIT(2)
#define MXT_T100_CTRL_RPTEN       BIT(1)
#define MXT_T100_CTRL_ENABLE      BIT(0)
 
#define MXT_T100_CFG_INVERTX       BIT(7)
#define MXT_T100_CFG_INVERTY       BIT(6)
#define MXT_T100_CFG_SWITCHXY      BIT(5)
#define MXT_T100_CFG_DISLOCK       BIT(4)
#define MXT_T100_CFG_ATCHTHRSEL    BIT(3)
#define MXT_T100_CFG_RPTEACHCYCLE  BIT(0)

// Touch events reported in the t100 messages
enum t100_touch_event {
    NO_EVENT,
    MOVE,
    UNSUP,
    SUP,
    DOWN,
    UP,
    UNSUPSUP,
    UNSUPUP,
    DOWNSUP,
    DOWNUP
};

enum t100_touch_type {
    MXT_FINGER = 1,
    MXT_PASSIVE_STYLUS,
    MXT_GLOVE = 5,
    MXT_LARGE_TOUCH
};
