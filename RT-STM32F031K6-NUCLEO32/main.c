/*
 * NeaPolis Innovation Summer Campus Examples
 * Copyright (C) 2020-2026
 *
 * VERSIONE "COMANDO SERVOMOTORI" (dead-man switch, un asse alla volta)
 * ---------------------------------------------------------------------
 * Rimossa tutta la logica di "penna follower" (pan/tilt gain,
 * deadband/LPF/rate-limiter, zero manuale/automatico).
 *
 * Il firmware:
 *  1) legge l'IMU (ISM330DLC)
 *  2) calcola pitch/roll (per debug) e la velocita' angolare
 *     istantanea (gx, gy) in gradi/secondo
 *  3) SOLO se il pulsante e' premuto, accumula il gesto su
 *     entrambi gli assi (roll e pitch)
 *  4) al RILASCIO, classifica la MEDIA del gesto su ciascun asse
 *     in: fermo / lento / veloce, con verso (0..4, vedi sotto)
 *  5) manda via Bluetooth (HC-05 su SD1) UN SOLO codice per gesto,
 *     quello dell'asse che si e' mosso di piu' (non roll+pitch
 *     insieme): l'altro asse viene ignorato per quel gesto.
 *
 * Perche' un solo asse alla volta: mandare sempre 2 codici (uno per
 * asse) creava confusione, sia perche' un gesto "puro" spesso ha
 * comunque un po' di rumore/cross-talk sull'altro asse (quindi
 * arrivava un codice "quasi fermo" ma non zero anche sull'asse che
 * non stavi muovendo), sia perche' i due codici condividevano lo
 * stesso spazio di cifre 0..9 con un offset, difficile da leggere.
 * Ora si manda un codice solo, con un carattere che dice ESPLICITA-
 * MENTE di quale asse si tratta: zero ambiguita' in lettura.
 *
 * Formato inviato: 2 caratteri ASCII + \r\n, SOLO al rilascio:
 *   'X' + cifra (0..4)  -> gesto sul ROLL  (fusto), pilota servo PB7
 *   'Y' + cifra (0..4)  -> gesto sul PITCH (testa), pilota servo PB6
 *   'S' + '0'           -> nessun asse sopra soglia: fermo esplicito
 * Il \r\n finale e' innocuo per un parser che legge le cifre/lettere
 * una alla volta, serve solo per leggere a occhio da un terminale.
 *
 * Cifra (0..4), stesso significato su entrambi gli assi:
 *   0 = fermo
 *   1 = verso positivo, lento   (roll: orario   | pitch: su)
 *   2 = verso positivo, veloce  (roll: orario   | pitch: su)
 *   3 = verso negativo, lento   (roll: antiorario | pitch: giu)
 *   4 = verso negativo, veloce  (roll: antiorario | pitch: giu)
 *
 * Come si sceglie l'asse "dominante": si confrontano i valori
 * assoluti delle due medie di velocita' angolare (roll_rate_avg_dps
 * vs pitch_rate_avg_dps) appena calcolate al rilascio, e si manda
 * SOLO il codice di quello con il valore piu' alto. Se sono
 * entrambi sotto soglia (nessun asse classificato come mosso) si
 * manda 'S''0' invece di scegliere un asse a caso.
 *
 * Il verso (segno) dipende da come e' montato il sensore: se
 * "orario" ti esce come negativo, basta invertire il segno in
 * classify_gesture() oppure scambiare 1<->3 e 2<->4 nella tabella.
 * ATTENZIONE: se durante i test un gesto che pensi sia "roll" (X)
 * arriva invece etichettato come "pitch" (Y) o viceversa, il
 * sensore e' probabilmente montato con gli assi fisici scambiati
 * rispetto a quanto assunto qui sotto: scambia gx/gy in acceleration_angles()
 * e nel calcolo di roll_rate_sum_dps/pitch_rate_sum_dps piu' in basso.
 *
 * Mapping fisico (penna):
 *  - ROLL  -> rotazione della penna sul suo asse   -> rotazione fusto lampada
 *  - PITCH -> punta della penna su/giu             -> inclinazione testa lampada
 */

#include "ch.h"
#include "hal.h"
#include "platform.h"
#include "ism330dlc_reg.h"
#include "chprintf.h"

#include <string.h>
#include <math.h>
#include <stdbool.h>

/* ===================== Config ===================== */
#define BOOT_TIME_MS             15U

#define GYRO_CAL_SAMPLES         300U
#define GYRO_CAL_DELAY_MS        5U

#define MAIN_LOOP_DELAY_MS       2U

#define COMP_ALPHA               0.98f          /* filtro complementare */

/* Pulsante "dead-man switch": tenuto premuto = catturo il gesto.
 * Al RILASCIO mando UN SOLO comando, calcolato sulla media di
 * tutto il gesto appena fatto (da quando hai premuto a quando
 * hai mollato). Se non muovi nulla e molli subito, la media
 * resta sotto soglia -> manda 0 (stop). */
#define ENABLE_BUTTON_LINE       LINE_ARD_A0    /* PA0, pulsante verso GND */
#define BUTTON_DEBOUNCE_MS       30U

/* ===== Soglie classificazione movimento (DA TARARE) =====
 * Sono espresse in gradi/secondo di velocita' angolare (gyro),
 * calcolate come media sull'intero gesto (dal press al release).
 * Usa DEBUG_PRINT_RATES qui sotto per vedere il numero finale
 * dopo ogni rilascio e regolare questi valori di conseguenza. */
#define ROLL_RATE_DEADBAND_DPS   18.0f   /* sotto: fermo (codice 0) */
#define ROLL_RATE_FAST_DPS       80.0f   /* sopra: veloce, tra i due: lento */

#define PITCH_RATE_DEADBAND_DPS  18.0f
#define PITCH_RATE_FAST_DPS      80.0f

/* ATTENZIONE: questa board ha UNA SOLA USART (SD1), che e' la STESSA
 * usata per il protocollo verso lo slave via Bluetooth (chp = &SD1,
 * vedi sotto). Qualunque chprintf fatto qui finisce quindi SULLA
 * STESSA LINEA dei 2 caratteri di comando: lo slave legge un
 * carattere alla volta e interpreta come comando anche le cifre
 * che compaiono dentro un testo di debug, causando movimenti
 * spuri dei servo. Percio' questi flag DEVONO restare a 0 quando
 * lo slave e' collegato via BT: accendili solo per una prova a
 * banco, da soli, con un terminale seriale al posto dello slave. */
#define DEBUG_PRINT_RATES        0   /* riga RATE_AVG dopo ogni rilascio */
#define DEBUG_BOOT_MESSAGES      0   /* banner di avvio, OK sensore, calibrazione */


/* ===================== Globals ==================== */
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;

static float_t acceleration_mg[3];
static float_t angular_rate_mdps[3];
static float_t temperature_degC;

static float gyro_bias_dps[3] = {0.0f, 0.0f, 0.0f};

static float roll_deg  = 0.0f;   /* rotazione fusto */
static float pitch_deg = 0.0f;   /* inclinazione testa */
static float yaw_deg   = 0.0f;   /* solo diagnostica, non usato */

static uint8_t whoamI = 0, rst = 0;
static BaseSequentialStream *chp = (BaseSequentialStream *)&SD1;

/* stato pulsante (debounce a livello, non a fronte:
 * ci serve sapere se e' TENUTO premuto, non solo il click) */
static bool button_last_raw = false;
static bool button_stable   = false;
static systime_t button_changed_t = 0;

/* accumulatori del gesto corrente (azzerati alla pressione,
 * mediati e usati al rilascio) */
static float roll_rate_sum_dps  = 0.0f;
static float pitch_rate_sum_dps = 0.0f;
static uint16_t rate_sample_count = 0U;

/* stato precedente del pulsante, per rilevare i fronti
 * press (falso->vero) e release (vero->falso) */
static bool button_prev_pressed = false;


/* ===================== HW config ================== */
static const I2CConfig i2ccfg = {
  STM32_TIMINGR_PRESC(8U) |
  STM32_TIMINGR_SCLDEL(3U) |
  STM32_TIMINGR_SDADEL(3U) |
  STM32_TIMINGR_SCLH(3U) |
  STM32_TIMINGR_SCLL(9U),
  0,
  0
};

static const ISM330DLC_Platform ism330dlccfg = {
  .i2cp = &I2CD1,
  .i2ccfg = &i2ccfg,
  .address = ISM330DLC_I2C_ADD_L >> 1
};

static const SerialConfig bt_cfg = {
    38400,
    0,
    0,
    0
};

/* ===================== Helpers ==================== */
static inline float rad2deg(float x) {
  return x * 57.2957795f;
}

static float wrap_deg(float a) {
  while (a >= 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

static void acceleration_angles(float ax,
                                float ay,
                                float az,
                                float *roll,
                                float *pitch) {

  *roll  = rad2deg(atan2f(ay, az));

  *pitch = rad2deg(
      atan2f(-ax, sqrtf((ay * ay) + (az * az)))
  );
}


/* pulsante verso GND -> LOW = premuto.
 * Ritorna lo stato STABILE corrente (true = tenuto premuto),
 * non un evento singolo: va chiamata ogni ciclo del loop. */
static bool button_is_pressed(void) {

  systime_t now = chVTGetSystemTimeX();
  bool raw = (palReadLine(ENABLE_BUTTON_LINE) == PAL_LOW);

  if (raw != button_last_raw) {
    button_last_raw = raw;
    button_changed_t = now;
  }

  if (chTimeDiffX(button_changed_t, now) >=
      TIME_MS2I(BUTTON_DEBOUNCE_MS)) {

    button_stable = button_last_raw;
  }

  return button_stable;
}


/* Classifica la velocita' angolare MEDIA di un intero gesto
 * (dal press al release) in un codice 0..4:
 *   0 = fermo (sotto la deadband)
 *   1 = positivo lento    2 = positivo veloce
 *   3 = negativo lento    4 = negativo veloce
 *
 * Nessuna isteresi qui: e' un calcolo one-shot fatto una sola
 * volta al rilascio, sulla media di tutto il gesto (che gia'
 * media via il rumore), quindi non serve memoria tra chiamate. */
static uint8_t classify_gesture(float avg_rate_dps,
                                float deadband_dps,
                                float fast_dps) {

  float mag = fabsf(avg_rate_dps);

  if (mag < deadband_dps) {
    return 0U;
  }

  if (avg_rate_dps > 0.0f) {
    return (mag >= fast_dps) ? 2U : 1U;
  }

  return (mag >= fast_dps) ? 4U : 3U;
}


/* =========================================================
 * CALIBRAZIONE GYRO (bias a bordo fermo, una sola volta)
 * ========================================================= */

static void calibrate_gyro_bias(stmdev_ctx_t *dev_ctx) {

  int32_t sx = 0, sy = 0, sz = 0;
  int16_t graw[3];

#if DEBUG_BOOT_MESSAGES
  chprintf(chp, "\r\n[CAL] Keep pen still...\r\n");
#endif

  for (uint16_t k = 0; k < GYRO_CAL_SAMPLES; k++) {

    if (ism330dlc_angular_rate_raw_get(dev_ctx, graw) == 0) {
      sx += graw[0];
      sy += graw[1];
      sz += graw[2];
    }

    chThdSleepMilliseconds(GYRO_CAL_DELAY_MS);
  }

  int16_t mx =
      (int16_t)(sx / (int32_t)GYRO_CAL_SAMPLES);

  int16_t my =
      (int16_t)(sy / (int32_t)GYRO_CAL_SAMPLES);

  int16_t mz =
      (int16_t)(sz / (int32_t)GYRO_CAL_SAMPLES);

  gyro_bias_dps[0] =
      ism330dlc_from_fs2000dps_to_mdps(mx) / 1000.0f;

  gyro_bias_dps[1] =
      ism330dlc_from_fs2000dps_to_mdps(my) / 1000.0f;

  gyro_bias_dps[2] =
      ism330dlc_from_fs2000dps_to_mdps(mz) / 1000.0f;

#if DEBUG_BOOT_MESSAGES
  chprintf(chp,
           "[CAL] done Bias[dps*1000]=%d,%d,%d\r\n",
           (int)(gyro_bias_dps[0] * 1000.0f),
           (int)(gyro_bias_dps[1] * 1000.0f),
           (int)(gyro_bias_dps[2] * 1000.0f));
#endif
}


/* =========================================================
 * LETTURA IMU
 * ========================================================= */

static bool read_imu_sample(stmdev_ctx_t *dev_ctx,
                            bool *have_acc,
                            bool *have_gyr) {

  ism330dlc_reg_t reg;

  *have_acc = false;
  *have_gyr = false;

  if (ism330dlc_status_reg_get(dev_ctx,
                               &reg.status_reg) != 0) {
    return false;
  }

  if (reg.status_reg.xlda &&
      (ism330dlc_acceleration_raw_get(
          dev_ctx,
          data_raw_acceleration) == 0)) {

    acceleration_mg[0] =
        ism330dlc_from_fs2g_to_mg(
            data_raw_acceleration[0]);

    acceleration_mg[1] =
        ism330dlc_from_fs2g_to_mg(
            data_raw_acceleration[1]);

    acceleration_mg[2] =
        ism330dlc_from_fs2g_to_mg(
            data_raw_acceleration[2]);

    *have_acc = true;
  }

  if (reg.status_reg.gda &&
      (ism330dlc_angular_rate_raw_get(
          dev_ctx,
          data_raw_angular_rate) == 0)) {

    angular_rate_mdps[0] =
        ism330dlc_from_fs2000dps_to_mdps(
            data_raw_angular_rate[0]);

    angular_rate_mdps[1] =
        ism330dlc_from_fs2000dps_to_mdps(
            data_raw_angular_rate[1]);

    angular_rate_mdps[2] =
        ism330dlc_from_fs2000dps_to_mdps(
            data_raw_angular_rate[2]);

    *have_gyr = true;
  }

  if (reg.status_reg.tda &&
      (ism330dlc_temperature_raw_get(
          dev_ctx,
          &data_raw_temperature) == 0)) {

    temperature_degC =
        ism330dlc_from_lsb_to_celsius(
            data_raw_temperature);
  }

  return true;
}


/* ===================== LED thread ================= */
/* Solo un heartbeat visivo, non serve al calcolo.
 * Puoi eliminarlo senza problemi se vuoi il minimo indispensabile. */

static THD_WORKING_AREA(waThread1, 128);

static THD_FUNCTION(Thread1, arg) {

  (void)arg;

  chRegSetThreadName("blinker");

  while (true) {

    palClearLine(LINE_LED_GREEN);

    chThdSleepMilliseconds(500);

    palSetLine(LINE_LED_GREEN);

    chThdSleepMilliseconds(500);
  }
}


/* ======================== MAIN ==================== */

int main(void) {

  halInit();
  chSysInit();


  sdStart(&SD1, &bt_cfg);


  /* Pulsante dead-man switch su PA0/LINE_ARD_A0 verso GND */
  palSetLineMode(
      ENABLE_BUTTON_LINE,
      PAL_MODE_INPUT_PULLUP
  );


  /* I2C pins */
  palSetLineMode(
      PAL_LINE(GPIOA, 9U),

      PAL_MODE_ALTERNATE(4) |
      PAL_STM32_OTYPE_OPENDRAIN |
      PAL_STM32_OSPEED_HIGHEST |
      PAL_STM32_PUPDR_PULLUP
  );


  palSetLineMode(
      PAL_LINE(GPIOA, 10U),

      PAL_MODE_ALTERNATE(4) |
      PAL_STM32_OTYPE_OPENDRAIN |
      PAL_STM32_OSPEED_HIGHEST |
      PAL_STM32_PUPDR_PULLUP
  );


  /*
  PB6 = TX
  PB7 = RX
  AF0*/

  palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(0));
  palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(0));


  /* =====================================================
   * THREAD LED (heartbeat)
   * ===================================================== */

  chThdCreateStatic(
      waThread1,
      sizeof(waThread1),
      NORMALPRIO + 1,
      Thread1,
      NULL
  );


  stmdev_ctx_t dev_ctx;

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg  = platform_read;
  dev_ctx.mdelay    = platform_delay;
  dev_ctx.handle    = (void *)&ism330dlccfg;


  platform_delay(BOOT_TIME_MS);


  /* WHO_AM_I */

  ism330dlc_device_id_get(
      &dev_ctx,
      &whoamI
  );


  if (whoamI != ISM330DLC_ID) {

    chprintf(
        chp,
        "ISM330DLC not found (0x%02X)\r\n",
        whoamI
    );

    while (1) {
    }
  }


#if DEBUG_BOOT_MESSAGES
  chprintf(
      chp,
      "ISM330DLC OK (0x%02X)\r\n",
      whoamI
  );
#endif


  /* Reset & config sensore */

  ism330dlc_reset_set(
      &dev_ctx,
      PROPERTY_ENABLE
  );


  do {

    ism330dlc_reset_get(
        &dev_ctx,
        &rst
    );

  } while (rst);


  ism330dlc_block_data_update_set(
      &dev_ctx,
      PROPERTY_ENABLE
  );


  ism330dlc_xl_data_rate_set(
      &dev_ctx,
      ISM330DLC_XL_ODR_104Hz
  );


  ism330dlc_gy_data_rate_set(
      &dev_ctx,
      ISM330DLC_GY_ODR_104Hz
  );


  ism330dlc_xl_full_scale_set(
      &dev_ctx,
      ISM330DLC_2g
  );


  ism330dlc_gy_full_scale_set(
      &dev_ctx,
      ISM330DLC_2000dps
  );


  ism330dlc_xl_filter_analog_set(
      &dev_ctx,
      ISM330DLC_XL_ANA_BW_400Hz
  );


  ism330dlc_xl_lp2_bandwidth_set(
      &dev_ctx,
      ISM330DLC_XL_LOW_NOISE_LP_ODR_DIV_100
  );


  ism330dlc_gy_band_pass_set(
      &dev_ctx,
      ISM330DLC_HP_260mHz_LP1_STRONG
  );


  /* calibrazione gyro una sola volta */

  calibrate_gyro_bias(
      &dev_ctx
  );


  /* flush eventuale vecchio sample accel */

  (void)ism330dlc_acceleration_raw_get(
      &dev_ctx,
      data_raw_acceleration
  );


  systime_t last_t =
      chVTGetSystemTimeX();

  bool attitude_initialized = false;


#if DEBUG_BOOT_MESSAGES
  chprintf(
      chp,
      "[RUN] Premi il pulsante, muovi la penna, rilascia per inviare il comando\r\n"
  );
#endif


  /* =====================================================
   * MAIN LOOP
   * ===================================================== */

  while (1) {

    bool have_acc = false;
    bool have_gyr = false;


    /*
     * Lettura IMU
     */

    if (!read_imu_sample(
            &dev_ctx,
            &have_acc,
            &have_gyr)) {

      chThdSleepMilliseconds(
          MAIN_LOOP_DELAY_MS
      );

      continue;
    }


    if (have_acc && have_gyr) {

      systime_t now_t =
          chVTGetSystemTimeX();


      float dt =
          ((float)(now_t - last_t)) /
          (float)CH_CFG_ST_FREQUENCY;


      last_t = now_t;


      if ((dt <= 0.0f) ||
          (dt > 0.2f)) {

        dt = 0.01f;
      }


      float ax =
          acceleration_mg[0] / 1000.0f;

      float ay =
          acceleration_mg[1] / 1000.0f;

      float az =
          acceleration_mg[2] / 1000.0f;


      float gx =
          (angular_rate_mdps[0] / 1000.0f)
          - gyro_bias_dps[0];

      float gy =
          (angular_rate_mdps[1] / 1000.0f)
          - gyro_bias_dps[1];

      float gz =
          (angular_rate_mdps[2] / 1000.0f)
          - gyro_bias_dps[2];


      float roll_acc;
      float pitch_acc;


      acceleration_angles(
          ax,
          ay,
          az,
          &roll_acc,
          &pitch_acc
      );


      /* =================================================
       * FILTRO COMPLEMENTARE (roll + pitch)
       * ================================================= */

      if (!attitude_initialized) {

        roll_deg =
            wrap_deg(roll_acc);

        pitch_deg =
            pitch_acc;

        attitude_initialized =
            true;

      }

      else {

        float roll_pred =
            wrap_deg(
                roll_deg +
                gx * dt
            );


        roll_deg =
            wrap_deg(
                roll_pred +
                (1.0f - COMP_ALPHA) *
                wrap_deg(
                    roll_acc -
                    roll_pred
                )
            );


        pitch_deg =
            COMP_ALPHA *
            (pitch_deg + gy * dt)
            +
            (1.0f - COMP_ALPHA) *
            pitch_acc;
      }


      yaw_deg += gz * dt; /* non usato, solo diagnostica */


      /* =================================================
       * CATTURA GESTO (press -> release) + INVIO ONE-SHOT
       *
       * Mentre il pulsante e' tenuto premuto, accumulo i
       * campioni di gx/gy (roll/pitch rate). Al RILASCIO
       * calcolo la media di tutto il gesto appena fatto,
       * la classifico, e mando UN SOLO comando.
       *
       * Se premi e molli subito senza muovere: media ~0 ->
       * manda 0 (stop). Se muovi e poi molli: manda il codice
       * corrispondente, una volta sola, finche' non rifai
       * un altro gesto (nuovo press+release).
       * ================================================= */

      bool now_pressed = button_is_pressed();


      if (now_pressed && !button_prev_pressed) {

        /* fronte di pressione: inizio un nuovo gesto,
         * azzero gli accumulatori del gesto precedente */

        roll_rate_sum_dps  = 0.0f;
        pitch_rate_sum_dps = 0.0f;
        rate_sample_count  = 0U;
      }


      if (now_pressed) {

        /* pulsante tenuto: accumulo il campione nel gesto
         * in corso */

        roll_rate_sum_dps  += gx;
        pitch_rate_sum_dps += gy;
        rate_sample_count++;
      }


      if (!now_pressed && button_prev_pressed) {

        /* fronte di rilascio: gesto concluso, calcolo la
         * media e mando il comando, una volta sola */

        float roll_rate_avg_dps  = 0.0f;
        float pitch_rate_avg_dps = 0.0f;

        if (rate_sample_count > 0U) {

          roll_rate_avg_dps =
              roll_rate_sum_dps /
              (float)rate_sample_count;

          pitch_rate_avg_dps =
              pitch_rate_sum_dps /
              (float)rate_sample_count;
        }


        uint8_t roll_cmd = classify_gesture(
            roll_rate_avg_dps,
            ROLL_RATE_DEADBAND_DPS,
            ROLL_RATE_FAST_DPS
        );

        uint8_t pitch_cmd = classify_gesture(
            pitch_rate_avg_dps,
            PITCH_RATE_DEADBAND_DPS,
            PITCH_RATE_FAST_DPS
        );


        /* Un solo asse per gesto: quello con la media di velocita'
         * angolare piu' alta (in valore assoluto) vince, l'altro
         * viene ignorato per questo gesto. Se nessuno dei due
         * supera la propria soglia (entrambi classificati 0),
         * mando 'S' '0' invece di scegliere un asse a caso: e'
         * il codice esplicito di "fermo, nessun asse mosso". */

        char axis_char;
        uint8_t axis_cmd;

        if ((roll_cmd == 0U) && (pitch_cmd == 0U)) {

          axis_char = 'S';
          axis_cmd  = 0U;

        } else if (fabsf(roll_rate_avg_dps) >=
                   fabsf(pitch_rate_avg_dps)) {

          axis_char = 'X';   /* roll (fusto) vince */
          axis_cmd  = roll_cmd;

        } else {

          axis_char = 'Y';   /* pitch (testa) vince */
          axis_cmd  = pitch_cmd;
        }

        char code_char = (char)('0' + axis_cmd);

        chprintf(
            chp,
            "%c%c\r\n",
            axis_char,
            code_char
        );

#if DEBUG_PRINT_RATES
        chprintf(
            chp,
            "RATE_AVG[cdeg/s] roll=%d code=%d pitch=%d code=%d -> inviato asse=%c cmd=%d\r\n",
            (int)(roll_rate_avg_dps * 100.0f),
            (int)roll_cmd,
            (int)(pitch_rate_avg_dps * 100.0f),
            (int)pitch_cmd,
            axis_char,
            (int)axis_cmd
        );
#endif

        /* pronto per il prossimo gesto */

        roll_rate_sum_dps  = 0.0f;
        pitch_rate_sum_dps = 0.0f;
        rate_sample_count  = 0U;
      }


      button_prev_pressed = now_pressed;
    }


    chThdSleepMilliseconds(
        MAIN_LOOP_DELAY_MS
    );
  }
}
