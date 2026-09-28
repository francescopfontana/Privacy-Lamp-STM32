#include "ch.h"
#include "hal.h"
#include "chprintf.h"

/*
 * ============================================================================
 *  Pan/Tilt a 2 servomotori STANDARD (posizionali, con potenziometro interno)
 *  Scheda: NUCLEO-STM32G474RE - ChibiOS/RT
 *
 *  Differenza rispetto alla versione precedente:
 *  I servo montati PRIMA erano a rotazione continua (senza potenziometro,
 *  pilotati "a velocita'": 1500us = fermo, <1500 = gira CCW, >1500 = gira CW).
 *  Per questo il vecchio codice li "spingeva" per 700ms e poi li fermava,
 *  cercando di indovinare l'angolo (angolo_servo_x/y era solo un contatore
 *  software, nessun controllo reale di posizione).
 *
 *  I servo NUOVI sono posizionali standard: hanno un potenziometro interno
 *  che chiude l'anello di controllo DENTRO al servo stesso. Al microcontrollore
 *  basta mandare l'impulso PWM corrispondente all'angolo voluto (1-2 ms ogni
 *  20 ms) e il servo ci va da solo e ci resta. Niente ADC, niente lettura
 *  del potenziometro da parte nostra: e' tutto interno al servo.
 *
 *  Per questo qui sotto NON si "spinge a tempo" ma si comanda un angolo
 *  assoluto, raggiunto pero' a piccoli passi (slew-rate limiting) per non
 *  fare scatti bruschi che assorbono un picco di corrente sui cavi.
 *
 *  COMANDO DA BLUETOOTH (master = penna con IMU, su SD1/USART1 PB6-PB7):
 *  il master manda un frame di 2 caratteri per gesto:
 *    'X' + cifra(0..4)  -> Servo X (canale 0, PA6): 0=fermo 1=+lento
 *                          2=+veloce 3=-lento 4=-veloce
 *    'Y' + cifra(0..4)  -> Servo Y (canale 1, PA7), stessa scala
 *    'S' + '0'           -> nessun asse mosso, nessuna azione
 *  "veloce" = passo doppio in un solo comando rispetto a "lento", vedi
 *  applica_comando_master(). Lo stesso parser legge anche SD2 (terminale
 *  PC), utile per testare mandando a mano "X1", "Y3", "S0" senza il
 *  modulo HC-05 collegato.
 *
 *  PULSANTI MANUALI (PC7/PA9/PB10/PA8, switch A0 = HIGH): un tocco fa
 *  un passo, tenuti premuti continuano a fare un passo ogni
 *  MANUAL_REPEAT_PERIOD_MS finche' non li rilasci (vedi step_dovuto()).
 * ============================================================================
 */

BaseSequentialStream *chp = (BaseSequentialStream*) &SD2;

/* ============================================================================
 * [1. SOGLIE DI SICUREZZA E TEMPORIZZAZIONI - TOCCA SOLO QUI]
 * ============================================================================ */

/* --- Base tempi PWM: NON toccare, e' lo standard dei servo a 50 Hz --- */
#define PWM_TIMER_FREQUENCY      1000000U  /* Clock PWM 1 MHz -> 1 tick = 1 us */
#define PWM_PERIOD                 20000U  /* Periodo 20 ms => 50 Hz           */

/*
 * --- Limiti elettrici/meccanici dell'impulso (in microsecondi) ---
 * Il datasheet del tuo servo di solito dichiara 500-2500us come corsa
 * massima teorica, ma spingerlo fino ai valori estremi lo fa sbattere contro
 * il fondo corsa meccanico: li' il motorino stalla e assorbe la corrente di
 * stallo (che sui cavetti sottili dei servo piccoli e' quello che "fa
 * esplodere i cavi"/scalda i connettori). Valori prudenti, si possono
 * allargare con cautela dopo aver verificato la corsa libera del tuo servo:
 */
#define SERVO_MIN_PULSE_US            900U   /* impulso al SERVO_MIN_ANGLE_DEG */
#define SERVO_MAX_PULSE_US           2100U   /* impulso al SERVO_MAX_ANGLE_DEG */
#define SERVO_CENTER_PULSE_US         1500U  /* impulso a 90 deg (centro)      */

/*
 * --- Limiti logici di angolo che il codice puo' comandare ---
 * IMPORTANTE: questi limiti NON sono ridondanti col potenziometro interno
 * del servo. Il potenziometro serve al servo per sapere DOV'E' e chiudere
 * il suo anello di controllo, ma non gli impedisce di provare ad andare
 * oltre la corsa meccanica se glielo comandiamo noi: se mandiamo un
 * impulso che corrisponde a un angolo oltre il fine corsa, il servo spinge
 * comunque contro il fermo meccanico e stalla, assorbendo la corrente di
 * stallo sugli stessi cavetti sottili che vogliamo proteggere. Il limite
 * software resta quindi necessario: usalo per fermarti UN PO' PRIMA del
 * fine corsa reale del tuo servo (misuralo a mano ruotando l'albero) invece
 * di toglierlo.
 */
#define SERVO_MIN_ANGLE_DEG              0
#define SERVO_MAX_ANGLE_DEG            180
#define SERVO_START_ANGLE_DEG            90   /* posizione di parcheggio all'accensione */

/* --- Passo di movimento per ogni pressione pulsante / comando seriale --- */
#define SERVO_STEP_DEG                   10

/* --- Moltiplicatore del passo per i comandi Bluetooth "veloce" (cifra
 * 2 o 4, vedi applica_comando_master) rispetto a "lento" (cifra 1 o 3).
 * Es. con 2: "veloce" = 20 gradi in un comando, "lento" = 10. Non tocca
 * i pulsanti manuali, che usano sempre SERVO_STEP_DEG. --- */
#define SERVO_STEP_VELOCE_MULT            2

/*
 * --- Slew-rate limiting (LA vera protezione dei cavi) ---
 * Invece di saltare di colpo alla posizione target, ci si arriva a piccoli
 * passi. Questo limita quanta corrente di picco il servo puo' chiedere in
 * un istante e ammortizza gli scatti meccanici. Aumenta SERVO_SLEW_STEP_DELAY_MS
 * (o riduci SERVO_SLEW_STEP_DEG) se senti/vedi i servo scattare troppo forte
 * o se l'alimentazione va in crisi (LED che sfarfalla, reset random della board).
 */
#define SERVO_SLEW_STEP_DEG               1   /* gradi per ogni incremento */
#define SERVO_SLEW_STEP_DELAY_MS         15   /* ms tra un incremento e il successivo */

/* --- Stagger dei due servo all'avvio: non partono nello stesso istante --- */
#define SERVO_STARTUP_STAGGER_MS        250

/* --- Debounce pulsanti e periodo del loop principale --- */
#define BUTTON_DEBOUNCE_MS                20
#define LOOP_PERIOD_MS                    20

/* --- Ripetizione pulsanti manuali tenuti premuti ---
 * Un pulsante manuale (PC7/PA9/PB10/PA8) fa subito un passo al
 * primo tocco, poi, se resta premuto, continua a fare un passo
 * ogni MANUAL_REPEAT_PERIOD_MS finche' non lo rilasci (prima
 * faceva un solo passo per pressione, serviva ripremere ogni volta). */
#define MANUAL_REPEAT_PERIOD_MS          150

/*
 * --- Bit del registro RCC->CSR (causa dell'ultimo reset) ---
 * Usati per capire se la board si e' riavviata da sola (es. brown-out per
 * assorbimento eccessivo dei servo) invece che per un bug software: in tal
 * caso le variabili statiche si reinizializzano e il servo "torna" alla
 * posizione di parcheggio, dando l'impressione che il comando non resti
 * settato. Scritti come maschere sui bit (uguali su tutta la famiglia
 * STM32, layout da RM0440) per non dipendere dai nomi esatti delle macro
 * nella versione di CMSIS installata. */
#define RESETCAUSE_BOR_MASK    (1UL << 27)  /* Brown-out / calo di tensione   */
#define RESETCAUSE_PIN_MASK    (1UL << 26)  /* Reset da pin NRST              */
#define RESETCAUSE_SFT_MASK    (1UL << 28)  /* Reset software                 */
#define RESETCAUSE_IWDG_MASK   (1UL << 29)  /* Watchdog indipendente          */
#define RESETCAUSE_WWDG_MASK   (1UL << 30)  /* Window watchdog                */
#define RESETCAUSE_LPWR_MASK   (1UL << 31)  /* Reset da low-power             */
#define RESETCAUSE_RMVF_MASK   (1UL << 24)  /* Bit di clear dei flag (RMVF)   */

/* ============================================================================
 * [2. PINOUT COMPLETO - NUCLEO-STM32G474RE]
 * ============================================================================
 *
 *  FUNZIONE                    | PIN MCU | PIN Arduino | Note
 *  -----------------------------------------------------------------------
 *  Servo X - segnale PWM       | PA6     | D12         | TIM3_CH1 (AF2)
 *  Servo Y - segnale PWM       | PA7     | D11         | TIM3_CH2 (AF2)
 *  Servo X/Y - alimentazione   | ESTERNA | -           | VEDI NOTA ALIMENTAZIONE sotto
 *  Servo X/Y - massa (GND)     | GND     | GND         | Comune tra Nucleo e alimentatore esterno
 *  Switch modalita' manuale/seriale | PA0 | A0         | INPUT_PULLUP (HIGH=manuale, LOW=seriale)
 *  Pulsante Servo X +step      | PC7     | -           | INPUT_PULLUP (attivo basso)
 *  Pulsante Servo X -step      | PA9     | D8          | INPUT_PULLUP (attivo basso)
 *  Pulsante Servo Y +step      | PB10    | D6          | INPUT_PULLUP (attivo basso)
 *  Pulsante Servo Y -step      | PA8     | D7          | INPUT_PULLUP (attivo basso)
 *  LED di stato                | PA5     | LD2 (onboard)| Toggle a ogni comando seriale ricevuto
 *  USART2 TX (debug console)   | PA2     | D1/TX       | 38400 8N1, verso ST-LINK VCP
 *  USART2 RX (debug console)   | PA3     | D0/RX       | 38400 8N1
 *
 *  NOTA ALIMENTAZIONE SERVO (leggere prima di collegare!):
 *  - Il filo di SEGNALE dei servo va SEMPRE e SOLO su PA6/PA7 (uscita 3.3V
 *    della Nucleo). Va benissimo: quasi tutti i servo leggono un segnale
 *    3.3V come "alto" valido.
 *  - Il filo di ALIMENTAZIONE (rosso, V+) dei servo NON va preso dal pin 5V
 *    della Nucleo: quel pin e' alimentato dall'USB/ST-LINK e regge si' e no
 *    ~300-500mA in totale. Due servo, soprattutto sotto sforzo o allo
 *    stallo, possono chiedere anche diversi Ampere: si rischia di far
 *    resettare o friggere la board, oltre che gli stessi cavi sottili del
 *    connettore servo.
 *  - Usa un alimentatore ESTERNO da 5V-6V (o quanto richiesto dal tuo servo)
 *    dimensionato sulla corrente di stallo del servo (guarda il datasheet),
 *    con GND messo IN COMUNE con il GND della Nucleo (altrimenti il segnale
 *    PWM non ha un riferimento e i servo si comportano in modo erratico).
 *  - Un condensatore da 470-1000uF tra V+ e GND vicino ai servo aiuta ad
 *    assorbire i picchi di corrente allo spunto.
 * ============================================================================ */

#define SWITCH_MODE_PORT       GPIOA
#define SWITCH_MODE_PAD        0

#define BTN_LEFT_PORT          GPIOC   /* PC7  -> Servo X +step */
#define BTN_LEFT_PAD           7

#define BTN_RIGHT_PORT         GPIOA   /* PA9 (D8) -> Servo X -step */
#define BTN_RIGHT_PAD          9

#define BTN_UP_PORT            GPIOB   /* PB10 (D6) -> Servo Y +step */
#define BTN_UP_PAD             10

#define BTN_DOWN_PORT          GPIOA   /* PA8 (D7) -> Servo Y -step */
#define BTN_DOWN_PAD           8

/* ============================================================================
 * [3. VARIABILI DI STATO GLOBALI]
 * ============================================================================ */

/* Angolo comandato attuale (0-180). A differenza della versione precedente
 * questo E' l'angolo reale del servo (i servo posizionali vanno esattamente
 * dove li mandi), non una stima software. */
static int angolo_servo_x = SERVO_START_ANGLE_DEG;
static int angolo_servo_y = SERVO_START_ANGLE_DEG;

static bool prev_left  = false;
static bool prev_right = false;
static bool prev_up    = false;
static bool prev_down  = false;

/* ============================================================================
 * [3bis. DEBOUNCE PULSANTI]
 * ============================================================================
 * Senza un debounce a tempo, un contatto che "rimbalza" puo' generare piu'
 * fronti di salita ravvicinati: sembra che il servo "torni indietro" ma in
 * realta' ha ricevuto un secondo comando (magari nella direzione opposta)
 * a pochi millisecondi dal primo. Qui si accetta un nuovo stato solo se
 * resta stabile per almeno BUTTON_DEBOUNCE_MS.
 */
typedef struct {
  bool     stato_stabile;
  bool     ultimo_letto;
  systime_t ultimo_cambio;
  systime_t ultimo_step;   /* istante dell'ultimo passo mandato, per la ripetizione */
} debounce_t;

static debounce_t db_left, db_right, db_up, db_down;

static bool debounce_leggi(debounce_t *db, bool raw) {
  systime_t adesso = chVTGetSystemTimeX();

  if (raw != db->ultimo_letto) {
    db->ultimo_letto  = raw;
    db->ultimo_cambio = adesso;
  }

  if (chTimeDiffX(db->ultimo_cambio, adesso) >= TIME_MS2I(BUTTON_DEBOUNCE_MS)) {
    db->stato_stabile = db->ultimo_letto;
  }

  return db->stato_stabile;
}

/* Un pulsante manuale deve fare un passo: subito al primo tocco
 * (fronte premuto), poi di nuovo ogni MANUAL_REPEAT_PERIOD_MS finche'
 * resta premuto (tenuto = "premuto_adesso" stabile a true per piu'
 * cicli). Stesso schema usato per il gesto del master via BT, solo
 * applicato ai pulsanti fisici invece che alla classificazione IMU. */
static bool step_dovuto(debounce_t *db, bool premuto_adesso, bool premuto_prima) {

  systime_t adesso = chVTGetSystemTimeX();

  if (premuto_adesso && !premuto_prima) {
    /* fronte di pressione: passo immediato, riparte il timer */
    db->ultimo_step = adesso;
    return true;
  }

  if (premuto_adesso && premuto_prima) {
    /* tenuto premuto: un passo in piu' ogni MANUAL_REPEAT_PERIOD_MS */
    if (chTimeDiffX(db->ultimo_step, adesso) >= TIME_MS2I(MANUAL_REPEAT_PERIOD_MS)) {
      db->ultimo_step = adesso;
      return true;
    }
  }

  return false;
}

/* ============================================================================
 * [4. CONFIGURAZIONI PERIFERICHE]
 * ============================================================================ */

static const SerialConfig sdcfg = {
  38400, 0, 0, 0
};

/* SD1 = USART1 su PB6(TX)/PB7(RX), stesso baud del MASTER (penna+IMU) */
static const SerialConfig bt_cfg = {
  38400, 0, 0, 0
};

/* TIM3 (PWMD3): CH1 su PA6 (D12) e CH2 su PA7 (D11) */
static PWMConfig pwmcfg_tim3 = {
  .frequency = PWM_TIMER_FREQUENCY,
  .period    = PWM_PERIOD,
  .callback  = NULL,
  .channels  = {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},          /* CH1 -> PA6 (D12 - Servo X) */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},          /* CH2 -> PA7 (D11 - Servo Y) */
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL}
  },
  .cr2  = 0,
  .dier = 0
};

/* ============================================================================
 * [5. CONVERSIONE ANGOLO -> IMPULSO PWM]
 * ============================================================================ */

static int clamp_int(int val, int min_val, int max_val) {
  if (val < min_val) return min_val;
  if (val > max_val) return max_val;
  return val;
}

static uint16_t angolo_a_impulso_us(int angolo) {
  int angolo_clamp = clamp_int(angolo, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG);

  /* Interpolazione lineare tra i due estremi meccanici sicuri */
  long range_pulse = (long)SERVO_MAX_PULSE_US - (long)SERVO_MIN_PULSE_US;
  long range_angle = (long)SERVO_MAX_ANGLE_DEG - (long)SERVO_MIN_ANGLE_DEG;

  long impulso = (long)SERVO_MIN_PULSE_US +
                 ((long)(angolo_clamp - SERVO_MIN_ANGLE_DEG) * range_pulse) / range_angle;

  return (uint16_t)impulso;
}

/* ============================================================================
 * [6. MOVIMENTO A RAMPA (slew-rate limiting) - QUESTO PROTEGGE I CAVI]
 * ============================================================================ */

static void muovi_servo_a_step_n(uint8_t canale, int *angolo_attuale, bool incrementa,
                                  int passo_deg, const char *sorgente) {
  int angolo_target = *angolo_attuale + (incrementa ? passo_deg : -passo_deg);

  if (incrementa && *angolo_attuale >= SERVO_MAX_ANGLE_DEG) {
    chprintf(chp, " [%s] Limite massimo (+%d deg) RAGGIUNTO!\r\n", sorgente, SERVO_MAX_ANGLE_DEG);
    return;
  }
  if (!incrementa && *angolo_attuale <= SERVO_MIN_ANGLE_DEG) {
    chprintf(chp, " [%s] Limite minimo (%d deg) RAGGIUNTO!\r\n", sorgente, SERVO_MIN_ANGLE_DEG);
    return;
  }

  angolo_target = clamp_int(angolo_target, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG);

  chprintf(chp, " [%s] CH%d -> Target %d deg (da %d deg)\r\n",
           sorgente, canale + 1, angolo_target, *angolo_attuale);

  /* Rampa a piccoli passi invece di un salto secco: e' questo che tiene
   * bassa la corrente di picco richiesta ai cavi/alimentatore. Il passo
   * della RAMPA (SERVO_SLEW_STEP_DEG) resta sempre lo stesso, piccolo:
   * quello che cambia con "passo_deg" e' solo QUANTO LONTANO arriva il
   * target (piu' gradi per un comando "veloce" dal master via BT). */
  while (*angolo_attuale != angolo_target) {
    if (*angolo_attuale < angolo_target) {
      *angolo_attuale = clamp_int(*angolo_attuale + SERVO_SLEW_STEP_DEG,
                                   SERVO_MIN_ANGLE_DEG, angolo_target);
    } else {
      *angolo_attuale = clamp_int(*angolo_attuale - SERVO_SLEW_STEP_DEG,
                                   angolo_target, SERVO_MAX_ANGLE_DEG);
    }
    pwmEnableChannel(&PWMD3, canale, angolo_a_impulso_us(*angolo_attuale));
    chThdSleepMilliseconds(SERVO_SLEW_STEP_DELAY_MS);
  }
}

/* Wrapper compatibile con le vecchie chiamate (pulsanti manuali):
 * sempre un passo di SERVO_STEP_DEG. */
static void muovi_servo_a_step(uint8_t canale, int *angolo_attuale, bool incrementa,
                                const char *sorgente) {
  muovi_servo_a_step_n(canale, angolo_attuale, incrementa, SERVO_STEP_DEG, sorgente);
}

/* ============================================================================
 * [6bis. PARSER PROTOCOLLO MASTER (Bluetooth / test da terminale PC)]
 * ============================================================================
 * Il master (penna con IMU) manda, ad ogni gesto, un frame di 2 caratteri:
 *
 *   'X' + cifra(0..4)  -> gesto sul ROLL  (fusto) -> Servo X (canale 0, PA6)
 *   'Y' + cifra(0..4)  -> gesto sul PITCH (testa) -> Servo Y (canale 1, PA7)
 *   'S' + '0'          -> nessun asse mosso, fermo esplicito (nessuna azione)
 *
 * cifra: 0=fermo (nessuna azione) 1=+lento 2=+veloce 3=-lento 4=-veloce.
 * "Veloce" muove semplicemente un passo doppio rispetto a "lento" in un
 * solo comando (stessa rampa piccola SERVO_SLEW_STEP_DEG, target piu' lontano).
 *
 * Il parser e' a 2 stati (aspetto il tag 'X'/'Y'/'S', poi aspetto la
 * cifra che segue) cosi' da leggere il frame di 2 byte indipendentemente
 * da dove arriva ('\r'/'\n' o rumore ricevuti mentre aspettiamo il tag
 * vengono ignorati senza rompere la sincronizzazione). Ogni sorgente
 * (Bluetooth / terminale PC) ha il proprio stato, cosi' se sono
 * entrambe cablate non si mescolano a meta' frame. */

typedef struct {
  bool in_attesa_cifra;
  char asse;
} parser_master_t;

static parser_master_t parser_bt = { false, 0 };
static parser_master_t parser_pc = { false, 0 };

static void applica_comando_master(char asse, char cifra_char, const char *sorgente) {

  if (asse == 'S') {
    /* fermo esplicito: nessuna azione, i servo tengono la posizione */
    return;
  }

  if ((cifra_char < '0') || (cifra_char > '4')) {
    /* frame corrotto/non riconosciuto: ignora senza toccare i servo */
    return;
  }

  int cifra = cifra_char - '0';

  if (cifra == 0) {
    return; /* fermo su questo asse */
  }

  int *angolo;
  uint8_t canale;

  if (asse == 'X') {
    angolo = &angolo_servo_x;
    canale = 0;
  } else if (asse == 'Y') {
    angolo = &angolo_servo_y;
    canale = 1;
  } else {
    return; /* tag non riconosciuto */
  }

  bool incrementa = (cifra == 1) || (cifra == 2);   /* 1,2 = verso positivo */
  bool veloce     = (cifra == 2) || (cifra == 4);   /* 2,4 = veloce */

  int passo = veloce ? (SERVO_STEP_DEG * SERVO_STEP_VELOCE_MULT) : SERVO_STEP_DEG;

  muovi_servo_a_step_n(canale, angolo, incrementa, passo, sorgente);
}

static void elabora_char_master(parser_master_t *stato, char c, const char *sorgente) {

  if (!stato->in_attesa_cifra) {

    if ((c == 'X') || (c == 'Y') || (c == 'S')) {
      stato->asse = c;
      stato->in_attesa_cifra = true;
    }
    /* altro carattere (\r \n rumore) mentre aspettiamo il tag: ignora */

  } else {

    /* Log del frame COSI' COM'E' ARRIVATO, prima di interpretarlo:
     * utile per vedere sul terminale PC (SD2) tutto quello che il
     * master manda via Bluetooth, anche i comandi "fermo" (S0) o un
     * frame corrotto, non solo i movimenti effettivi. */
    chprintf(chp, "[%s] RX: %c%c\r\n", sorgente, stato->asse, c);

    applica_comando_master(stato->asse, c, sorgente);
    stato->in_attesa_cifra = false;
  }
}

/* ============================================================================
 * [MAIN]
 * ============================================================================ */

int main(void) {

  /* Leggo la causa dell'ultimo reset PRIMA di inizializzare qualunque cosa,
   * cosi' i flag non vengono persi, e li ripulisco subito dopo per il
   * prossimo giro. Serve a diagnosticare un eventuale riavvio inatteso
   * della board (es. brown-out) invece di un bug nel codice. */
  uint32_t causa_reset = RCC->CSR;
  RCC->CSR |= RESETCAUSE_RMVF_MASK;

  halInit();
  chSysInit();

  /* Abilitazione Clock per GPIOA, GPIOB e GPIOC */
  rccEnableAHB2(RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN, true);

  /* LED2 di stato (PA5) */
  palSetPadMode(GPIOA, 5, PAL_MODE_OUTPUT_PUSHPULL);

  /* USART2 per Console/Debugging (PA2 = TX, PA3 = RX) */
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

  /* Configurazione Pin PWM TIM3: PA6/D12 (CH1) e PA7/D11 (CH2) */
  palSetPadMode(GPIOA, 6, PAL_MODE_ALTERNATE(2)); /* PA6 AF2 -> TIM3_CH1 */
  palSetPadMode(GPIOA, 7, PAL_MODE_ALTERNATE(2)); /* PA7 AF2 -> TIM3_CH2 */

  /* Configurazione Switch Modalita' (A0 / PA0) */
  palSetPadMode(SWITCH_MODE_PORT, SWITCH_MODE_PAD, PAL_MODE_INPUT_PULLUP);

  /* Configurazione Pulsanti Manuali */
  palSetPadMode(BTN_LEFT_PORT,  BTN_LEFT_PAD,  PAL_MODE_INPUT_PULLUP); /* PC7 */
  palSetPadMode(BTN_RIGHT_PORT, BTN_RIGHT_PAD, PAL_MODE_INPUT_PULLUP); /* PA9 */
  palSetPadMode(BTN_UP_PORT,    BTN_UP_PAD,    PAL_MODE_INPUT_PULLUP); /* PB10 */
  palSetPadMode(BTN_DOWN_PORT,  BTN_DOWN_PAD,  PAL_MODE_INPUT_PULLUP); /* PA8 */

  sdStart(&SD2, &sdcfg);

  /* Bluetooth HC-05 verso il MASTER: PB6=USART1_TX, PB7=USART1_RX, AF7 su G4 */
  palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(7));
  sdStart(&SD1, &bt_cfg);

  /* Diagnostica causa dell'ultimo reset: se vedi comparire questo banner
   * da solo (senza che tu abbia premuto reset o ricollegato l'USB) proprio
   * quando muovi un servo, il problema NON e' nel codice ma nell'alimentazione
   * (la board si sta resettando per calo di tensione). */
  chprintf(chp, "\r\nCausa ultimo reset (raw=0x%08lX): ", (unsigned long)causa_reset);
  if (causa_reset & RESETCAUSE_BOR_MASK)  chprintf(chp, "BROWN-OUT(ALIMENTAZIONE!) ");
  if (causa_reset & RESETCAUSE_PIN_MASK)  chprintf(chp, "PIN-RESET/NRST ");
  if (causa_reset & RESETCAUSE_SFT_MASK)  chprintf(chp, "SOFTWARE ");
  if (causa_reset & RESETCAUSE_IWDG_MASK) chprintf(chp, "IWDG ");
  if (causa_reset & RESETCAUSE_WWDG_MASK) chprintf(chp, "WWDG ");
  if (causa_reset & RESETCAUSE_LPWR_MASK) chprintf(chp, "LOW-POWER ");
  chprintf(chp, "\r\n");
  if (causa_reset & RESETCAUSE_BOR_MASK) {
    chprintf(chp, "!!! ATTENZIONE: brown-out rilevato. Controlla alimentazione\r\n");
    chprintf(chp, "!!! esterna dei servo e massa (GND) in comune con la Nucleo.\r\n");
  }

  /* Avvio Timer PWM TIM3 */
  pwmStart(&PWMD3, &pwmcfg_tim3);
  TIM3->BDTR |= TIM_BDTR_MOE;

  /* Inizializzazione servomotori in posizione di parcheggio, SFALSATA nel
   * tempo tra i due canali per non far assorbire a entrambi i servo il
   * picco di corrente di spunto nello stesso identico istante. */
  pwmEnableChannel(&PWMD3, 0, angolo_a_impulso_us(angolo_servo_x)); /* PA6 / D12 */
  chThdSleepMilliseconds(SERVO_STARTUP_STAGGER_MS);
  pwmEnableChannel(&PWMD3, 1, angolo_a_impulso_us(angolo_servo_y)); /* PA7 / D11 */

  chprintf(chp, "\r\n===================================================\r\n");
  chprintf(chp, "   PAN/TILT SERVO POSIZIONALI (con potenziometro)   \r\n");
  chprintf(chp, "   FIRMWARE v4 - compilato il %s %s\r\n", __DATE__, __TIME__);
  chprintf(chp, "===================================================\r\n");
  chprintf(chp, "SWITCH (A0): HIGH = MANUALE | LOW = SERIALE\r\n");
  chprintf(chp, "MANUALE    : PC7 (+%dX) | PA9 (-%dX) | PB10 (+%dY) | PA8 (-%dY) -- tienili premuti per continuare a girare\r\n",
           SERVO_STEP_DEG, SERVO_STEP_DEG, SERVO_STEP_DEG, SERVO_STEP_DEG);
  chprintf(chp, "SERIALE    : Bluetooth SD1 (PB6/PB7) + terminale PC SD2, comandi X/Y+cifra 0-4, S0=fermo\r\n");
  chprintf(chp, "Angolo iniziale: X=%d deg Y=%d deg (limiti %d-%d deg)\r\n\r\n",
           angolo_servo_x, angolo_servo_y, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG);

  bool modalita_manuale_attiva = false;
  bool prima_esecuzione = true;

  while (true) {

    bool stato_switch = (palReadPad(SWITCH_MODE_PORT, SWITCH_MODE_PAD) == PAL_HIGH);

    if (stato_switch != modalita_manuale_attiva || prima_esecuzione) {
      modalita_manuale_attiva = stato_switch;
      prima_esecuzione = false;

      if (modalita_manuale_attiva) {
        chprintf(chp, "\r\n>>> MODALITA MANUALE (PC7, PA9, PB10, PA8) <<<\r\n");
      } else {
        chprintf(chp, "\r\n>>> MODALITA SERIALE (Bluetooth master: X/Y+cifra 0-4, S0=fermo) <<<\r\n");
      }
      /* Nota: qui NON si rimanda a STOP come nella versione a rotazione
       * continua: un servo posizionale deve continuare a tenere la sua
       * posizione corrente, altrimenti perde l'aggancio meccanico. */
    }

    if (modalita_manuale_attiva) {

      bool curr_left  = debounce_leggi(&db_left,  palReadPad(BTN_LEFT_PORT,  BTN_LEFT_PAD)  == PAL_LOW);
      bool curr_right = debounce_leggi(&db_right, palReadPad(BTN_RIGHT_PORT, BTN_RIGHT_PAD) == PAL_LOW);
      bool curr_up    = debounce_leggi(&db_up,    palReadPad(BTN_UP_PORT,    BTN_UP_PAD)    == PAL_LOW);
      bool curr_down  = debounce_leggi(&db_down,  palReadPad(BTN_DOWN_PORT,  BTN_DOWN_PAD)  == PAL_LOW);

      /* PC7: Servo X +step (subito al tocco, poi ogni
       * MANUAL_REPEAT_PERIOD_MS finche' resta premuto) */
      if (step_dovuto(&db_left, curr_left, prev_left)) {
        muovi_servo_a_step(0, &angolo_servo_x, true, "PULSANTE PC7");
      }

      /* PA9: Servo X -step */
      if (step_dovuto(&db_right, curr_right, prev_right)) {
        muovi_servo_a_step(0, &angolo_servo_x, false, "PULSANTE PA9");
      }

      /* PB10 (D6): Servo Y +step */
      if (step_dovuto(&db_up, curr_up, prev_up)) {
        muovi_servo_a_step(1, &angolo_servo_y, true, "PULSANTE PB10");
      }

      /* PA8 (D7): Servo Y -step */
      if (step_dovuto(&db_down, curr_down, prev_down)) {
        muovi_servo_a_step(1, &angolo_servo_y, false, "PULSANTE PA8");
      }

      prev_left  = curr_left;
      prev_right = curr_right;
      prev_up    = curr_up;
      prev_down  = curr_down;

    } else {

      /* Bluetooth (master vero) e terminale PC (per testare a mano
       * mandando "X1", "Y3", "S0", ...) condividono lo stesso parser
       * a 2 stati, ma con stato indipendente per non mescolarsi. */

      msg_t char_bt = sdGetTimeout(&SD1, TIME_IMMEDIATE);

      if (char_bt != MSG_TIMEOUT) {
        palTogglePad(GPIOA, 5);
        elabora_char_master(&parser_bt, (char)char_bt, "BT");
      }

      msg_t char_pc = sdGetTimeout(&SD2, TIME_IMMEDIATE);

      if (char_pc != MSG_TIMEOUT) {
        palTogglePad(GPIOA, 5);
        elabora_char_master(&parser_pc, (char)char_pc, "PC");
      }
    }

    chThdSleepMilliseconds(LOOP_PERIOD_MS);
  }
}
