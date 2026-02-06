#include <SPI.h>
#include <LoRa.h>
#include <lmic.h>
#include <hal/hal.h>
#include <Preferences.h>

// ---------------- LoRa Pins ----------------
#define LORA_SS   10
#define LORA_RST  14
#define LORA_DIO0 37
#define LORA_DIO1 36
#define LORA_DIO2 35
#define LORA_SCK  12
#define LORA_MISO 13
#define LORA_MOSI 11


#define LORA_SYNC_WORD 0x34

// ---------------- State Machine ----------------
enum State {
  STATE_P2P_RX,
  STATE_SWITCH_TO_LMIC,
  STATE_LMIC_RUN,
  STATE_SWITCH_TO_P2P
};

State state = STATE_P2P_RX;

// ---------------- Buffers ----------------
uint8_t rxBuf[64];
uint8_t rxLen = 0;

// ---------------- Preferences ----------------
Preferences prefs;
#define PREFS_NAMESPACE "lmic"
#define MAGIC_KEY "magic"
#define MAGIC_VAL 0xA5A5

// ---------------- OTAA Profiles ----------------
struct OtaaProfile {
  uint8_t appEUI[8];
  uint8_t devEUI[8];
  uint8_t appKey[16];
  const char* name;
};

OtaaProfile profiles[] = {
  {
    {0x35,0x12,0x06,0xD0,0x7E,0xD5,0xB3,0x70},
    {0x14,0x75,0x24,0x30,0x31,0x0A,0x61,0xA8},
    {0x10,0x2B,0x3C,0x4D,0x5E,0x6F,0x78,0x90,0x12,0x34,0x56,0x78,0x9A,0xBC,0xDE,0xF0},
    "Profile0"
  },
  {
    {0x60,0xB3,0xD5,0x7E,0xD0,0x06,0x12,0x35},
    {0xA7,0x61,0x0A,0x31,0x30,0x24,0x75,0x16},
    {0xA1,0x2B,0x3C,0x4D,0x5E,0x6F,0x78,0x90,0x12,0x34,0x56,0x78,0x9A,0xBC,0xDE,0xF0},
    "Profile1"
  }
};

#define PROFILE_COUNT (sizeof(profiles) / sizeof(OtaaProfile))
int activeProfile = -1;

// ---------------- LMIC Callbacks ----------------
void os_getArtEui(u1_t* buf) {
  memcpy(buf, profiles[activeProfile].appEUI, 8);
}

void os_getDevEui(u1_t* buf) {
  memcpy(buf, profiles[activeProfile].devEUI, 8);
}

void os_getDevKey(u1_t* buf) {
  memcpy(buf, profiles[activeProfile].appKey, 16);
}

// ---------------- LMIC Pins ----------------
const lmic_pinmap lmic_pins = {
  .nss = LORA_SS,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LORA_RST,
  .dio = { LORA_DIO0, LORA_DIO1, LORA_DIO2 }
};

static osjob_t sendjob;

// ---------------- Session Helpers ----------------
String profileKey(const char* key) {
  return String(key) + "_" + String(activeProfile);
}

bool hasSavedSession() {
  prefs.begin(PREFS_NAMESPACE, true);
  uint16_t magic = prefs.getUInt(profileKey(MAGIC_KEY).c_str(), 0);
  prefs.end();
  return magic == MAGIC_VAL;
}

void saveSession() {
  prefs.begin(PREFS_NAMESPACE, false);
  prefs.putUInt(profileKey(MAGIC_KEY).c_str(), MAGIC_VAL);
  prefs.putUInt(profileKey("netid").c_str(), LMIC.netid);
  prefs.putUInt(profileKey("devaddr").c_str(), LMIC.devaddr);
  prefs.putBytes(profileKey("nwkKey").c_str(), LMIC.nwkKey, 16);
  prefs.putBytes(profileKey("artKey").c_str(), LMIC.artKey, 16);
  prefs.putUInt(profileKey("seqUp").c_str(), LMIC.seqnoUp);
  prefs.putUInt(profileKey("seqDn").c_str(), LMIC.seqnoDn);
  prefs.end();
}

bool restoreSession() {
  if (!hasSavedSession()) return false;

  prefs.begin(PREFS_NAMESPACE, true);
  uint32_t netid = prefs.getUInt(profileKey("netid").c_str(), 0);
  uint32_t devaddr = prefs.getUInt(profileKey("devaddr").c_str(), 0);
  uint8_t nwkKey[16], artKey[16];
  prefs.getBytes(profileKey("nwkKey").c_str(), nwkKey, 16);
  prefs.getBytes(profileKey("artKey").c_str(), artKey, 16);
  uint32_t seqUp = prefs.getUInt(profileKey("seqUp").c_str(), 0);
  uint32_t seqDn = prefs.getUInt(profileKey("seqDn").c_str(), 0);
  prefs.end();

  if (devaddr == 0) return false;

  os_init();
  LMIC_reset();
  LMIC_setSession(netid, devaddr, nwkKey, artKey);
  LMIC.seqnoUp = seqUp;
  LMIC.seqnoDn = seqDn;
  LMIC_setAdrMode(0);
  LMIC_setLinkCheckMode(0);

  Serial.println("ABP session restored");
  return true;
}

// ---------------- LMIC Send ----------------
void do_send(osjob_t*) {
  LMIC_setTxData2(1, rxBuf, rxLen, 0);
  Serial.println("LMIC uplink queued");
}

// ---------------- LMIC Events ----------------
void onEvent(ev_t ev) {
  Serial.print("LMIC Event: ");
  Serial.println(ev);

  if (ev == EV_JOINED) {
    Serial.println("LMIC joined");
    LMIC_setLinkCheckMode(0);
    saveSession();
    do_send(nullptr);
    LMIC.rxDelay = savedLMICState.rxDelay; // necessary to receive downlink

    // Restore current radio state
    //LMIC.channelMap = savedLMICState.channelMap;
    //LMIC.datarate = savedLMICState.datarate;
    //LMIC.txpow = savedLMICState.txpow;
    //LMIC.initBandplanAfterReset = savedLMICState.initBandplanAfterReset;

    // Restore downlink confirmation state (critical for ACK mechanism)
    LMIC.dnConf = savedLMICState.dnConf; // necessary 
    LMIC.lastDnConf = savedLMICState.lastDnConf;
  }

  if (ev == EV_TXCOMPLETE) {
    Serial.println("LMIC TX complete");
    saveSession();
    state = STATE_SWITCH_TO_P2P;
  }
}

// ---------------- P2P Setup ----------------
void setupP2P() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  LoRa.begin(433E6);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  Serial.println("P2P RX ready");
}

// ---------------- Start LMIC ----------------
void startLMIC() {
  Serial.print("Switching to LMIC using ");
  Serial.println(profiles[activeProfile].name);

  LoRa.end();
  SPI.end();
  delay(10);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);

  if (restoreSession()) {
    do_send(nullptr);
    return;
  }

  os_init();
  LMIC_reset();
  LMIC_setAdrMode(0);
  LMIC_startJoining();
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, HIGH);

  setupP2P();
}

// ---------------- Main Loop ----------------
void loop() {

  if (state == STATE_P2P_RX) {
    int packetSize = LoRa.parsePacket();
    if (packetSize > 0) {

      rxLen = 0;
      while (LoRa.available() && rxLen < sizeof(rxBuf)) {
        rxBuf[rxLen++] = LoRa.read();
      }

      uint8_t profileId = rxBuf[0];
      if (profileId >= PROFILE_COUNT) {
        Serial.println("Invalid profile ID");
        return;
      }

      activeProfile = profileId;

      uint8_t msgLen = rxLen - 1;
      memmove(rxBuf, rxBuf + 1, msgLen);
      rxLen = msgLen;

      Serial.print("Profile selected: ");
      Serial.println(profiles[activeProfile].name);

      state = STATE_SWITCH_TO_LMIC;
    }
  }

  if (state == STATE_SWITCH_TO_LMIC) {
    startLMIC();
    state = STATE_LMIC_RUN;
  }

  if (state == STATE_LMIC_RUN) {
    os_runloop_once();
  }

  if (state == STATE_SWITCH_TO_P2P) {
    os_init();
    LMIC_reset();
    SPI.end();
    delay(10);
    setupP2P();
    state = STATE_P2P_RX;
  }

  delay(1);
}




