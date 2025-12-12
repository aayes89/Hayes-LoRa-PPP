/* Hayes-LoRa-PPP Modem
   Firmware completo (Arduino) para RPi Pico + SX1278
   - Emula modem Hayes (AT)
   - Discovery (BEACON), handshake (SYN/SYNACK), modo TRANSPARENT PPP, Modo Dummy
   - Logs en Serial
   Ajusta pines LoRa y nodeMac según tu hardware.
*/
#include <SPI.h>
#include <LoRa.h>
// ------------------ Ajustes hardware ------------------
#define LORA_SS 5    // CS
#define LORA_RST 6   // RST
#define LORA_DIO0 7  // DIO0
#define LORA_SCK 2
#define LORA_MISO 4
#define LORA_MOSI 3
#define LED_PIN 25
// Frecuencia LoRa (ajusta a tu país)
#define LORA_FREQ 433E6
// Tipos de frame
#define T_BEACON 0x01
#define T_SYN 0x02
#define T_SYNACK 0x03
#define T_ACK 0x04
#define T_DATA 0x05
#define T_FIN 0x06
// Parámetros operativos
#define BEACON_INTERVAL_MS 2000
#define HANDSHAKE_RETRIES 6
#define HANDSHAKE_TIMEOUT_MS 2000
#define NEIGHBOR_TIMEOUT_MS (BEACON_INTERVAL_MS * 6)  // > beacon window
#define TX_POWER_DBM 17
#define SF 9
#define BANDWIDTH 125E3
#define CODERATE 5
#define LORA_HAVE_DUMP_REGS 1
// PPP constantes
#define PPP_FLAG 0x7E
#define PPP_ESC 0x7D
#define PPP_ADDR 0xFF
#define PPP_CTRL 0x03
#define PPP_LCP 0xC021
#define PPP_IPCP 0x8021
#define PPP_IP 0x0021
#define PPP_PAP 0xC023
#define PPP_CHAP 0x8081

// PAP códigos
#define PAP_AUTHREQ 1
#define PAP_AUTHACK 2
#define PAP_AUTHNAK 3
// CHAP códigos
#define CHAP_CHALLENGE 1
#define CHAP_RESPONSE 2
#define CHAP_SUCCESS 3
#define CHAP_FAILURE 4

// PPP códigos
#define CONF_REQ 1
#define CONF_ACK 2
#define CONF_NAK 3
#define CONF_REJ 4
#define TERM_REQ 5
#define TERM_ACK 6
#define CODE_REJ 7
#define PROTO_REJ 8
#define ECHO_REQ 9
#define ECHO_REP 10
#define DISCARD_REQ 11
// Opciones LCP conocidas
#define LCP_OPTION_MRU 1
#define LCP_OPTION_ACCM 2
#define LCP_OPTION_AUTH 3
#define LCP_OPTION_MAGIC 5
#define LCP_OPTION_PFC 7
#define LCP_OPTION_ACFC 8

// Estados LED MODEM
#define DCD_PIN 25
#define DSR_PIN 26
#define CTS_PIN 27

// Opciones
#define IPCP_OPTION_IPADDRESS 3
enum LcpCode {
  LCP_CONFREQ = 1,
  LCP_CONFACK = 2,
  LCP_CONFNAK = 3,
  LCP_CONFREJ = 4,
  LCP_TERMREQ = 5,
  LCP_TERMACK = 6,
  LCP_ECHOREQ = 9,
  LCP_ECHOREP = 10
};
enum LcpState {
  PPP_LCP_DOWN,
  PPP_LCP_REQ,
  PPP_LCP_ACK
};
enum IPCPState {
  PPP_IPCP_DOWN,
  PPP_IPCP_REQ,
  PPP_IPCP_ACK
};

LcpState lcpState = PPP_LCP_DOWN;
IPCPState ipcpState = PPP_IPCP_DOWN;
int lcpRetries = 0;
int ipcpRetries = 0;
uint8_t lcpID = 1;
uint16_t peerMRU = 1500;
uint32_t peerMagic = 0;
bool pppLcpUp = false;
uint8_t papMyId = 0;
uint8_t chapMyId = 0;
bool papAuthenticated = false;
bool chapAuthenticated = false;
// Ayuda IPCP
bool ipcpRequested = false;   // hemos enviado ConfReq
bool ipcpAckedLocal = false;  // peer acked nuestro ConfReq
bool ipcpAckedPeer = false;   // nosotros ackamos/aceptamos su ConfReq
// Buffers
uint8_t rxBuffer[256];  // No usado ahora, pero mantenido
int rxIndex = 0;        // No usado
// --- Emulación de línea / modo servidor PPP ---
bool virtualCarrier = false;   // true = DCD on (emulado)
bool pppServerActive = false;  // si true manejamos PPP como servidor activo
bool permissiveAT = true;      // si true: responder OK a AT no reconocidos (mejora compatibilidad)
unsigned long carrierTimestamp = 0;

bool verboseMode = true;                        // ATV1 = respuestas verbales, ATV0 = numéricas
uint8_t SREG[100];                              // registros S (S0, S7, S10, etc.)
uint8_t resultCodeLevel = 4;                    // ATX nivel 0–4 (solo efecto decorativo)
String storedProfile = "DEFAULT 115200,N,8,1";  // para AT&V
static String plusCount = "";
bool echoOn = false;
bool dummyMode = true;  // PPP server con loopback
// IDs de control IPCP
uint8_t ipcpMyId = 0;
uint8_t myIp[4] = { 10, 0, 0, 1 };
uint8_t peerIp[4] = { 10, 0, 0, 2 };
uint32_t localIP = 0x0A000001;  // 10.0.0.1
uint32_t peerIP = 0x0A000002;   // 10.0.0.2
uint32_t dns1IP = 0x08080808;   // 8.8.8.8
uint32_t dns2IP = 0x08080404;   // 8.8.4.4

// ------------------ Identidad / MAC -> Dial number ------------------
uint8_t nodeMac[6] = { 0x0A, 0x00, 0x27, 0x00, 0x00, 0x02 };  // CAMBIA por tu valor real
String myDialID;
String macToDialNumber(uint8_t *mac, int len = 6, int digits = 11) {
  String result = "";
  for (int i = 0; i < len; ++i) {
    int val = mac[i];
    result += String(val);
  }
  if (result.length() > digits) {
    result = result.substring(result.length() - digits);
  }
  while (result.length() < digits) result = "0" + result;
  return result;
}
// ------------------ Estructura Neighbor ------------------
struct Neighbor {
  String id;
  String macStr;
  long lastSeen;
  int rssi;
};
#define MAX_NEIGHBORS 16
Neighbor neighbors[MAX_NEIGHBORS];
void updateNeighbor(String id, String macStr, int rssi) {
  long now = millis();
  for (int i = 0; i < MAX_NEIGHBORS; i++) {
    if (neighbors[i].id == id) {
      neighbors[i].lastSeen = now;
      neighbors[i].rssi = rssi;
      return;
    }
  }
  int oldest = 0;
  long oldestTime = LONG_MAX;
  for (int i = 0; i < MAX_NEIGHBORS; i++) {
    if (neighbors[i].id.length() == 0) {
      neighbors[i].id = id;
      neighbors[i].macStr = macStr;
      neighbors[i].lastSeen = now;
      neighbors[i].rssi = rssi;
      return;
    }
    if (neighbors[i].lastSeen < oldestTime) {
      oldestTime = neighbors[i].lastSeen;
      oldest = i;
    }
  }
  neighbors[oldest].id = id;
  neighbors[oldest].macStr = macStr;
  neighbors[oldest].lastSeen = now;
  neighbors[oldest].rssi = rssi;
}
void pruneNeighbors() {
  long now = millis();
  for (int i = 0; i < MAX_NEIGHBORS; i++) {
    if (neighbors[i].id.length() && (now - neighbors[i].lastSeen > NEIGHBOR_TIMEOUT_MS)) {
      neighbors[i].id = "";
      neighbors[i].macStr = "";
      neighbors[i].lastSeen = 0;
      neighbors[i].rssi = 0;
    }
  }
}
int findNeighborByID(String id) {
  for (int i = 0; i < MAX_NEIGHBORS; i++)
    if (neighbors[i].id == id) return i;
  return -1;
  //return 1; // siempre hay alguien aunque no exista
}
// ------------------ Estado del módem ------------------
enum ModemState { S_IDLE,
                  S_DIALING,
                  S_RINGING,
                  S_CONNECTED };

ModemState modemState = S_IDLE;
String pendingDialID = "";
int handshakeTries = 0;
unsigned long handshakeTimestamp = 0;
int connectedNeighborIdx = -1;
// ------------------ PPP Estados ------------------
// PPP FSM básico
enum PppPhase { PPP_DEAD,
                PPP_ESTABLISH,
                PPP_AUTHENTICATE,
                PPP_AUTH,
                PPP_NETWORK,
                PPP_OPEN,
                PPP_TERMINATE };
PppPhase pppPhase = PPP_DEAD;
uint8_t pppRxBuffer[512];
int pppRxLen = 0;
bool pppInEsc = false;
unsigned long pppTimer = 0;
int pppRetries = 0;
bool lcpOpen = false;
bool ipcpOpen = false;
uint8_t lcpMyId = 0;
uint32_t magicNumber;
uint16_t mru = 1500;
// CRC Table
uint16_t crcTable[256] = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
  0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
  0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
  0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
  0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
  0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
  0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
  0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
  0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

// ------------------ Timers ------------------
unsigned long lastBeacon = 0;
unsigned long lastSerialLog = 0;
unsigned long ipcpTimer = 0;
// ------------------ Funciones PPP ------------------
uint16_t crc16(const uint8_t *data, int len) {
  uint16_t fcs = 0xFFFF;
  for (int i = 0; i < len; i++) {
    fcs = (fcs >> 8) ^ crcTable[(fcs ^ data[i]) & 0xFF];
  }
  return ~fcs;
}

String atBuffer = "";
// --- Helpers ----
void sendOK() {
  Serial.println("\r\nOK");
}
void sendERROR() {
  Serial.println("\r\nERROR");
}
void sendCONNECT(int baud = 115200) {
  Serial.print("\r\nCONNECT ");
  Serial.println(baud);
}

void startVirtualCarrier() {
  virtualCarrier = true;
  digitalWrite(LED_PIN, HIGH);  // Emula DCD ON (conecta a pin DCD)
  digitalWrite(DSR_PIN, HIGH);  // Refuerza DSR
  carrierTimestamp = millis();
  Serial.print("\r\nCONNECT 115200\r\n");  // Limpio, con \r\n final
  Serial.flush();
  delay(100);  // Tiempo para Windows poll DCD
  pppServerActive = true;
  pppPhase = PPP_ESTABLISH;
  lcpOpen = false;
  ipcpOpen = false;
  pppRetries = 0;  // Usa lcpRetries/ipcpRetries si agregaste
  lcpRetries = 1; ipcpRetries = 1;
  pppTimer = millis();

  // Dummy: Fuerza open inmediato (evita auth/LCP delays)
  if (dummyMode && connectedNeighborIdx == -1) {
    pppPhase = PPP_OPEN;
    lcpOpen = true;
    ipcpOpen = true;
    ipcpState = PPP_IPCP_ACK;
    lcpState = PPP_LCP_ACK;
    papAuthenticated = true;
    chapAuthenticated = true;
    sendLcpConfReq();  // Trigger client response
    Serial.println("[DUMMY] PPP OPEN + ConfReq enviado");
  } else {
    sendLcpConfReq();  // Real: Inicia LCP
  }
  Serial.flush();
}

void stopVirtualCarrier() {
  virtualCarrier = false;
  digitalWrite(LED_PIN, LOW);  // DCD OFF
  pppServerActive = false;
  pppPhase = PPP_DEAD;
  lcpOpen = false;
  ipcpOpen = false;
  Serial.print("\r\nNO CARRIER\r\nOK\r\n");
  Serial.flush();
}
// ------------- PAP -----------------
void sendPapAck(uint8_t id, const char *msg) {
  // PAP Auth-Ack: code 2
  uint8_t pkt[256];
  int lon = 0;
  pkt[lon++] = PAP_AUTHACK;
  pkt[lon++] = id;
  // longitud
  pkt[lon++] = 0;
  pkt[lon++] = 0;
  // mensaje
  int mlon = strlen(msg);
  pkt[lon++] = (mlon >> 8) & 0xFF;  // PAP usa longitud diferente hoy en día
  // Mensajes en crudo en mayoría de implementaciones
  // Añaden los bytes al mensaje y establece longitud
  memcpy(pkt + lon, msg, mlon);
  lon += mlon;
  uint16_t totalLon = lon;
  pkt[2] = (totalLon >> 8) & 0xFF;
  pkt[3] = totalLon & 0xFF;
  sendPPPToSerial(PPP_PAP, pkt, totalLon);
}

void sendPapNak(uint8_t id, const char *msg) {
  uint8_t pkt[256];
  int len = 0;
  pkt[len++] = PAP_AUTHNAK;
  pkt[len++] = id;
  pkt[len++] = 0;
  pkt[len++] = 0;
  int mlen = strlen(msg);
  memcpy(pkt + len, msg, mlen);
  len += mlen;
  uint16_t totalLen = len;
  pkt[2] = (totalLen >> 8) & 0xFF;
  pkt[3] = totalLen & 0xFF;
  sendPPPToSerial(PPP_PAP, pkt, totalLen);
}

// --- CHAP Handler para Dummy Auth ---
void chapInput(uint8_t *pkt, int len) {
  if (len < 4) return;
  uint8_t code = pkt[0];
  uint8_t id = pkt[1];
  uint16_t plen = ((uint16_t)pkt[2] << 8) | pkt[3];
  if (plen > len) return;
  uint8_t *info = pkt + 4;
  int infoLen = plen - 4;

  switch (code) {
    case CHAP_CHALLENGE:
      {
        // Windows (cliente) envía Challenge: id, len, value (16B nonce + name)
        chapMyId = id;
        // Dummy: No chequeamos nonce/name; preparamos response fake (en real, computa MD4/RC4)
        // Response: code=2, id, len=50 (estándar MSv2: LM/NT hashes fake)
        uint8_t respPkt[64];
        int rlen = 0;
        respPkt[rlen++] = CHAP_RESPONSE;
        respPkt[rlen++] = id;
        respPkt[rlen++] = 0;
        respPkt[rlen++] = 0;  // Len placeholder (50 total)
        // Info: Value len=49, Value (48B: LM=24, NT=24, flags=1), Name len=1, Name="DummyUser"
        respPkt[rlen++] = 49;  // Value len
        // Fake hashes (zeros + random; en real: DES(MD4(pass)) + SHA)
        memset(respPkt + rlen, 0x00, 24);
        rlen += 24;  // LM hash fake
        memset(respPkt + rlen, 0x11, 24);
        rlen += 24;              // NT hash fake
        respPkt[rlen++] = 0x01;  // Flags (use NT)
        uint8_t nameLen = 9;     // "DummyUser"
        respPkt[rlen++] = nameLen;
        const char *name = "DummyUser";
        memcpy(respPkt + rlen, name, nameLen);
        rlen += nameLen;
        uint16_t totalLen = rlen;
        respPkt[2] = (totalLen >> 8) & 0xFF;
        respPkt[3] = totalLen & 0xFF;
        sendPPPToSerial(PPP_CHAP, respPkt, totalLen);
        Serial.println("[CHAPv2] Dummy Response enviado.");
        chapAuthenticated = true;
        pppPhase = PPP_NETWORK;
        break;
      }

    case CHAP_SUCCESS:
      {
        chapAuthenticated = true;
        Serial.println("[CHAPv2] Success recibido - Auth OK");
        pppPhase = PPP_NETWORK;  // Avanza a IPCP
        break;
      }

    case CHAP_FAILURE:
      {
        chapAuthenticated = false;
        Serial.println("[CHAPv2] Failure - Terminate");
        pppPhase = PPP_TERMINATE;
        break;
      }
  }
}
// --- PAP Handler Corregido para Dummy Auth ---
void papInput(uint8_t *pkt, int len) {
  if (len < 4) return;
  uint8_t code = pkt[0];
  uint8_t id = pkt[1];
  uint16_t plen = ((uint16_t)pkt[2] << 8) | pkt[3];
  if (plen > len || plen < 5) return;  // Mínimo: code,id,len,peerLen=1,peerId=0

  uint8_t *info = pkt + 4;
  int infoLen = plen - 4;

  switch (code) {
    case PAP_AUTHREQ:
      {
        // Parseo correcto PAP: peer-id-len (1B), peer-id (N), passwd-len (1B), passwd (M)
        if (infoLen < 2) return;  // Sin peerLen
        uint8_t peerLen = info[0];
        if (peerLen > infoLen - 1) return;  // Len inválido
        uint8_t *peerId = info + 1;
        int peerIdLen = peerLen;

        int passwdOffset = 1 + peerLen;
        if (passwdOffset + 1 > infoLen) return;  // Sin passwdLen
        uint8_t passwdLen = info[passwdOffset];
        uint8_t *passwd = info + passwdOffset + 1;
        int passwdStrLen = passwdLen;

        // Dummy: Acepta CUALQUIER credencial (no chequea valores)
        // Log opcional para debug (comenta en prod para no contaminar serial)
        // Serial.print("[PAP] Dummy accept: User='");
        // Serial.write(peerId, peerIdLen);
        // Serial.print("' (len=");
        // Serial.print(peerIdLen);
        // Serial.println(")");

        papAuthenticated = true;
        papMyId = id;

        // ACK con mensaje estándar "Login OK" (longitud correcta)
        const char *msg = "Login OK";
        int mlen = strlen(msg);
        uint8_t ackPkt[256];
        int alen = 0;
        ackPkt[alen++] = PAP_AUTHACK;
        ackPkt[alen++] = id;
        ackPkt[alen++] = 0;  // Len high
        ackPkt[alen++] = 0;  // Len low
        // Payload: msgLen (1B, pero PAP ACK usa raw msg sin len prefix; total len = 4 + mlen)
        memcpy(ackPkt + alen, msg, mlen);
        alen += mlen;
        uint16_t totalLen = alen;
        ackPkt[2] = (totalLen >> 8) & 0xFF;
        ackPkt[3] = totalLen & 0xFF;

        sendPPPToSerial(PPP_PAP, ackPkt, totalLen);
        Serial.println("[PAP] Dummy Auth-Ack enviado.");

        // Avanza fase: Si dummy, salta directo a NETWORK/IPCP
        if (dummyMode) {
          pppPhase = PPP_NETWORK;
          // Fuerza IPCP start si no iniciado
          if (!ipcpRequested) {
            sendIpcpConfReq();
            ipcpRequested = true;
            ipcpTimer = millis();
          }
        } else if (!ipcpRequested) {
          sendIpcpConfReq();
          ipcpRequested = true;
          pppTimer = millis();
        }
        break;
      }
    case PAP_AUTHACK:
      {
        // Cliente aceptó nuestras creds (si enviamos; dummy no envía)
        papAuthenticated = true;
        pppPhase = PPP_NETWORK;  // Avanza
        break;
      }
    case PAP_AUTHNAK:
      {
        papAuthenticated = false;
        Serial.println("[PAP] Auth rechazada por peer, terminate.");
        pppPhase = PPP_TERMINATE;
        break;
      }
  }
}

// ------------------ Envío IP sobre LoRa ------------------
void sendIPDataToPeer(const uint8_t *ipData, int len) {
  if (connectedNeighborIdx == -1 && dummyMode) {
    // Dummy loopback: Ecoa IP de vuelta como si viniera del peer
    sendPPPToSerial(PPP_IP, ipData, len);
    return;
  }
  if (len <= 0 || len > 256) return;
  uint8_t header[12];
  header[0] = T_DATA;
  for (int i = 0; i < 11; i++) header[1 + i] = (i < myDialID.length()) ? myDialID.charAt(i) : '0';
  LoRa.beginPacket();
  LoRa.write(header, 12);
  LoRa.write(ipData, len);
  LoRa.endPacket();
}

void sendPPPToSerial(uint16_t proto, const uint8_t *data, int dataLen) {
  uint8_t frame[512];
  int frameLen = 0;
  frame[frameLen++] = PPP_ADDR;
  frame[frameLen++] = PPP_CTRL;
  frame[frameLen++] = (proto >> 8) & 0xFF;
  frame[frameLen++] = proto & 0xFF;
  if (dataLen > 0 && dataLen < 500) {
    memcpy(frame + frameLen, data, dataLen);
    frameLen += dataLen;
  }
  uint16_t fcs = crc16(frame, frameLen);
  frame[frameLen++] = fcs & 0xFF;
  frame[frameLen++] = (fcs >> 8) & 0xFF;
  // Enviar escapado
  Serial.write(PPP_FLAG);
  for (int i = 0; i < frameLen; i++) {
    uint8_t b = frame[i];
    if (b == PPP_FLAG || b == PPP_ESC || b < 0x20) {
      Serial.write(PPP_ESC);
      Serial.write(b ^ 0x20);
    } else {
      Serial.write(b);
    }
  }
  Serial.write(PPP_FLAG);
}

void pppSerialInput() {
  // Handle ATH en PPP mode (escape simple)
  if (Serial.available() >= 4) {
    String peekStr = "";
    for (int i = 0; i < 4; i++) peekStr += (char)Serial.peek();
    if (peekStr == "ATH\r" || peekStr == "ATH\n") {
      Serial.readBytes((uint8_t *)peekStr.c_str(), 4);  // Consume
      stopVirtualCarrier();
      sendOK();
      modemState = S_IDLE;
      return;
    }
  }
  while (Serial.available()) {
    uint8_t raw = Serial.read();
    if (raw == '+') {
      plusCount += '+';
      if (plusCount.length() == 3) {
        delay(1000);  // Pausa guard (Hayes: 1s silencio después)
        if (Serial.available() && Serial.peek() == '+') {
          // Confirmado? No, más + → reset count
          Serial.read();  // Consume extra +
          plusCount = "";
        } else {
          // Escape confirmado: back to command
          pppPhase = PPP_TERMINATE;
          stopVirtualCarrier();
          sendOK();  // Command mode
          modemState = S_IDLE;
          plusCount = "";
          return;  // Sal de PPP input
        }
      }
      continue;
    } else {
      plusCount = "";  // Reset on non-+
    }
    // Flag -> fin de frame
    if (raw == PPP_FLAG) {
      if (pppRxLen >= 6) {
        // calc FCS sobre bytes recibidos (sin los 2 bytes FCS al final)
        uint16_t calcFcs = crc16(pppRxBuffer, pppRxLen - 2);
        // recvFcs: LSB en [len-2], MSB en [len-1]
        uint16_t recvFcs = (uint16_t)pppRxBuffer[pppRxLen - 2] | ((uint16_t)pppRxBuffer[pppRxLen - 1] << 8);
        if (calcFcs == recvFcs && pppRxBuffer[0] == PPP_ADDR && pppRxBuffer[1] == PPP_CTRL) {
          uint16_t proto = ((uint16_t)pppRxBuffer[2] << 8) | pppRxBuffer[3];
          uint8_t *info = pppRxBuffer + 4;
          int infoLen = pppRxLen - 6;
          if (proto == PPP_LCP) {
            lcpInput(info, infoLen);
          } else if (proto == PPP_IPCP) {
            ipcpInput(info, infoLen);
          } else if (proto == PPP_CHAP || proto == 0x80C2) {  // CHAP gen/MSv2
            chapInput(info, infoLen);
          } else if (proto == PPP_PAP) {
            papInput(info, infoLen);
          } else if (proto == PPP_IP && pppPhase == PPP_OPEN && connectedNeighborIdx >= 0) {
            sendIPDataToPeer(info, infoLen);
          } else {
            protoReject(proto);
          }
        } else {
          Serial.print("[PPP] FCS mismatch o header inválido (calc=");
          Serial.print(calcFcs, HEX);
          Serial.print(", recv=");
          Serial.print(recvFcs, HEX);
          Serial.println(")");
        }
      }
      pppRxLen = 0;
      pppInEsc = false;
      continue;
    }

    // Process escape logic BEFORE appending bytes to buffer
    uint8_t b;
    if (pppInEsc) {
      // previous byte was ESC -> unescape current
      b = raw ^ 0x20;
      pppInEsc = false;
    } else {
      if (raw == PPP_ESC) {
        // mark escape and continue (do NOT store escape byte)
        pppInEsc = true;
        continue;
      } else {
        b = raw;
      }
    }

    // append byte (siempre el byte/unescaped)
    if (pppRxLen < (int)sizeof(pppRxBuffer) - 2) {
      pppRxBuffer[pppRxLen++] = b;
    } else {
      pppRxLen = 0;
      Serial.println("[PPP] Buffer overflow, reset.");
    }
  }
}

void protoReject(uint16_t proto) {
  uint8_t pkt[6];
  pkt[0] = PROTO_REJ;
  pkt[1] = ++lcpMyId;
  pkt[2] = 0;
  pkt[3] = 6;
  pkt[4] = (proto >> 8) & 0xFF;
  pkt[5] = proto & 0xFF;
  sendPPPToSerial(PPP_LCP, pkt, 6);
}
// LCP
void lcpInput(uint8_t *data, uint16_t len) {
  if (len < 4) return;
  uint8_t code = data[0];
  uint8_t id = data[1];
  uint16_t lcpLen = ((uint16_t)data[2] << 8) | data[3];
  // Validación de longitud
  if (lcpLen > len) return;
  switch (code) {
    // 1. Configure-Request (del peer)
    case LCP_CONFREQ:
      {
        uint8_t response[32];  // Buffer fijo
        int ri = 0;
        response[ri++] = LCP_CONFACK;  // Asumimos ACK
        response[ri++] = id;
        response[ri++] = 0;
        response[ri++] = 0;
        // Revisar opciones (con manejo AUTH y overflow)
        int optPtr = 4;
        while (optPtr + 1 < lcpLen) {
          uint8_t optType = data[optPtr];
          uint8_t optLen = data[optPtr + 1];
          if (optLen < 2 || optPtr + optLen > lcpLen) break;
          if (ri + optLen > sizeof(response)) {
            //Serial.println("[LCP] Response buffer overflow, envía REJ.");
            uint8_t rej[4] = { LCP_CONFREJ, id, 0, 4 };
            sendPPPToSerial(PPP_LCP, rej, 4);
            return;
          }
          if (optType == LCP_OPTION_AUTH) {
            // Parsea subopt: data[optPtr+2..3] == proto (0x80=CHAP, 0x81=MSv2)
            if (optPtr + 3 < lcpLen) {
              uint16_t authProto = ((uint16_t)data[optPtr + 2] << 8) | data[optPtr + 3];
              if (authProto == 0xC223 || authProto == PPP_CHAP) {  // CHAP o MS-CHAPv2
                // Dummy: ACK directo (acepta MS-CHAPv2 sin NAK)
                memcpy(&response[ri], &data[optPtr], optLen);
                ri += optLen;
                Serial.println("[LCP] MS-CHAPv2 ACKed (dummy).");
              } else {
                // Otros (PAP): ACK
                memcpy(&response[ri], &data[optPtr], optLen);
                ri += optLen;
              }
            } else {
              break;  // Opción inválida
            }
          }
        }
        // Escribir longitud
        uint16_t respLen = ri;
        response[2] = (respLen >> 8) & 0xFF;
        response[3] = respLen & 0xFF;
        sendPPPToSerial(PPP_LCP, response, respLen);
        lcpState = PPP_LCP_ACK;
        lcpOpen = true;  // Set flag para FSM
        break;
      }
    // 2. Configure-Ack (a nuestro ConfReq)
    case LCP_CONFACK:
      {
        if (id == lcpMyId && lcpState == PPP_LCP_REQ) {
          lcpState = PPP_LCP_ACK;
          lcpOpen = true;  // Fix: Set flag
          pppTimer = millis();
          Serial.println("[LCP] Configure-Ack recibido. LCP completado.");
          // Avanzar a siguiente fase (IPCP o autenticación)
          if (papAuthenticated || pppPhase == PPP_ESTABLISH) {
            pppPhase = PPP_NETWORK;
          } else {
            pppPhase = PPP_AUTHENTICATE;
          }
        }
        break;
      }
    // 3. Configure-Nak/Rej
    case LCP_CONFNAK:
    case LCP_CONFREJ:
      {
        Serial.println("[LCP] Peer rechazó configuración, reenviando ajuste.");
        lcpState = PPP_LCP_DOWN;  // Reset state
        delay(100);
        sendLcpConfReq();
        break;
      }
    // 4. Echo-Request
    case LCP_ECHOREQ:
      {
        // Fix: Extrae y chequea magic para loop detect
        uint32_t reqMagic = 0;
        if (lcpLen > 8) {  // Magic opt len=6, data=4
          reqMagic = ((uint32_t)data[8] << 24) | ((uint32_t)data[9] << 16) | ((uint32_t)data[10] << 8) | data[11];
          if (reqMagic == magicNumber) {
            Serial.println("[LCP] Loop detectado en ECHO, discard.");
            return;
          }
          peerMagic = reqMagic;  // Actualiza peer magic
        }
        uint8_t resp[16];
        int ri = 0;
        resp[ri++] = LCP_ECHOREP;
        resp[ri++] = id;
        resp[ri++] = 0;
        resp[ri++] = 0;
        if (lcpLen > 4) {
          memcpy(&resp[ri], &data[4], lcpLen - 4);
          ri += lcpLen - 4;
        }
        uint16_t respLen = ri;
        resp[2] = (respLen >> 8) & 0xFF;
        resp[3] = respLen & 0xFF;
        sendPPPToSerial(PPP_LCP, resp, respLen);
        break;
      }
    // 5. Terminate-Request
    case LCP_TERMREQ:
      {
        uint8_t resp[4] = { LCP_TERMACK, id, 0, 4 };
        sendPPPToSerial(PPP_LCP, resp, 4);
        Serial.println("[LCP] Terminate-Request recibido, cerrando.");
        lcpState = PPP_LCP_DOWN;
        lcpOpen = false;
        pppPhase = PPP_TERMINATE;  // Trigger FSM
        break;
      }
    case LCP_TERMACK:
      {
        lcpOpen = false;
        pppPhase = PPP_DEAD;
        //Serial.println("[LCP] Terminate-Ack recibido, desconectado.");
        break;
      }
    default:
      {
        Serial.print("[LCP] Código no soportado: ");
        Serial.println(code, HEX);
        break;
      }
  }
}

void sendLcpConfReq() {
  uint8_t pkt[32];
  int i = 0;

  // Código PPP: LCP Configure-Request
  pkt[i++] = LCP_CONFREQ;

  // ID del paquete (se incrementa por cada ConfReq)
  pkt[i++] = ++lcpMyId;

  // Placeholder para longitud
  pkt[i++] = 0;
  pkt[i++] = 0;

  // ---- Opción MRU ----
  pkt[i++] = LCP_OPTION_MRU;  // Tipo = 1
  pkt[i++] = 4;               // Longitud = 4 bytes (tipo + len + 2 datos)
  pkt[i++] = (mru >> 8) & 0xFF;
  pkt[i++] = mru & 0xFF;

  // ---- Opción Magic Number ----
  pkt[i++] = LCP_OPTION_MAGIC;  // Tipo = 5
  pkt[i++] = 6;                 // Longitud = 6 bytes (tipo + len + 4 datos)
  //magicNumber = (uint32_t)millis() ^ 0xA5A5A5A5; // inicializado en setup()
  pkt[i++] = (magicNumber >> 24) & 0xFF;
  pkt[i++] = (magicNumber >> 16) & 0xFF;
  pkt[i++] = (magicNumber >> 8) & 0xFF;
  pkt[i++] = magicNumber & 0xFF;

  // ---- (Opcional) Opción PFC (Protocol Field Compression) ----
  pkt[i++] = LCP_OPTION_PFC;  // Tipo = 7
  pkt[i++] = 2;               // Longitud = 2 bytes (sin datos)

  // ---- (Opcional) Opción ACFC (Address/Control Field Compression) ----
  pkt[i++] = LCP_OPTION_ACFC;  // Tipo = 8
  pkt[i++] = 2;                // Longitud = 2 bytes (sin datos)

  // Establecer longitud total
  uint16_t len = i;
  pkt[2] = (len >> 8) & 0xFF;
  pkt[3] = len & 0xFF;

  // Enviar frame completo (proto = LCP)
  sendPPPToSerial(PPP_LCP, pkt, len);

  // Registrar tiempo de envío (para timeout/retransmisión)
  pppTimer = millis();
  Serial.println("[LCP] Configure-Request enviado.");
}

void sendLcpConfAck(uint8_t id, uint8_t *options, int optLen) {
  uint8_t pkt[16];
  pkt[0] = CONF_ACK;
  pkt[1] = id;
  pkt[2] = 0;
  pkt[3] = 4 + optLen;
  memcpy(pkt + 4, options, optLen);
  sendPPPToSerial(PPP_LCP, pkt, 4 + optLen);
}
void sendLcpTermAck(uint8_t id) {
  uint8_t pkt[4] = { TERM_ACK, id, 0, 4 };
  sendPPPToSerial(PPP_LCP, pkt, 4);
}
void sendLcpEchoRep(uint8_t id, uint8_t *data, int dataLen) {
  if (dataLen > 256) dataLen = 256;
  int pktLen = 4 + dataLen;
  //uint8_t *pkt = (uint8_t *)malloc(pktLen);
  uint8_t pkt[260];
  //if (!pkt) return;
  pkt[0] = ECHO_REP;
  pkt[1] = id;
  pkt[2] = (pktLen >> 8) & 0xFF;
  pkt[3] = pktLen & 0xFF;
  if (dataLen > 0) memcpy(pkt + 4, data, dataLen);
  sendPPPToSerial(PPP_LCP, pkt, pktLen);
  //free(pkt);
}
// IPCP
void ipcpInput(uint8_t *pkt, int len) {
  if (len < 4) return;
  uint8_t code = pkt[0];
  uint8_t id = pkt[1];
  uint16_t pktLen = ((uint16_t)pkt[2] << 8) | pkt[3];
  if (pktLen > len) return;
  uint8_t *opts = pkt + 4;
  int optsLen = pktLen - 4;

  switch (code) {
    case CONF_REQ:
      {
        // parse options and accept IP-Address if present
        bool haveIp = false;
        uint8_t ackBuf[64] = { 0 };
        int ackLen = 0;
        for (int i = 0; i < optsLen;) {
          uint8_t t = opts[i];
          if (i + 1 >= optsLen) break;
          uint8_t l = opts[i + 1];
          if (l < 2 || i + l > optsLen) break;
          if (t == IPCP_OPTION_IPADDRESS && l == 6) {
            // peer suggests an IP for itself -> we accept and set peerIp
            peerIp[0] = opts[i + 2];
            peerIp[1] = opts[i + 3];
            peerIp[2] = opts[i + 4];
            peerIp[3] = opts[i + 5];
            haveIp = true;
            // copy option to ackBuf (we ack it)
            memcpy(ackBuf + ackLen, opts + i, 6);
            ackLen += 6;
          } else {
            // unknown option: we reject it (ConfNak could be used to propose alternative, but we ACK known ones)
            // For now, we will ignore unknowns and not fail the negotiation.
          }
          i += l;
        }
        // send Conf-Ack with options we accepted
        sendIpcpConfAck(id, ackBuf, ackLen);
        ipcpAckedPeer = true;
        // if we haven't sent our ConfReq yet, send it now
        if (!ipcpRequested) { sendIpcpConfReq(); }
        break;
      }
    case CONF_ACK:
      {
        // peer acked our ConfReq (they accepted our myIp)
        ipcpAckedLocal = true;
        if (id == ipcpMyId && ipcpState == PPP_IPCP_REQ) {
          ipcpState = PPP_IPCP_ACK;
          Serial.println("[IPCP] Configure-Ack recibido. IPCP completado.");
          pppPhase = PPP_OPEN;
        }
        break;
      }
    case CONF_NAK:
      {
        // peer suggests alternatives: parse and respond accordingly (if MRU/NAT etc)
        // For IP address NAK we can parse suggested address and adopt.
        // Simple policy: if peer NAKs our IP, accept their proposed IP (if present)
        if (optsLen >= 6 && opts[0] == IPCP_OPTION_IPADDRESS && opts[1] == 6) {
          myIp[0] = opts[2];
          myIp[1] = opts[3];
          myIp[2] = opts[4];
          myIp[3] = opts[5];
        }
        // retry request with new myIp
        sendIpcpConfReq();
        break;
      }
    case CONF_REJ:
      {
        // peer rejected options: fallback to basic (no options)
        sendIpcpConfReq();
        break;
      }
  }

  // check if both ack states are true => IPCP negotiated
  if (ipcpAckedLocal && ipcpAckedPeer && lcpOpen) {
    ipcpOpen = true;
    pppPhase = PPP_OPEN;
    // signal to host: nothing to do because host already sees CONNECT; but we can log
    Serial.println("\r\nIPCP open - PPP established");
    // optionally print assigned IPs
    Serial.print("\r\nLocal IP: ");
    Serial.print(myIp[0]);
    Serial.print('.');
    Serial.print(myIp[1]);
    Serial.print('.');
    Serial.print(myIp[2]);
    Serial.print('.');
    Serial.println(myIp[3]);
    Serial.print("\r\nPeer IP: ");
    Serial.print(peerIp[0]);
    Serial.print('.');
    Serial.print(peerIp[1]);
    Serial.print('.');
    Serial.print(peerIp[2]);
    Serial.print('.');
    Serial.println(peerIp[3]);
  }
}

void sendIpcpConfReq() {
  uint8_t pkt[40];
  int i = 0;

  pkt[i++] = CONF_REQ;
  pkt[i++] = ++ipcpMyId;
  pkt[i++] = 0;  // length high placeholder
  pkt[i++] = 0;  // length low placeholder

  // ---- Opción IP Address (Type 3) ----
  pkt[i++] = 3;  // Option Type
  pkt[i++] = 6;  // Length (Type+Len+4 bytes)
  pkt[i++] = (localIP >> 24) & 0xFF;
  pkt[i++] = (localIP >> 16) & 0xFF;
  pkt[i++] = (localIP >> 8) & 0xFF;
  pkt[i++] = localIP & 0xFF;

  // ---- Opción Primary DNS (Type 129) ----
  pkt[i++] = 129;  // Primary DNS
  pkt[i++] = 6;
  pkt[i++] = (dns1IP >> 24) & 0xFF;
  pkt[i++] = (dns1IP >> 16) & 0xFF;
  pkt[i++] = (dns1IP >> 8) & 0xFF;
  pkt[i++] = dns1IP & 0xFF;

  // ---- Opción Secondary DNS (Type 131) ----
  pkt[i++] = 131;  // Secondary DNS
  pkt[i++] = 6;
  pkt[i++] = (dns2IP >> 24) & 0xFF;
  pkt[i++] = (dns2IP >> 16) & 0xFF;
  pkt[i++] = (dns2IP >> 8) & 0xFF;
  pkt[i++] = dns2IP & 0xFF;

  // ---- Longitud total ----
  uint16_t len = i;
  pkt[2] = (len >> 8) & 0xFF;
  pkt[3] = len & 0xFF;

  sendPPPToSerial(PPP_IPCP, pkt, len);
  ipcpTimer = millis();
  ipcpState = PPP_IPCP_REQ;

  Serial.println("[IPCP] Configure-Request enviado.");
}

void sendIpcpConfAck(uint8_t id, uint8_t *options, int optLen) {
  uint8_t pkt[16];
  pkt[0] = CONF_ACK;
  pkt[1] = id;
  pkt[2] = 0;
  pkt[3] = 4 + optLen;
  memcpy(pkt + 4, options, optLen);
  sendPPPToSerial(PPP_IPCP, pkt, 4 + optLen);
}
void sendIpcpConfNak(uint8_t id, uint8_t *options, int optLen) {
  uint8_t pkt[16];
  pkt[0] = CONF_NAK;
  pkt[1] = id;
  pkt[2] = 0;
  pkt[3] = 4 + optLen;
  memcpy(pkt + 4, options, optLen);
  sendPPPToSerial(PPP_IPCP, pkt, 4 + optLen);
}

void pppStateMachine() {
  if (dummyMode && connectedNeighborIdx == -1) {
    // Dummy: Fuerza OPEN inmediato post-CONNECT
    if (pppPhase == PPP_ESTABLISH) {
      pppPhase = PPP_OPEN;
      lcpOpen = true;
      ipcpOpen = true;
      ipcpState = PPP_IPCP_ACK;
      lcpState = PPP_LCP_ACK;
      papAuthenticated = true;  // Skip auth
      Serial.println("\r\nPPP DUMMY OPEN - Local IPs assigned");
      Serial.print("\r\nLocal: 10.0.0.1 Peer: 10.0.0.2");
      return;  // No retries/echoes
    }
    // Dummy skip en otras fases si atascado
    if (pppPhase == PPP_AUTHENTICATE) {
      pppPhase = PPP_NETWORK;
      papAuthenticated = true;
      Serial.println("[PPP] Dummy: Skip AUTH → NETWORK");
      if (!ipcpRequested) {
        sendIpcpConfReq();
        ipcpRequested = true;
        ipcpTimer = millis();
      }
      return;
    }
    if (pppPhase == PPP_NETWORK) {
      ipcpState = PPP_IPCP_ACK;
      ipcpOpen = true;
      pppPhase = PPP_OPEN;
      Serial.println("[PPP] Dummy: Force IPCP ACK → OPEN");
      return;
    }
  }

  static unsigned long lastEcho = 0;
  unsigned long now = millis();

  // Echo en OPEN (opcional skip en dummy para menos ruido)
  if (pppPhase == PPP_OPEN && now - lastEcho > 5000 && !(dummyMode && connectedNeighborIdx == -1)) {
    uint8_t echo[4] = { ECHO_REQ, ++lcpMyId, 0, 4 };
    sendPPPToSerial(PPP_LCP, echo, 4);
    lastEcho = now;
  }

  // LCP negotiation (ESTABLISH)
  if (pppPhase == PPP_ESTABLISH && lcpState != PPP_LCP_ACK && now - pppTimer > 3000) {
    if (lcpRetries < 10) {
      sendLcpConfReq();
      lcpState = PPP_LCP_REQ;  // Marca como enviado
      lcpRetries++;
      pppTimer = now;
    } else {
      Serial.println("[PPP] LCP retries agotados, DEAD.");
      pppPhase = PPP_DEAD;
      lcpRetries = 0;  // Reset
    }
  }

  // Auth phase (si aplica) - Dummy
  if (pppPhase == PPP_AUTHENTICATE && now - pppTimer > 3000) {
    if (dummyMode && connectedNeighborIdx == -1) {
      pppPhase = PPP_NETWORK;
      chapAuthenticated = true;
      return;
    }
    if (!papAuthenticated && !chapAuthenticated) {
      //Serial.println("[PPP] Auth timeout, TERMINATE.");
      pppPhase = PPP_TERMINATE;
    } else {
      pppPhase = PPP_NETWORK;  // Avanza si auth OK
    }
  }

  // IPCP negotiation (NETWORK)
  if (pppPhase == PPP_NETWORK && ipcpState != PPP_IPCP_ACK && now - ipcpTimer > 3000) {
    if (ipcpRetries < 10) {
      sendIpcpConfReq();
      ipcpState = PPP_IPCP_REQ;
      ipcpRetries++;
      ipcpTimer = now;
    } else {
      Serial.println("[PPP] IPCP retries agotados, DEAD.");
      pppPhase = PPP_DEAD;
      ipcpRetries = 0;  // Reset
    }
  }

  // Transiciones basadas en estados
  if (lcpState == PPP_LCP_ACK && ipcpState == PPP_IPCP_ACK && pppPhase != PPP_OPEN) {
    lcpOpen = true;
    ipcpOpen = true;
    pppPhase = PPP_OPEN;
    Serial.println("[PPP] LCP+IPCP completos -> OPEN.");
    // Print IPs aquí para ambos modos (opcional)
    Serial.print("\r\nLocal: 10.0.0.1 Peer: 10.0.0.2");
  }

  // TERMINATE (skip TERM_REQ en dummy)
  if (pppPhase == PPP_TERMINATE) {
    if (!(dummyMode && connectedNeighborIdx == -1)) {
      uint8_t term[4] = { TERM_REQ, ++lcpMyId, 0, 4 };
      sendPPPToSerial(PPP_LCP, term, 4);
    }
    pppPhase = PPP_DEAD;
    lcpState = PPP_LCP_DOWN;
    ipcpState = PPP_IPCP_DOWN;
    lcpOpen = false;
    ipcpOpen = false;
    lcpRetries = 0;
    ipcpRetries = 0;
    papAuthenticated = false;
  }

  // Fallback si server activo pero LCP down (skip en dummy)
  if (pppServerActive && lcpState == PPP_LCP_DOWN && now - pppTimer > 3000 && !(dummyMode && connectedNeighborIdx == -1)) {
    sendLcpConfReq();
    pppTimer = now;
  }
}


// ------------------ Hayes-like AT handler ------------------
void processAT(String cmd) {
  delay(10);  //10ms
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.startsWith("AT")) cmd = cmd.substring(2);
  if (cmd.length() == 0) {
    sendOK();
    return;
  }

  // Parser simple: divide por '&' para subcomandos, pero maneja '=' dentro
  int ampPos = 0;
  while (ampPos < cmd.length()) {
    int nextAmp = cmd.indexOf('&', ampPos);
    if (nextAmp == -1) nextAmp = cmd.length();
    String subcmd = cmd.substring(ampPos, nextAmp);
    processSubCommand(subcmd);
    ampPos = nextAmp + 1;
  }
  Serial.flush();
}

// Helper nueva para subcomandos (añádela después de processAT)
void processSubCommand(String subcmd) {
  subcmd.trim();

  // --- comandos básicos ---
  if (subcmd == "Z") {
    // Reset full: estados PPP, LoRa, etc.
    modemState = S_IDLE;
    connectedNeighborIdx = -1;
    pppPhase = PPP_DEAD;
    lcpState = PPP_LCP_DOWN;
    ipcpState = PPP_IPCP_DOWN;
    lcpOpen = false;
    ipcpOpen = false;
    papAuthenticated = false;
    virtualCarrier = false;
    pppServerActive = false;
    pruneNeighbors();  // Limpia tabla neighbors
    sendOK();
    return;
  }
  if (subcmd == "&F") {
    // Factory defaults: resetea SREG, etc.
    for (int i = 0; i < 100; i++) SREG[i] = 0;
    SREG[0] = 0;
    SREG[7] = 50;
    SREG[10] = 14;        // Defaults Hayes
    verboseMode = true;   // V1 default
    resultCodeLevel = 4;  // X4 default
    sendOK();
    return;
  }
  if (subcmd.startsWith("&C")) {
    // &C1: DCD on carrier (default ya, pero confirma)
    sendOK();
    return;
  }
  if (subcmd.startsWith("&K")) {
    // &K3: XON/XOFF (ignora CTS)
    sendOK();
    return;
  }
  if (subcmd.startsWith("&D")) {
    // &D2: DTR normal
    sendOK();
    return;
  }
  if (subcmd.startsWith("D2")) {  // &D2: DTR control
    // Ignora o setea flag para DTR drop on disconnect
    sendOK();
    return;
  }
  if (subcmd.startsWith("C1")) {  // &C1: DCD on carrier
    // Ya emulado con virtualCarrier
    sendOK();
    return;
  }
  if (subcmd.startsWith("S0=")) {
    int eq = subcmd.indexOf('=');
    int v = (eq > 0) ? subcmd.substring(eq + 1).toInt() : 0;
    SREG[0] = v;  // Auto-answer rings
    sendOK();
    return;
  }
  if (subcmd.startsWith("X")) {
    int v = subcmd.substring(1).toInt();
    resultCodeLevel = v;  // 0-4 para verbose results
    sendOK();
    return;
  }
  if (subcmd == "M1") {  // Speaker on until carrier (ignora, simula con log?)
    sendOK();
    return;
  }
  if (subcmd == "E1") {  // Echo on
    // Implementa eco: en loop(), si !pppServerActive, Serial.print char
    echoOn = true;
    sendOK();
    return;
  }
  if (subcmd == "E0") {  // Echo off
    echoOn = false;
    sendOK();
    return;
  }
  // --- comandos básicos ---

  if (subcmd == "&V") {
    String number = subcmd.substring(2);
    number.trim();
    if (number.length() == 0) {
      sendERROR();
      return;
    }
    if (number == "0") {
      Serial.println("\r\nCTRL_VERB OFF");
      sendOK();
      return;
    }
    if (number == "1") {
      Serial.println(storedProfile);
      sendOK();
      return;
    }
  }
  if (subcmd == "&W") {
    storedProfile = "SAVED";
    sendOK();
    return;
  }
  if (subcmd == "E") {
    sendOK();
    return;
  }

  if (subcmd == "V0") {
    verboseMode = false;
    sendOK();
    return;
  }
  if (subcmd == "V1") {
    verboseMode = true;
    sendOK();
    return;
  }
  // --- registros S ---
  if (subcmd.startsWith("S")) {
    int eq = subcmd.indexOf('=');
    int q = subcmd.indexOf('?');
    if (eq > 0) {  // ATSn=v
      String regStr = subcmd.substring(1, eq);
      int n = regStr.toInt();
      int v = subcmd.substring(eq + 1).toInt();
      if (n >= 0 && n < 100) SREG[n] = constrain(v, 0, 255);
      sendOK();
      return;
    } else if (q > 0) {  // ATSn?
      String regStr = subcmd.substring(1, q);
      int n = regStr.toInt();
      if (n >= 0 && n < 100) Serial.println(SREG[n]);
      sendOK();
      return;
    }
  }
  if (subcmd.startsWith("Q")) {  // quiet mode
    // always OK
    // 0 - quite mode OFF
    // 1 - quiet mode ON
    sendOK();
    return;
  }

  if (subcmd.startsWith("I")) {
    Serial.print("\r\nHAYES-LORA-PPP-MODEM 1.0");
    Serial.print("\r\nID: ");
    Serial.println(myDialID);
    //#ifdef LORA_HAVE_DUMP_REGS
    //LoRa.dumpRegisters(Serial);
    //#endif
    sendOK();
    return;
  }
  // --- dial commands ---
  if (subcmd.startsWith("DT") || subcmd.startsWith("DP") || subcmd.startsWith("DS")) {  // Line or Pulse
    String number = subcmd.substring(2);
    number.trim();
    if (number.length() == 0) {
      sendERROR();
      return;
    }
    //sendOK();
    startDial(number);
    return;
  }
  if (subcmd.startsWith("D")) {  // Dial (general, after DT/DP handled)
    String number = subcmd.substring(1);
    number.trim();
    if (number.length() == 0) {
      sendERROR();
      return;
    }
    sendOK();
    startDial(number);
    return;
  }
  if (subcmd == "A") {
    if (modemState == S_RINGING) {
      Serial.println("\r\nANSWERING");
      uint8_t payload[12];
      for (int i = 0; i < 11; i++) payload[1 + i] = myDialID.charAt(i);
      LoRa.beginPacket();
      LoRa.write(T_SYNACK);
      LoRa.write(payload, 11);
      LoRa.endPacket();
      delay(50);
      modemState = S_CONNECTED;
      connectedNeighborIdx = findNeighborByID(pendingDialID);  // from RING
      // responder CONNECT
      startVirtualCarrier();
      return;
    } else {
      sendERROR();
      return;
    }
  }
  if (subcmd.startsWith("H") || subcmd == "H0" || subcmd == "ATH" || subcmd.startsWith("$RB")) {  // Hang up a call
    if (modemState == S_CONNECTED || virtualCarrier) {
      if (!pppServerActive) {
        LoRa.beginPacket();
        LoRa.write(T_FIN);
        LoRa.endPacket();
        pppPhase = PPP_TERMINATE;
      }
      modemState = S_IDLE;
      connectedNeighborIdx = -1;
      stopVirtualCarrier();
      Serial.flush();
    }
    sendOK();  // OK post-hangup
    return;
  }
  if (subcmd.startsWith("E")) {  // echo
    String number = subcmd.substring(1);
    number.trim();
    if (number.length() == 0) {
      sendERROR();
      return;
    }
    if (number == "0") {
      Serial.println("\r\nECHO OFF");
      sendOK();
      return;
    }
    if (number == "1") {
      Serial.println("\r\nECHO ON");
      sendOK();
      return;
    }
  }
  if (subcmd.startsWith("Z") || subcmd == "&Z") {
    modemState = S_IDLE;
    connectedNeighborIdx = -1;
    pppPhase = PPP_DEAD;
    lcpOpen = false;
    lcpState = PPP_LCP_DOWN;
    pppPhase = PPP_DEAD;
    ipcpOpen = false;
    pruneNeighbors();
    sendOK();
    return;
  }
  if (subcmd.startsWith("I1")) {
    Serial.println("\r\nHAYES-LoRa-PPP v1.0");
    sendOK();
    return;
  }
  if (subcmd.startsWith("I2")) {
    sendOK();
    return;
  }
  if (subcmd.startsWith("I3")) {
    Serial.println("\r\nHAYES-LoRa-PPP MODEM");
    sendOK();
    return;
  }
  if (subcmd.startsWith("&F")) {  // factory defaults
    sendOK();
    return;
  }
  if (subcmd.startsWith("P")) {  // Polarity
    // always OK
    // 0 - INVERTED
    // 1 - NORMAL
    sendOK();
    return;
  }
  if (subcmd.startsWith("K")) {  // Flow Control
    // always OK
    // Modes: 0,1,2
    sendOK();
    return;
  }
  if (subcmd.startsWith("$SB?") || subcmd.startsWith("$BM=")) {  // Baudrate
    // always OK
    // Modes: 115200 | 57600
    Serial.println("\r\n115200");
    sendOK();
    return;
  }
  if (subcmd.startsWith("$BM?")) {  // Busy message
    // always OK
    Serial.println("\r\nModem Busy!");
    sendOK();
    return;
  }
  if (subcmd.startsWith("+FCLASS=?")) {
    Serial.println("\r\n0");  // modo datos
    sendOK();
    return;
  }
  if (subcmd.startsWith("+FCLASS?")) {
    Serial.println("\r\n0");  // modo datos
    sendOK();
    return;
  }
  if (subcmd.startsWith("+FCLASS=0")) {
    sendOK();
    return;
  }
  if (subcmd.startsWith("+MS")) {
    Serial.println("\r\nB103");  // mentira necesaria
    return;
  }
  // --- manufacturer info ---
  if (subcmd.startsWith("+GMI")) {
    Serial.println("\r\nCUMXTV Labs");
    sendOK();
    return;
  }
  if (subcmd.startsWith("+GMM")) {
    Serial.println("\r\nLoRa PPP Modem");
    sendOK();
    return;
  }
  if (subcmd.startsWith("+GMR")) {  // Revision
    Serial.println("\r\nv1.0");
    sendOK();
    return;
  }
  if (subcmd.startsWith("+GSN")) {  // Serial number
    Serial.println(myDialID);
    sendOK();
    return;
  }
  if (subcmd.startsWith("+IPR")) {  // set baud (ignore)
    sendOK();
    return;
  }
  if (subcmd.startsWith("S0")) {  // auto-answer
    // always OK
    // ATS0=1 ON
    // ATS0? Display auto-answer
    sendOK();
    return;
  }
  if (permissiveAT) {
    sendOK();
    return;
  }
  sendERROR();
}
// ------------------ Dial / Handshake ------------------
void startDial(String number) {
  pendingDialID = number;
  int idx = findNeighborByID(number);
  bool hasPeer = (idx >= 0);

  if (!dummyMode && !hasPeer) {
    if (resultCodeLevel >= 3) Serial.println("\r\nNO DIALTONE");
    else Serial.println("\r\nNO CARRIER");
    sendERROR();
    return;
  }

  //Serial.print("\r\nDIALING "); Serial.println(number);  // Feedback para rasdial
  delay(2000);  // Simula "llamando" (ajusta S7 si quieres)

  if (dummyMode || !hasPeer) {
    // Dummy: Simula conexión local inmediata
    Serial.println("\r\nCONNECT 115200");  // Emula carrier
    modemState = S_CONNECTED;
    connectedNeighborIdx = -1;  // Marca como dummy
    startVirtualCarrier();
    return;
  }

  // Real LoRa: Procede como antes
  modemState = S_DIALING;
  connectedNeighborIdx = idx;
  sendSYN(number);
  handshakeTimestamp = millis();
  handshakeTries = 1;
}

void sendSYN(String targetID) {
  uint8_t payload[23];
  payload[0] = T_SYN;
  for (int i = 0; i < 11; i++) payload[1 + i] = (i < targetID.length()) ? targetID.charAt(i) : '0';
  for (int i = 0; i < 11; i++) payload[12 + i] = myDialID.charAt(i);
  LoRa.beginPacket();
  LoRa.write(payload, 23);
  LoRa.endPacket();
  //Serial.println("\r\nSYN sent");
}
// ------------------ LoRa receive handler ------------------
void handleLoRaPacket() {
  int packetSize = LoRa.parsePacket();
  if (!packetSize) return;
  int rssi = LoRa.packetRssi();
  uint8_t type = LoRa.read();
  if (type == T_BEACON) {
    uint8_t mac6[6];
    for (int i = 0; i < 6; i++) mac6[i] = LoRa.read();
    char macStr[32];
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
            mac6[0], mac6[1], mac6[2], mac6[3], mac6[4], mac6[5]);
    char idBuf[12];
    idBuf[11] = 0;
    for (int i = 0; i < 11; i++) idBuf[i] = (char)LoRa.read();
    String idStr = String(idBuf);
    updateNeighbor(idStr, String(macStr), rssi);
    /*Serial.print("NEIGHBOR: ID=");
    Serial.print(idStr);
    Serial.print(" MAC=");
    Serial.print(macStr);
    Serial.print(" RSSI=");
    Serial.println(rssi);*/
    return;
  }
  if (type == T_SYN) {
    char targetID[12];
    targetID[11] = 0;
    char callerID[12];
    callerID[11] = 0;
    for (int i = 0; i < 11; i++) targetID[i] = (char)LoRa.read();
    for (int i = 0; i < 11; i++) callerID[i] = (char)LoRa.read();
    String tgt = String(targetID);
    String caller = String(callerID);
    if (tgt == myDialID) {
      Serial.print("RING from ID=");
      Serial.println(caller);
      updateNeighbor(caller, String("unknown"), rssi);
      connectedNeighborIdx = findNeighborByID(caller);
      modemState = S_RINGING;
      pendingDialID = caller;  // for ATA
      Serial.println("\r\nRING");
    }
    return;
  }
  if (type == T_SYNACK) {
    char buf[12];
    buf[11] = 0;
    for (int i = 0; i < 11; i++) buf[i] = (char)LoRa.read();
    String replyID = String(buf);
    if (modemState == S_DIALING && replyID == pendingDialID) {
      //Serial.println("\r\nSYNACK received -> CONNECT");
      modemState = S_CONNECTED;
      startVirtualCarrier();  // ÚNICO lugar para CONNECT
      pppPhase = PPP_ESTABLISH;
      pppRetries = 0;
      delay(1500);       // Espera para PPP stack de Windows
      sendLcpConfReq();  // Inicia negociación inmediatamente
      pppTimer = millis();
      return;
    }
    return;
  }
  if (type == T_FIN) {
    Serial.println("\r\nREMOTE HANGUP");
    modemState = S_IDLE;
    connectedNeighborIdx = -1;
    pppPhase = PPP_DEAD;
    lcpOpen = false;
    ipcpOpen = false;
    return;
  }
  if (type == T_DATA) {
    char senderID[12];
    senderID[11] = 0;
    for (int i = 0; i < 11; i++) senderID[i] = (char)LoRa.read();
    uint8_t ipData[256];
    int ipLen = 0;
    while (LoRa.available() && ipLen < sizeof(ipData)) {
      ipData[ipLen++] = LoRa.read();
    }
    if (pppPhase == PPP_OPEN) {
      sendPPPToSerial(PPP_IP, ipData, ipLen);
    }
    return;
  }
}


// ------------------ Setup & Loop ------------------
void setup() {
  Serial.begin(115200);
  Serial.println("Starting HAYES-LoRa-PPP Modem...");
  while (!Serial)
    ;

  pinMode(LED_PIN, OUTPUT);  // DCD R-1kohm. Pull-up 10kohm a 3.3v (Carrier Detect) Pin 8 DB9
  pinMode(DSR_PIN, OUTPUT);  // DSR R-1kohm. Pull-up 10kohm a 3.3v (Data Set Ready) Pin 6 DB9
  pinMode(CTS_PIN, OUTPUT);  // CTS R-1kohm. Pull-up 10kohm a 3.3v (simula always ready) Pin 7 DB9

  digitalWrite(LED_PIN, LOW);
  digitalWrite(DSR_PIN, HIGH);
  digitalWrite(CTS_PIN, HIGH);
  Serial.flush();

  randomSeed(millis());
  magicNumber = ((uint32_t)random(65536) << 16) | random(65536);

  SPI.setSCK(LORA_SCK);
  SPI.setMISO(LORA_MISO);
  SPI.setMOSI(LORA_MOSI);
  SPI.begin();
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed!");
    while (1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
    }
  }
  LoRa.setTxPower(TX_POWER_DBM);
  LoRa.setSpreadingFactor(SF);
  LoRa.setSignalBandwidth(BANDWIDTH);
  LoRa.setCodingRate4(CODERATE);
  SREG[0] = 0;    // rings before auto-answer
  SREG[7] = 50;   // tiempo de espera de carrier (s)
  SREG[10] = 14;  // delay desconexión (decenas de ms)
  myDialID = macToDialNumber(nodeMac, 6, 11);
  //Serial.println("HAYES-LoRa-PPP Modem Ready");
  //Serial.print("My Dial ID: ");
  //Serial.println(myDialID);
  sendOK();
  lastBeacon = millis();
}
void loop() {
  unsigned long now = millis();

  // Beacons y mantenimiento LoRa (siempre)
  if (now - lastBeacon >= BEACON_INTERVAL_MS) {
    sendBeacon();
    lastBeacon = now;
  }
  pruneNeighbors();
  handleLoRaPacket();

  // Handshake retry (solo si DIALING y no dummy)
  if (modemState == S_DIALING && dummyMode == false) {
    if (now - handshakeTimestamp > HANDSHAKE_TIMEOUT_MS) {
      if (handshakeTries < HANDSHAKE_RETRIES) {
        handshakeTries++;
        handshakeTimestamp = now;
        sendSYN(pendingDialID);
      } else {
        Serial.println("\r\nNO CARRIER");
        modemState = S_IDLE;
        connectedNeighborIdx = -1;
        handshakeTries = 0;
      }
    }
  }

  // --- MODO COMMAND (IDLE/RINGING): Input AT no-bloqueante con eco condicional ---
  if (modemState == S_IDLE || modemState == S_RINGING) {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\r' || c == '\n') {  // Terminador Hayes: CR o LF
        if (atBuffer.length() > 0) {
          processAT(atBuffer);
          atBuffer = "";  // Limpia inmediatamente
        }
      } else {
        atBuffer += c;
        if (echoOn) {
          Serial.print(c);  // Eco char-a-char (solo si E1)
        }
      }
    }
  }

  // Auto-answer si S0 > 0 y RINGING (opcional, para server)
  static int ringCount = 0;
  static unsigned long ringTime = 0;
  if (modemState == S_RINGING && SREG[0] > 0) {
    if (now - ringTime > 5000) {  // Ring cada 5s
      Serial.println("\r\nRING");
      ringCount++;
      ringTime = now;
      if (ringCount >= SREG[0]) {
        Serial.println("\r\nAUTO ANSWER");
        // Simula ATA: SYNACK si LoRa, o dummy CONNECT
        int idx = findNeighborByID(pendingDialID);
        if (dummyMode || idx < 0) {
          startVirtualCarrier();  // Dummy o directo
        } else {
          uint8_t payload[12];
          for (int i = 0; i < 11; i++) payload[1 + i] = myDialID.charAt(i);
          LoRa.beginPacket();
          LoRa.write(T_SYNACK);
          LoRa.write(payload, 11);
          LoRa.endPacket();
          delay(50);
          modemState = S_CONNECTED;
          connectedNeighborIdx = idx;
          startVirtualCarrier();
        }
        ringCount = 0;
        modemState = S_CONNECTED;
      }
    }
  }

  // --- MODO CONNECTED: PPP input y state machine ---
  if (modemState == S_CONNECTED) {
    pppSerialInput();   // Procesa PPP frames (incluye +++ escape)
    pppStateMachine();  // Negociación/retries

    if (pppServerActive) {
      // Fallback para LCP si down (pero no en dummy)
      if (!dummyMode && lcpState == PPP_LCP_DOWN && now - pppTimer > 3000) {
        sendLcpConfReq();
        pppTimer = now;
      }
      // Dummy force-open ya en pppStateMachine()
    }
  }

  delay(1);  // Pequeño delay para estabilidad (no bloquea)
}
// ------------------ Funciones auxiliares (sin cambios) ------------------
void sendBeacon() {
  uint8_t payload[18];
  payload[0] = T_BEACON;
  for (int i = 0; i < 6; i++) payload[1 + i] = nodeMac[i];
  for (int i = 0; i < 11; i++) payload[7 + i] = (i < myDialID.length()) ? myDialID.charAt(i) : '0';
  LoRa.beginPacket();
  LoRa.write(payload, 18);  // enviar todo el FRAME
  LoRa.endPacket();
}
