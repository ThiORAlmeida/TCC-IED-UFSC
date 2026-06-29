/*25-06-26
 * IED — Monitor de Energia + Proteções ANSI
 * ESP32 + W5500 + UART2 + Modbus TCP + Web simples via Ethernet
 *
 * VERSÃO ULTRA OTIMIZADA:
 * - Sem lógica física de relé de TRIP
 * - Sem WiFi.h
 * - Sem WebServer.h
 * - Web dashboard servido pelo W5500 na porta 80
 * - Modbus TCP pelo W5500 na porta 502
 * - UART2 recebe os canais do módulo de aquisição
 * - Opção de debug para receber os dados pelo Monitor Serial USB
 * - Mantém TRIP lógico, saída de bobina, proteções ANSI e registradores Modbus
 *
 * Entrada de dados:
 *   DEBUG_SERIAL_CONSOLE = 1 -> recebe pelo Monitor Serial USB
 *   DEBUG_SERIAL_CONSOLE = 0 -> recebe pela UART2 RX=16, TX=17, baud=115200
 *   Protocolo: $;I1;I2;I3;IA;IB;IC;V1;V2;V3;IN;TEMP;FREQ;FLAG;\n
 *   FREQ é opcional. Se não vier, fica 60 Hz.
 *   FLAG é opcional. Se vier 0 = bobina aberta / normal; 1 = fecha bobina de TRIP.
 *
 * W5500:
 *   SCK=18, MISO=19, MOSI=23, CS=5
 *   IP: 192.168.0.200
 *
 * Holding Registers Modbus:
 *   Reg 0–1   I1       Reg 12–13 V1
 *   Reg 2–3   I2       Reg 14–15 V2
 *   Reg 4–5   I3       Reg 16–17 V3
 *   Reg 6–7   IA       Reg 18–19 IN
 *   Reg 8–9   IB       Reg 20–21 Temp
 *   Reg 10–11 IC
 *   Reg 22    TRIP geral lógico 0/1
 *   Reg 23    Máscara de bits das proteções em TRIP
 *             bit0=50, bit1=51, bit2=50N, bit3=51N,
 *             bit4=59, bit5=27, bit6=26, bit7=81, bit8=24
 *   Reg 24    FLAG recebida / comando bobina 0/1
 */

#include <SPI.h>
#include <Ethernet.h>
#include <string.h>
#include <stdlib.h>

// Deixe 1 para usar WS2812B com FastLED. Se ainda ficar grande, troque para 0.
#define USAR_LEDS_WS2812B 1

#if USAR_LEDS_WS2812B
#include <FastLED.h>
#define LED_PIN 4
#define NUM_LEDS 8
CRGB leds[NUM_LEDS];

// Inverte a ordem física da fita/módulo de LEDs.
// LED lógico 0 vira LED físico 7, lógico 1 vira físico 6, etc.
// Assim a numeração visual fica: 1,2,3,4,5,6,7,8 no sentido correto.
// Usei macro em vez de função para evitar erro de protótipos automáticos do Arduino IDE.
#define LED_FISICO(ledLogico) ((NUM_LEDS - 1) - (ledLogico))
#endif

// ==================== REDE W5500 ====================
#define CS_PIN 5

byte mac[]        = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip      (192, 168,   0, 200);
IPAddress dns_ip  (  8,   8,   8,   8);
IPAddress gateway (192, 168,   0,   1);
IPAddress subnet  (255, 255, 255,   0);

EthernetServer webServer(80);
EthernetServer mbServer(502);
EthernetClient mbClient;

// ==================== UART2 ====================
#define UART2_RX   16
#define UART2_TX   17
#define UART2_BAUD 115200

// DEBUG_SERIAL_CONSOLE:
// 1 = recebe os dados pelo Monitor Serial USB, útil para debug no computador.
// 0 = recebe os dados pela UART2 RX=16/TX=17, modo normal com outro ESP32.
#define DEBUG_SERIAL_CONSOLE 0

// ==================== SAÍDA FÍSICA DA BOBINA DE TRIP ====================
// Ajuste o pino conforme seu hardware. GPIO32 foi deixado como padrão.
#define TRIP_COIL_PIN 32

// Se seu módulo de relé for ativo em LOW, troque para LOW.
#define TRIP_COIL_ATIVO HIGH
#define TRIP_COIL_INATIVO (TRIP_COIL_ATIVO == HIGH ? LOW : HIGH)

// true = bobina fechada / comando de TRIP físico ativo
bool flagTripBobina = false;

// ==================== CANAIS ====================
#define NUM_CANAIS 10
float valoresRMS[NUM_CANAIS] = {0.0f};  // I1,I2,I3,IA,IB,IC,V1,V2,V3,IN
float tempCh10 = 0.0f;
float frequenciaMedida = 60.0f;

#define IDX_I1 0
#define IDX_I2 1
#define IDX_I3 2
#define IDX_IA 3
#define IDX_IB 4
#define IDX_IC 5
#define IDX_V1 6
#define IDX_V2 7
#define IDX_V3 8
#define IDX_IN 9

char linhaSerial[180];
uint16_t posSerial = 0;

// ==================== PROTEÇÕES ANSI ====================
struct ProtecaoStatus {
  bool habilitada;
  bool pickup;
  bool trip;
  uint32_t tPickupMs;
};

struct AjustesProtecao {
  // Ajustes das correntes de entrada do transformador: I1, I2, I3
  float I_entrada_pickup_50;
  float I_entrada_pickup_51;

  // Ajustes das correntes de saída do transformador: IA, IB, IC
  float I_saida_pickup_50;
  float I_saida_pickup_51;

  // Temporização da função 51 para entrada e saída
  uint32_t t_ajuste_51_ms;

  // Ajustes da corrente de neutro IN
  float IN_pickup_50N;
  float IN_pickup_51N;
  uint32_t t_ajuste_51N_ms;

  float V_max;
  uint32_t t_ajuste_59_ms;

  float V_min;
  uint32_t t_ajuste_27_ms;

  float T_trip;

  float f_min;
  float f_max;
  uint32_t t_ajuste_81_ms;

  float VHz_ajuste;
  uint32_t t_ajuste_24_ms;
};

AjustesProtecao ajustes = {
  1.9f, 1.2f,          // 50/51 entrada: I1, I2, I3
  1.9f, 1.2f,          // 50/51 saída: IA, IB, IC
  20000,               // tempo 51 fase, em ms
  4.0f, 2.0f, 3000,    // 50N/51N neutro IN
  240.0f, 1000,        // 59
  70.0f, 1000,         // 27
  80.0f,               // 26
  57.0f, 63.0f, 500,   // 81
  3.8f, 2000           // 24
};

ProtecaoStatus st_50      = {true,  false, false, 0};
ProtecaoStatus st_51      = {true,  false, false, 0};
ProtecaoStatus st_50N     = {false, false, false, 0};
ProtecaoStatus st_51N     = {false, false, false, 0};
ProtecaoStatus st_59      = {false, false, false, 0};
ProtecaoStatus st_27      = {false, false, false, 0};
ProtecaoStatus st_26      = {false, false, false, 0};
ProtecaoStatus st_81      = {false, false, false, 0};
ProtecaoStatus st_24      = {false, false, false, 0};

bool tripGeralLatched = false;
char causaUltimoTrip[120] = "";

void atualizarSaidaBobina() {
  // A bobina fecha se a FLAG recebida for 1 OU se alguma proteção interna gerar TRIP.
  bool fecharBobina = flagTripBobina || tripGeralLatched;
  digitalWrite(TRIP_COIL_PIN, fecharBobina ? TRIP_COIL_ATIVO : TRIP_COIL_INATIVO);
}

// ==================== UTILITÁRIOS DE PROTEÇÃO ====================
bool avaliarProtecao(ProtecaoStatus& st, bool condicaoAtiva, uint32_t tAjusteMs) {
  if (!st.habilitada) {
    st.pickup = false;
    return false;
  }

  if (condicaoAtiva) {
    if (!st.pickup) {
      st.pickup = true;
      st.tPickupMs = millis();
    }

    if (tAjusteMs == 0 || (millis() - st.tPickupMs) >= tAjusteMs) {
      st.trip = true;
    }
  } else {
    st.pickup = false;
  }

  return st.trip;
}

void adicionarCausa(const char* texto) {
  if (strlen(causaUltimoTrip) + strlen(texto) + 2 < sizeof(causaUltimoTrip)) {
    strcat(causaUltimoTrip, texto);
    strcat(causaUltimoTrip, " ");
  }
}

void avaliarTodasProtecoes() {
  // Correntes de entrada e saída são avaliadas com pickups independentes,
  // porque em transformadores os lados primário e secundário podem ter
  // ordens de grandeza diferentes.
  float ImaxEntrada = max(valoresRMS[IDX_I1], max(valoresRMS[IDX_I2], valoresRMS[IDX_I3]));
  float ImaxSaida   = max(valoresRMS[IDX_IA], max(valoresRMS[IDX_IB], valoresRMS[IDX_IC]));

  bool cond50Entrada = ImaxEntrada > ajustes.I_entrada_pickup_50;
  bool cond50Saida   = ImaxSaida   > ajustes.I_saida_pickup_50;
  bool cond50Fase    = cond50Entrada || cond50Saida;

  // A função 51 atua apenas abaixo da faixa de 50, evitando que uma falta
  // instantânea acenda também o LED/estado da temporizada.
  bool cond51Entrada = (ImaxEntrada > ajustes.I_entrada_pickup_51) &&
                       (ImaxEntrada <= ajustes.I_entrada_pickup_50);
  bool cond51Saida   = (ImaxSaida > ajustes.I_saida_pickup_51) &&
                       (ImaxSaida <= ajustes.I_saida_pickup_50);
  bool cond51Fase    = cond51Entrada || cond51Saida;

  avaliarProtecao(st_50, cond50Fase, 0);
  avaliarProtecao(st_51, cond51Fase, ajustes.t_ajuste_51_ms);

  float IN = valoresRMS[IDX_IN];
  bool cond50N = IN > ajustes.IN_pickup_50N;
  bool cond51N = (IN > ajustes.IN_pickup_51N) && (IN <= ajustes.IN_pickup_50N);

  avaliarProtecao(st_50N, cond50N, 0);
  avaliarProtecao(st_51N, cond51N, ajustes.t_ajuste_51N_ms);

  float Vmax = max(valoresRMS[IDX_V1], max(valoresRMS[IDX_V2], valoresRMS[IDX_V3]));
  float Vmin = min(valoresRMS[IDX_V1], min(valoresRMS[IDX_V2], valoresRMS[IDX_V3]));

  avaliarProtecao(st_59, Vmax > ajustes.V_max, ajustes.t_ajuste_59_ms);

  bool sistemaEnergizado = Vmax > 30.0f;
  avaliarProtecao(st_27, sistemaEnergizado && Vmin < ajustes.V_min, ajustes.t_ajuste_27_ms);

  avaliarProtecao(st_26, tempCh10 > ajustes.T_trip, 0);

  bool condFreq = frequenciaMedida < ajustes.f_min || frequenciaMedida > ajustes.f_max;
  avaliarProtecao(st_81, condFreq, ajustes.t_ajuste_81_ms);

  float vHz = 0.0f;
  if (frequenciaMedida > 1.0f) vHz = Vmax / frequenciaMedida;
  avaliarProtecao(st_24, vHz > ajustes.VHz_ajuste, ajustes.t_ajuste_24_ms);

  bool novoTrip = st_50.trip || st_51.trip || st_50N.trip || st_51N.trip ||
                  st_59.trip || st_27.trip || st_26.trip || st_81.trip || st_24.trip;

  if (novoTrip && !tripGeralLatched) {
    causaUltimoTrip[0] = '\0';
    if (st_50.trip)  adicionarCausa("ANSI50");
    if (st_51.trip)  adicionarCausa("ANSI51");
    if (st_50N.trip) adicionarCausa("ANSI50N");
    if (st_51N.trip) adicionarCausa("ANSI51N");
    if (st_59.trip)  adicionarCausa("ANSI59");
    if (st_27.trip)  adicionarCausa("ANSI27");
    if (st_26.trip)  adicionarCausa("ANSI26");
    if (st_81.trip)  adicionarCausa("ANSI81");
    if (st_24.trip)  adicionarCausa("ANSI24");
  }

  tripGeralLatched = tripGeralLatched || novoTrip;
  atualizarSaidaBobina();
}

bool rearmarProtecoes() {
  bool aindaEmFalta = st_50.pickup || st_51.pickup || st_50N.pickup || st_51N.pickup ||
                      st_59.pickup || st_27.pickup || st_26.pickup || st_81.pickup || st_24.pickup;

  if (aindaEmFalta) return false;

  st_50.trip = false;
  st_51.trip = false;
  st_50N.trip = false;
  st_51N.trip = false;
  st_59.trip = false;
  st_27.trip = false;
  st_26.trip = false;
  st_81.trip = false;
  st_24.trip = false;

  tripGeralLatched = false;
  causaUltimoTrip[0] = '\0';
  atualizarSaidaBobina();
  return true;
}

// ==================== LEDS ====================
#if USAR_LEDS_WS2812B
uint32_t ultimoBlinkMs = 0;
bool fasePiscaAtual = false;

// Autoteste periódico dos LEDs:
// - No funcionamento normal, os LEDs ficam indicando status fixo.
// - A cada INTERVALO_AUTOTESTE_LEDS_MS, roda um sequencial rápido nos 8 LEDs.
// - Depois do teste, volta automaticamente para os LEDs de status.
const uint32_t INTERVALO_AUTOTESTE_LEDS_MS = 10000;  // tempo entre autotestes: 10 s
const uint32_t TEMPO_PASSO_AUTOTESTE_MS    = 150;    // tempo de cada LED no teste
const uint8_t  BRILHO_AUTOTESTE_LED        = 6;      // brilho baixo para não incomodar

bool autotesteLEDsAtivo = false;
uint8_t indiceAutotesteLED = 0;
uint32_t ultimoAutotesteLEDsMs = 0;
uint32_t ultimoPassoAutotesteMs = 0;

bool executarAutotesteLEDs() {
  uint32_t agora = millis();

  if (!autotesteLEDsAtivo) {
    if (agora - ultimoAutotesteLEDsMs < INTERVALO_AUTOTESTE_LEDS_MS) {
      return false;  // segue mostrando status normal
    }

    autotesteLEDsAtivo = true;
    indiceAutotesteLED = 0;
    ultimoPassoAutotesteMs = 0;
  }

  if (agora - ultimoPassoAutotesteMs < TEMPO_PASSO_AUTOTESTE_MS) {
    return true;  // mantém o LED atual do autoteste aceso
  }

  ultimoPassoAutotesteMs = agora;
  FastLED.clear();

  if (indiceAutotesteLED < NUM_LEDS) {
    leds[LED_FISICO(indiceAutotesteLED)] = CRGB(BRILHO_AUTOTESTE_LED, 0, 0);
    FastLED.show();
    indiceAutotesteLED++;
    return true;
  }

  autotesteLEDsAtivo = false;
  ultimoAutotesteLEDsMs = agora;
  FastLED.clear();
  FastLED.show();
  return false;  // acabou o autoteste, pode voltar para status normal
}

void corLED(uint8_t idx, const ProtecaoStatus& st) {
  if (st.trip) {
    leds[LED_FISICO(idx)] = CRGB(5, 0, 0);
  } else if (st.pickup) {
    leds[LED_FISICO(idx)] = CRGB(5, 3, 0);
  } else {
    leds[LED_FISICO(idx)] = CRGB(0, 0, 0);
  }
}

void corLEDCombinado(uint8_t idx, const ProtecaoStatus& stA, const ProtecaoStatus& stB) {
  if (stA.trip || stB.trip) {
    leds[LED_FISICO(idx)] = CRGB(5, 0, 0);
  } else if (stA.pickup || stB.pickup) {
    leds[LED_FISICO(idx)] = CRGB(5, 3, 0);
  } else {
    leds[LED_FISICO(idx)] = CRGB(0, 0, 0);
  }
}

void corLEDBobina(uint8_t idx) {
  bool bobinaAcionada = flagTripBobina || tripGeralLatched;
  leds[LED_FISICO(idx)] = bobinaAcionada ? CRGB(5, 0, 0) : CRGB(0, 0, 0);
}

void atualizarLEDs() {
  if (millis() - ultimoBlinkMs > 1000) {
    ultimoBlinkMs = millis();
    fasePiscaAtual = !fasePiscaAtual;
  }

  // Durante o autoteste, os LEDs piscam em sequência para verificar se nenhum queimou.
  // Fora do autoteste, os LEDs voltam para o status fixo abaixo.
  if (executarAutotesteLEDs()) return;

  // LED 0 = TRIP físico / bobina acionada
  // LED 1 = ANSI 50      Sobrecorrente instantânea, fase ou neutro
  // LED 2 = ANSI 51      Sobrecorrente temporizada, fase ou neutro
  // LED 3 = ANSI 59      Sobretensão
  // LED 4 = ANSI 27      Subtensão
  // LED 5 = ANSI 26      Temperatura
  // LED 6 = ANSI 81      Sub/sobrefrequência
  // LED 7 = ANSI 24      V/Hz
  corLEDBobina(0);
  corLEDCombinado(1, st_50, st_50N);
  corLEDCombinado(2, st_51, st_51N);
  corLED(3, st_59);
  corLED(4, st_27);
  corLED(5, st_26);
  corLED(6, st_81);
  corLED(7, st_24);

  FastLED.show();
}
#endif

// ==================== SERIAL ====================
bool parseDados(char* linha) {
  if (linha[0] != '$') return false;

  char* saveptr = NULL;
  char* token = strtok_r(linha, ";", &saveptr);  // pega "$"
  if (!token) return false;

  for (uint8_t ch = 0; ch <= NUM_CANAIS; ch++) {
    token = strtok_r(NULL, ";", &saveptr);
    if (!token) return false;

    float val = atof(token);
    if (ch < NUM_CANAIS) valoresRMS[ch] = val;
    else tempCh10 = val;
  }

  // Campo opcional FREQ
  token = strtok_r(NULL, ";", &saveptr);
  if (token) {
    float f = atof(token);
    if (f > 1.0f) frequenciaMedida = f;
  }

  // Campo opcional FLAG: 0 = bobina aberta / normal; 1 = fecha bobina de TRIP
  token = strtok_r(NULL, ";", &saveptr);
  if (token) {
    int flag = atoi(token);
    flagTripBobina = (flag != 0);
  }

  avaliarTodasProtecoes();
  atualizarSaidaBobina();
  return true;
}

void lerEntradaSerial() {
#if DEBUG_SERIAL_CONSOLE
  Stream& entrada = Serial;   // Monitor Serial USB
#else
  Stream& entrada = Serial2;  // UART2 normal
#endif

  while (entrada.available()) {
    char c = (char)entrada.read();

    if (c == '\n') {
      linhaSerial[posSerial] = '\0';

      if (parseDados(linhaSerial)) {
#if DEBUG_SERIAL_CONSOLE
        Serial.print(F("Linha recebida OK | FlagBobina="));
        Serial.print(flagTripBobina ? F("1") : F("0"));
        Serial.print(F(" | TripGeral="));
        Serial.println(tripGeralLatched ? F("1") : F("0"));
#endif
      } else {
#if DEBUG_SERIAL_CONSOLE
        Serial.print(F("Linha invalida: "));
        Serial.println(linhaSerial);
#endif
      }

      posSerial = 0;
    } else if (c != '\r') {
      if (posSerial < sizeof(linhaSerial) - 1) {
        linhaSerial[posSerial++] = c;
      } else {
        posSerial = 0;
#if DEBUG_SERIAL_CONSOLE
        Serial.println(F("Buffer serial estourou. Linha descartada."));
#endif
      }
    }
  }
}

// ==================== JSON / WEB ====================
void enviarCabecalhoHTTP(EthernetClient& c, const __FlashStringHelper* tipo) {
  c.println(F("HTTP/1.1 200 OK"));
  c.print(F("Content-Type: "));
  c.println(tipo);
  c.println(F("Connection: close"));
  c.println(F("Access-Control-Allow-Origin: *"));
  c.println();
}

void enviarJSONDados(EthernetClient& c) {
  enviarCabecalhoHTTP(c, F("application/json"));
  c.print(F("{\"I1\":")); c.print(valoresRMS[0], 3);
  c.print(F(",\"I2\":")); c.print(valoresRMS[1], 3);
  c.print(F(",\"I3\":")); c.print(valoresRMS[2], 3);
  c.print(F(",\"IA\":")); c.print(valoresRMS[3], 3);
  c.print(F(",\"IB\":")); c.print(valoresRMS[4], 3);
  c.print(F(",\"IC\":")); c.print(valoresRMS[5], 3);
  c.print(F(",\"V1\":")); c.print(valoresRMS[6], 2);
  c.print(F(",\"V2\":")); c.print(valoresRMS[7], 2);
  c.print(F(",\"V3\":")); c.print(valoresRMS[8], 2);
  c.print(F(",\"IN\":")); c.print(valoresRMS[9], 3);
  c.print(F(",\"Temp\":")); c.print(tempCh10, 2);
  c.print(F(",\"Freq\":")); c.print(frequenciaMedida, 3);
  c.print(F(",\"TripGeral\":")); c.print(tripGeralLatched ? 1 : 0);
  c.print(F(",\"FlagBobina\":")); c.print(flagTripBobina ? 1 : 0);
  c.print(F(",\"BobinaTrip\":")); c.print((flagTripBobina || tripGeralLatched) ? 1 : 0);
  c.print(F(",\"Causa\":\"")); c.print(causaUltimoTrip); c.print(F("\"}"));
}

void printStatus(EthernetClient& c, const char* nome, const ProtecaoStatus& st, bool ultima) {
  c.print(F("\"")); c.print(nome); c.print(F("\":{"));
  c.print(F("\"pickup\":")); c.print(st.pickup ? F("true") : F("false"));
  c.print(F(",\"trip\":")); c.print(st.trip ? F("true") : F("false"));
  c.print(F(",\"hab\":")); c.print(st.habilitada ? F("true") : F("false"));
  c.print(F("}"));
  if (!ultima) c.print(F(","));
}

void enviarJSONProtecoes(EthernetClient& c) {
  enviarCabecalhoHTTP(c, F("application/json"));
  c.print(F("{\"tripGeral\":")); c.print(tripGeralLatched ? F("true") : F("false"));
  c.print(F(",\"flagBobina\":")); c.print(flagTripBobina ? F("true") : F("false"));
  c.print(F(",\"bobinaTrip\":")); c.print((flagTripBobina || tripGeralLatched) ? F("true") : F("false"));
  c.print(F(",\"causa\":\"")); c.print(causaUltimoTrip); c.print(F("\","));
  printStatus(c, "ANSI50", st_50, false);
  printStatus(c, "ANSI51", st_51, false);
  printStatus(c, "ANSI50N", st_50N, false);
  printStatus(c, "ANSI51N", st_51N, false);
  printStatus(c, "ANSI59", st_59, false);
  printStatus(c, "ANSI27", st_27, false);
  printStatus(c, "ANSI26", st_26, false);
  printStatus(c, "ANSI81", st_81, false);
  printStatus(c, "ANSI24", st_24, true);
  c.print(F("}"));
}

// ATENÇÃO: toda a parte "pesada" (gráfico, animações, cálculo de histórico)
// roda no NAVEGADOR do usuário via JavaScript/Chart.js (CDN), não no ESP32.
// O ESP32 só envia texto fixo (vindo da Flash, via F()) + JSON pequeno em /dados.
// Isso mantém o custo de CPU/RAM do ESP32 igual ao da versão anterior.
void enviarPaginaWeb(EthernetClient& c) {
  enviarCabecalhoHTTP(c, F("text/html; charset=utf-8"));

  c.print(F("<!doctype html><html lang='pt-br'><head><meta charset='utf-8'>"));
  c.print(F("<meta name='viewport' content='width=device-width,initial-scale=1'>"));
  c.print(F("<title>IED ESP32 + W5500</title>"));

  // ---------- CSS ----------
  c.print(F("<style>"));
  c.print(F(":root{--bg:#0b0d10;--panel:#15181d;--panel2:#1b1f26;--line:#262b33;"));
  c.print(F("--txt:#e8eaed;--muted:#8a93a3;--blue:#4da3ff;--green:#2ecc71;--yellow:#e8b339;--red:#e74c3c}"));
  c.print(F("*{box-sizing:border-box}body{font-family:-apple-system,Segoe UI,Roboto,Arial,sans-serif;"));
  c.print(F("background:var(--bg);color:var(--txt);margin:0;padding:14px 14px 40px}"));
  c.print(F(".top{display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:8px;margin-bottom:14px}"));
  c.print(F("h1{font-size:18px;margin:0;color:var(--blue);font-weight:600}"));
  c.print(F(".sub{font-size:12px;color:var(--muted)}"));
  c.print(F("#dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:var(--green);margin-right:6px;vertical-align:middle}"));
  c.print(F(".off#dot{background:var(--red)}"));
  c.print(F(".banner{display:none;background:linear-gradient(90deg,#7a1010,#9a1414);border:1px solid #c0392b;"));
  c.print(F("border-radius:10px;padding:12px 14px;margin-bottom:14px;font-weight:600;animation:pulse 1.4s infinite}"));
  c.print(F("@keyframes pulse{0%,100%{opacity:1}50%{opacity:.75}}"));
  c.print(F(".section{margin-bottom:18px}"));
  c.print(F(".section h2{font-size:13px;color:var(--muted);text-transform:uppercase;letter-spacing:.04em;margin:0 0 8px;font-weight:600}"));
  c.print(F(".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(95px,1fr));gap:8px}"));
  c.print(F(".card{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:10px 8px;text-align:center}"));
  c.print(F(".card .l{font-size:11px;color:var(--muted);margin-bottom:4px}"));
  c.print(F(".card .v{font-size:21px;font-weight:700;line-height:1.1}"));
  c.print(F(".card .u{font-size:10px;color:var(--muted);margin-top:2px}"));
  c.print(F(".card.alarm{border-color:var(--red);box-shadow:0 0 0 1px var(--red) inset}"));
  c.print(F(".panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:10px}"));
  c.print(F(".chartwrap{position:relative;height:200px}"));
  c.print(F(".plist{display:flex;flex-direction:column;gap:6px}"));
  c.print(F(".prow{display:flex;align-items:center;justify-content:space-between;background:var(--panel2);"));
  c.print(F("border-radius:8px;padding:8px 10px;font-size:13px}"));
  c.print(F(".pname{font-weight:600}.pname small{color:var(--muted);font-weight:400}"));
  c.print(F(".badge{font-size:11px;padding:3px 9px;border-radius:20px;font-weight:600}"));
  c.print(F(".b-ok{background:#1f3a28;color:var(--green)}.b-pick{background:#3a3320;color:var(--yellow)}"));
  c.print(F(".b-trip{background:#3a1f22;color:var(--red)}.b-off{background:#23262b;color:var(--muted)}"));
  c.print(F("button{padding:11px 18px;border:0;border-radius:8px;font-weight:700;background:var(--blue);"));
  c.print(F("color:#06121f;cursor:pointer;font-size:14px}button:active{opacity:.8}"));
  c.print(F("footer{margin-top:18px;font-size:11px;color:var(--muted);text-align:center}"));
  c.print(F("</style></head><body>"));

  // ---------- HTML ----------
  c.print(F("<div class='top'><div><h1>IED &middot; ESP32 + W5500</h1>"));
  c.print(F("<div class='sub' id='connSub'><span id='dot'></span>conectando...</div></div>"));
  c.print(F("<button onclick='rearmar()'>Rearmar TRIP</button></div>"));

  c.print(F("<div class='banner' id='banner'></div>"));

  c.print(F("<div class='section'><h2>Correntes (A)</h2><div class='grid' id='gI'></div></div>"));
  c.print(F("<div class='section'><h2>Tensoes (V) / Outros</h2><div class='grid' id='gV'></div></div>"));

  c.print(F("<div class='section'><h2>Historico (tempo real)</h2><div class='panel chartwrap'>"));
  c.print(F("<canvas id='chart'></canvas></div></div>"));

  c.print(F("<div class='section'><h2>Protecoes ANSI</h2><div class='panel plist' id='plist'></div></div>"));

  c.print(F("<footer>Modbus TCP em "));
  c.print(Ethernet.localIP());
  c.print(F(":502 &nbsp;|&nbsp; Atualiza a cada 1s</footer>"));

  // Chart.js via CDN: processado pelo navegador do usuário, não pelo ESP32.
  c.print(F("<script src='https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.0/chart.umd.min.js'></script>"));

  c.print(F("<script>"));
  c.print(F("const corrCanais=[['I1','I1'],['I2','I2'],['I3','I3'],['IA','IA'],['IB','IB'],['IC','IC'],['IN','IN']];"));
  c.print(F("const voltCanais=[['V1','V1','V'],['V2','V2','V'],['V3','V3','V'],['Temp','Temp','C'],['Freq','Freq','Hz']];"));
  c.print(F("const protNomes={ANSI50:'50 Sobrecorrente instantanea',ANSI51:'51 Sobrecorrente temporizada',"));
  c.print(F("ANSI50N:'50N Sobrecorrente neutro instantanea',ANSI51N:'51N Sobrecorrente neutro temporizada',"));
  c.print(F("ANSI59:'59 Sobretensao',ANSI27:'27 Subtensao',ANSI26:'26 Temperatura',ANSI81:'81 Sub/sobrefrequencia',ANSI24:'24 V/Hz'};"));

  c.print(F("function mkCard(label,val,unit,alarm){"));
  c.print(F("return `<div class='card${alarm?' alarm':''}'><div class=l>${label}</div><div class=v>${val}</div><div class=u>${unit}</div></div>`}"));

  c.print(F("function setCards(host,defs,d,alarmKeys){let h='';defs.forEach(([label,key,unit])=>{"));
  c.print(F("let v=Number(d[key]);let dec=(key=='Freq')?2:(key[0]=='V'||key=='Temp'?1:2);"));
  c.print(F("h+=mkCard(label,isFinite(v)?v.toFixed(dec):'--',unit||(key[0]=='I'?'A':''),(alarmKeys||[]).includes(key))});"));
  c.print(F("host.innerHTML=h}"));

  // Histórico mantido só em memória do navegador (arrays JS), sem custo no ESP32.
  c.print(F("const MAXP=40;const hist={t:[],I:[],V:[]};"));
  c.print(F("const ctx=document.getElementById('chart');"));
  c.print(F("const chart=new Chart(ctx,{type:'line',data:{labels:hist.t,datasets:["));
  c.print(F("{label:'I max (A)',data:hist.I,borderColor:'#4da3ff',backgroundColor:'transparent',tension:.25,pointRadius:0,yAxisID:'y'},"));
  c.print(F("{label:'V max (V)',data:hist.V,borderColor:'#e8b339',backgroundColor:'transparent',tension:.25,pointRadius:0,yAxisID:'y1'}"));
  c.print(F("]},options:{animation:false,responsive:true,maintainAspectRatio:false,"));
  c.print(F("interaction:{mode:'index',intersect:false},"));
  c.print(F("scales:{x:{ticks:{color:'#8a93a3',maxTicksLimit:6}},"));
  c.print(F("y:{position:'left',ticks:{color:'#4da3ff'},grid:{color:'#262b33'}},"));
  c.print(F("y1:{position:'right',ticks:{color:'#e8b339'},grid:{display:false}}},"));
  c.print(F("plugins:{legend:{labels:{color:'#e8eaed',boxWidth:12,font:{size:11}}}}}});"));

  c.print(F("function pushHist(d){const now=new Date();const lbl=now.toLocaleTimeString().slice(0,8);"));
  c.print(F("const imax=Math.max(d.I1,d.I2,d.I3,d.IA,d.IB,d.IC,d.IN);"));
  c.print(F("const vmax=Math.max(d.V1,d.V2,d.V3);"));
  c.print(F("hist.t.push(lbl);hist.I.push(imax);hist.V.push(vmax);"));
  c.print(F("if(hist.t.length>MAXP){hist.t.shift();hist.I.shift();hist.V.shift()}"));
  c.print(F("chart.update()}"));

  c.print(F("function badge(st){if(!st.hab)return\"<span class='badge b-off'>desabilitada</span>\";"));
  c.print(F("if(st.trip)return\"<span class='badge b-trip'>TRIP</span>\";"));
  c.print(F("if(st.pickup)return\"<span class='badge b-pick'>pickup</span>\";"));
  c.print(F("return \"<span class='badge b-ok'>normal</span>\"}"));

  c.print(F("function setProtecoes(p){let h='';for(const key in protNomes){const st=p[key];if(!st)continue;"));
  c.print(F("h+=`<div class=prow><div class=pname>${protNomes[key]}</div>${badge(st)}</div>`}"));
  c.print(F("plist.innerHTML=h}"));

  c.print(F("async function poll(){"));
  c.print(F("try{"));
  c.print(F("const d=await (await fetch('/dados')).json();"));
  c.print(F("const p=await (await fetch('/protecoes')).json();"));
  c.print(F("const alarmI=[];if(p.ANSI50&&p.ANSI50.trip||p.ANSI51&&p.ANSI51.trip)alarmI.push('I1','I2','I3','IA','IB','IC');"));
  c.print(F("if(p.ANSI50N&&p.ANSI50N.trip||p.ANSI51N&&p.ANSI51N.trip)alarmI.push('IN');"));
  c.print(F("const alarmV=[];if(p.ANSI59&&p.ANSI59.trip||p.ANSI27&&p.ANSI27.trip)alarmV.push('V1','V2','V3');"));
  c.print(F("if(p.ANSI26&&p.ANSI26.trip)alarmV.push('Temp');if(p.ANSI81&&p.ANSI81.trip)alarmV.push('Freq');"));
  c.print(F("setCards(gI,corrCanais,d,alarmI);"));
  c.print(F("setCards(gV,voltCanais,d,alarmV);"));
  c.print(F("setProtecoes(p);"));
  c.print(F("pushHist(d);"));
  c.print(F("if(d.TripGeral){banner.style.display='block';banner.innerHTML='&#9888; TRIP GERAL ATIVO &mdash; '+(d.Causa||'')}"));
  c.print(F("else{banner.style.display='none'}"));
  c.print(F("dot.className='';connSub.innerHTML=\"<span id='dot'></span>online\";"));
  c.print(F("}catch(e){connSub.innerHTML=\"<span id='dot'></span>sem conexao\";connSub.className='off';"));
  c.print(F("connSub.firstElementChild.className='off'}"));
  c.print(F("}"));

  c.print(F("async function rearmar(){"));
  c.print(F("try{const d=await (await fetch('/rearmar')).json();"));
  c.print(F("alert(d.ok?'TRIP rearmado.':'Rearme negado: ainda ha falta ativa (pickup).')}"));
  c.print(F("catch(e){alert('Falha de comunicacao.')}"));
  c.print(F("}"));

  c.print(F("setInterval(poll,1000);poll();"));
  c.print(F("</script></body></html>"));
}

void enviar404(EthernetClient& c) {
  c.println(F("HTTP/1.1 404 Not Found"));
  c.println(F("Content-Type: text/plain"));
  c.println(F("Connection: close"));
  c.println();
  c.println(F("404"));
}

void tratarHTTP(EthernetClient& c) {
  char req[96];
  uint8_t n = 0;
  uint32_t t0 = millis();

  while (c.connected() && millis() - t0 < 120) {
    if (c.available()) {
      char ch = (char)c.read();
      if (ch == '\n') break;
      if (ch != '\r' && n < sizeof(req) - 1) req[n++] = ch;
    }
  }
  req[n] = '\0';

  while (c.available()) c.read();

  if (strncmp(req, "GET /dados", 10) == 0) {
    enviarJSONDados(c);
  } else if (strncmp(req, "GET /protecoes", 14) == 0) {
    enviarJSONProtecoes(c);
  } else if (strncmp(req, "GET /rearmar", 12) == 0 || strncmp(req, "POST /rearmar", 13) == 0) {
    bool ok = rearmarProtecoes();
    enviarCabecalhoHTTP(c, F("application/json"));
    c.print(F("{\"ok\":")); c.print(ok ? F("true") : F("false")); c.print(F("}"));
  } else if (strncmp(req, "GET / ", 6) == 0 || strncmp(req, "GET /HTTP", 9) == 0 || strncmp(req, "GET /?", 6) == 0) {
    enviarPaginaWeb(c);
  } else {
    enviar404(c);
  }

  delay(1);
  c.stop();
}

void atenderWeb() {
  EthernetClient c = webServer.available();
  if (c) tratarHTTP(c);
}

// ==================== MODBUS TCP ====================
#define MB_NUM_REGS 25

void floatParaRegs(float f, uint16_t& hi, uint16_t& lo) {
  uint32_t bits;
  memcpy(&bits, &f, 4);
  hi = (uint16_t)(bits >> 16);
  lo = (uint16_t)(bits & 0xFFFF);
}

void montarRegistradores(uint16_t regs[MB_NUM_REGS]) {
  float vals[11] = {
    valoresRMS[0], valoresRMS[1], valoresRMS[2],
    valoresRMS[3], valoresRMS[4], valoresRMS[5],
    valoresRMS[6], valoresRMS[7], valoresRMS[8],
    valoresRMS[9], tempCh10
  };

  for (uint8_t i = 0; i < 11; i++) {
    floatParaRegs(vals[i], regs[i * 2], regs[i * 2 + 1]);
  }

  regs[22] = tripGeralLatched ? 1 : 0;

  uint16_t mascara = 0;
  if (st_50.trip)  mascara |= (1 << 0);
  if (st_51.trip)  mascara |= (1 << 1);
  if (st_50N.trip) mascara |= (1 << 2);
  if (st_51N.trip) mascara |= (1 << 3);
  if (st_59.trip)  mascara |= (1 << 4);
  if (st_27.trip)  mascara |= (1 << 5);
  if (st_26.trip)  mascara |= (1 << 6);
  if (st_81.trip)  mascara |= (1 << 7);
  if (st_24.trip)  mascara |= (1 << 8);
  regs[23] = mascara;
  regs[24] = flagTripBobina ? 1 : 0;
}

void enviarExcecaoMB(EthernetClient& c, uint8_t* req, uint8_t fc, uint8_t exc) {
  uint8_t resp[9] = { req[0], req[1], 0x00, 0x00, 0x00, 0x03, req[6], (uint8_t)(fc | 0x80), exc };
  c.write(resp, 9);
}

void tratarModbus(EthernetClient& c) {
  if (c.available() < 12) return;

  uint8_t req[12];
  for (uint8_t i = 0; i < 12; i++) req[i] = c.read();

  if (req[2] != 0x00 || req[3] != 0x00) return;

  uint8_t unitID = req[6];
  uint8_t fc = req[7];
  uint16_t startReg = ((uint16_t)req[8] << 8) | req[9];
  uint16_t qty = ((uint16_t)req[10] << 8) | req[11];

  if (fc != 0x03) {
    enviarExcecaoMB(c, req, fc, 0x01);
    return;
  }

  if (qty == 0 || qty > MB_NUM_REGS || startReg + qty > MB_NUM_REGS) {
    enviarExcecaoMB(c, req, fc, 0x02);
    return;
  }

  uint16_t regs[MB_NUM_REGS];
  montarRegistradores(regs);

  uint8_t byteCount = qty * 2;
  uint16_t pduLen = 3 + byteCount;
  uint8_t resp[9 + MB_NUM_REGS * 2];

  resp[0] = req[0];
  resp[1] = req[1];
  resp[2] = 0x00;
  resp[3] = 0x00;
  resp[4] = (pduLen >> 8) & 0xFF;
  resp[5] = pduLen & 0xFF;
  resp[6] = unitID;
  resp[7] = 0x03;
  resp[8] = byteCount;

  for (uint16_t i = 0; i < qty; i++) {
    uint16_t v = regs[startReg + i];
    resp[9 + i * 2] = (v >> 8) & 0xFF;
    resp[10 + i * 2] = v & 0xFF;
  }

  c.write(resp, 9 + byteCount);
}

void atenderModbus() {
  if (!mbClient || !mbClient.connected()) {
    mbClient = mbServer.available();
  }

  if (mbClient && mbClient.connected() && mbClient.available() >= 12) {
    tratarModbus(mbClient);
  }
}

// ==================== SETUP / LOOP ====================
void setup() {
  Serial.begin(115200);

#if !DEBUG_SERIAL_CONSOLE
  Serial2.begin(UART2_BAUD, SERIAL_8N1, UART2_RX, UART2_TX);
#endif

  pinMode(TRIP_COIL_PIN, OUTPUT);
  digitalWrite(TRIP_COIL_PIN, TRIP_COIL_INATIVO);

  delay(300);

#if USAR_LEDS_WS2812B
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();
#endif

  SPI.begin(18, 19, 23, CS_PIN);
  Ethernet.init(CS_PIN);
  Ethernet.begin(mac, ip, dns_ip, gateway, subnet);
  delay(800);

  webServer.begin();
  mbServer.begin();

  Serial.println(F("IED ESP32 + W5500 iniciado"));
#if DEBUG_SERIAL_CONSOLE
  Serial.println(F("Modo entrada: Monitor Serial USB"));
  Serial.println(F("Envie: $;I1;I2;I3;IA;IB;IC;V1;V2;V3;IN;TEMP;FREQ;FLAG;"));
#else
  Serial.println(F("Modo entrada: UART2 RX=16/TX=17"));
#endif
  Serial.print(F("Web: http://"));
  Serial.println(Ethernet.localIP());
  Serial.print(F("Modbus TCP: "));
  Serial.print(Ethernet.localIP());
  Serial.println(F(":502"));
}

void loop() {
  lerEntradaSerial();
  atenderWeb();
  atenderModbus();

#if USAR_LEDS_WS2812B
  atualizarLEDs();
#endif
}