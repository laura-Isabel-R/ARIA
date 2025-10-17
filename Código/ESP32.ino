#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <WebServer.h>

// ================= PCA9685 =================
#define PCA9685_ADDR 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);
#define SERVOMIN 125
#define SERVOMAX 575

// ================= CANALES =================
const uint8_t CANAL_SERVO1 = 1;
const uint8_t CANAL_SERVO2 = 2;
const uint8_t CANAL_SERVO3 = 3; // invertido en degToPwmCh
const uint8_t CANAL_SERVO4 = 4; // hombro
const uint8_t CANAL_SERVO5 = 5;
const uint8_t CANAL_SERVO6 = 6; // gripper

// ================= PARÁMETROS =================
const uint16_t VEL_MS = 10;

// ===== PASOS POR RUTINA =====
const int PASOS_SALUDO         = 7; //ya
const int PASOS_BOTON_ROJO     = 6; //ya
const int PASOS_BOTON_VERDE    = 6; //ya
const int PASOS_BOTON_AZUL     = 6;
const int PASOS_BOTON_AMARILLO = 6; //ya

// Límites seguros
const int LIM_MIN_S4 = 10;
const int LIM_MAX_S4 = 170;
const int LIM_MIN_S6 = 90;
const int LIM_MAX_S6 = 135;

// ================= ESTADO SERVOS =================
int anguloActual[16];
volatile bool enEjecucion = false;

// ================= HOME =================
int HOME[16];
void initHome() {
  HOME[CANAL_SERVO1] = 70;
  HOME[CANAL_SERVO2] = 70;
  HOME[CANAL_SERVO3] = 140;
  HOME[CANAL_SERVO4] = 90;
  HOME[CANAL_SERVO5] = 150;
  HOME[CANAL_SERVO6] = 150;
}
int clampByChannel(uint8_t canal, int deg){
  if (deg < 0) deg = 0; if (deg > 180) deg = 180;
  if (canal == CANAL_SERVO4) { if (deg < LIM_MIN_S4) deg = LIM_MIN_S4; if (deg > LIM_MAX_S4) deg = LIM_MAX_S4; }
  if (canal == CANAL_SERVO6) { if (deg < LIM_MIN_S6) deg = LIM_MIN_S6; if (deg > LIM_MAX_S6) deg = LIM_MAX_S6; }
  return deg;
}
uint16_t degToPwmCh(uint8_t canal, int deg){
  if (deg < 0) deg = 0; if (deg > 180) deg = 180;
  if (canal == CANAL_SERVO3) deg = 180 - deg;
  return map(deg, 0, 180, SERVOMIN, SERVOMAX);
}
void centrarTodoEnHomeInstant(){
  for (uint8_t ch = 0; ch < 16; ch++){
    int target = clampByChannel(ch, HOME[ch]);
    anguloActual[ch] = target;
    pwm.setPWM(ch, 0, degToPwmCh(ch, target));
    delay(2);
  }
}

// ================= SESIÓN / GATE =================
bool gate_open = false;        // puerta desde la Pi
bool has_owner = false;        // dueño (IP)
IPAddress owner_ip;
uint32_t session_id = 0;       // invalida páginas viejas

// ================ COOLDOWN ================
// (ms) — puedes cambiarlos aquí o con /cooldown
uint32_t cd_saludo_ms   = 10000;   // tras saludo/apertura (déjalo como prefieras)
uint32_t cd_rojo_ms     = 30000;   // 🔴 primer video: 30 s
uint32_t cd_verde_ms    = 141000;  // 🟢 segundo video: 2 min 21 s = 141000 ms
uint32_t cd_amarillo_ms = 10000;   // 🟡 imagen: 10 s
uint32_t cd_azul_ms     = 10000;   // 🔵 imagen: 10 s

volatile unsigned long cooldown_until_ms = 0;
inline bool isCooldownActive() { return millis() < cooldown_until_ms; }
inline uint32_t remainingCooldownMs(){
  unsigned long now = millis();
  return (now < cooldown_until_ms) ? (cooldown_until_ms - now) : 0;
}
inline void startCooldown(uint32_t ms){
  if (ms == 0) return;
  cooldown_until_ms = millis() + ms;
}

// ========= AUTO-CIERRE (tras 3 botones + timeout) =========
uint32_t auto_exit_delay_ms = 20000; // ajusta a 40000 o 50000 ms
uint8_t  presses_mask  = 0;          // bits: rojo=1, verde=2, amarillo=4, azul=8
uint8_t  presses_count = 0;          // cuántos botones DISTINTOS
uint32_t last_action_ms = 0;

inline void on_session_open(){
  presses_mask = 0;
  presses_count = 0;
  last_action_ms = millis();
}

inline void note_action(uint8_t bitflag){
  last_action_ms = millis();
  if ((presses_mask & bitflag) == 0){
    presses_mask |= bitflag;
    presses_count++;
    Serial.print("[AUTOEXIT] Distintos usados: ");
    Serial.println(presses_count);
  }
}

inline void maybe_auto_exit(){
  if (!gate_open) return;
  if (presses_count < 3) return; // aún no se alcanzaron 3 botones distintos
  if (millis() - last_action_ms >= auto_exit_delay_ms){
    Serial.println("[AUTOEXIT] Cierre automático por inactividad tras 3+ botones.");
    Serial2.println("DONE");
    gate_open = false;
    has_owner = false;
  }
}

// ================= HTTP HELPERS =================
WebServer server(80);
const char* AP_SSID = "ARIA-ESP32"; // AP abierto

void sendTextOK(const char* msg){ server.sendHeader("Access-Control-Allow-Origin", "*"); server.send(200, "text/plain", msg); }
void sendBad(const char* msg){ server.sendHeader("Access-Control-Allow-Origin", "*"); server.send(400, "text/plain", msg); }
void sendForbidden(const char* msg){ server.sendHeader("Access-Control-Allow-Origin", "*"); server.send(403, "text/plain", msg); }
void sendLocked(const char* msg){ server.sendHeader("Access-Control-Allow-Origin", "*"); server.send(423, "text/plain", msg); } // 423 Locked

// Dueño + gate + cooldown
bool checkAccessAndMaybeClaim(){
  if (!gate_open){
    sendLocked("Bloqueado: espera a que se detecten 4 personas y se genere un QR nuevo.");
    return false;
  }
  if (isCooldownActive()){
    char buf[96];
    snprintf(buf, sizeof(buf), "Bloqueado por %lu ms (cooldown).", (unsigned long)remainingCooldownMs());
    sendLocked(buf);
    return false;
  }
  IPAddress ip = server.client().remoteIP();
  if (!has_owner){
    owner_ip = ip; has_owner = true;
    Serial.print("[SESSION] Dueño reclamado por IP "); Serial.println(owner_ip);
    return true;
  }
  if (ip != owner_ip){
    sendForbidden("Interfaz tomada por otro dispositivo. Espera a 'Salida' y nuevo QR.");
    return false;
  }
  return true;
}

// SID en query (?sid=)
bool checkSidOrReject(){
  if (!server.hasArg("sid")){
    sendForbidden("Falta sid de sesión. Recarga desde el QR.");
    return false;
  }
  uint32_t sid = (uint32_t) server.arg("sid").toInt();
  if (sid != session_id){
    sendForbidden("Sesión inválida. Recarga desde el QR nuevo.");
    return false;
  }
  return true;
}

// ================= UI =================
const char* INDEX_HTML = R"HTML(
<!doctype html><html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Aria · ESP32</title>
<style>
  :root{--bg:#0b0b10;--card:#151520;--fg:#f5f5f7;}
  body{font-family:system-ui,Arial;margin:24px;background:var(--bg);color:var(--fg)}
  h1{font-size:22px;margin:0 0 10px}
  p{opacity:.85;margin:0 0 14px}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:10px;margin-top:10px}
  .btn{padding:14px 16px;font-size:16px;border-radius:12px;border:0;cursor:pointer}
  .card{background:var(--card);border-radius:16px;padding:18px;margin-top:14px;box-shadow:0 8px 24px rgba(0,0,0,.2)}
  .primary{background:#2196f3;color:#fff}
  .red{background:#e53935;color:#fff}
  .green{background:#43a047;color:#fff}
  .blue{background:#1e88e5;color:#fff}
  .yellow{background:#fdd835;color:#222}
  .warn{background:#fb8c00;color:#fff}
  .muted{opacity:.7}
</style></head><body>
<h1>Aria · ESP32</h1>
<p id="state" class="muted">Cargando estado…</p>

<div class="card">
  <b>Acciones del brazo</b>
  <div class="grid" style="margin-top:8px">
    <!-- Orden: Rojo, Verde, Amarillo, Azul -->
    <button class="btn red"     onclick="go('/BotonRojo')">🔴 Mecatrónica UAO</button>
    <button class="btn green"   onclick="go('/BotonVerde')">🟢 Proyectos UAO</button>
    <button class="btn yellow"  onclick="go('/BotonAmarillo')">🟡 Conéctate con Mecatrónica UAO</button>
    <button class="btn blue"    onclick="go('/BotonAzul')">🔵 Cuéntanos tu experiencia</button>
    <button class="btn warn"    onclick="go('/salida')">⏏️ Salida</button>
  </div>
</div>

<script>
let SID = null;
async function go(p){
  try{
    if(!SID) await updateState();
    const url = SID ? (p + '?sid='+encodeURIComponent(SID)) : p;
    const r = await fetch(url,{cache:'no-store'});
    const t = await r.text();
    if(!r.ok) throw new Error(t||('HTTP '+r.status));
    alert(t);
    updateState();
  }catch(e){
    alert((e?.message)||'error');
    updateState();
  }
}
async function updateState(){
  try{
    const r = await fetch('/status',{cache:'no-store'});
    const j = await r.json();
    SID = j.session_id;
    const me = await (await fetch('/who',{cache:'no-store'})).text();
    let txt = `Puerta: ${j.gate_open?'ABIERTA':'CERRADA'} · ` +
              `Owner: ${j.has_owner?(j.owner_ip || 'asignado'):'(libre)'} · ` +
              `Cooldown restante: ${j.cooldown_ms_remaining} ms · ` +
              `SID: ${j.session_id} · Tú: ${me}`;
    document.getElementById('state').textContent = txt;
  }catch(e){
    document.getElementById('state').textContent = 'Estado no disponible';
  }
}
updateState();
</script>
</body></html>
)HTML";

// ================= MOVIMIENTO =================
void moverTodosServos(int servoObjetivo[6], int stepDelay = 10){
  int servoInicio[6];
  for (int i=0;i<6;i++) servoInicio[i] = anguloActual[i+1];
  int maxPasos=0;
  for (int i=0;i<6;i++){ int d = abs(servoObjetivo[i]-servoInicio[i]); if (d>maxPasos) maxPasos=d; }
  if (maxPasos<1) maxPasos=1;
  for (int s=0; s<=maxPasos; s++){
    for (int i=0;i<6;i++){
      int pos = servoInicio[i] + (s*(servoObjetivo[i]-servoInicio[i]))/maxPasos;
      uint8_t canal = i+1;
      pos = clampByChannel(canal, pos);
      pwm.setPWM(canal, 0, degToPwmCh(canal, pos));
      anguloActual[canal] = pos;
    }
    delay(stepDelay);
  }
}
void goHomeSmooth(uint16_t pausaAntes=150, uint16_t pausaDespues=50){
  delay(pausaAntes);
  int home6[6] = {HOME[1], HOME[2], HOME[3], HOME[4], HOME[5], HOME[6]};
  moverTodosServos(home6, VEL_MS);
  delay(pausaDespues);
}
void ejecutarMovimientos(int movimientos[6][15], int pasos){
  for (int p=0; p<pasos; p++){
    int servos[6];
    for (int i=0;i<6;i++) servos[i] = movimientos[i][p];
    moverTodosServos(servos, VEL_MS);
    delay(200);
  }
}
void ejecutarMovimientosYHome(int movimientos[6][15], int pasos){
  if (enEjecucion) return;
  enEjecucion = true;
  ejecutarMovimientos(movimientos, pasos);
  goHomeSmooth(150, 50);
  enEjecucion = false;
}

// ================= MATRICES =================
int saludo[6][15] = {
  { 70,  90,  90,  90,  90,  90,  90, 0,0,0,0,0,0,0,0}, // S1
  { 90,  90,  90,  90,  90,  90,  90, 0,0,0,0,0,0,0,0}, // S2
  {140, 135, 180, 135, 180, 135, 180, 0,0,0,0,0,0,0,0}, // S3
  { 90,   0,   0,   0,   0,   0,   0, 0,0,0,0,0,0,0,0}, // S4
  {150,  90,  90,  90,  90,  90,  90, 0,0,0,0,0,0,0,0}, // S5
  {150, 135, 135, 135, 135, 135, 135, 0,0,0,0,0,0,0,0}  // S6
};
int BotonRojo[6][15] = {
  { 70,  30,  42,   46,  46,  58, 0,0,0,0,0,0,0,0,0}, // S1
  { 90,  48, 135,  135, 120, 125, 0,0,0,0,0,0,0,0,0}, // S2
  {140, 120, 100,    5,   5,  90, 0,0,0,0,0,0,0,0,0}, // S3
  { 90,  90,  90,   90,  90,  90, 0,0,0,0,0,0,0,0,0}, // S4
  {150, 128,  70,   25,  25,  30, 0,0,0,0,0,0,0,0,0}, // S5
  {150, 150,  15,   15,  15,  90, 0,0,0,0,0,0,0,0,0}  // S6
};
int BotonVerde[6][15] = {
  { 70,  90,  90,  90,  90,  90,  0, 0,0,0,0,0,0,0,0}, // S1
  { 90, 120, 145, 145, 160, 120,  0, 0,0,0,0,0,0,0,0}, // S2
  {140, 160, 170, 175, 178, 130,  0, 0,0,0,0,0,0,0,0}, // S3
  { 90,  80,  90,  90,  90,  90,  0, 0,0,0,0,0,0,0,0}, // S4
  {150, 140, 155, 155, 155, 160,  0, 0,0,0,0,0,0,0,0}, // S5
  {150,  90,  90,  90,  90,  90,  0, 0,0,0,0,0,0,0,0}  // S6
};
int BotonAzul[6][15] = {
  { 70, 100,  95,  95,  95,  95,  90, 0,0,0,0,0,0,0,0}, // S1
  { 90,  90, 140, 120,  90, 180,  90, 0,0,0,0,0,0,0,0}, // S2
  {140, 120,  50,  35,  28,  30,  90, 0,0,0,0,0,0,0,0}, // S3
  { 90,  90,  90,  90,  90,  90,  90, 0,0,0,0,0,0,0,0}, // S4
  {150,  40,  40,  42,  42,  50,  90, 0,0,0,0,0,0,0,0}, // S5
  {150,  90,  90,  90,  90,  90,  90, 0,0,0,0,0,0,0,0}  // S6
};
int BotonAmarillo[6][15] = {
  { 70,  78,   78,  78,  75,  75,  90, 0,0,0,0,0,0,0,0}, // S1
  { 90,  122, 120, 120, 110,  90,  90, 0,0,0,0,0,0,0,0}, // S2
  {140,  150,  80,  80, 100, 150,  90, 0,0,0,0,0,0,0,0}, // S3
  { 90,  90,   90,  90,  90,  90,  90, 0,0,0,0,0,0,0,0}, // S4
  {150,  20,   70,  70,  70,  50,  90, 0,0,0,0,0,0,0,0}, // S5
  {150,  80,   80,  80,  80,  80,  90, 0,0,0,0,0,0,0,0}  // S6
};

// ================= RUTAS ACCIÓN =================
void addActionRoute(const char* path, int (*mat)[15], int pasos, uint32_t* cooldown_ms, const char* okmsg){
  server.on(path, HTTP_GET, [=](){
    if (!checkSidOrReject()) return;
    if (!checkAccessAndMaybeClaim()) return;
    ejecutarMovimientosYHome(mat, pasos);
    startCooldown(*cooldown_ms);
    sendTextOK(okmsg);
  });
}

// Variante que además cuenta el botón (bitflag)
void addActionRouteCounted(const char* path, int (*mat)[15], int pasos,
                           uint32_t* cooldown_ms, const char* okmsg, uint8_t bitflag){
  server.on(path, HTTP_GET, [=](){
    if (!checkSidOrReject()) return;
    if (!checkAccessAndMaybeClaim()) return;
    ejecutarMovimientosYHome(mat, pasos);
    startCooldown(*cooldown_ms);
    note_action(bitflag);
    sendTextOK(okmsg);
  });
}

// Estado (para la UI)
void handleStatus(){
  String oip = has_owner ? owner_ip.toString() : String("");
  uint32_t cd_ms = remainingCooldownMs();
  String json = String("{\"gate_open\":") + (gate_open?"true":"false")
              + ",\"has_owner\":" + (has_owner?"true":"false")
              + ",\"owner_ip\":\"" + oip + "\""
              + ",\"cooldown_ms_remaining\":" + String(cd_ms)
              + ",\"session_id\":" + String(session_id)
              + ",\"cd_saludo\":" + String(cd_saludo_ms)
              + ",\"cd_rojo\":" + String(cd_rojo_ms)
              + ",\"cd_verde\":" + String(cd_verde_ms)
              + ",\"cd_amarillo\":" + String(cd_amarillo_ms)
              + ",\"cd_azul\":" + String(cd_azul_ms)
              + "}";
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}
void handleWho(){
  IPAddress ip = server.client().remoteIP();
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", ip.toString());
}

// ======= Salida (respeta cooldown) =======
void handleSalida(){
  if (!checkSidOrReject()) return;
  if (!gate_open){ sendLocked("Ya está cerrado."); return; }
  if (isCooldownActive()){
    char buf[96];
    snprintf(buf, sizeof(buf), "Salida bloqueada por cooldown (%lu ms restantes).", (unsigned long)remainingCooldownMs());
    sendLocked(buf);
    return;
  }
  IPAddress ip = server.client().remoteIP();
  if (!has_owner || ip != owner_ip){
    sendForbidden("Solo el dueño puede salir.");
    return;
  }
  // Despedida
  ejecutarMovimientosYHome(saludo, PASOS_SALUDO);
  Serial2.println("DONE");
  gate_open = false;
  has_owner = false;
  sendTextOK("Salida OK. Despedida ejecutada. Cierre; requiere nuevo QR.");
}

// ================= UART (desde Pi) =================
void procesaEntradaSerie(String entrada, Stream &out){
  entrada.trim();
  entrada.toUpperCase();

  if (entrada == "SALUDO" || entrada == "G"){
    out.println("RX: SALUDO");
    ejecutarMovimientosYHome(saludo, PASOS_SALUDO);
    gate_open = true;
    has_owner = false;     // liberar dueño anterior
    session_id++;          // invalida páginas antiguas
    on_session_open();     // <<<<<< reset contadores auto-exit
    startCooldown(cd_saludo_ms); // bloqueo breve inicial
    out.print("OK. GATE OPEN. SID="); out.println(session_id);
    return;
  }
  if (entrada == "UNLOCK"){
    gate_open = true; has_owner = false; session_id++;
    on_session_open();     // <<<<<<
    startCooldown(cd_saludo_ms);
    out.print("OK UNLOCK. SID="); out.println(session_id);
    return;
  }
  if (entrada == "LOCK"){
    gate_open = false; has_owner = false;
    out.println("OK LOCK");
    return;
  }

  // Acciones por UART (opcional: contar también)
  if (entrada == "BOTON_ROJO" || entrada == "ROJO" || entrada == "R"){
    ejecutarMovimientosYHome(BotonRojo, PASOS_BOTON_ROJO); startCooldown(cd_rojo_ms); note_action(0x01); out.println("OK ROJO"); return;
  }
  if (entrada == "BOTON_VERDE" || entrada == "VERDE" || entrada == "V"){
    ejecutarMovimientosYHome(BotonVerde, PASOS_BOTON_VERDE); startCooldown(cd_verde_ms); note_action(0x02); out.println("OK VERDE"); return;
  }
  if (entrada == "BOTON_AZUL" || entrada == "AZUL" || entrada == "Z"){
    ejecutarMovimientosYHome(BotonAzul, PASOS_BOTON_AZUL); startCooldown(cd_azul_ms); note_action(0x08); out.println("OK AZUL"); return;
  }
  if (entrada == "BOTON_AMARILLO" || entrada == "AMARILLO" || entrada == "AM" || entrada == "Y"){
    ejecutarMovimientosYHome(BotonAmarillo, PASOS_BOTON_AMARILLO); startCooldown(cd_amarillo_ms); note_action(0x04); out.println("OK AMARILLO"); return;
  }
}

// ================= SETUP =================
void setup(){
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  Wire.begin(21, 22);
  Wire.setClock(400000);
  pwm.begin();
  pwm.setPWMFreq(60);

  initHome();
  centrarTodoEnHomeInstant();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID); // OPEN
  IPAddress ip = WiFi.softAPIP();
  Serial.print("[WEB] AP SSID: "); Serial.println(AP_SSID);
  Serial.print("[WEB] AP IP:   "); Serial.println(ip);

  server.on("/", HTTP_GET, [](){ server.send(200, "text/html", INDEX_HTML); });
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/who", HTTP_GET, handleWho);

  server.on("/home", HTTP_GET, [](){
    if (!checkSidOrReject()) return;
    if (!checkAccessAndMaybeClaim()) return;
    goHomeSmooth(); sendTextOK("OK HOME");
  });

  // Rutas que cuentan botones distintos (bitflags: rojo=1, verde=2, amarillo=4, azul=8)
  addActionRouteCounted("/BotonRojo",     BotonRojo,     PASOS_BOTON_ROJO,     &cd_rojo_ms,     "OK Mecatrónica UAO + HOME",               0x01);
  addActionRouteCounted("/BotonVerde",    BotonVerde,    PASOS_BOTON_VERDE,    &cd_verde_ms,    "OK Proyectos UAO + HOME",                 0x02);
  addActionRouteCounted("/BotonAmarillo", BotonAmarillo, PASOS_BOTON_AMARILLO, &cd_amarillo_ms, "OK Conéctate con Mecatrónica UAO + HOME", 0x04);
  addActionRouteCounted("/BotonAzul",     BotonAzul,     PASOS_BOTON_AZUL,     &cd_azul_ms,     "OK Cuéntanos tu experiencia + HOME",      0x08);

  // /saludo existe, pero no cuenta como “uno de los 3”:
  addActionRoute("/saludo", saludo, PASOS_SALUDO, &cd_saludo_ms, "OK SALUDO + HOME");

  server.on("/salida", HTTP_GET, handleSalida);

  // eco manual a Pi
  server.on("/pi", HTTP_GET, [](){
    String c = server.hasArg("c") ? server.arg("c") : "";
    if (!c.length()) { sendBad("Falta ?c=valor"); return; }
    Serial2.print(c); Serial2.print("\n");
    sendTextOK((String("Enviado a Pi: ") + c).c_str());
  });

  // Ajuste de cooldowns en vivo:
  server.on("/cooldown", HTTP_GET, [](){
    if (server.hasArg("rojo"))     cd_rojo_ms     = (uint32_t) server.arg("rojo").toInt();
    if (server.hasArg("verde"))    cd_verde_ms    = (uint32_t) server.arg("verde").toInt();
    if (server.hasArg("amarillo")) cd_amarillo_ms = (uint32_t) server.arg("amarillo").toInt();
    if (server.hasArg("azul"))     cd_azul_ms     = (uint32_t) server.arg("azul").toInt();
    if (server.hasArg("saludo"))   cd_saludo_ms   = (uint32_t) server.arg("saludo").toInt();

    if (server.hasArg("apply")){
      String a = server.arg("apply");
      if (a == "clear"){
        cooldown_until_ms = millis(); // limpia cooldown actual
      } else if (a == "shrink" && isCooldownActive()){
        uint32_t rem = remainingCooldownMs();
        uint32_t min_cd = cd_rojo_ms;
        if (cd_verde_ms    < min_cd) min_cd = cd_verde_ms;
        if (cd_amarillo_ms < min_cd) min_cd = cd_amarillo_ms;
        if (cd_azul_ms     < min_cd) min_cd = cd_azul_ms;
        uint32_t new_rem = (rem < min_cd) ? rem : min_cd;
        cooldown_until_ms = millis() + new_rem;
      }
    }

    char buf[240];
    snprintf(buf, sizeof(buf),
      "OK cooldowns — saludo:%lu, rojo:%lu, verde:%lu, amarillo:%lu, azul:%lu (ms). Cooldown restante: %lu",
      (unsigned long)cd_saludo_ms,
      (unsigned long)cd_rojo_ms,
      (unsigned long)cd_verde_ms,
      (unsigned long)cd_amarillo_ms,
      (unsigned long)cd_azul_ms,
      (unsigned long)remainingCooldownMs()
    );
    sendTextOK(buf);
  });

  server.begin();
  Serial.println("[WEB] Server iniciado (http://192.168.4.1/)");
}

// ================= LOOP =================
void loop(){
  server.handleClient();

  if (Serial.available()){
    String s = Serial.readStringUntil('\n');
    procesaEntradaSerie(s, Serial);
  }
  if (Serial2.available()){
    String s2 = Serial2.readStringUntil('\n');
    procesaEntradaSerie(s2, Serial2);
  }

  // Revisión periódica del auto-cierre
  maybe_auto_exit();
}
