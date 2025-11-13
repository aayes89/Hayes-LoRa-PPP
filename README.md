# Hayes-LoRa-PPP
Hayes-LoRa-PPP Modem
<p>
Este es un firmware completo y open-source para transformar un Raspberry Pi Pico + módulo LoRa SX1278 en un módem inalámbrico emulado Hayes (comandos AT) que soporta PPP transparente sobre LoRa.<br>
Imagina conectar tu PC a una red IP remota vía radio de largo alcance, como un "dial-up" moderno pero off-grid y sin cables.<br><br>
Perfecto para IoT, experimentos hackers, redes mesh rurales o survival tech.<br>
Desarrollado en Arduino IDE, este proyecto emula un módem clásico de los 90s para que funcione seamless con Windows (o Linux) como una conexión telefónica, pero usando LoRa para el transporte.<br>
¡Prueba conectar dos Picos a 10km de distancia y hacer ping a IPs vía aire!
</p>

✨ <b>Características Principales</b>

* Emulación Hayes completa: Soporta comandos AT estándar (ATD, ATA, ATH, ATZ, ATI, ATSn, ATV, ATX, etc.) con modo "permissivo" para máxima compatibilidad.
* Discovery automático: Envía beacons LoRa cada 2s para detectar nodos vecinos (hasta 16 en tabla, con RSSI y timeout).
* Handshake seguro: Protocolo SYN/SYNACK/FIN para conexiones P2P, con retries y timeouts.
* PPP over LoRa: Negociación LCP/IPCP/PAP completa (IPs dinámicas, MRU 1500, Magic Number, Echo). Transporta paquetes IP transparentes (hasta 256 bytes por frame).
* Modo servidor/cliente: Marca ("dial") o contesta llamadas vía AT commands. Emula "CONNECT 115200" para que Windows lo vea como un módem real.
* Logs detallados: Todo en Serial (115200 baud) para debug: neighbors, handshakes, PPP states, IPs asignadas.
* Configurable: Pines LoRa ajustables, frecuencia (433/915 MHz), SF/BW/CR, potencia TX (hasta 17dBm).

🚀 <b>Ventajas y Desventajas</b>

Para que sepas exactamente qué esperar antes de probarlo, aquí una tabla honesta:

<table>Ventajas,Desventajas
"Fácil integración: Funciona out-of-the-box con Windows ""Acceso telefónico"" – ¡como un módem dial-up real!","Single-hop solo: No routing mesh (P2P directo); para redes multi-hop, necesitarás extensiones."
"Bajo consumo: Ideal para battery-powered (Pico ~10mA en RX, LoRa duty-cycle bajo). Alcance ~1-10km en línea de vista.",Ancho de banda limitado: LoRa es lento (~5-10kbps en SF9); no para streaming/video. Paquetes IP max 256 bytes (sin fragmentación full).
"Off-grid puro: Sin WiFi/celular; perfecto para emergencias, drones o sensores remotos.",No encriptado: Frames LoRa planos; vulnerable a eavesdropping (agrega AES si necesitas).
"Open-source & extensible: Código limpio en Arduino; fácil agregar features (e.g., CHAP, auto-answer).",Dependiente de hardware: Requiere SX1278 exacto; frecuencias reguladas por país (ajusta LORA_FREQ).
"Debug amigable: Logs verbose + S-regs para tuning (e.g., S0 para auto-answer).","PPP básico: Soporta PAP pero no CHAP avanzado; puede fallar en setups estrictos (e.g., Windows Server)."
</table>

📋 <b>Requisitos</b>

<b>Hardware</b>
* Raspberry Pi Pico (o Pico W, pero no usa WiFi aquí).
* Módulo LoRa SX1278 (e.g., Heltec o Seeed, ~$10 en AliExpress).

<b>Conexiones (ajusta pines en código):</b>
CS (SS): GPIO 5
RST: GPIO 6
DIO0: GPIO 7
SPI: SCK=GP2, MISO=GP4, MOSI=GP3
LED debug: GP25

Antena LoRa (433/915 MHz, ~3-5dBi para mejor alcance).
Cable USB para Serial (COM en Windows).

<b>Software</b>

<b>Arduino IDE (v2.x) configuración:</b>
 - Board: "Raspberry Pi RP2040 Boards" (via Board Manager).
 - Librería: "LoRa" by Sandeep Mistry (via Library Manager).

<b>Para pruebas:</b> 
* TeraTERM/SmarTTY/PuTTY/HyperTerminal para AT commands.
* Windows "Configuración de Red" para PPP.

🛠️ <b>Instalación</b>

1. Clona éste repositorio.
2. Abre en Arduino IDE:
* Archivo > Abrir > y carga el archivo 'HayesLoRaPPPModem.ino'.
* Selecciona board: "Raspberry Pi Pico".
* Puerto: El COM/USB del Pico (mantén BOOTSEL presionado para upload si es necesario).
3. Configura:
* Cambia nodeMac[6] a un MAC único (e.g., {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}).
* Ajusta LORA_FREQ (433E6 para EU/Asia, 915E6 para US).
* Pines si tu wiring difiere.
4. Compila y sube:
* Verify > Upload.
* Abre Serial Monitor (115200 baud): Deberías ver "HAYES-LoRa-PPP Modem Ready" y tu "My Dial ID" (e.g., "AABBCCDDEEFF" como número de 11 dígitos).
5. Opcional pero más eficiente:
  * Skecth > Exportar Binario Compilado (para generar el UF2), será más simple copiar el binario resultanto y copiarlo por BOOTSEL.


¡Listo! Enciende dos Picos cerca; verás logs de "NEIGHBOR" vía beacons.

🔧 Uso
1. <b>Pruebas Básicas (AT Commands)</b>
Usa TeraTERM/SmarTTY/PuTTY (Serial, 115200, 8N1) para simular un terminal modem:

AT → OK
ATI → Info del modem y Dial ID.
ATD <ID_remoto> → Marca (SYN); espera "CONNECT 115200" si handshake OK.
ATA → Contesta si "RING" recibido.
ATH → Cuelga.

2. <b>Conexión PPP en Windows</b>

* Ve a Configuración > Red e Internet > Acceso telefónico > Configurar nueva conexión.
* Número de teléfono: Dial ID del nodo remoto (del ATI).
* Puerto: COM del Pico.
* Baud: 115200.
* Autenticación: "Sin" o "Nombre de usuario/contraseña" (usa dummy; PAP es permissive).
* Conecta: Windows envía ATD, handshake LoRa, negocia PPP → ¡IP asignada (e.g., 192.168.1.1/2)!

Prueba: Abre CMD y ping 192.168.1.2 – ¡llega vía LoRa!
3. <b>Ejemplo Multi-Nodo</b>

Nodo A (Dial ID: 12345678901): Conecta a PC Windows.
Nodo B (Dial ID: 98765432109): Corre solo (o con otra PC).
En A: ATD 98765432109 → PPP up, ping desde Windows a IP de B.

Consejo: Asegura línea de vista; ajusta SF=12 para más alcance (pero más lento).

🐛 <b>Troubleshooting</b>

* No init LoRa: Verifica wiring/pines; prueba LoRa.dumpRegisters(Serial) en ATI.
* No neighbors: Chequea frecuencia legal; beacons cada 2s, espera 10-20s.
* PPP falla: Logs muestran "CONF_NAK" o "NO CARRIER". Prueba IPs hardcode (myIp/peerIp). Para PAP, usa creds dummy.
* Windows no conecta: Desactiva "Compresión" en config; verifica driver COM.
* Alcance bajo: Sube TX_POWER_DBM=20; usa antena externa.

Si atascas, abre un Issue con logs Serial.

🤝 <b>Contribuir</b>

<b>¡Ayuda bienvenida!</b>

* Forkea, crea branch (e.g., feat/mesh-routing).
* Commits claros, PR con descripción.
* Ideas: CHAP full, fragmentación IP, encriptación LoRa, integración LoRaWAN.

📄 <b>Licencia</b>
MIT License – Usa, modifica, comparte libremente. © 2025 [Slam].

🙏 <b>Agradecimientos</b>

Inspirado en mods Hayes vintage y libs LoRa. Shoutout a Sandeep Mistry por la lib LoRa. ¡Estrellas y forks apreciados para promocionar off-grid hacking!
