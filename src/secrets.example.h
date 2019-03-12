#define DEBUGPRINT

// Sensor UDP API server address and port
const IPAddress udpHost(127,0,0,1);
const int udpPort(1234);

// Local UDP response port
const int udpLocalPort = 4567;

// Wifi AP's
const char* wifi_SSID = "Wifi AP SSID"; // Default Wifi AP SSID
const char* wifi_PWD = "wifipassword";   // Defult Wifi AP password
const char* wifi_SSID_ALT = "Wifi AP2 SSID"; // Secondary Wifi AP SSID
const char* wifi_PWD_ALT = "wifipassword2";   // Secondary Wifi AP password
const char* wifi_SSID_ALT_2 = "Helsingin kaupungin WLAN"; // Tertiary passwordless open wifi

// Using a static IP avoids DHCP and speeds up connection. Only used on primary wifi AP.
const IPAddress localIp(10,0,0,200);
const IPAddress gatewayIp(10,0,0,1); 
const IPAddress subnetIp(255,255,255,0);
const IPAddress dnsIp(127,0,0,1);

// AES Encryption Key
const byte aes_key[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// AES initialization vector
const byte aes_iv[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
