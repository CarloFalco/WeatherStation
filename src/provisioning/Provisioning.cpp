/**
 * @file Provisioning.cpp
 * @brief Implementation of the first-time setup captive portal.
 */

#include "Provisioning.h"
#include <Arduino.h>

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

Provisioning::Provisioning() : _http(PROV_HTTP_PORT) {}

// ---------------------------------------------------------------------------
// begin()
// ---------------------------------------------------------------------------

void Provisioning::begin() {
    Serial.println("[Provisioning] Starting setup wizard…");

    // ---- Configure Soft-AP -------------------------------------------------
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(PROV_AP_IP, PROV_AP_IP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(PROV_AP_SSID, PROV_AP_PASSWORD);

    Serial.printf("[Provisioning] AP started  SSID='%s'  IP=%s\n",
                  PROV_AP_SSID,
                  WiFi.softAPIP().toString().c_str());

    // ---- DNS server – wildcard redirect to AP IP ---------------------------
    _dns.setErrorReplyCode(DNSReplyCode::NoError);
    _dns.start(PROV_DNS_PORT, "*", PROV_AP_IP);

    // ---- HTTP routes -------------------------------------------------------
    _http.on("/",           HTTP_GET,  [this]() { handleRoot(); });
    _http.on("/index.html", HTTP_GET,  [this]() { handleRoot(); });
    _http.on("/save",       HTTP_POST, [this]() { handleSave(); });
    _http.onNotFound(               [this]() { handleNotFound(); });

    _http.begin();
    Serial.println("[Provisioning] HTTP server started on port 80");
    Serial.println("[Provisioning] Connect to 'WeatherStation-Setup' and open http://192.168.4.1");
}

// ---------------------------------------------------------------------------
// handle()
// ---------------------------------------------------------------------------

void Provisioning::handle() {
    _dns.processNextRequest();
    _http.handleClient();
}

// ---------------------------------------------------------------------------
// handleRoot()
// ---------------------------------------------------------------------------

void Provisioning::handleRoot() {
    // Prefer serving the page from LittleFS so it can be updated without
    // recompiling the firmware.
    if (LittleFS.exists("/index.html")) {
        File f = LittleFS.open("/index.html", "r");
        _http.streamFile(f, "text/html");
        f.close();
        return;
    }

    // Fallback: minimal inline form (should not happen in normal operation)
    String html =
        "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
        "<title>Weather Station Setup</title></head><body>"
        "<h1>Weather Station Setup</h1>"
        "<p><b>Warning:</b> /index.html not found on LittleFS.<br>"
        "Please upload the filesystem image (<code>pio run -t uploadfs</code>).</p>"
        "</body></html>";
    _http.send(200, "text/html", html);
}

// ---------------------------------------------------------------------------
// handleSave()
// ---------------------------------------------------------------------------

void Provisioning::handleSave() {
    // Collect form fields
    String wifiSsid     = formParam("wifi_ssid");
    String wifiPass     = formParam("wifi_password");
    String mqttHost     = formParam("mqtt_host");
    String mqttPortStr  = formParam("mqtt_port");
    String mqttUser     = formParam("mqtt_user");
    String mqttPass     = formParam("mqtt_password");
    String mqttClientId = formParam("mqtt_client_id");
    String baseTopic    = formParam("base_topic");
    String githubRepo   = formParam("github_repo");
    String githubToken  = formParam("github_token");

    // Sampling fields
    String bme280s  = formParam("bme280_s");
    String rains    = formParam("rain_s");
    String winds    = formParam("wind_s");
    String pm25s    = formParam("pm25_s");
    String ccs811s  = formParam("ccs811_s");
    String micss    = formParam("mics_s");
    String inas     = formParam("ina_s");
    String publishs = formParam("publish_s");
    String sleeps   = formParam("sleep_s");

    // Validate mandatory fields
    if (wifiSsid.isEmpty() || mqttHost.isEmpty()) {
        sendPage("Error",
                 "<p>Wi-Fi SSID and MQTT host are required. Please go back and fill in all mandatory fields.</p>"
                 "<p><a href='/'>&#8592; Back</a></p>",
                 true);
        return;
    }

    // Populate AppConfig singleton
    AppConfig& cfg = AppConfig::getInstance();
    cfg.wifiSsid        = wifiSsid;
    cfg.wifiPassword    = wifiPass;
    cfg.mqttHost        = mqttHost;
    cfg.mqttPort        = mqttPortStr.isEmpty() ? 1883 : (uint16_t)mqttPortStr.toInt();
    cfg.mqttUser        = mqttUser;
    cfg.mqttPassword    = mqttPass;
    cfg.mqttClientId    = mqttClientId.isEmpty() ? "weather-station-01" : mqttClientId;
    cfg.mqttBaseTopic   = baseTopic.isEmpty()    ? "weather/station01"  : baseTopic;
    cfg.otaGithubRepo   = githubRepo;
    cfg.otaGithubToken  = githubToken;

    if (!bme280s.isEmpty())  cfg.intervalBme280S      = (uint32_t)bme280s.toInt();
    if (!rains.isEmpty())    cfg.intervalRainS         = (uint32_t)rains.toInt();
    if (!winds.isEmpty())    cfg.intervalWindS         = (uint32_t)winds.toInt();
    if (!pm25s.isEmpty())    cfg.intervalPm25S         = (uint32_t)pm25s.toInt();
    if (!ccs811s.isEmpty())  cfg.intervalCcs811S       = (uint32_t)ccs811s.toInt();
    if (!micss.isEmpty())    cfg.intervalMics6814S     = (uint32_t)micss.toInt();
    if (!inas.isEmpty())     cfg.intervalIna3221S      = (uint32_t)inas.toInt();
    if (!publishs.isEmpty()) cfg.mqttPublishIntervalS  = (uint32_t)publishs.toInt();
    if (!sleeps.isEmpty())   cfg.deepSleepDurationS    = (uint32_t)sleeps.toInt();

    // Save to LittleFS
    if (!cfg.save("/config.ini")) {
        sendPage("Error",
                 "<p>Failed to write configuration to filesystem. "
                 "Check LittleFS partition.</p>"
                 "<p><a href='/'>&#8592; Back</a></p>",
                 true);
        return;
    }

    _configured = true;

    // Send success page before restarting
    sendPage("Configuration Saved",
             "<p>&#10003; Configuration saved successfully!</p>"
             "<p>The weather station will now restart and connect to your Wi-Fi network.</p>"
             "<p><i>You can reconnect to your normal Wi-Fi network.</i></p>");

    // Give the browser time to receive the response
    delay(3000);

    Serial.println("[Provisioning] Config saved – restarting…");
    ESP.restart();
}

// ---------------------------------------------------------------------------
// handleNotFound()
// ---------------------------------------------------------------------------

void Provisioning::handleNotFound() {
    // Captive-portal redirect: any unknown URL → root config page
    _http.sendHeader("Location", "http://192.168.4.1/", true);
    _http.send(302, "text/plain", "");
}

// ---------------------------------------------------------------------------
// sendPage()
// ---------------------------------------------------------------------------

void Provisioning::sendPage(const String& title,
                            const String& body,
                            bool isError) {
    String headingColor = isError ? "#c0392b" : "#27ae60";
    String html =
        "<!DOCTYPE html><html><head>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>" + title + " – Weather Station</title>"
        "<style>"
        "body{font-family:Arial,sans-serif;max-width:480px;margin:40px auto;padding:0 20px;}"
        "h1{color:" + headingColor + ";}"
        "p{line-height:1.6;}"
        "</style></head><body>"
        "<h1>" + title + "</h1>" +
        body +
        "</body></html>";
    _http.send(200, "text/html", html);
}

// ---------------------------------------------------------------------------
// formParam()
// ---------------------------------------------------------------------------

String Provisioning::formParam(const String& name) const {
    if (!_http.hasArg(name)) return String();
    String v = _http.arg(name);
    v.trim();
    return v;
}
