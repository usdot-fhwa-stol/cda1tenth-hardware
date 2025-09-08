#include "web_debug_server.h"

// Global instance
WebDebugServer webDebugServer;

// Static instance pointer for callbacks
WebDebugServer* WebDebugServer::instance = nullptr;

// =============================================================================
// WEB DEBUG SERVER IMPLEMENTATION
// =============================================================================

WebDebugServer::WebDebugServer() 
    : Initializable("WebDebugServer"),
      server(nullptr), serverRunning(false), logIndex(0), logCount(0),
      systemStatus("Initializing..."), rosStatus("Not connected"), 
      motorStatus("Disabled"), performanceMetrics("N/A"),
      wifiStarted(false), wifiIP("") {
}

WebDebugServer::~WebDebugServer() {
    cleanup();
}

bool WebDebugServer::initialize() {
    // Set static instance for callbacks
    instance = this;
    
    // Use base class initialization
    return Initializable::initialize();
}

void WebDebugServer::cleanup() {
    stopServer();
    stopWiFi();
    Initializable::cleanup();
}

// Virtual method implementations from Initializable
bool WebDebugServer::doInitialize() {
    // Initialize log storage
    for (int i = 0; i < MAX_LOG_ENTRIES; i++) {
        logEntries[i] = LogEntry();
    }
    logIndex = 0;
    logCount = 0;
    
    return true;
}

void WebDebugServer::doCleanup() {
    if (server != nullptr) {
        delete server;
        server = nullptr;
    }
}

bool WebDebugServer::doHealthCheck() const {
    return serverRunning && wifiStarted;
}

// Logging methods
void WebDebugServer::log(const String& level, const String& component, const String& message) {
    addLogEntry(level, component, message);
}

void WebDebugServer::logInfo(const String& component, const String& message) {
    log("INFO", component, message);
}

void WebDebugServer::logWarning(const String& component, const String& message) {
    log("WARN", component, message);
}

void WebDebugServer::logError(const String& component, const String& message) {
    log("ERROR", component, message);
}

void WebDebugServer::logDebug(const String& component, const String& message) {
    log("DEBUG", component, message);
}

// System status methods
void WebDebugServer::updateSystemStatus(const String& status) {
    systemStatus = status;
}

void WebDebugServer::updateROSStatus(const String& status) {
    rosStatus = status;
}

void WebDebugServer::updateMotorStatus(const String& status) {
    motorStatus = status;
}

void WebDebugServer::updatePerformanceMetrics(const String& metrics) {
    performanceMetrics = metrics;
}

// Web server control
void WebDebugServer::startServer() {
    if (serverRunning) return;
    
    server = new AsyncWebServer(WEB_SERVER_PORT);
    setupWebRoutes();
    server->begin();
    serverRunning = true;
    
    logInfo("WEB", "Web server started on port " + String(WEB_SERVER_PORT));
}

void WebDebugServer::stopServer() {
    if (server != nullptr) {
        server->end();
        delete server;
        server = nullptr;
    }
    serverRunning = false;
}

bool WebDebugServer::isServerRunning() {
    return serverRunning;
}

// WiFi control
void WebDebugServer::startWiFiAP() {
    if (wifiStarted) return;
    
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
    wifiIP = WiFi.softAPIP().toString();
    wifiStarted = true;
    
    logInfo("WIFI", "WiFi AP started: " + String(WIFI_SSID) + " IP: " + wifiIP);
}

void WebDebugServer::stopWiFi() {
    if (wifiStarted) {
        WiFi.softAPdisconnect(true);
        WiFi.mode(WIFI_OFF);
        wifiStarted = false;
        wifiIP = "";
    }
}

String WebDebugServer::getWiFiIP() {
    return wifiIP;
}

// Private methods
void WebDebugServer::setupWebRoutes() {
    if (server == nullptr) return;
    
    // Root page
    server->on("/", HTTP_GET, handleRootCallback);
    
    // API endpoints
    server->on("/api/status", HTTP_GET, handleStatusCallback);
    server->on("/api/logs", HTTP_GET, handleLogsCallback);
    server->on("/api/data", HTTP_GET, handleAPICallback);
    
    // 404 handler
    server->onNotFound(handleNotFoundCallback);
}

void WebDebugServer::addLogEntry(const String& level, const String& component, const String& message) {
    logEntries[logIndex] = LogEntry(millis(), level, component, message);
    logIndex = (logIndex + 1) % MAX_LOG_ENTRIES;
    if (logCount < MAX_LOG_ENTRIES) {
        logCount++;
    }
}

String WebDebugServer::generateHTML() {
    String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 Debug Console</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        body { font-family: 'Courier New', monospace; margin: 0; padding: 20px; background: #1a1a1a; color: #00ff00; }
        .header { text-align: center; margin-bottom: 20px; }
        .status-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; margin-bottom: 20px; }
        .status-card { background: #2a2a2a; padding: 15px; border-radius: 5px; border-left: 4px solid #00ff00; }
        .status-card h3 { margin: 0 0 10px 0; color: #00ff00; }
        .status-value { font-size: 14px; color: #ffffff; }
        .logs-container { background: #000000; padding: 15px; border-radius: 5px; height: 400px; overflow-y: auto; }
        .log-entry { margin-bottom: 5px; font-size: 12px; }
        .log-timestamp { color: #666666; }
        .log-level-INFO { color: #00ff00; }
        .log-level-WARN { color: #ffff00; }
        .log-level-ERROR { color: #ff0000; }
        .log-level-DEBUG { color: #00ffff; }
        .log-component { color: #ffffff; font-weight: bold; }
        .log-message { color: #cccccc; }
        .refresh-btn { background: #00ff00; color: #000000; border: none; padding: 10px 20px; border-radius: 5px; cursor: pointer; margin: 10px 0; }
        .refresh-btn:hover { background: #00cc00; }
        .auto-refresh { margin-left: 10px; }
    </style>
</head>
<body>
    <div class="header">
        <h1>🤖 ESP32 Debug Console</h1>
        <p>Real-time system monitoring and logging</p>
    </div>
    
    <div class="status-grid">
        <div class="status-card">
            <h3>System Status</h3>
            <div class="status-value" id="system-status">Loading...</div>
        </div>
        <div class="status-card">
            <h3>ROS Connection</h3>
            <div class="status-value" id="ros-status">Loading...</div>
        </div>
        <div class="status-card">
            <h3>Motor Control</h3>
            <div class="status-value" id="motor-status">Loading...</div>
        </div>
        <div class="status-card">
            <h3>Performance</h3>
            <div class="status-value" id="performance-metrics">Loading...</div>
        </div>
    </div>
    
    <button class="refresh-btn" onclick="refreshData()">Refresh</button>
    <label class="auto-refresh">
        <input type="checkbox" id="auto-refresh" checked> Auto-refresh (2s)
    </label>
    
    <div class="logs-container" id="logs-container">
        <div class="log-entry">Loading logs...</div>
    </div>
    
    <script>
        let autoRefreshInterval;
        
        function refreshData() {
            fetch('/api/data')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('system-status').textContent = data.systemStatus;
                    document.getElementById('ros-status').textContent = data.rosStatus;
                    document.getElementById('motor-status').textContent = data.motorStatus;
                    document.getElementById('performance-metrics').textContent = data.performanceMetrics;
                    
                    const logsContainer = document.getElementById('logs-container');
                    logsContainer.innerHTML = '';
                    
                    data.logs.forEach(log => {
                        const logEntry = document.createElement('div');
                        logEntry.className = 'log-entry';
                        logEntry.innerHTML = `
                            <span class="log-timestamp">[${new Date(log.timestamp).toLocaleTimeString()}]</span>
                            <span class="log-level-${log.level}">[${log.level}]</span>
                            <span class="log-component">${log.component}:</span>
                            <span class="log-message">${log.message}</span>
                        `;
                        logsContainer.appendChild(logEntry);
                    });
                    
                    logsContainer.scrollTop = logsContainer.scrollHeight;
                })
                .catch(error => console.error('Error:', error));
        }
        
        function toggleAutoRefresh() {
            const checkbox = document.getElementById('auto-refresh');
            if (checkbox.checked) {
                autoRefreshInterval = setInterval(refreshData, 2000);
            } else {
                clearInterval(autoRefreshInterval);
            }
        }
        
        document.getElementById('auto-refresh').addEventListener('change', toggleAutoRefresh);
        
        // Initial load and start auto-refresh
        refreshData();
        toggleAutoRefresh();
    </script>
</body>
</html>
)";
    return html;
}

String WebDebugServer::generateJSON() {
    DynamicJsonDocument doc(4096);
    
    doc["systemStatus"] = systemStatus;
    doc["rosStatus"] = rosStatus;
    doc["motorStatus"] = motorStatus;
    doc["performanceMetrics"] = performanceMetrics;
    
    JsonArray logs = doc.createNestedArray("logs");
    int startIndex = (logCount < MAX_LOG_ENTRIES) ? 0 : logIndex;
    int count = logCount;
    
    for (int i = 0; i < count; i++) {
        int index = (startIndex + i) % MAX_LOG_ENTRIES;
        JsonObject log = logs.createNestedObject();
        log["timestamp"] = logEntries[index].timestamp;
        log["level"] = logEntries[index].level;
        log["component"] = logEntries[index].component;
        log["message"] = logEntries[index].message;
    }
    
    String jsonString;
    serializeJson(doc, jsonString);
    return jsonString;
}

String WebDebugServer::escapeHTML(const String& str) {
    String escaped = str;
    escaped.replace("&", "&amp;");
    escaped.replace("<", "&lt;");
    escaped.replace(">", "&gt;");
    escaped.replace("\"", "&quot;");
    escaped.replace("'", "&#39;");
    return escaped;
}

// Web route handlers
void WebDebugServer::handleRoot(AsyncWebServerRequest* request) {
    request->send(200, "text/html", generateHTML());
}

void WebDebugServer::handleAPI(AsyncWebServerRequest* request) {
    request->send(200, "application/json", generateJSON());
}

void WebDebugServer::handleLogs(AsyncWebServerRequest* request) {
    request->send(200, "application/json", generateJSON());
}

void WebDebugServer::handleStatus(AsyncWebServerRequest* request) {
    request->send(200, "application/json", generateJSON());
}

void WebDebugServer::handleNotFound(AsyncWebServerRequest* request) {
    request->send(404, "text/plain", "Not Found");
}

// Static callback functions
void WebDebugServer::handleRootCallback(AsyncWebServerRequest* request) {
    if (instance != nullptr) {
        instance->handleRoot(request);
    }
}

void WebDebugServer::handleAPICallback(AsyncWebServerRequest* request) {
    if (instance != nullptr) {
        instance->handleAPI(request);
    }
}

void WebDebugServer::handleLogsCallback(AsyncWebServerRequest* request) {
    if (instance != nullptr) {
        instance->handleLogs(request);
    }
}

void WebDebugServer::handleStatusCallback(AsyncWebServerRequest* request) {
    if (instance != nullptr) {
        instance->handleStatus(request);
    }
}

void WebDebugServer::handleNotFoundCallback(AsyncWebServerRequest* request) {
    if (instance != nullptr) {
        instance->handleNotFound(request);
    }
}
