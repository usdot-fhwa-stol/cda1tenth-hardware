#ifndef WEB_DEBUG_SERVER_H
#define WEB_DEBUG_SERVER_H

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "system_config.h"
#include "initializable.h"

// =============================================================================
// LOG ENTRY STRUCTURE
// =============================================================================

struct LogEntry {
    uint32_t timestamp;
    String level;
    String component;
    String message;
    
    LogEntry() : timestamp(0), level(""), component(""), message("") {}
    LogEntry(uint32_t ts, const String& lvl, const String& comp, const String& msg)
        : timestamp(ts), level(lvl), component(comp), message(msg) {}
};

// =============================================================================
// WEB DEBUG SERVER CLASS
// =============================================================================

class WebDebugServer : public Initializable {
public:
    WebDebugServer();
    ~WebDebugServer();
    
    // Initialization and cleanup
    bool initialize();
    void cleanup();
    
    // Logging methods
    void log(const String& level, const String& component, const String& message);
    void logInfo(const String& component, const String& message);
    void logWarning(const String& component, const String& message);
    void logError(const String& component, const String& message);
    void logDebug(const String& component, const String& message);
    
    // System status methods
    void updateSystemStatus(const String& status);
    void updateROSStatus(const String& status);
    void updateMotorStatus(const String& status);
    void updatePerformanceMetrics(const String& metrics);
    
    // Web server control
    void startServer();
    void stopServer();
    bool isServerRunning();
    
    // WiFi control
    void startWiFiAP();
    void stopWiFi();
    String getWiFiIP();
    
protected:
    // Virtual method implementations from Initializable
    virtual bool doInitialize() override;
    virtual void doCleanup() override;
    virtual bool doHealthCheck() const override;
    
private:
    // Web server
    AsyncWebServer* server;
    bool serverRunning;
    
    // Log storage
    LogEntry logEntries[MAX_LOG_ENTRIES];
    uint8_t logIndex;
    uint8_t logCount;
    
    // System status
    String systemStatus;
    String rosStatus;
    String motorStatus;
    String performanceMetrics;
    
    // WiFi
    bool wifiStarted;
    String wifiIP;
    
    // Private methods
    void setupWebRoutes();
    void addLogEntry(const String& level, const String& component, const String& message);
    String generateHTML();
    String generateJSON();
    String escapeHTML(const String& str);
    
    // Web route handlers
    void handleRoot(AsyncWebServerRequest* request);
    void handleAPI(AsyncWebServerRequest* request);
    void handleLogs(AsyncWebServerRequest* request);
    void handleStatus(AsyncWebServerRequest* request);
    void handleNotFound(AsyncWebServerRequest* request);
    
    // Static callback functions
    static void handleRootCallback(AsyncWebServerRequest* request);
    static void handleAPICallback(AsyncWebServerRequest* request);
    static void handleLogsCallback(AsyncWebServerRequest* request);
    static void handleStatusCallback(AsyncWebServerRequest* request);
    static void handleNotFoundCallback(AsyncWebServerRequest* request);
    
    // Static instance pointer for callbacks
    static WebDebugServer* instance;
};

// Global instance
extern WebDebugServer webDebugServer;

#endif // WEB_DEBUG_SERVER_H
