/*
 * MCP Server Implementation with ESP-NOW and Camera Gate Servo (GPIO 21)
 * Reference: https://modelcontextprotocol.io/specification/2024-11-05
 */

#include "mcp_server.h"
#include <esp_app_desc.h>
#include <esp_log.h>
#include <esp_pthread.h>
#include <driver/ledc.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <algorithm>
#include <cstring>
#include <iterator>
#include <optional>

#include "application.h"
#include "board.h"
#include "display.h"
#include "lvgl_image.h"
#include "lvgl_theme.h"
#include "settings.h"

#define TAG "MCP"

void RegisterAnimaBodyControl(McpServer* server);

#define CAMERA_SERVO_PIN 21
#define LEDC_TIMER_SERVO LEDC_TIMER_0
#define LEDC_MODE_SERVO LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_SERVO LEDC_CHANNEL_0
#define LEDC_FREQUENCY 50 // 50Hz for Servo Motor

static void init_camera_servo() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE_SERVO,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .timer_num = LEDC_TIMER_SERVO,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num = CAMERA_SERVO_PIN,
        .speed_mode = LEDC_MODE_SERVO,
        .channel = LEDC_CHANNEL_SERVO,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_SERVO,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
}

static void set_camera_gate_angle(int angle) {
    uint32_t duty = 1638 + (angle * (8192 - 1638) / 180);
    ledc_set_duty(LEDC_MODE_SERVO, LEDC_CHANNEL_SERVO, duty);
    ledc_update_duty(LEDC_MODE_SERVO, LEDC_CHANNEL_SERVO);
}

McpServer::McpServer() {}

McpServer::~McpServer() = default;

void McpServer::AddCommonTools() {
    auto original_tools = std::move(tools_);
    auto& board = Board::GetInstance();

    RegisterAnimaBodyControl(this);

    AddTool("self.get_device_status",
            "Provides the real-time information of the device...",
            PropertyList(), [&board](const PropertyList& properties) -> ReturnValue {
                return board.GetDeviceStatusJson();
            });

    AddTool("self.audio_speaker.set_volume",
            "Set the volume of the audio speaker.",
            PropertyList({Property("volume", kPropertyTypeInteger, 0, 100)}),
            [&board](const PropertyList& properties) -> ReturnValue {
                auto codec = board.GetAudioCodec();
                codec->SetOutputVolume(properties["volume"].value<int>());
                return true;
            });

    auto backlight = board.GetBacklight();
    if (backlight) {
        AddTool("self.screen.set_brightness", "Set the brightness of the screen.",
                PropertyList({Property("brightness", kPropertyTypeInteger, 0, 100)}),
                [backlight](const PropertyList& properties) -> ReturnValue {
                    uint8_t brightness = static_cast<uint8_t>(properties["brightness"].value<int>());
                    backlight->SetBrightness(brightness, true);
                    return true;
                });
    }

#ifdef HAVE_LVGL
    auto camera = board.GetCamera();
    if (camera) {
        init_camera_servo();
        set_camera_gate_angle(0);

        AddTool("self.camera.take_photo",
                "Take a photo and explain it.",
                PropertyList({Property("question", kPropertyTypeString)}),
                [camera](const PropertyList& properties) -> ToolResult {
                    TaskPriorityReset priority_reset(1);

                    ESP_LOGI(TAG, "Opening camera gate on GPIO 21...");
                    set_camera_gate_angle(90); 
                    vTaskDelay(pdMS_TO_TICKS(400)); 

                    if (!camera->Capture()) {
                        set_camera_gate_angle(0); 
                        return std::unexpected("Failed to capture photo");
                    }
                    auto question = properties["question"].value<std::string>();
                    auto result = camera->Explain(question);

                    ESP_LOGI(TAG, "Closing camera gate on GPIO 21...");
                    set_camera_gate_angle(0);

                    if (!result) {
                        return std::unexpected(std::move(result.error()));
                    }
                    return std::move(*result);
                });
    }
#endif

    tools_.insert(tools_.end(), std::make_move_iterator(original_tools.begin()),
                  std::make_move_iterator(original_tools.end()));
}

void McpServer::AddUserOnlyTools() {
    AddUserOnlyTool("self.get_system_info", "Get the system information", PropertyList(),
                    [this](const PropertyList& properties) -> ReturnValue {
                        auto& board = Board::GetInstance();
                        return board.GetSystemInfoJson();
                    });

    AddUserOnlyTool("self.reboot", "Reboot the system", PropertyList(),
                    [this](const PropertyList& properties) -> ReturnValue {
                        auto& app = Application::GetInstance();
                        app.Schedule([&app]() {
                            vTaskDelay(pdMS_TO_TICKS(1000));
                            app.Reboot();
                        });
                        return true;
                    });
}

void McpServer::AddTool(std::unique_ptr<McpTool> tool) {
    if (std::find_if(tools_.begin(), tools_.end(), [&tool](const auto& existing) {
            return existing->name() == tool->name();
        }) != tools_.end()) {
        return;
    }
    tools_.push_back(std::move(tool));
}

void McpServer::AddTool(const std::string& name, const std::string& description,
                        const PropertyList& properties, ToolCallback callback) {
    AddTool(std::make_unique<McpTool>(name, description, properties, std::move(callback)));
}

void McpServer::AddUserOnlyTool(const std::string& name, const std::string& description,
                                const PropertyList& properties, ToolCallback callback) {
    auto tool = std::make_unique<McpTool>(name, description, properties, std::move(callback));
    tool->set_user_only(true);
    AddTool(std::move(tool));
}

void McpServer::ParseMessage(const std::string& message, ResponseSender response_sender) {
    CJsonUniquePtr json(cJSON_Parse(message.c_str()));
    if (json == nullptr) return;
    ParseMessage(json.get(), std::move(response_sender));
}

void McpServer::ParseCapabilities(const cJSON* capabilities) {
    auto vision = cJSON_GetObjectItem(capabilities, "vision");
    if (cJSON_IsObject(vision)) {
        auto url = cJSON_GetObjectItem(vision, "url");
        auto token = cJSON_GetObjectItem(vision, "token");
        if (cJSON_IsString(url)) {
            auto camera = Board::GetInstance().GetCamera();
            if (camera) {
                std::string url_str = std::string(url->valuestring);
                std::string token_str = cJSON_IsString(token) ? std::string(token->valuestring) : "";
                camera->SetExplainUrl(url_str, token_str);
            }
        }
    }
}

void McpServer::ParseMessage(const cJSON* json, ResponseSender response_sender) {
    auto version = cJSON_GetObjectItem(json, "jsonrpc");
    if (version == nullptr || !cJSON_IsString(version) || strcmp(version->valuestring, "2.0") != 0) return;

    auto method = cJSON_GetObjectItem(json, "method");
    if (method == nullptr || !cJSON_IsString(method)) return;

    auto method_str = std::string(method->valuestring);
    if (method_str.find("notifications") == 0) return;

    auto params = cJSON_GetObjectItem(json, "params");
    auto id = cJSON_GetObjectItem(json, "id");
    if (id == nullptr || !cJSON_IsNumber(id)) return;
    auto id_int = id->valueint;

    if (method_str == "initialize") {
        if (cJSON_IsObject(params)) {
            auto capabilities = cJSON_GetObjectItem(params, "capabilities");
            if (cJSON_IsObject(capabilities)) ParseCapabilities(capabilities);
        }
        auto app_desc = esp_app_get_description();
        std::string message = "{\"protocolVersion\":\"2024-11-05\",\"capabilities\":{\"tools\":{}},\"serverInfo\":{\"name\":\"" BOARD_NAME "\",\"version\":\"";
        message += app_desc->version;
        message += "\"}}";
        ReplyResult(id_int, message, response_sender);
    } else if (method_str == "tools/list") {
        std::string cursor_str = "";
        bool list_user_only_tools = false;
        if (params != nullptr) {
            auto cursor = cJSON_GetObjectItem(params, "cursor");
            if (cJSON_IsString(cursor)) cursor_str = std::string(cursor->valuestring);
            auto with_user_tools = cJSON_GetObjectItem(params, "withUserTools");
            if (cJSON_IsBool(with_user_tools)) list_user_only_tools = with_user_tools->valueint == 1;
        }
        GetToolsList(id_int, cursor_str, list_user_only_tools, response_sender);
    } else if (method_str == "tools/call") {
        auto tool_name = cJSON_GetObjectItem(params, "name");
        auto tool_arguments = cJSON_GetObjectItem(params, "arguments");
        DoToolCall(id_int, std::string(tool_name->valuestring), tool_arguments, std::move(response_sender));
    }
}

void McpServer::SendResponse(const std::string& payload, const ResponseSender& response_sender) {
    if (response_sender) response_sender(payload);
    else Application::GetInstance().SendMcpMessage(payload);
}

void McpServer::ReplyResult(int id, const std::string& result, const ResponseSender& response_sender) {
    std::string payload = "{\"jsonrpc\":\"2.0\",\"id\":" + std::to_string(id) + ",\"result\":" + result + "}";
    SendResponse(payload, response_sender);
}

void McpServer::ReplyError(int id, int code, const std::string& message, const ResponseSender& response_sender) {
    std::string payload = "{\"jsonrpc\":\"2.0\",\"id\":" + std::to_string(id) + ",\"error\":{\"code\":" + std::to_string(code) + ",\"message\":\"" + message + "\"}}";
    SendResponse(payload, response_sender);
}

void McpServer::ReplyError(int id, const std::string& message, const ResponseSender& response_sender) {
    ReplyError(id, -32603, message, response_sender);
}

void McpServer::GetToolsList(int id, const std::string& cursor, bool list_user_only_tools, const ResponseSender& response_sender) {
    std::string json = "{\"tools\":[";
    for (auto& tool : tools_) {
        if (!list_user_only_tools && tool->user_only()) continue;
        json += tool->to_json() + ",";
    }
    if (json.back() == ',') json.pop_back();
    json += "]}";
    ReplyResult(id, json, response_sender);
}

void McpServer::DoToolCall(int id, const std::string& tool_name, const cJSON* tool_arguments, ResponseSender response_sender) {
    auto tool_iter = std::find_if(tools_.begin(), tools_.end(), [&tool_name](const auto& tool) {
        return tool->name() == tool_name;
    });
    if (tool_iter == tools_.end()) {
        ReplyError(id, -32602, "Unknown tool: " + tool_name, response_sender);
        return;
    }
    McpTool* tool = tool_iter->get();
    PropertyList arguments = tool->properties();
    for (auto& argument : arguments) {
        if (cJSON_IsObject(tool_arguments)) {
            auto value = cJSON_GetObjectItem(tool_arguments, argument.name().c_str());
            if (value) {
                if (argument.type() == kPropertyTypeInteger) argument.set_value<int>(value->valueint);
                else if (argument.type() == kPropertyTypeString) argument.set_value<std::string>(value->valuestring);
            }
        }
    }
    auto result = tool->Call(arguments);
    if (!result) {
        ReplyError(id, result.error(), response_sender);
        return;
    }
    ReplyResult(id, *result, response_sender);
}

// ==================== ANIMA 6 ESP-NOW WIRELESS BODY CONTROL ====================
static const char* ANIMA_TAG = "ANIMA_ESP_NOW";
static uint8_t receiver_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

void Anima_ESP_NOW_Init() {
    if (esp_now_init() != ESP_OK) {
        ESP_LOGE(ANIMA_TAG, "ESP-NOW Initialization Failed");
        return;
    }
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiver_mac, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
}

void SendAnimaCommandViaEspNow(const char* action_cmd) {
    char packet[64];
    snprintf(packet, sizeof(packet), "CMD:%s", action_cmd);
    esp_now_send(receiver_mac, (uint8_t*)packet, strlen(packet) + 1);
}

void RegisterAnimaBodyControl(McpServer* server) {
    Anima_ESP_NOW_Init();
    server->AddTool("control_anima_body", 
        "Control Anima 6 movements like jaw, neck, arms, scary mode via wireless ESP-NOW", 
        PropertyList({Property("action", kPropertyTypeString)}),
        [](const PropertyList& properties) -> ReturnValue {
            auto action = properties["action"].value<std::string>();
            SendAnimaCommandViaEspNow(action.c_str());
            return true;
        }
    );
}
