#include <mcp_can.h>
#include <SPI.h>
#include "xiaomi_cybergear_defs.h"

// CAN controller CS pin definition
MCP_CAN CAN0(10); // Pin 10 for CS, modify as per your setup

struct XiaomiCyberGearStatus {
    float position;
    float speed;
    float torque;
    uint16_t temperature;
};

struct XiaomiCyberGearMotionCommand {
    float position;
    float speed;
    float torque;
    float kp;
    float kd;
};

class XiaomiCyberGearDriver {
    public:
        XiaomiCyberGearDriver();
        XiaomiCyberGearDriver(uint8_t master_can_id, uint8_t cybergear_can_id);
        virtual ~XiaomiCyberGearDriver();

        /**
         * @retval -1 Error
         * @retval 0 OK
         */
        int init_mcp2515(); // Replaced TWAI init function with MCP2515
        void init_motor(uint8_t mode);
        void enable_motor();
        void stop_motor();
        void set_run_mode(uint8_t mode);

        void set_limit_speed(float speed);
        void set_limit_current(float current);
        void set_limit_torque(float torque);

        // MODE MOTION
        void send_motion_control(XiaomiCyberGearMotionCommand cmd);

        // MODE_CURRENT
        void set_current_kp(float kp);
        void set_current_ki(float ki);
        void set_current_filter_gain(float gain);
        void set_current_ref(float current);

        // MODE_POSITION
        void set_position_kp(float kp);
        void set_position_ref(float position);

        // MODE_SPEED
        void set_speed_kp(float kp);
        void set_speed_ki(float ki);
        void set_speed_ref(float speed);

        uint8_t get_run_mode() const;
        uint8_t get_motor_can_id() const;
        void set_motor_can_id(uint8_t can_id);

        void request_status();
        void process_message(); // Updated for MCP2515
        XiaomiCyberGearStatus get_status() const;
        
    private:
        uint16_t _float_to_uint(float x, float x_min, float x_max, int bits);
        float _uint_to_float(uint16_t x, float x_min, float x_max);
        void _send_can_package(uint8_t can_id, uint8_t cmd_id, uint16_t option, uint8_t len, uint8_t* data);
        void _send_can_float_package(uint8_t can_id, uint16_t addr, float value, float min, float max);

        uint8_t _cybergear_can_id;
        uint8_t _master_can_id;
        uint8_t _run_mode;
        bool _use_serial_debug;
        XiaomiCyberGearStatus _status;
};

// Constructor
XiaomiCyberGearDriver::XiaomiCyberGearDriver() {}

XiaomiCyberGearDriver::XiaomiCyberGearDriver(uint8_t master_can_id, uint8_t cybergear_can_id) {
    _master_can_id = master_can_id;
    _cybergear_can_id = cybergear_can_id;
}

XiaomiCyberGearDriver::~XiaomiCyberGearDriver() {}

// Initialize MCP2515 CAN controller
int XiaomiCyberGearDriver::init_mcp2515() {
    if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
        Serial.println("MCP2515 CAN Initialized Successfully");
        return 0;
    } else {
        Serial.println("MCP2515 CAN Initialization Failed");
        return -1;
    }
}

// Send CAN package
void XiaomiCyberGearDriver::_send_can_package(uint8_t can_id, uint8_t cmd_id, uint16_t option, uint8_t len, uint8_t* data) {
    uint8_t txBuf[8] = { cmd_id, (uint8_t)(option & 0xFF), (uint8_t)((option >> 8) & 0xFF), data[0], data[1], data[2], data[3], data[4] };
    CAN0.sendMsgBuf(can_id, 0, len, txBuf);
}

// Send CAN package with float data
void XiaomiCyberGearDriver::_send_can_float_package(uint8_t can_id, uint16_t addr, float value, float min, float max) {
    uint16_t intValue = _float_to_uint(value, min, max, 16);
    uint8_t data[2] = { (uint8_t)(intValue & 0xFF), (uint8_t)((intValue >> 8) & 0xFF) };
    _send_can_package(can_id, (uint8_t)(addr & 0xFF), (uint16_t)((addr >> 8) & 0xFF), 2, data);
}

// Process incoming CAN messages
void XiaomiCyberGearDriver::process_message() {
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];

    if (CAN0.checkReceive() == CAN_MSGAVAIL) {
        CAN0.readMsgBuf(&rxId, &len, rxBuf);
        // Handle received data here
        _status.position = _uint_to_float(((uint16_t)rxBuf[1] << 8) | rxBuf[0], -180.0, 180.0);
        _status.speed = _uint_to_float(((uint16_t)rxBuf[3] << 8) | rxBuf[2], -100.0, 100.0);
        _status.torque = _uint_to_float(((uint16_t)rxBuf[5] << 8) | rxBuf[4], -50.0, 50.0);
        _status.temperature = ((uint16_t)rxBuf[7] << 8) | rxBuf[6];
    }
}

// Convert float to uint for CAN transmission
uint16_t XiaomiCyberGearDriver::_float_to_uint(float x, float x_min, float x_max, int bits) {
    return (uint16_t)((x - x_min) * ((1 << bits) - 1) / (x_max - x_min));
}

// Convert uint back to float after CAN reception
float XiaomiCyberGearDriver::_uint_to_float(uint16_t x, float x_min, float x_max) {
    return ((float)x / (float)((1 << 16) - 1)) * (x_max - x_min) + x_min;
}

// Other methods (like init_motor, enable_motor, etc.) remain the same
