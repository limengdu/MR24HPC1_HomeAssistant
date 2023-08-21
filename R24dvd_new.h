#include "esphome.h"

static const char *const TAG = "R24dvd";
#define FRAME_BUF_MAX_SIZE 128
#define PRODUCT_BUF_MAX_SIZE 32

#define FRAME_HEADER1_VALUE 0x53
#define FRAME_HEADER2_VALUE 0x59
#define FRAME_TAIL1_VALUE 0x54
#define FRAME_TAIL2_VALUE 0x43

#define FRAME_CONTROL_WORD_INDEX 2
#define FRAME_COMMAND_WORD_INDEX 3
#define FRAME_DATA_INDEX 6

enum
{
    FRAME_IDLE,
    FRAME_HEADER2,
    FRAME_CTL_WORLD,
    FRAME_CMD_WORLD,
    FRAME_DATA_LEN_H,
    FRAME_DATA_LEN_L,
    FRAME_DATA_BYTES,
    FRAME_DATA_CRC,
    FRAME_TAIL1,
    FRAME_TAIL2,
};

enum
{
    STANDARD_FUNCTION_QUERY_PRODUCT_MODE = 0,
    STANDARD_FUNCTION_QUERY_PRODUCT_ID,
    STANDARD_FUNCTION_QUERY_FIRMWARE_VERDION,
    STANDARD_FUNCTION_QUERY_HARDWARE_MODE,
    // STANDARD_FUNCTION_QUERY_PROTOCOL_TYPE,
    STANDARD_FUNCTION_QUERY_HUMAN_STATUS,
    STANDARD_FUNCTION_QUERY_SCENE_MODE,
    STANDARD_FUNCTION_QUERY_SENSITIVITY,
    STANDARD_FUNCTION_QUERY_RADAR_INIT_STATUS,
    STANDARD_FUNCTION_QUERY_MOV_TARGET_DETECTION_MAX_DISTANCE,
    STANDARD_FUNCTION_QUERY_STATIC_TARGET_DETECTION_MAX_DISTANCE,
    STANDARD_FUNCTION_QUERY_UNMANNED_TIME,
    STANDARD_FUNCTION_QUERY_RADAR_OUITPUT_INFORMATION_SWITCH,
    STANDARD_FUNCTION_MAX,

    CUSTOM_FUNCTION_QUERY_RADAR_OUITPUT_INFORMATION_SWITCH,
    // CUSTOM_FUNCTION_QUERY_SPATIAL_STATIC_VALUE,
    // CUSTOM_FUNCTION_QUERY_SPATIAL_MOTION_AMPLITUDE,
    CUSTOM_FUNCTION_QUERY_PRESENCE_OF_DETECTION_RANGE,
    // CUSTOM_FUNCTION_QUERY_DISTANCE_OF_MOVING_OBJECT,
    // CUSTOM_FUNCTION_QUERY_TARGET_MOVEMENT_SPEED,
    CUSTOM_FUNCTION_QUERY_JUDGMENT_THRESHOLD_EXISTS,
    CUSTOM_FUNCTION_QUERY_MOTION_AMPLITUDE_TRIGGER_THRESHOLD,
    CUSTOM_FUNCTION_QUERY_PRESENCE_OF_PERCEPTION_BOUNDARY,
    CUSTOM_FUNCTION_QUERY_MOTION_TRIGGER_BOUNDARY,
    CUSTOM_FUNCTION_QUERY_MOTION_TRIGGER_TIME,
    CUSTOM_FUNCTION_QUERY_MOVEMENT_TO_REST_TIME,
    CUSTOM_FUNCTION_QUERY_TIME_OF_ENTER_UNMANNED,
    CUSTOM_FUNCTION_MAX,
};

enum
{
    OUTPUT_SWITCH_INIT,
    OUTPUT_SWTICH_ON,
    OUTPUT_SWTICH_OFF,
};
static char s_heartbeat_str[2][20] = {"0", "1"};
static char s_scene_str[5][20] = {"0", "1", "2", "3", "4"};
static char s_someoneExists_str[2][20] = {"0", "1"};
static char s_motion_status_str[3][20] = {"none", "0", "1"};
static char s_keep_away_str[3][20] = {"none", "0", "1"};
static char s_unmanned_time_str[9][20] = {"none", "10s", "30s", "1min", "2min", "5min", "10min", "30min", "1hour"};
static char s_motion_trig_boundary_str[10][5] = {"0.5m", "1.0m", "1.5m", "2.0m", "2.5m", "3.0m", "3.5m", "4.0m", "4.5m", "5.0m"};
static char s_presence_of_perception_boundary_str[10][5] = {"0.5m", "1.0m", "1.5m", "2.0m", "2.5m", "3.0m", "3.5m", "4.0m", "4.5m", "5.0m"};
static char s_presence_of_detection_range_str[7][10] = {"none", "0.5m", "1.0m", "1.5m", "2.0m", "2.5m", "3.0m"};

static uint8_t s_output_info_switch_flag = OUTPUT_SWITCH_INIT;

static uint8_t sg_recv_data_state = FRAME_IDLE;
static uint8_t sg_frame_len = 0;
static uint8_t sg_data_len = 0;
static uint8_t sg_frame_buf[FRAME_BUF_MAX_SIZE] = {0};
static uint8_t sg_frame_prase_buf[FRAME_BUF_MAX_SIZE] = {0};
static bool sg_init_flag = false;
static int sg_start_query_data = -1;
static int sg_start_query_data_max = -1;
static uint8_t sg_movementSigns_bak;
static uint32_t sg_motion_trigger_time_bak;
static uint32_t sg_move_to_rest_time_bak;
static uint32_t sg_enter_unmanned_time_bak;
static uint8_t sg_spatial_static_value_bak;
static uint8_t sg_static_distance_bak;
static uint8_t sg_spatial_motion_value_bak;
static uint8_t sg_motion_distance_bak;
static uint8_t sg_motion_speed_bak;
static uint8_t sg_heartbeat_flag = 255;
static uint8_t s_power_on_status = 0;

static uint8_t get_frame_crc_sum(uint8_t *data, int len)
{
    unsigned int crc_sum = 0;
    for (int i = 0; i < len - 3; i++)
    {
        crc_sum += data[i];
    }
    return crc_sum & 0xff;
}

static int get_frame_check_status(uint8_t *data, int len)
{
    uint8_t crc_sum = get_frame_crc_sum(data, len);
    uint8_t verified = data[len - 3];
    return (verified == crc_sum) ? 1 : 0;
}

static void show_frame_data(uint8_t *data, int len)
{
    printf("[%s] FRAME: %d, ", __FUNCTION__, len);
    for (int i = 0; i < len; i++)
    {
        printf("%02X ", data[i] & 0xff);
    }
    printf("\r\n");
}

class MyCustomTextSensor : public PollingComponent, public TextSensor
{
public:
    // constructor
    MyCustomTextSensor() : PollingComponent(8000) {}
    float get_setup_priority() const override { return esphome::setup_priority::LATE; }
    TextSensor *Heartbeat = new TextSensor();

    void setup() override
    {
        s_power_on_status = 0;
    }
    void update() override
    {
        if (!sg_init_flag)
            return;
        if (sg_init_flag && (255 != sg_heartbeat_flag))
        {
            this->Heartbeat->publish_state(s_heartbeat_str[sg_heartbeat_flag]);
            sg_heartbeat_flag = 0;
        }
        if (s_power_on_status < 4)
        {
            if (s_output_info_switch_flag == OUTPUT_SWITCH_INIT)
            {
                sg_start_query_data = CUSTOM_FUNCTION_QUERY_RADAR_OUITPUT_INFORMATION_SWITCH;
                sg_start_query_data_max = CUSTOM_FUNCTION_MAX;
            }
            else if (s_output_info_switch_flag == OUTPUT_SWTICH_OFF)
            {
                sg_start_query_data = STANDARD_FUNCTION_QUERY_PRODUCT_MODE;
                sg_start_query_data_max = STANDARD_FUNCTION_MAX;
            }
            else if (s_output_info_switch_flag == OUTPUT_SWTICH_ON)
            {
                sg_start_query_data = CUSTOM_FUNCTION_QUERY_RADAR_OUITPUT_INFORMATION_SWITCH;
                sg_start_query_data_max = CUSTOM_FUNCTION_MAX;
            }
            s_power_on_status++;
        }
        else
        {
            sg_start_query_data = STANDARD_FUNCTION_QUERY_PRODUCT_MODE;
            sg_start_query_data_max = STANDARD_FUNCTION_QUERY_HARDWARE_MODE;
        }
    }
};

class UartReadLineSensor : public Component, public UARTDevice, public Sensor
{
private:
    char c_product_mode[PRODUCT_BUF_MAX_SIZE + 1];
    char c_product_id[PRODUCT_BUF_MAX_SIZE + 1];
    char c_hardware_model[PRODUCT_BUF_MAX_SIZE + 1];
    char c_firmware_version[PRODUCT_BUF_MAX_SIZE + 1];

public:
    UartReadLineSensor(UARTComponent *parent) : UARTDevice(parent) {}
    float get_setup_priority() const override { return esphome::setup_priority::LATE; }

    Sensor *movementSigns = new Sensor();
    Sensor *inited = new Sensor();

    void setup();
    void loop();
    void send_query(uint8_t *query, size_t string_length);
    void get_init_finial(void);
    void get_heartbeat_packet(void);
    void get_product_mode(void);
    void get_product_id(void);
    void get_hardware_model(void);
    void get_firmware_version(void);
    void get_protocol_type(void);
    void get_human_status(void);
    void get_scene_mode(void);
    void get_sensitivity(void);
    void get_radar_init_status(void);
    void get_unmanned_time(void);
    void get_movingTargetDetectionMaxDistance(void);
    void get_staticTargetDetectionMaxDistance(void);
    void get_radar_output_information_switch(void);

    void get_spatial_static_value(void);
    void get_spatial_motion_amplitude(void);
    void get_presence_of_detection_range(void);
    void get_distance_of_moving_object(void);
    void get_target_movement_speed(void);
    void get_judgment_threshold_exists(void);
    void get_motion_amplitude_trigger_threshold(void);
    void get_presence_of_perception_boundary(void);
    void get_motion_trigger_boundary(void);
    void get_motion_trigger_time(void);
    void get_movement_to_rest_time(void);
    void get_time_of_enter_unmanned(void);

    void R24_split_data_frame(uint8_t value);
    void R24_frame_parse_work_status(uint8_t *data);
    void R24_frame_parse_human_information(uint8_t *data);
    void R24_frame_parse_detection_range(uint8_t *data);
    void R24_frame_parse_product_Information(uint8_t *data);
    void R24_frame_parse_open_underlying_information(uint8_t *data);
    void R24_parse_data_frame(uint8_t *data, uint8_t len);
};

void UartReadLineSensor::setup()
{
    ESP_LOGCONFIG(TAG, "uart_settings is 115200");
    this->check_uart_settings(115200);
    sg_init_flag = true;

    memset(this->c_product_mode, 0, PRODUCT_BUF_MAX_SIZE);
    memset(this->c_product_id, 0, PRODUCT_BUF_MAX_SIZE);
    memset(this->c_firmware_version, 0, PRODUCT_BUF_MAX_SIZE);
    memset(this->c_hardware_model, 0, PRODUCT_BUF_MAX_SIZE);
}

void UartReadLineSensor::loop()
{
    uint8_t byte;
    while (this->available())
    {
        this->read_byte(&byte);
        this->R24_split_data_frame(byte);
    }
    if (!s_output_info_switch_flag && sg_start_query_data == CUSTOM_FUNCTION_QUERY_RADAR_OUITPUT_INFORMATION_SWITCH)
    {
        this->get_radar_output_information_switch();
        sg_start_query_data++;
    }
    if ((s_output_info_switch_flag == OUTPUT_SWTICH_OFF) && (sg_start_query_data <= sg_start_query_data_max) && (sg_start_query_data >= STANDARD_FUNCTION_QUERY_PRODUCT_MODE))
    {
        switch (sg_start_query_data)
        {
        case STANDARD_FUNCTION_QUERY_PRODUCT_MODE:
            if (strlen(this->c_product_mode) > 0)
            {
                id(product_mode).publish_state(this->c_product_mode);
            }
            else
            {
                this->get_product_mode();
            }

            break;
        case STANDARD_FUNCTION_QUERY_PRODUCT_ID:
            if (strlen(this->c_product_id) > 0)
            {
                id(product_id).publish_state(this->c_product_id);
            }
            else
            {
                this->get_product_id();
            }

            break;
        case STANDARD_FUNCTION_QUERY_FIRMWARE_VERDION:
            if (strlen(this->c_firmware_version) > 0)
            {
                id(firmware_version).publish_state(this->c_firmware_version);
            }
            else
            {
                this->get_firmware_version();
            }

            break;
        case STANDARD_FUNCTION_QUERY_HARDWARE_MODE:
            if (strlen(this->c_hardware_model) > 0)
            {
                id(hardware_model).publish_state(this->c_hardware_model);
            }
            else
            {
                this->get_hardware_model();
            }
            break;
        case STANDARD_FUNCTION_QUERY_HUMAN_STATUS:
            this->get_human_status();
            break;
        case STANDARD_FUNCTION_QUERY_SCENE_MODE:
            this->get_scene_mode();
            break;
        case STANDARD_FUNCTION_QUERY_SENSITIVITY:
            this->get_sensitivity();
            break;
        case STANDARD_FUNCTION_QUERY_RADAR_INIT_STATUS:
            this->get_radar_init_status();
            break;
        case STANDARD_FUNCTION_QUERY_MOV_TARGET_DETECTION_MAX_DISTANCE:
            this->get_movingTargetDetectionMaxDistance();
            break;
        case STANDARD_FUNCTION_QUERY_STATIC_TARGET_DETECTION_MAX_DISTANCE:
            this->get_staticTargetDetectionMaxDistance();
            break;
        case STANDARD_FUNCTION_QUERY_UNMANNED_TIME:
            this->get_unmanned_time();
            break;
        case STANDARD_FUNCTION_QUERY_RADAR_OUITPUT_INFORMATION_SWITCH:
            this->get_radar_output_information_switch();
            break;
        case STANDARD_FUNCTION_MAX:
            this->get_heartbeat_packet();
            break;
        default:
            break;
        }
        sg_start_query_data++;
    }
    if ((s_output_info_switch_flag == OUTPUT_SWTICH_ON) && (sg_start_query_data <= sg_start_query_data_max) && (sg_start_query_data >= CUSTOM_FUNCTION_QUERY_RADAR_OUITPUT_INFORMATION_SWITCH))
    {
        switch (sg_start_query_data)
        {
        case CUSTOM_FUNCTION_QUERY_RADAR_OUITPUT_INFORMATION_SWITCH:
            this->get_radar_output_information_switch();
            break;
        // case CUSTOM_FUNCTION_QUERY_SPATIAL_STATIC_VALUE:
        //     this->get_spatial_static_value();
        //     break;
        // case CUSTOM_FUNCTION_QUERY_SPATIAL_MOTION_AMPLITUDE:
        //     this->get_spatial_motion_amplitude();
        //     break;
        case CUSTOM_FUNCTION_QUERY_PRESENCE_OF_DETECTION_RANGE:
            this->get_presence_of_detection_range();
            break;
        // case CUSTOM_FUNCTION_QUERY_DISTANCE_OF_MOVING_OBJECT:
        //     this->get_distance_of_moving_object();
        //     break;
        // case CUSTOM_FUNCTION_QUERY_TARGET_MOVEMENT_SPEED:
        //     this->get_target_movement_speed();
        //     break;
        case CUSTOM_FUNCTION_QUERY_JUDGMENT_THRESHOLD_EXISTS:
            this->get_judgment_threshold_exists();
            break;
        case CUSTOM_FUNCTION_QUERY_MOTION_AMPLITUDE_TRIGGER_THRESHOLD:
            this->get_motion_amplitude_trigger_threshold();
            break;
        case CUSTOM_FUNCTION_QUERY_PRESENCE_OF_PERCEPTION_BOUNDARY:
            this->get_presence_of_perception_boundary();
            break;
        case CUSTOM_FUNCTION_QUERY_MOTION_TRIGGER_BOUNDARY:
            this->get_motion_trigger_boundary();
            break;
        case CUSTOM_FUNCTION_QUERY_MOTION_TRIGGER_TIME:
            this->get_motion_trigger_time();
            break;
        case CUSTOM_FUNCTION_QUERY_MOVEMENT_TO_REST_TIME:
            this->get_movement_to_rest_time();
            break;
        case CUSTOM_FUNCTION_QUERY_TIME_OF_ENTER_UNMANNED:
            this->get_time_of_enter_unmanned();
            break;
        case CUSTOM_FUNCTION_MAX:
            this->get_heartbeat_packet();
            break;
        default:
            break;
        }
        sg_start_query_data++;
    }
}

void UartReadLineSensor::send_query(uint8_t *query, size_t string_length)
{
    int i;
    for (i = 0; i < string_length; i++)
    {
        write(query[i]);
    }
    show_frame_data(query, i);
}

void UartReadLineSensor::get_heartbeat_packet(void)
{
    uint8_t send_data_len = 10;
    uint8_t send_data[10] = {0x53, 0x59, 0x01, 0x01, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_init_finial(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x05, 0x81, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_product_mode(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x02, 0xA1, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_product_id(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x02, 0xA2, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_hardware_model(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x02, 0xA3, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_firmware_version(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x02, 0xA4, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_protocol_type(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x02, 0xA5, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_human_status(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x80, 0x81, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_scene_mode(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x05, 0x87, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_sensitivity(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x05, 0x88, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_radar_init_status(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x05, 0x81, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_movingTargetDetectionMaxDistance(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x07, 0x81, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_staticTargetDetectionMaxDistance(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x07, 0x84, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_unmanned_time(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x80, 0x8a, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_radar_output_information_switch(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x08, 0x80, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_spatial_static_value(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x08, 0x81, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_spatial_motion_amplitude(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x08, 0x82, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_presence_of_detection_range(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x08, 0x83, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_distance_of_moving_object(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x08, 0x84, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_target_movement_speed(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x08, 0x85, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_judgment_threshold_exists(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x08, 0x88, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_motion_amplitude_trigger_threshold(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x08, 0x89, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_presence_of_perception_boundary(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x08, 0x8a, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_motion_trigger_boundary(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x08, 0x8b, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_motion_trigger_time(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x08, 0x8c, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_movement_to_rest_time(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x08, 0x8d, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::get_time_of_enter_unmanned(void)
{
    unsigned char send_data_len = 10;
    unsigned char send_data[10] = {0x53, 0x59, 0x08, 0x8e, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    send_data[FRAME_DATA_INDEX + 1] = get_frame_crc_sum(send_data, send_data_len);
    this->send_query(send_data, send_data_len);
}

void UartReadLineSensor::R24_split_data_frame(uint8_t value)
{
    switch (sg_recv_data_state)
    {
    case FRAME_IDLE:
        if (FRAME_HEADER1_VALUE == value)
        {
            sg_recv_data_state = FRAME_HEADER2;
        }
        break;
    case FRAME_HEADER2:
        if (FRAME_HEADER2_VALUE == value)
        {
            sg_frame_buf[0] = FRAME_HEADER1_VALUE;
            sg_frame_buf[1] = FRAME_HEADER2_VALUE;
            sg_recv_data_state = FRAME_CTL_WORLD;
        }
        else
        {
            sg_recv_data_state = FRAME_IDLE;
            ESP_LOGD(TAG, "FRAME_IDLE ERROR value:%x", value);
        }
        break;
    case FRAME_CTL_WORLD:
        sg_frame_buf[2] = value;
        sg_recv_data_state = FRAME_CMD_WORLD;
        break;
    case FRAME_CMD_WORLD:
        sg_frame_buf[3] = value;
        sg_recv_data_state = FRAME_DATA_LEN_H;
        break;
    case FRAME_DATA_LEN_H:
        if (value <= 4)
        {
            sg_data_len = value * 256;
            sg_frame_buf[4] = value;
            sg_recv_data_state = FRAME_DATA_LEN_L;
        }
        else
        {
            sg_data_len = 0;
            sg_recv_data_state = FRAME_IDLE;
            ESP_LOGD(TAG, "FRAME_DATA_LEN_H ERROR value:%x", value);
        }
        break;
    case FRAME_DATA_LEN_L:
        sg_data_len += value;
        if (sg_data_len > 32)
        {
            ESP_LOGD(TAG, "len=%d, FRAME_DATA_LEN_L ERROR value:%x", sg_data_len, value);
            sg_data_len = 0;
            sg_recv_data_state = FRAME_IDLE;
        }
        else
        {
            sg_frame_buf[5] = value;
            sg_frame_len = 6;
            sg_recv_data_state = FRAME_DATA_BYTES;
        }
        break;
    case FRAME_DATA_BYTES:
        sg_data_len -= 1;
        sg_frame_buf[sg_frame_len++] = value;
        if (sg_data_len <= 0)
        {
            sg_recv_data_state = FRAME_DATA_CRC;
        }
        break;
    case FRAME_DATA_CRC:
        sg_frame_buf[sg_frame_len++] = value;
        sg_recv_data_state = FRAME_TAIL1;
        break;
    case FRAME_TAIL1:
        if (FRAME_TAIL1_VALUE == value)
        {
            sg_recv_data_state = FRAME_TAIL2;
        }
        else
        {
            sg_recv_data_state = FRAME_IDLE;
            sg_frame_len = 0;
            sg_data_len = 0;
            ESP_LOGD(TAG, "FRAME_TAIL1 ERROR value:%x", value);
        }
        break;
    case FRAME_TAIL2:
        if (FRAME_TAIL2_VALUE == value)
        {
            sg_frame_buf[sg_frame_len++] = FRAME_TAIL1_VALUE;
            sg_frame_buf[sg_frame_len++] = FRAME_TAIL2_VALUE;
            memcpy(sg_frame_prase_buf, sg_frame_buf, sg_frame_len);
            if (get_frame_check_status(sg_frame_prase_buf, sg_frame_len))
            {
                // show_frame_data(sg_frame_prase_buf, sg_frame_len);
                this->R24_parse_data_frame(sg_frame_prase_buf, sg_frame_len);
            }
            else
            {
                ESP_LOGD(TAG, "frame check failer!");
            }
        }
        else
        {
            ESP_LOGD(TAG, "FRAME_TAIL2 ERROR value:%x", value);
        }
        memset(sg_frame_prase_buf, 0, FRAME_BUF_MAX_SIZE);
        memset(sg_frame_buf, 0, FRAME_BUF_MAX_SIZE);
        sg_frame_len = 0;
        sg_data_len = 0;
        sg_recv_data_state = FRAME_IDLE;
        break;
    default:
        sg_recv_data_state = FRAME_IDLE;
    }
}

void UartReadLineSensor::R24_frame_parse_work_status(uint8_t *data)
{
    if (data[FRAME_COMMAND_WORD_INDEX] == 0x01)
    {

        this->inited->publish_state(data[FRAME_DATA_INDEX]);
        ESP_LOGD(TAG, "Report: radar init status 0x%02X", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x07)
    {

        if (id(scene_mode).has_index(data[FRAME_DATA_INDEX] - 1))
        {
            id(scene_mode).publish_state(s_scene_str[data[FRAME_DATA_INDEX] - 1]);
        }
        else
        {
            ESP_LOGD(TAG, "Select has index offset %d Error", data[FRAME_DATA_INDEX]);
        }
        ESP_LOGD(TAG, "Reply: set scene_mode 0x%02X", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x08)
    {
        // 1-3
        id(sensitivity).publish_state(data[FRAME_DATA_INDEX]);
        ESP_LOGD(TAG, "Reply: set sensitivity 0x%02X", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x09)
    {
        // 1-4
        id(custom_mode_settings).publish_state(data[FRAME_DATA_INDEX]);
        ESP_LOGD(TAG, "Reply: set custom_mode_settings 0x%02X", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x81)
    {

        this->inited->publish_state(data[FRAME_DATA_INDEX]);
        ESP_LOGD(TAG, "Reply: get radar init status 0x%02X", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x87)
    {

        if (id(scene_mode).has_index(data[FRAME_DATA_INDEX] - 1))
        {
            id(scene_mode).publish_state(s_scene_str[data[FRAME_DATA_INDEX] - 1]);
        }
        else
        {
            ESP_LOGD(TAG, "Select has index offset %d Error", data[FRAME_DATA_INDEX]);
        }
        ESP_LOGD(TAG, "Reply: get scene_mode 0x%02X", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x88)
    {

        id(sensitivity).publish_state(data[FRAME_DATA_INDEX]);
        ESP_LOGD(TAG, "Reply: get sensitivity 0x%02X", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x89)
    {
        // 1-4
        id(custom_mode_settings).publish_state(data[FRAME_DATA_INDEX]);
        ESP_LOGD(TAG, "Reply: get custom_mode_settings 0x%02X", data[FRAME_DATA_INDEX]);
    }
    else
    {
        ESP_LOGD(TAG, "[%s] No found COMMAND_WORD(%02X) in Frame", __FUNCTION__, data[FRAME_COMMAND_WORD_INDEX]);
    }
}

void UartReadLineSensor::R24_frame_parse_human_information(uint8_t *data)
{
    if (data[FRAME_COMMAND_WORD_INDEX] == 0x01)
    {

        id(someoneExists).publish_state(s_someoneExists_str[data[FRAME_DATA_INDEX]]);
        ESP_LOGD(TAG, "Report: someoneExists %d", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x02)
    {

        if (data[FRAME_DATA_INDEX] < 3 && data[FRAME_DATA_INDEX] >= 0)
        {
            id(motion_status).publish_state(s_motion_status_str[data[FRAME_DATA_INDEX]]);
        }
        ESP_LOGD(TAG, "Report: motion_status %d", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x03)
    {

        if (sg_movementSigns_bak != data[FRAME_DATA_INDEX])
        {
            this->movementSigns->publish_state(data[FRAME_DATA_INDEX]);
            sg_movementSigns_bak = data[FRAME_DATA_INDEX];
        }
        ESP_LOGD(TAG, "Report: movementSigns %d", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x0A)
    {
        // none:0x00  1s:0x01 30s:0x02 1min:0x03 2min:0x04 5min:0x05 10min:0x06 30min:0x07 1hour:0x08
        if (data[FRAME_DATA_INDEX] < 9 && data[FRAME_DATA_INDEX] >= 0)
        {
            id(unmanned_time).publish_state(s_unmanned_time_str[data[FRAME_DATA_INDEX]]);
        }
        ESP_LOGD(TAG, "Reply: set enter unmanned time %d", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x0B)
    {
        // none:0x00  close_to:0x01  far_away:0x02
        if (data[FRAME_DATA_INDEX] < 3 && data[FRAME_DATA_INDEX] >= 0)
        {
            id(keep_away).publish_state(s_keep_away_str[data[FRAME_DATA_INDEX]]);
        }

        ESP_LOGD(TAG, "Report:  moving direction  %d", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x81)
    {

        id(someoneExists).publish_state(s_someoneExists_str[data[FRAME_DATA_INDEX]]);
        ESP_LOGD(TAG, "Reply: get someoneExists %d", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x82)
    {

        if (data[FRAME_DATA_INDEX] < 3 && data[FRAME_DATA_INDEX] >= 0)
        {
            id(motion_status).publish_state(s_motion_status_str[data[FRAME_DATA_INDEX]]);
        }
        ESP_LOGD(TAG, "Reply: get motion_status %d", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x83)
    {

        if (sg_movementSigns_bak != data[FRAME_DATA_INDEX])
        {
            this->movementSigns->publish_state(data[FRAME_DATA_INDEX]);
            sg_movementSigns_bak = data[FRAME_DATA_INDEX];
        }
        ESP_LOGD(TAG, "Reply: get movementSigns %d", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x8A)
    {
        // none:0x00  1s:0x01 30s:0x02 1min:0x03 2min:0x04 5min:0x05 10min:0x06 30min:0x07 1hour:0x08
        if (data[FRAME_DATA_INDEX] < 9 && data[FRAME_DATA_INDEX] >= 0)
        {
            id(unmanned_time).publish_state(s_unmanned_time_str[data[FRAME_DATA_INDEX]]);
        }
        ESP_LOGD(TAG, "Report: get enter unmanned time %d", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x8B)
    {
        // none:0x00  close_to:0x01  far_away:0x02
        if (data[FRAME_DATA_INDEX] < 3 && data[FRAME_DATA_INDEX] >= 0)
        {
            id(keep_away).publish_state(s_keep_away_str[data[FRAME_DATA_INDEX]]);
        }

        ESP_LOGD(TAG, "Reply: get moving direction  %d", data[FRAME_DATA_INDEX]);
    }
    else
    {
        ESP_LOGD(TAG, "[%s] No found COMMAND_WORD(%02X) in Frame", __FUNCTION__, data[FRAME_COMMAND_WORD_INDEX]);
    }
}

void UartReadLineSensor::R24_frame_parse_detection_range(uint8_t *data)
{
    if (data[FRAME_COMMAND_WORD_INDEX] == 0x01)
    {
        id(moving_target_detection_max_distance).publish_state(data[FRAME_DATA_INDEX] * 256 + data[FRAME_DATA_INDEX + 1]);
        ESP_LOGD(TAG, "Reply: set movingTargetDetectionMaxDistance %d", data[FRAME_DATA_INDEX] * 256 + data[FRAME_DATA_INDEX + 1]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x04)
    {
        id(static_target_detection_max_distance).publish_state(data[FRAME_DATA_INDEX] * 256 + data[FRAME_DATA_INDEX + 1]);
        ESP_LOGD(TAG, "Reply: set staticTargetDetectionMaxDistance %d", data[FRAME_DATA_INDEX] * 256 + data[FRAME_DATA_INDEX + 1]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x81)
    {
        id(moving_target_detection_max_distance).publish_state(data[FRAME_DATA_INDEX] * 256 + data[FRAME_DATA_INDEX + 1]);
        ESP_LOGD(TAG, "Reply: get movingTargetDetectionMaxDistance %d", data[FRAME_DATA_INDEX] * 256 + data[FRAME_DATA_INDEX + 1]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x84)
    {
        id(static_target_detection_max_distance).publish_state(data[FRAME_DATA_INDEX] * 256 + data[FRAME_DATA_INDEX + 1]);
        ESP_LOGD(TAG, "Reply: get staticTargetDetectionMaxDistance %d", data[FRAME_DATA_INDEX] * 256 + data[FRAME_DATA_INDEX + 1]);
    }
    else
    {
        ESP_LOGD(TAG, "[%s] No found COMMAND_WORD(%02X) in Frame", __FUNCTION__, data[FRAME_COMMAND_WORD_INDEX]);
    }
}

void UartReadLineSensor::R24_frame_parse_product_Information(uint8_t *data)
{
    uint8_t product_len = 0;
    if (data[FRAME_COMMAND_WORD_INDEX] == 0xA1)
    {
        product_len = data[FRAME_COMMAND_WORD_INDEX + 1] * 256 + data[FRAME_COMMAND_WORD_INDEX + 2];
        if (product_len < PRODUCT_BUF_MAX_SIZE)
        {
            memset(this->c_product_mode, 0, PRODUCT_BUF_MAX_SIZE);
            memcpy(this->c_product_mode, &data[FRAME_DATA_INDEX], product_len);
            ESP_LOGD(TAG, "Reply: get product_mode :%s", this->c_product_mode);
            id(product_mode).publish_state(this->c_product_mode);
        }
        else
        {
            ESP_LOGD(TAG, "Reply: get product_mode length too long!");
        }
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0xA2)
    {
        product_len = data[FRAME_COMMAND_WORD_INDEX + 1] * 256 + data[FRAME_COMMAND_WORD_INDEX + 2];
        if (product_len < PRODUCT_BUF_MAX_SIZE)
        {

            memset(this->c_product_id, 0, PRODUCT_BUF_MAX_SIZE);
            memcpy(this->c_product_id, &data[FRAME_DATA_INDEX], product_len);
            id(product_id).publish_state(this->c_product_id);
            ESP_LOGD(TAG, "Reply: get productId :%s", this->c_product_id);
        }
        else
        {
            ESP_LOGD(TAG, "Reply: get productId length too long!");
        }
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0xA3)
    {
        product_len = data[FRAME_COMMAND_WORD_INDEX + 1] * 256 + data[FRAME_COMMAND_WORD_INDEX + 2];
        if (product_len < PRODUCT_BUF_MAX_SIZE)
        {

            memset(this->c_hardware_model, 0, PRODUCT_BUF_MAX_SIZE);
            memcpy(this->c_hardware_model, &data[FRAME_DATA_INDEX], product_len);
            id(hardware_model).publish_state(this->c_hardware_model);
            ESP_LOGD(TAG, "Reply: get hardware_model :%s", this->c_hardware_model);
        }
        else
        {
            ESP_LOGD(TAG, "Reply: get hardwareModel length too long!");
        }
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0xA4)
    {
        product_len = data[FRAME_COMMAND_WORD_INDEX + 1] * 256 + data[FRAME_COMMAND_WORD_INDEX + 2];
        if (product_len < PRODUCT_BUF_MAX_SIZE)
        {

            memset(this->c_firmware_version, 0, PRODUCT_BUF_MAX_SIZE);
            memcpy(this->c_firmware_version, &data[FRAME_DATA_INDEX], product_len);
            id(firmware_version).publish_state(this->c_firmware_version);
            ESP_LOGD(TAG, "Reply: get firmware_version :%s", this->c_firmware_version);
        }
        else
        {
            ESP_LOGD(TAG, "Reply: get firmwareVersion length too long!");
        }
    }
}

void UartReadLineSensor::R24_frame_parse_open_underlying_information(uint8_t *data)
{
    if (data[FRAME_COMMAND_WORD_INDEX] == 0x00)
    {
        id(output_info_switch).publish_state(data[FRAME_DATA_INDEX]);
        if (data[FRAME_DATA_INDEX])
        {
            s_output_info_switch_flag = OUTPUT_SWTICH_ON;
        }
        else
        {
            s_output_info_switch_flag = OUTPUT_SWTICH_OFF;
        }
        ESP_LOGD(TAG, "Reply: output switch %d", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x01)
    {

        if (sg_spatial_static_value_bak != data[FRAME_DATA_INDEX])
        {
            sg_spatial_static_value_bak = data[FRAME_DATA_INDEX];
            id(custom_spatial_static_value).publish_state(sg_spatial_static_value_bak);
        }
        if (sg_static_distance_bak != data[FRAME_DATA_INDEX + 1])
        {
            sg_static_distance_bak = data[FRAME_DATA_INDEX + 1];
            id(custom_static_distance).publish_state(sg_static_distance_bak * 0.5);
        }
        if (sg_spatial_motion_value_bak != data[FRAME_DATA_INDEX + 2])
        {
            sg_spatial_motion_value_bak = data[FRAME_DATA_INDEX + 2];
            id(custom_spatial_motion_value).publish_state(sg_spatial_motion_value_bak);
        }
        if (sg_motion_distance_bak != data[FRAME_DATA_INDEX + 3])
        {
            sg_motion_distance_bak = data[FRAME_DATA_INDEX + 3];
            id(custom_motion_distance).publish_state(sg_motion_distance_bak * 0.5);
        }
        if (sg_motion_speed_bak != data[FRAME_DATA_INDEX + 4])
        {
            sg_motion_speed_bak = data[FRAME_DATA_INDEX + 4];
            id(custom_motion_speed).publish_state((sg_motion_speed_bak - 10) * 0.5);
        }
        ESP_LOGD(TAG, "Reply: get output info %d  %d  %d  %d", data[FRAME_DATA_INDEX], data[FRAME_DATA_INDEX + 1], data[FRAME_DATA_INDEX + 2], data[FRAME_DATA_INDEX + 3]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x06)
    {
        // none:0x00  close_to:0x01  far_away:0x02
        if (data[FRAME_DATA_INDEX] < 3 && data[FRAME_DATA_INDEX] >= 0)
        {
            id(keep_away).publish_state(s_keep_away_str[data[FRAME_DATA_INDEX]]);
        }
        ESP_LOGD(TAG, "Report:  moving direction  %d", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x07)
    {

        if (sg_movementSigns_bak != data[FRAME_DATA_INDEX])
        {
            this->movementSigns->publish_state(data[FRAME_DATA_INDEX]);
            sg_movementSigns_bak = data[FRAME_DATA_INDEX];
        }
        ESP_LOGD(TAG, "Report: get movementSigns %d", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x08)
    {
        id(custom_judgment_threshold_exists).publish_state(data[FRAME_DATA_INDEX]);
        ESP_LOGD(TAG, "Reply: set judgment threshold exists %d", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x09)
    {
        id(custom_motion_amplitude_trigger_threshold).publish_state(data[FRAME_DATA_INDEX]);
        ESP_LOGD(TAG, "Reply: set motion amplitude trigger threshold %d", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x0a)
    {
        if (id(custom_presence_of_perception_boundary).has_index(data[FRAME_DATA_INDEX] - 1))
        {
            id(custom_presence_of_perception_boundary).publish_state(s_presence_of_perception_boundary_str[data[FRAME_DATA_INDEX] - 1]);
        }
        ESP_LOGD(TAG, "Reply: set presence awareness boundary %d", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x0b)
    {
        if (id(custom_motion_trigger_boundary).has_index(data[FRAME_DATA_INDEX] - 1))
        {
            id(custom_motion_trigger_boundary).publish_state(s_motion_trig_boundary_str[data[FRAME_DATA_INDEX] - 1]);
        }
        ESP_LOGD(TAG, "Reply: set motion trigger boundary %d", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x0c)
    {
        uint32_t motion_trigger_time = (uint32_t)(data[FRAME_DATA_INDEX] << 24) + (uint32_t)(data[FRAME_DATA_INDEX + 1] << 16) + (uint32_t)(data[FRAME_DATA_INDEX + 2] << 8) + data[FRAME_DATA_INDEX + 3];
        if (sg_motion_trigger_time_bak != motion_trigger_time)
        {
            sg_motion_trigger_time_bak = motion_trigger_time;
            id(custom_motion_trigger_time).publish_state(motion_trigger_time);
        }
        ESP_LOGD(TAG, "Reply: set motion trigger time %u", motion_trigger_time);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x0d)
    {
        uint32_t move_to_rest_time = (uint32_t)(data[FRAME_DATA_INDEX] << 24) + (uint32_t)(data[FRAME_DATA_INDEX + 1] << 16) + (uint32_t)(data[FRAME_DATA_INDEX + 2] << 8) + data[FRAME_DATA_INDEX + 3];
        if (sg_move_to_rest_time_bak != move_to_rest_time)
        {
            id(custom_movement_to_rest_time).publish_state(move_to_rest_time);
            sg_move_to_rest_time_bak = move_to_rest_time;
        }
        ESP_LOGD(TAG, "Reply: set movement to rest time %u", move_to_rest_time);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x0e)
    {
        uint32_t enter_unmanned_time = (uint32_t)(data[FRAME_DATA_INDEX] << 24) + (uint32_t)(data[FRAME_DATA_INDEX + 1] << 16) + (uint32_t)(data[FRAME_DATA_INDEX + 2] << 8) + data[FRAME_DATA_INDEX + 3];
        if (sg_enter_unmanned_time_bak != enter_unmanned_time)
        {
            id(custom_time_of_enter_unmanned).publish_state(enter_unmanned_time);
            sg_enter_unmanned_time_bak = enter_unmanned_time;
        }
        ESP_LOGD(TAG, "Reply: set Time of entering unmanned state %u", enter_unmanned_time);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x80)
    {
        if (data[FRAME_DATA_INDEX])
        {
            s_output_info_switch_flag = OUTPUT_SWTICH_ON;
        }
        else
        {
            s_output_info_switch_flag = OUTPUT_SWTICH_OFF;
        }
        id(output_info_switch).publish_state(data[FRAME_DATA_INDEX]);
        ESP_LOGD(TAG, "Reply: get output switch %d", data[FRAME_DATA_INDEX]);

    } else if (data[FRAME_COMMAND_WORD_INDEX] == 0x81) {
        if (sg_spatial_static_value_bak != data[FRAME_DATA_INDEX]) {
            sg_spatial_static_value_bak = data[FRAME_DATA_INDEX];
            id(custom_spatial_static_value).publish_state(sg_spatial_static_value_bak);
        }
        ESP_LOGD(TAG, "Reply: get spatial static value %d", data[FRAME_DATA_INDEX]);

    } else if (data[FRAME_COMMAND_WORD_INDEX] == 0x82) {
        if (sg_spatial_motion_value_bak != data[FRAME_DATA_INDEX]) {
            sg_spatial_motion_value_bak = data[FRAME_DATA_INDEX];
            id(custom_spatial_motion_value).publish_state(sg_spatial_motion_value_bak);
        }
        ESP_LOGD(TAG, "Reply: get spatial motion amplitude %d", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x83)
    {
        id(custom_presence_of_detection).publish_state(s_presence_of_detection_range_str[data[FRAME_DATA_INDEX]]);
        ESP_LOGD(TAG, "Reply: get Presence of detection range %d", data[FRAME_DATA_INDEX]);

    } else if (data[FRAME_COMMAND_WORD_INDEX] == 0x84) { 
        // if (sg_motion_distance_bak != data[FRAME_DATA_INDEX]) {
            sg_motion_distance_bak = data[FRAME_DATA_INDEX];
            id(custom_motion_distance).publish_state(sg_motion_distance_bak * 0.5);
        // }
        ESP_LOGD(TAG, "Report: get distance of moving object %lf", data[FRAME_DATA_INDEX]*0.5);

    } else if (data[FRAME_COMMAND_WORD_INDEX] == 0x85) {  
        if (sg_motion_speed_bak != data[FRAME_DATA_INDEX]) {
            sg_motion_speed_bak = data[FRAME_DATA_INDEX];
            id(custom_motion_speed).publish_state((sg_motion_speed_bak - 10) * 0.5);
        }
        ESP_LOGD(TAG, "Reply: get target movement speed %d", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x86)
    {
        ESP_LOGD(TAG, "Reply: get keep_away %d", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x87)
    {
        if (sg_movementSigns_bak != data[FRAME_DATA_INDEX])
        {
            this->movementSigns->publish_state(data[FRAME_DATA_INDEX]);
            sg_movementSigns_bak = data[FRAME_DATA_INDEX];
        }
        ESP_LOGD(TAG, "Reply: get movementSigns %d", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x88)
    {
        id(custom_judgment_threshold_exists).publish_state(data[FRAME_DATA_INDEX]);
        ESP_LOGD(TAG, "Reply: get judgment threshold exists %d", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x89)
    {
        id(custom_motion_amplitude_trigger_threshold).publish_state(data[FRAME_DATA_INDEX]);
        ESP_LOGD(TAG, "Reply: get motion amplitude trigger threshold setting %d", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x8a)
    {
        if (id(custom_presence_of_perception_boundary).has_index(data[FRAME_DATA_INDEX] - 1))
        {
            id(custom_presence_of_perception_boundary).publish_state(s_presence_of_perception_boundary_str[data[FRAME_DATA_INDEX] - 1]);
        }
        ESP_LOGD(TAG, "Reply: get presence awareness boundary %d", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x8b)
    {
        if (id(custom_motion_trigger_boundary).has_index(data[FRAME_DATA_INDEX] - 1))
        {
            id(custom_motion_trigger_boundary).publish_state(s_motion_trig_boundary_str[data[FRAME_DATA_INDEX] - 1]);
        }
        ESP_LOGD(TAG, "Reply: get motion trigger boundary %d", data[FRAME_DATA_INDEX]);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x8c)
    {
        uint32_t motion_trigger_time = (uint32_t)(data[FRAME_DATA_INDEX] << 24) + (uint32_t)(data[FRAME_DATA_INDEX + 1] << 16) + (uint32_t)(data[FRAME_DATA_INDEX + 2] << 8) + data[FRAME_DATA_INDEX + 3];
        if (sg_motion_trigger_time_bak != motion_trigger_time)
        {
            id(custom_motion_trigger_time).publish_state(motion_trigger_time);
            sg_motion_trigger_time_bak = motion_trigger_time;
        }
        ESP_LOGD(TAG, "Reply: get motion trigger time %u", motion_trigger_time);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x8d)
    {
        uint32_t move_to_rest_time = (uint32_t)(data[FRAME_DATA_INDEX] << 24) + (uint32_t)(data[FRAME_DATA_INDEX + 1] << 16) + (uint32_t)(data[FRAME_DATA_INDEX + 2] << 8) + data[FRAME_DATA_INDEX + 3];
        if (sg_move_to_rest_time_bak != move_to_rest_time)
        {
            id(custom_movement_to_rest_time).publish_state(move_to_rest_time);
            sg_move_to_rest_time_bak = move_to_rest_time;
        }
        ESP_LOGD(TAG, "Reply: get movement to rest time %u", move_to_rest_time);
    }
    else if (data[FRAME_COMMAND_WORD_INDEX] == 0x8e)
    {
        uint32_t enter_unmanned_time = (uint32_t)(data[FRAME_DATA_INDEX] << 24) + (uint32_t)(data[FRAME_DATA_INDEX + 1] << 16) + (uint32_t)(data[FRAME_DATA_INDEX + 2] << 8) + data[FRAME_DATA_INDEX + 3];
        if (sg_enter_unmanned_time_bak != enter_unmanned_time)
        {
            id(custom_time_of_enter_unmanned).publish_state(enter_unmanned_time);
            sg_enter_unmanned_time_bak = enter_unmanned_time;
        }
        ESP_LOGD(TAG, "Reply: get Time of entering unmanned state %u", enter_unmanned_time);
    }
}

void UartReadLineSensor::R24_parse_data_frame(uint8_t *data, uint8_t len)
{
    switch (data[FRAME_CONTROL_WORD_INDEX])
    {

    case 0x01:
    {
        if (data[FRAME_COMMAND_WORD_INDEX] == 0x01)
        {
            sg_heartbeat_flag = 1;
            ESP_LOGD(TAG, "Reply: query Heartbeat packet");
        }
        else if (data[FRAME_COMMAND_WORD_INDEX] == 0x02)
        {
            ESP_LOGD(TAG, "Reply: query reset packet");
        }
    }
    break;
    case 0x02:
    {
        this->R24_frame_parse_product_Information(data);
    }
    break;
    case 0x05:
    {
        this->R24_frame_parse_work_status(data);
    }
    break;
    case 0x07:
    {
        this->R24_frame_parse_detection_range(data);
    }
    break;
    case 0x08:
    {
        this->R24_frame_parse_open_underlying_information(data);
    }
    break;
    case 0x80:
    {
        this->R24_frame_parse_human_information(data);
    }
    break;
    default:
        ESP_LOGD(TAG, "control world:0x%02X not found", data[FRAME_CONTROL_WORD_INDEX]);
        break;
    }
}
