/**
 * @file crossfire.h
 * @author Timo Lehnertz
 * @brief 
 * @version 0.1
 * @date 2022-01-01
 * 
 * @copyright Copyright (c) 2022
 * 
 * 
 * Crossfire wire protocol:
 *  https://github.com/ExpressLRS/ExpressLRS/wiki/CRSF-Protocol
 *  [dest] [len] [type] [payload] [crc8]
 *  dest: destination of frame(fc,receiver,transmitter,radio)
 *  len: size of bytes that follow (payload size + 2)
 *  type: type of frame
 * crc8: CRC8 using poly 0xD5, includes all bytes from type (buffer[2]) to end of payload.
 */
#ifndef CRSF
#define CRSF
#include <Arduino.h>

// #define CRSF_DEBUG //prints every received frame

typedef uint32_t timeUs_t;

typedef int32_t timeDelta_t;

#define CRSF_ADDRESS_CRSF_TRANSMITTER   0xEE
#define CRSF_ADDRESS_RADIO_TRANSMITTER  0xEA
#define CRSF_ADDRESS_FLIGHT_CONTROLLER  0xC8
#define CRSF_ADDRESS_CRSF_RECEIVER      0xEC

#define CRSF_FRAME_LENGTH_ADDRESS 1
#define CRSF_FRAME_LENGTH_FRAMELENGTH 1

#define CRSF_FRAME_LENGTH_TYPE_CRC 2 // length of TYPE and CRC fields combined

#define CRSF_TIME_NEEDED_PER_FRAME_US   1100 // 700 ms + 400 ms for potential ad-hoc request
#define CRSF_TIME_BETWEEN_FRAMES_US     6667 // At fastest, frames are sent by the transmitter every 6.667 milliseconds, 150 Hz

#define CRSF_FRAME_SIZE_MAX 64 // 62 bytes frame plus 2 bytes frame header(<length><type>)
#define CRSF_PAYLOAD_SIZE_MAX CRSF_FRAME_SIZE_MAX - 6

#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED           0x16
#define CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED    0x17
#define CRSF_FRAMETYPE_DEVICE_PING                  0x28
#define CRSF_FRAMETYPE_DEVICE_INFO                  0x29
#define CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY     0x2B
#define CRSF_FRAMETYPE_PARAMETER_READ               0x2C
#define CRSF_FRAMETYPE_PARAMETER_WRITE              0x2D
#define CRSF_FRAMETYPE_COMMAND                      0x32

#define MAX_CHANEL_COUNT 12
#define CHANEL_BITS 11

#define CRSF_SUBSET_RC_RES_CONF_11B                 1
#define CRSF_SUBSET_RC_RES_BITS_11B                 11
#define CRSF_SUBSET_RC_RES_MASK_11B                 0x07FF

#define CRSF_SUBSET_RC_STARTING_CHANNEL_BITS 5

#define crc8_dvb_s2(crc, a) crc8_calc(crc, a, 0xD5)

#define CRSF_BAUDRATE 420000

#define CRSF_SYNC_BYTE 0xC8

#define CRSF_FRAMETYPE_FLIGHT_MODE      0x21
#define CRSF_FRAMETYPE_BATTERY_SENSOR   0x08
#define CRSF_FRAMETYPE_ATTITUDE         0x1E
#define CRSF_FRAMETYPE_GPS              0x02

#define CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE 8
#define CRSF_FRAME_FLIGHT_MODE_PAYLOAD_SIZE 5 //assuming 4 chars
#define CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE 6
#define CRSF_FRAME_GPS_PAYLOAD_SIZE 15

#define CRSF_FAILSAFE_TIMEOUT_US 1000000 //1sek

static inline timeDelta_t cmpTimeUs(timeUs_t a, timeUs_t b) { return (timeDelta_t)(a - b); }

struct CRSF_TxChanels_Labels {
    uint16_t roll;
    uint16_t pitch;
    uint16_t throttle;
    uint16_t yaw;
    uint16_t aux1;
    uint16_t aux2;
    uint16_t aux3;
    uint16_t aux4;
    uint16_t aux5;
    uint16_t aux6;
    uint16_t aux7;
    uint16_t aux8;
};

struct CRSF_TxChanels_Converted {
    float roll;
    float pitch;
    float throttle;
    float yaw;
    float aux1;
    float aux2;
    float aux3;
    float aux4;
    float aux5;
    float aux6;
    float aux7;
    float aux8;
};

union CRSF_TxChanels {
    uint16_t chanels[MAX_CHANEL_COUNT];
    CRSF_TxChanels_Labels labels;
};

struct CRSF_FrameDef_t {
    uint8_t deviceAddress;
    uint8_t frameLength;
    uint8_t type;
    uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + 1]; // +1 for CRC at end of payload
};

union CRSF_Frame_t {
    uint8_t bytes[CRSF_FRAME_SIZE_MAX];
    CRSF_FrameDef_t frame;
};

class Crossfire {
public:

    Crossfire(HardwareSerial *h) : uart(h) {}

    bool firstFrameReceived = false;
    CRSF_TxChanels chanels; //always set to the latest received chanels

    // states
    void begin();
    void handle();
    void end();
    void encode(char c);

    void beginTransmition();
    void endTransmition();

    CRSF_TxChanels_Converted getChanelsCoverted();
    CRSF_TxChanels getChanels();

    /**
     * Failsafe
     **/
    bool isFailsafe();

    bool isRcConnected() { return rcConnected; }
    uint64_t timeSinceRcConnect() { return micros() - rcConnectedTime; }

    /**
     * telemetry updates
     **/
    void updateTelemetryFlightMode(const char* mode); //char array of length 4
    void updateTelemetryAttitude(float roll, float pitch, float yaw); 
    void updateTelemetryGPS(float lat, float lng, float groundSpeed, float heading, float altitude, int satelitesInUse);
    void updateTelemetryBattery(float vBat, float batCurrent, uint32_t mahDraw, int remainingPercent);
    void updateRcChannels(CRSF_TxChanels_Converted channels);
    
    double map(double x, double in_min, double in_max, double out_min, double out_max);
private:
    HardwareSerial* uart;
    CRSF_Frame_t crsfFrame;
    byte payloadLength = 0;
    byte crsfFramePosition = 0;
    timeUs_t crsfFrameStartAtUs = 0;

    uint8_t crsfFrameCRC(CRSF_Frame_t & frame);
    // uint8_t crsfFrameCRCTelemetry(CRSF_Frame_t & frame, int len);
    uint8_t crc8_calc(uint8_t crc, unsigned char a, uint8_t poly);

    uint16_t txRate = 500; //hz
    timeUs_t lastTxRcFrame = 0;
    bool isTransmitting = false;
    CRSF_Frame_t txCrsfFrame;

    void handleCrsfFrame(CRSF_Frame_t&, int);

    CRSF_TxChanels frameToChanels(CRSF_Frame_t& frame, int payloadLength);


    void writeU32BigEndian(uint8_t *dst, uint32_t val);
    void writeU16BigEndian(uint8_t *dst, uint16_t val);

    void printFrame(CRSF_Frame_t &frame);

    CRSF_TxChanels getChannelsFromConverted(CRSF_TxChanels_Converted channelsConverted);

    /**
     * Failsafe
     * has to be set to cut on rx
     **/
    timeUs_t lastRcFrame = 0;

    bool rcConnected = false;
    uint64_t rcConnectedTime = 0;

    /**
     * Telemetry
     **/
    // 0 -> no telemetry
    // 1 telemetry is send between every received frame
    // 10 -> telemetry is send after every tenth received frame
    int telemFrequency = 1;
    int telemInc = 0;

    bool useBatteryTelem = true;
    bool useAttitudeTelem = true;
    bool useGPSTelem = true;
    bool useFlightMode = true;

    bool telemBatDone = false;
    bool telemAttitudeDone = false;
    bool telemGPSDone = false;
    bool telemFlightModeDone = false;

    bool fmQued = false;
    const char* fm = "-";

    /**
     * Battery stats
     **/
    uint16_t batAvgCellVoltage = 0;     // Voltage ( mV * 100 )
    uint16_t batCurrent = 0;            // Current ( mA * 100 )
    uint32_t mahDraw = 0;               // Fuel ( drawn mAh )
    uint8_t  batRemainingPercentage = 0;// Battery remaining ( percent )

    /**
     * Attitude stats
     **/
    int16_t pitch = 0;                  // Pitch angle ( rad / 10000 )
    int16_t roll = 0;                   // Roll angle ( rad / 10000 )
    int16_t yaw = 0;                    // Yaw angle ( rad / 10000 )
 
    /**
     * GPS stats
     **/
    int32_t gpsLat = 508074330;         // Latitude ( degree / 10`000`000 )
    int32_t gpsLng = 68317330;          // Longitude (degree / 10`000`000 )
    uint16_t groundSpeed = 0;           // Groundspeed ( km/h / 10 )
    int16_t gpsHeading = 0;            // GPS heading ( degree / 100 )
    uint16_t altitude = 1000;           // Altitude ( meter + 1000 ­)
    uint8_t satelitesInUse = 0;         // Satellites in use ( counter )

    /**
     * Flight mode
     **/
    char flightMode[5] = "(-;>";   //exactly 4 chars

    void sendFrame(CRSF_Frame_t &frame);

    /**
     * Gets called every time after a crsf frame has been received
     * schedules the battery, gps, attitude and flight mode telemetry functions according to telemFrequency
     **/
    void handleTelemetry();

    void sendFlightMode();  //write flight mode to uart
    void sendBatteryInfo(); //write battery telemetry to uart
    void sendAttitude();    //write battery telemetry to uart
    void sendGpsFrame();    //write battery telemetry to uart
};
#endif