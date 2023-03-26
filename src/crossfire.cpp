/**
 * @file crossfire.cpp
 * @author Timo Lehnertz
 * @brief 
 * @version 0.1
 * @date 2022-01-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <Arduino.h>
#include "crossfire.h"

void Crossfire::encode(char c) {
    // Serial.write(c);
    static uint8_t crsfFramePosition = 0;
    timeUs_t currentTimeUs = micros();

    if (cmpTimeUs(currentTimeUs, crsfFrameStartAtUs) > CRSF_TIME_NEEDED_PER_FRAME_US) {
        // We've received a character after max time needed to complete a frame,
        // so this must be the start of a new frame.
        crsfFramePosition = 0;
    }

    if (crsfFramePosition == 0) {
        crsfFrameStartAtUs = currentTimeUs;
    }
    // assume frame is 5 bytes long until we have received the frame length
    // full frame length includes the length of the address and framelength fields
    const int fullFrameLength = crsfFramePosition < 3 ? 5 : crsfFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH;
    if (crsfFramePosition >= fullFrameLength) {
        return;
    }
    crsfFrame.bytes[crsfFramePosition++] = (uint8_t) c;
    if (crsfFramePosition >= fullFrameLength) {
        const uint8_t crc = crsfFrameCRC(crsfFrame);
        if (crc != crsfFrame.bytes[fullFrameLength - 1]) {
            // Serial.println("CRSF crc missmatch! "); //Serial.print(crc); Serial.print(", got: "); Serial.println(crsfFrame.bytes[fullFrameLength - 1]);
            return;
        }
        switch (crsfFrame.frame.type) {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
        case CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED:
            if (crsfFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
                handleCrsfFrame(crsfFrame, crsfFramePosition - 3);
            }
            break;
        }
        crsfFramePosition = 0;
    }
}

void Crossfire::handle() {
    if(micros() - lastRcFrame > 3 * 1000000) { // 3 seconds timeout
        rcConnected = false;
    }
    // then send
    if(isTransmitting && micros() - lastTxRcFrame > 1000000 / txRate) {
        sendFrame(txCrsfFrame);
        lastTxRcFrame = micros();
    }
}

CRSF_TxChanels Crossfire::getChannelsFromConverted(CRSF_TxChanels_Converted channelsConverted) {
    CRSF_TxChanels channels;
    for (size_t i = 0; i < sizeof(CRSF_TxChanels_Converted) / sizeof(float); i++) {
        float channelValue = *(((float*) &channelsConverted) + i);
        if(i == 2) { // throttle
            channels.chanels[i] = map(channelValue, 0, 1, 172, 1809);
        } else { //normal
            channels.chanels[i] = map(channelValue, -1, 1, 172, 1809);
        }
    }
}

void Crossfire::updateRcChannels(CRSF_TxChanels_Converted channelsConverted) {
    txCrsfFrame.frame.deviceAddress = CRSF_ADDRESS_CRSF_TRANSMITTER;
    txCrsfFrame.frame.type = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    txCrsfFrame.frame.frameLength = 24; // (16 channels * 11 bits) / 8 + 2

    CRSF_TxChanels channels = getChannelsFromConverted(channelsConverted);
    // thanks openai
    // txCrsfFrame.frame.payload
    for (int i = 0; i < MAX_CHANEL_COUNT; i++) {
        // Copy the 11 least significant bits of txChannels.chanels[i] to frame.payload[i*11:i*11+11]
        for (int j = 0; j < 11; j++) {
            txCrsfFrame.frame.payload[(i*11 + j) / 8] |= (channels.chanels[i] >> j) & 1 << j;
        }
    }
}

bool Crossfire::isFailsafe() {
    return micros() - lastRcFrame > CRSF_FAILSAFE_TIMEOUT_US;
}

void Crossfire::handleCrsfFrame(CRSF_Frame_t& frame, int payloadLength) {
    lastRcFrame = micros();
    firstFrameReceived = true;
    chanels = frameToChanels(frame, payloadLength);

    #ifdef CRSF_DEBUG //print out each frame
    Serial.print("received frame of type: "); Serial.print(frame.frame.type);
    Serial.print(", length: "); Serial.print(frame.frame.frameLength);
    Serial.print(" chanels: ");
    for (int i = 0; i < MAX_CHANEL_COUNT; i++) {
        Serial.print(chanels.chanels[i]); Serial.print(",");
    }
    Serial.println();
    #endif

    /**
     * Telemetry
     **/
    // handleTelemetry();
    if(!rcConnected) {
        rcConnected = true;
        rcConnectedTime = micros();
    }
}

void Crossfire::handleTelemetry() {
    if(telemFrequency <= 0) return;
    telemInc++;
    if(telemInc >= telemFrequency) {
        if(!telemBatDone && useBatteryTelem) {
            sendBatteryInfo();
            telemBatDone = true;
        } else if(!telemAttitudeDone && useAttitudeTelem) {
            sendAttitude();
            telemAttitudeDone = true;
        } else if(!telemGPSDone && useGPSTelem) {
            sendGpsFrame();
            telemGPSDone = true;
        } else if(!telemFlightModeDone && useFlightMode) {
            sendFlightMode();
            telemFlightModeDone = true;
        } else {
            telemInc = 0;
            telemBatDone = false;
            telemAttitudeDone = false;
            telemGPSDone = false;
            telemFlightModeDone = false;
        }
    }
}

CRSF_TxChanels Crossfire::frameToChanels(CRSF_Frame_t& frame, int payloadLength) {
    CRSF_TxChanels chanels;
    // uint8_t numOfChannels = ((frame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC) * 8 - CRSF_SUBSET_RC_STARTING_CHANNEL_BITS) / CHANEL_BITS;
    uint8_t numOfChannels = 12;
    uint8_t bitsMerged = 0;
    uint32_t readValue = 0;
    uint8_t readByteIndex = 0;
    for (int chanel = 0; chanel < numOfChannels; chanel++) {
        while (bitsMerged < CRSF_SUBSET_RC_RES_BITS_11B) {
            uint8_t readByte = frame.frame.payload[readByteIndex++];
            readValue |= ((uint32_t) readByte) << bitsMerged;
            bitsMerged += 8;
        }
        chanels.chanels[chanel] = readValue & CRSF_SUBSET_RC_RES_MASK_11B;
        readValue >>= CRSF_SUBSET_RC_RES_BITS_11B;
        bitsMerged -= CRSF_SUBSET_RC_RES_BITS_11B;
    }
    return chanels;
}

uint8_t Crossfire::crsfFrameCRC(CRSF_Frame_t &frame) {
     // CRC includes type and payload
    uint8_t crc = crc8_dvb_s2(0, frame.frame.type);
    for (int i = 0; i < frame.frame.frameLength - 2; i++) {
        crc = crc8_dvb_s2(crc, frame.frame.payload[i]);
    }
    return crc;
}

void Crossfire::begin() {
    Serial.println("starting crossfire");
    uart->begin(CRSF_BAUDRATE);
    // uart->addMemoryForRead(new char[100], 100);
}

void Crossfire::end() {
    uart->end();
}

void Crossfire::beginTransmition() {
    isTransmitting = true;
}

void Crossfire::endTransmition() {
    isTransmitting = false;
}

uint8_t Crossfire::crc8_calc(uint8_t crc, unsigned char a, uint8_t poly) {
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ poly;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

/**
 * Telemetry
 * max 4 chars
 **/
void Crossfire::updateTelemetryFlightMode(const char* fm) {
    size_t len = strlen(fm);
    for (size_t i = 0; i < 4; i++) {
        flightMode[i] = i < len ? fm[i] : ' ';
    }
}

void Crossfire::updateTelemetryAttitude(float roll, float pitch, float yaw) {
    this->roll  =   roll * 10000;
    this->pitch =   pitch * 10000;
    this->yaw   =   yaw * 10000;
}

void Crossfire::updateTelemetryGPS(float lat, float lng, float groundSpeed, float headingDeg, float altitude, int satelitesInUse) {
    this->gpsLat = lat * 10000000;
    this->gpsLng = lng * 10000000;
    this->groundSpeed = groundSpeed * 10;
    this->gpsHeading = headingDeg * 100;
    this->altitude = altitude + 1000;
    this->satelitesInUse = satelitesInUse;
}

void Crossfire::updateTelemetryBattery(float vBat, float batCurrent, uint32_t mahDraw, int remainingPercent) {
    this->batAvgCellVoltage = round(vBat * 10);
    this->batCurrent = round(batCurrent * 10);
    this->mahDraw = mahDraw;
    this->batRemainingPercentage = remainingPercent;
}

/*
    0x21 Flight mode text based
    Payload:
    char[]      Flight mode ( Null terminated string )
*/
void Crossfire::sendFlightMode() {
    CRSF_Frame_t frame;
    frame.frame.deviceAddress = CRSF_SYNC_BYTE;
    frame.frame.type = CRSF_FRAMETYPE_FLIGHT_MODE;
    frame.frame.frameLength = CRSF_FRAME_FLIGHT_MODE_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC;
    frame.frame.payload[0] = flightMode[0];
    frame.frame.payload[1] = flightMode[1];
    frame.frame.payload[2] = flightMode[2];
    frame.frame.payload[3] = flightMode[3];
    frame.frame.payload[4] = 0; //null terminated
    frame.frame.payload[5] = crsfFrameCRC(frame);
    sendFrame(frame);
}

/*
    0x08 Battery sensor
    Payload:
    uint16_t    Voltage ( mV * 100 )
    uint16_t    Current ( mA * 100 )
    uint24_t    Fuel ( drawn mAh )
    uint8_t     Battery remaining ( percent )
*/
void Crossfire::sendBatteryInfo() {
    // float pinVolt = analogRead(16) / 1024.0f * 3.3f; //actual volts at pin
    // float batVolt = pinVolt * 17.972f; //actual volts at pin

    // batAvgCellVoltage = batVolt * 10.0f; //crsf scaling


    // batAvgCellVoltage = analogRead(16) * 10.0;
    CRSF_Frame_t frame;
    frame.frame.deviceAddress = CRSF_SYNC_BYTE;
    frame.frame.type = CRSF_FRAMETYPE_BATTERY_SENSOR;
    frame.frame.frameLength = CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC;
    //voltage
    frame.frame.payload[0] = batAvgCellVoltage >> 8;
    frame.frame.payload[1] = (uint8_t) batAvgCellVoltage;
    //current
    frame.frame.payload[2] = batCurrent >> 8;
    frame.frame.payload[3] = (uint8_t) batCurrent;
    //fuel
    frame.frame.payload[4] = mahDraw >> 16;
    frame.frame.payload[5] = mahDraw >> 8;
    frame.frame.payload[6] = (uint8_t) mahDraw;
    //remaining percent
    frame.frame.payload[7] = batRemainingPercentage;
    // frame.frame.payload[8] = crsfFrameCRCTelemetry(frame, CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE);
    frame.frame.payload[8] = crsfFrameCRC(frame);
    sendFrame(frame);
}

/*
    0x1E Attitude
    Payload:
    int16_t     Pitch angle ( rad / 10000 )
    int16_t     Roll angle ( rad / 10000 )
    int16_t     Yaw angle ( rad / 10000 )
*/
void Crossfire::sendAttitude() {
    CRSF_Frame_t frame;
    frame.frame.deviceAddress = CRSF_SYNC_BYTE;
    frame.frame.type = CRSF_FRAMETYPE_ATTITUDE;
    frame.frame.frameLength = CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC;
    //pitch
    writeU16BigEndian(&frame.frame.payload[0], pitch);
    //roll
    writeU16BigEndian(&frame.frame.payload[2], roll);
    //yaw
    writeU16BigEndian(&frame.frame.payload[4], yaw);
    frame.frame.payload[6] = crsfFrameCRC(frame);
    sendFrame(frame);
}

/*
    0x02 GPS
    Payload:
    int32_t     Latitude ( degree / 10`000`000 )
    int32_t     Longitude (degree / 10`000`000 )
    uint16_t    Groundspeed ( km/h / 10 )
    uint16_t    GPS heading ( degree / 100 )
    uint16      Altitude ( meter ­1000m offset )
    uint8_t     Satellites in use ( counter )
*/
void Crossfire::sendGpsFrame() {
    CRSF_Frame_t frame;
    frame.frame.deviceAddress = CRSF_SYNC_BYTE;
    frame.frame.type = CRSF_FRAMETYPE_GPS;
    frame.frame.frameLength = CRSF_FRAME_GPS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC;//1 CRC 1 type
    //lat lng
    writeU32BigEndian(&frame.frame.payload[0], gpsLat);
    writeU32BigEndian(&frame.frame.payload[4], gpsLng);
    //groundspeed
    writeU16BigEndian(&frame.frame.payload[8], groundSpeed);
    writeU16BigEndian(&frame.frame.payload[10], gpsHeading * 10);  // gps heading is degrees * 10
    //altitude
    writeU16BigEndian(&frame.frame.payload[12], altitude);
    frame.frame.payload[14] = satelitesInUse;

    //CRC
    frame.frame.payload[15] = crsfFrameCRC(frame);
    sendFrame(frame);
}

void Crossfire::writeU32BigEndian(uint8_t *dst, uint32_t val) {
    dst[0] = val >> 24;
    dst[1] = val >> 16;
    dst[2] = val >> 8;
    dst[3] = (uint8_t) val;
}

void Crossfire::writeU16BigEndian(uint8_t *dst, uint16_t val) {
    dst[0] = val >> 8;
    dst[1] = (uint8_t) val;
}

/*
    CRSF frame has the structure:
    <Device address> <Frame length> <Type> <Payload> <CRC>
    Device address: (uint8_t)
    Frame length:   length in  bytes including Type (uint8_t)
    Type:           (uint8_t)
    CRC:            (uint8_t), crc of <Type> and <Payload>
*/
void Crossfire::sendFrame(CRSF_Frame_t &frame) {
    const int fullFrameLength = frame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH;
    uart->write(frame.bytes, fullFrameLength);
}

void Crossfire::printFrame(CRSF_Frame_t &frame) {
    Serial.println("Frame: --");
    Serial.print(frame.frame.deviceAddress); Serial.println(" addr");
    Serial.print(frame.frame.frameLength); Serial.println(" len");
    Serial.print(frame.frame.type); Serial.println(" t");
    for (int i = 0; i < frame.frame.frameLength - 1; i++) {
        Serial.print(", "); Serial.print(frame.frame.payload[i]);
    }
    Serial.println("<-crc");
    int crc = crsfFrameCRC(frame);
    Serial.print(crc); Serial.println(" calculated crc");
    if(crc != frame.frame.payload[frame.frame.frameLength - 2]) {
        // Serial.println("CRC match");
    // } else {
        #ifdef CRSF_DEBUG
        Serial.println("CRC MISSMATCH!");
        #endif
    }
    Serial.println("------");
}

CRSF_TxChanels Crossfire::getChanels() {
    return chanels;
}

CRSF_TxChanels_Converted Crossfire::getChanelsCoverted() {
    CRSF_TxChanels_Converted conv;
    if(!firstFrameReceived) { //default values
        conv.roll     = 0;
        conv.pitch    = 0;
        conv.throttle = 0;
        conv.yaw      = 0;
        conv.aux1     = 0;
        conv.aux2     = 0;
        conv.aux3     = 0;
        conv.aux4     = 0;
        conv.aux5     = 0;
        conv.aux6     = 0;
        conv.aux7     = 0;
        conv.aux8     = 0;
    } else {
        conv.roll     = map(chanels.labels.roll, 172.0, 1809.0, -1.0, 1.0);
        conv.pitch    = map(chanels.labels.pitch, 172.0, 1809.0, -1.0, 1.0);
        conv.throttle = map(chanels.labels.throttle, 172.0, 1809.0, 0.0, 1.0);
        conv.yaw      = map(chanels.labels.yaw,  172.0, 1809.0, -1.0, 1.0);
        conv.aux1     = map(chanels.labels.aux1, 172.0, 1809.0, -1.0, 1.0);
        conv.aux2     = map(chanels.labels.aux2, 172.0, 1809.0, -1.0, 1.0);
        conv.aux3     = map(chanels.labels.aux3, 172.0, 1809.0, -1.0, 1.0);
        conv.aux4     = map(chanels.labels.aux4, 172.0, 1809.0, -1.0, 1.0);
        conv.aux5     = map(chanels.labels.aux5, 172.0, 1809.0, -1.0, 1.0);
        conv.aux6     = map(chanels.labels.aux6, 172.0, 1809.0, -1.0, 1.0);
        conv.aux7     = map(chanels.labels.aux7, 172.0, 1809.0, -1.0, 1.0);
        conv.aux8     = map(chanels.labels.aux8, 172.0, 1809.0, -1.0, 1.0);
    }
    return conv;
}

double Crossfire::map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}