#include "CameraCommunication.h"
#include "Config.h"
#include <stdio.h>
#include <string.h>

void CameraCommunication::begin() {
    Serial1.begin(Config::CAMERA_BAUD, SERIAL_8N1,
                  Config::CAMERA_RX_PIN, Config::CAMERA_TX_PIN);
}

void CameraCommunication::poll() {
    while (Serial1.available() > 0) {
        const char incoming = static_cast<char>(Serial1.read());

        if (incoming == '\n') {
            lineBuffer_[lineLength_] = '\0';
            if (lineLength_ > 0) parseLine(lineBuffer_);
            lineLength_ = 0;
            continue;
        }

        if (incoming == '\r') continue;

        if (lineLength_ < sizeof(lineBuffer_) - 1) {
            lineBuffer_[lineLength_++] = incoming;
        } else {
            // Drop overlong lines rather than trying to parse a partial message.
            lineLength_ = 0;
        }
    }
}

void CameraCommunication::parseLine(char *line) {
    if (strcmp(line, "NONE") == 0) {
        data_.hasValidMessage = true;
        data_.detected = false;
        data_.confidence = 0.0f;
        data_.centerX = 0;
        data_.imageWidth = 0;
        strncpy(data_.color, "UNKNOWN", sizeof(data_.color) - 1);
        data_.color[sizeof(data_.color) - 1] = '\0';
        data_.lastMessageMs = millis();
        ++data_.messageCounter;
        return;
    }

    int centerX = 0;
    int imageWidth = 0;
    float confidence = 0.0f;
    char color[12] = "UNKNOWN";

    // Expected line: FLAG,centerX,imageWidth,confidence,color
    // Example: FLAG,160,320,0.95,RED
    const int fields = sscanf(line, "FLAG,%d,%d,%f,%11s",
                              &centerX, &imageWidth, &confidence, color);

    if (fields < 3 || imageWidth <= 0 || centerX < 0 || centerX >= imageWidth ||
        confidence < 0.0f || confidence > 1.0f) {
        return;  // Invalid messages do not refresh the camera timeout.
    }

    data_.hasValidMessage = true;
    data_.confidence = confidence;
    data_.centerX = centerX;
    data_.imageWidth = imageWidth;
    data_.detected = confidence >= Config::CAMERA_MIN_CONFIDENCE;

    if (fields >= 4) {
        strncpy(data_.color, color, sizeof(data_.color) - 1);
        data_.color[sizeof(data_.color) - 1] = '\0';
    } else {
        strncpy(data_.color, "UNKNOWN", sizeof(data_.color) - 1);
        data_.color[sizeof(data_.color) - 1] = '\0';
    }

    data_.lastMessageMs = millis();
    ++data_.messageCounter;
}

bool CameraCommunication::isFresh(uint32_t nowMs) const {
    return data_.hasValidMessage &&
           (nowMs - data_.lastMessageMs <= Config::CAMERA_STALE_MS);
}

const CameraData &CameraCommunication::data() const {
    return data_;
}
