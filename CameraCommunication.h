#pragma once

#include <Arduino.h>

struct CameraData {
    bool hasValidMessage = false;
    bool detected = false;
    int centerX = 0;
    int imageWidth = 0;
    float confidence = 0.0f;
    char color[12] = "UNKNOWN";
    uint32_t lastMessageMs = 0;
    uint32_t messageCounter = 0;
};

class CameraCommunication {
public:
    void begin();
    void poll();
    bool isFresh(uint32_t nowMs) const;
    const CameraData &data() const;

private:
    void parseLine(char *line);

    char lineBuffer_[80] = {0};
    size_t lineLength_ = 0;
    CameraData data_;
};
