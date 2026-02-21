#ifndef GPX_WRITER_H
#define GPX_WRITER_H

#ifdef USE_LVGL_UI

#include <Arduino.h>

namespace GPXWriter {

    bool isRecording();
    bool startRecording();  // Creates new GPX file, returns true on success
    void stopRecording();   // Closes current GPX file
    void addPoint(float lat, float lon, float alt, float hdop, float speed);

} // namespace GPXWriter

#endif // USE_LVGL_UI
#endif // GPX_WRITER_H
