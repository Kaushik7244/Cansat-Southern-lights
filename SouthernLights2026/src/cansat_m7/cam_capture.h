/**
 * cam_capture.h — SouthernLights CanSat 2026
 *
 * Thin wrapper around the Vision Shield HM0360 camera.
 * Isolates the Camera library (and its problematic CameraWire global)
 * into its own compilation unit so main.cpp never includes camera.h.
 */

#pragma once
#include <stdint.h>
#include <stddef.h>

/**
 * Initialise HM0360 camera. Call after all Wire-dependent peripherals are set up
 * (IIC-UART, Nicla, etc.) because cam.begin() reconfigures the I2C bus.
 * Returns true on success.
 */
bool camInit();

/**
 * Capture one QVGA grayscale frame and write it as an 8-bit BMP to SD.
 * @param path   Full filesystem path, e.g. "/fs/CAM_0001.bmp"
 * @return true on success
 */
bool camCaptureToSD(const char* path);
