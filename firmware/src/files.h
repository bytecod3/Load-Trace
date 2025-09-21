/**
 * @file files.h
 * @brief Creates header variables for files.cpp
 */

#ifndef FIRMWARE_FILES_H
#define FIRMWARE_FILES_H

#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>

extern char file_data_buffer[512];

/* file paths */
extern const char* wificonfig_folder_path;
extern const char* wifi_config_bit_filepath;
extern const char* wifi_config_filepath;

extern volatile uint8_t credentials_saved;
extern uint8_t device_configured_status;

void listDir(fs::FS &fs, const char *dirname, uint8_t levels);
void createDir(fs::FS &fs, const char *path);
void removeDir(fs::FS &fs, const char *path);
void readFile(fs::FS &fs, const char *path);
void writeFile(fs::FS &fs, const char *path, const char *message);
void appendFile(fs::FS &fs, const char *path, const char *message);
void renameFile(fs::FS &fs, const char *path1, const char *path2);
void deleteFile(fs::FS &fs, const char *path);
void writeFile2(fs::FS &fs, const char *path, const char *message);
void deleteFile2(fs::FS &fs, const char *path);
void testFileIO(fs::FS &fs, const char *path);
void FileOperationsInit();

#endif //FIRMWARE_FILES_H
