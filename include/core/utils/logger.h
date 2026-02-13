#pragma once
// Vehicle Dynamics Engine - Logging System

#include "core/math/math_base.h"
#include <stdarg.h>

//-------------------------
// Types
//-------------------------

typedef enum LogLevel {
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_FATAL
} LogLevel;

typedef struct Logger Logger;

// Log callback function
typedef void (*LogCallback)(LogLevel level, const char* message, void* user_data);

//-------------------------
// API Functions
//-------------------------

VDE_API Logger* logger_create(void);
VDE_API void logger_destroy(Logger* logger);

// Set log level filter
VDE_API void logger_set_level(Logger* logger, LogLevel level);
VDE_API LogLevel logger_get_level(const Logger* logger);

// Register callback
VDE_API void logger_register_callback(Logger* logger, LogCallback callback, void* user_data);

// Enable/disable file logging
VDE_API int logger_set_file(Logger* logger, const char* filename);
VDE_API void logger_close_file(Logger* logger);

// Log messages
VDE_API void logger_log(Logger* logger, LogLevel level, const char* format, ...);
VDE_API void logger_log_v(Logger* logger, LogLevel level, const char* format, va_list args);

// Convenience macros
#define LOG_DEBUG(logger, ...) logger_log(logger, LOG_LEVEL_DEBUG, __VA_ARGS__)
#define LOG_INFO(logger, ...)  logger_log(logger, LOG_LEVEL_INFO, __VA_ARGS__)
#define LOG_WARN(logger, ...)  logger_log(logger, LOG_LEVEL_WARNING, __VA_ARGS__)
#define LOG_ERROR(logger, ...) logger_log(logger, LOG_LEVEL_ERROR, __VA_ARGS__)
#define LOG_FATAL(logger, ...) logger_log(logger, LOG_LEVEL_FATAL, __VA_ARGS__)