#include "core/utils/logger.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

//-------------------------
// Internal State
//-------------------------

struct Logger {
    LogLevel min_level;      // Minimum level to log
    LogCallback callback;    // User callback
    void* user_data;         // User data for callback
    FILE* file;              // Log file handle (or NULL)
};

//-------------------------
// Lifecycle
//-------------------------

/**
 * Create a logger instance
 * 
 * Output:
 *   - Returns pointer to Logger, or NULL on failure
 */
Logger* logger_create(void) {
    Logger* logger = (Logger*)malloc(sizeof(Logger));
    if (!logger) return NULL;
    
    logger->min_level = LOG_LEVEL_INFO;
    logger->callback = NULL;
    logger->user_data = NULL;
    logger->file = NULL;
    
    return logger;
}

/**
 * Destroy logger and free resources
 * 
 * Input:
 *   - logger: Logger to destroy (can be NULL)
 */
void logger_destroy(Logger* logger) {
    if (!logger) return;
    
    if (logger->file) {
        logger_close_file(logger);
    }
    
    free(logger);
}

//-------------------------
// Configuration
//-------------------------

/**
 * Set minimum log level filter
 * 
 * Messages below this level will be ignored.
 * 
 * Input:
 *   - logger: Logger instance (must be non-NULL)
 *   - level: Minimum level to log
 */
void logger_set_level(Logger* logger, LogLevel level) {
    if (!logger) return;
    logger->min_level = level;
}

/**
 * Get current minimum log level
 * 
 * Input:
 *   - logger: Logger instance (must be non-NULL)
 * 
 * Output:
 *   - Returns current minimum log level
 */
LogLevel logger_get_level(const Logger* logger) {
    if (!logger) return LOG_LEVEL_INFO;
    return logger->min_level;
}

/**
 * Register a callback function for log messages
 * 
 * The callback will be called for each log message that passes
 * the level filter. Useful for forwarding logs to Unity or other systems.
 * 
 * Input:
 *   - logger: Logger instance (must be non-NULL)
 *   - callback: Callback function (can be NULL to unregister)
 *   - user_data: User data passed to callback (can be NULL)
 */
void logger_register_callback(Logger* logger, LogCallback callback, void* user_data) {
    if (!logger) return;
    
    logger->callback = callback;
    logger->user_data = user_data;
}

/**
 * Enable file logging to specified file
 * 
 * Opens a file for logging. If a file is already open, closes it first.
 * 
 * Input:
 *   - logger: Logger instance (must be non-NULL)
 *   - filename: Path to log file (must be non-NULL)
 * 
 * Output:
 *   - Returns 0 on success, -1 on failure
 */
int logger_set_file(Logger* logger, const char* filename) {
    if (!logger || !filename) return -1;
    
    // Close existing file if open
    if (logger->file) {
        logger_close_file(logger);
    }
    
    // Open new file
    logger->file = fopen(filename, "a");
    return logger->file ? 0 : -1;
}

/**
 * Close the log file
 * 
 * Input:
 *   - logger: Logger instance (must be non-NULL)
 */
void logger_close_file(Logger* logger) {
    if (!logger || !logger->file) return;
    
    fclose(logger->file);
    logger->file = NULL;
}

//-------------------------
// Logging
//-------------------------

// Helper to get level name
static const char* get_level_name(LogLevel level) {
    switch (level) {
        case LOG_LEVEL_DEBUG:   return "DEBUG";
        case LOG_LEVEL_INFO:    return "INFO";
        case LOG_LEVEL_WARNING: return "WARN";
        case LOG_LEVEL_ERROR:   return "ERROR";
        case LOG_LEVEL_FATAL:   return "FATAL";
        default:                return "UNKNOWN";
    }
}

/**
 * Log a message with printf-style formatting
 * 
 * Input:
 *   - logger: Logger instance (must be non-NULL)
 *   - level: Log level
 *   - format: Printf-style format string (must be non-NULL)
 *   - ...: Format arguments
 * 
 * Functionality:
 *   1. Check if level passes filter
 *   2. Format message with timestamp
 *   3. Call registered callback if present
 *   4. Write to file if file logging enabled
 *   5. Write to stderr for WARNING and above
 */
void logger_log(Logger* logger, LogLevel level, const char* format, ...) {
    if (!logger || !format) return;
    
    va_list args;
    va_start(args, format);
    logger_log_v(logger, level, format, args);
    va_end(args);
}

/**
 * Log a message with va_list arguments
 * 
 * Input:
 *   - logger: Logger instance (must be non-NULL)
 *   - level: Log level
 *   - format: Printf-style format string (must be non-NULL)
 *   - args: Variable argument list
 * 
 * Functionality:
 *   Same as logger_log but accepts va_list
 */
void logger_log_v(Logger* logger, LogLevel level, const char* format, va_list args) {
    if (!logger || !format) return;
    
    // Check level filter
    if (level < logger->min_level) return;
    
    // TODO: Implement full logging with timestamp
    // 1. Get current time and format timestamp
    // 2. Format the message with vsnprintf
    // 3. Call callback if registered
    // 4. Write to file if enabled
    // 5. Write to stderr for warnings and errors
    
    // Basic placeholder output
    char buffer[4096];
    vsnprintf(buffer, sizeof(buffer), format, args);
    
    // Write to stderr for important messages
    if (level >= LOG_LEVEL_WARNING) {
        fprintf(stderr, "[%s] %s\n", get_level_name(level), buffer);
    }
    
    // Call callback if registered
    if (logger->callback) {
        logger->callback(level, buffer, logger->user_data);
    }
    
    // Write to file if enabled
    if (logger->file) {
        fprintf(logger->file, "[%s] %s\n", get_level_name(level), buffer);
        fflush(logger->file);
    }
}
