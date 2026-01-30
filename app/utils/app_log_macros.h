#ifndef UTILS_APP_LOG_MACROS_H
#define UTILS_APP_LOG_MACROS_H

#include "log.h"

#define APP_LOG_LEVEL_OFF 0U
#define APP_LOG_LEVEL_ERROR 1U
#define APP_LOG_LEVEL_WARN 2U
#define APP_LOG_LEVEL_INFO 3U
#define APP_LOG_LEVEL_DEBUG 4U

#ifndef APP_LOG_LEVEL
#ifdef NDEBUG
#define APP_LOG_LEVEL APP_LOG_LEVEL_WARN
#else
#define APP_LOG_LEVEL APP_LOG_LEVEL_INFO
#endif
#endif

#if APP_LOG_LEVEL >= APP_LOG_LEVEL_DEBUG
#define APP_LOG_DEBUG(fmt, ...)                                                 \
  app_log_printf("[APP][DEBUG] " fmt "\r\n", ##__VA_ARGS__)
#else
#define APP_LOG_DEBUG(fmt, ...)                                                 \
  do                                                                           \
  {                                                                            \
    if (0)                                                                     \
    {                                                                          \
      app_log_printf(fmt, ##__VA_ARGS__);                                      \
    }                                                                          \
  } while (0)
#endif

#if APP_LOG_LEVEL >= APP_LOG_LEVEL_INFO
#define APP_LOG_INFO(fmt, ...)                                                 \
  app_log_printf("[APP][INFO] " fmt "\r\n", ##__VA_ARGS__)
#else
#define APP_LOG_INFO(fmt, ...)                                                 \
  do                                                                           \
  {                                                                            \
    if (0)                                                                     \
    {                                                                          \
      app_log_printf(fmt, ##__VA_ARGS__);                                      \
    }                                                                          \
  } while (0)
#endif

#if APP_LOG_LEVEL >= APP_LOG_LEVEL_WARN
#define APP_LOG_WARN(fmt, ...)                                                 \
  app_log_printf("[APP][WARN] " fmt "\r\n", ##__VA_ARGS__)
#else
#define APP_LOG_WARN(fmt, ...)                                                 \
  do                                                                           \
  {                                                                            \
    if (0)                                                                     \
    {                                                                          \
      app_log_printf(fmt, ##__VA_ARGS__);                                      \
    }                                                                          \
  } while (0)
#endif

#if APP_LOG_LEVEL >= APP_LOG_LEVEL_ERROR
#define APP_LOG_ERROR(fmt, ...)                                                \
  app_log_printf("[APP][ERROR] " fmt "\r\n", ##__VA_ARGS__)
#else
#define APP_LOG_ERROR(fmt, ...)                                                \
  do                                                                           \
  {                                                                            \
    if (0)                                                                     \
    {                                                                          \
      app_log_printf(fmt, ##__VA_ARGS__);                                      \
    }                                                                          \
  } while (0)
#endif

#endif /* UTILS_APP_LOG_MACROS_H */
