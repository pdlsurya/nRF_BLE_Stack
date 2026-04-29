
#ifndef USB_LOG_H
#define USB_LOG_H
void log_init(void);
void log_idle(void);

void log_printf(const char *format, ...);

#endif // USB_LOG_H
