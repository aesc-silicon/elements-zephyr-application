#ifndef HEARTBEAT_H
#define HEARTBEAT_H

/* size of stack area used by each thread */
#define HEARTBEAT_STACKSIZE	2048

/* scheduling priority used by each thread */
#define HEARTBEAT_PRIORITY	5

#define HEARTBEAT_NODE		DT_PATH(gpio_leds, heartbeat)

#if !DT_NODE_HAS_STATUS(HEARTBEAT_NODE, okay)
#error "Unsupported board: heartbeat devicetree alias is not defined"
#endif

void heartbeat(void);

#endif /* HEARTBEAT_H */
