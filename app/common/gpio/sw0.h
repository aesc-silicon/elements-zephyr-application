#ifndef SOFTWARE0_BUTTON_H
#define SOFTWARE0_BUTTON_H

/* size of stack area used by each thread */
#define SOFTWARE0_BUTTON_STACKSIZE		2048

/* scheduling priority used by each thread */
#define SOFTWARE0_BUTTON_PRIORITY		5

#define SOFTWARE0_BUTTON_NODE		DT_PATH(keys, sw0)

#if !DT_NODE_HAS_STATUS(SOFTWARE0_BUTTON_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

int software0_button(void);

#endif /* SOFTWARE0_BUTTON_H */
