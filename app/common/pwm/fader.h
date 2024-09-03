#ifndef FADER_H
#define FADER_H

/* size of stack area used by each thread */
#define FADER_STACKSIZE		2048

/* scheduling priority used by each thread */
#define FADER_PRIORITY		5

#define FADER_RED_NODE		DT_PATH(pwm_leds, pwm_red)

#if !DT_NODE_HAS_STATUS(FADER_RED_NODE, okay)
#error "Unsupported board: pwm-red devicetree alias is not defined"
#endif

#define FADER_GREEN_NODE	DT_PATH(pwm_leds, pwm_green)
#if !DT_NODE_HAS_STATUS(FADER_GREEN_NODE, okay)
#error "Unsupported board: pwm-green devicetree alias is not defined"
#endif

#define FADER_BLUE_NODE		DT_PATH(pwm_leds, pwm_blue)
#if !DT_NODE_HAS_STATUS(FADER_BLUE_NODE, okay)
#error "Unsupported board: pwm-blue devicetree alias is not defined"
#endif

#define STEP_SIZE PWM_USEC(200)

int fader(void);

#endif /* FADER_H */
