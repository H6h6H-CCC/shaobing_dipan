#ifndef NAV_CHECK_HAL_H
#define NAV_CHECK_HAL_H
#include <stdint.h>
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef struct { uint32_t unused; } CAN_HandleTypeDef;
uint32_t HAL_GetTick(void);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *can);
static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t value) { (void)value; }
#endif
