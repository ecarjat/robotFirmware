#include "main.h"
#include "sensors.h"
#if SENSOR_ENABLE_BMI270
#include "imu_bmi270.h"
#endif
#if SENSOR_ENABLE_ICM42688
#include "imu_icm42688.h"
#endif
#if SENSOR_ENABLE_BMM150
#include "imu_bmm150.h"
#endif

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
#if SENSOR_ENABLE_ICM42688
    if (GPIO_Pin == ICM42688_INT1_Pin)
    {
        imu_icm42688_handle_int1();
        return;
    }
#endif
#if SENSOR_ENABLE_BMI270
    if (GPIO_Pin == BMI270_INT1_Pin)
    {
        imu_bmi270_handle_int1();
        return;
    }
#endif
#if SENSOR_ENABLE_BMM150
    if (GPIO_Pin == BMM150_INT1_Pin)
    {
        imu_bmm150_handle_int1();
        return;
    }
#endif
}
