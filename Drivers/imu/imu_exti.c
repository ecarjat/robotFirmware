#include "imu_bmi270.h"
#include "imu_bmm150.h"
#include "imu_icm42688.h"
#include "main.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ICM42688_INT1_Pin)
    {
        imu_icm42688_handle_int1();
    }
    else if (GPIO_Pin == BMI270_INT1_Pin)
    {
        imu_bmi270_handle_int1();
    }
    else if (GPIO_Pin == BMM150_INT1_Pin)
    {
        imu_bmm150_handle_int1();
    }
}
