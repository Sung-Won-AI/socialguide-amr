#include "board_io.h"

#include <stddef.h>
#include <string.h>

#include "board_config.h"
#include "main.h"
#include "motor_driver.h"

static bool board_ready;

__weak uint16_t BoardIO_ReadBatteryVoltageMv(void)
{
    return 0U;
}

void BoardIO_Init(void)
{
    board_ready = false;

    /* Preserve the electronic team's precharge sequence. */
    HAL_GPIO_WritePin(
        PRECHARGE_RELAY_GPIO_Port,
        PRECHARGE_RELAY_Pin,
        GPIO_PIN_SET
    );
    HAL_Delay(AMR_PRECHARGE_TIME_MS);
    HAL_GPIO_WritePin(
        PRECHARGE_RELAY_GPIO_Port,
        PRECHARGE_RELAY_Pin,
        GPIO_PIN_RESET
    );

    board_ready = true;
}

bool BoardIO_IsReady(void)
{
    return board_ready;
}

static bool estop_active(void)
{
#if AMR_HAS_ESTOP_INPUT
    return HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin)
        == AMR_ESTOP_ACTIVE_LEVEL;
#else
    /* Missing E-Stop wiring must never be interpreted as safe. */
    return true;
#endif
}

static bool cliff_left_active(void)
{
#if AMR_HAS_CLIFF_LEFT_INPUT
    return HAL_GPIO_ReadPin(CLIFF_LEFT_GPIO_Port, CLIFF_LEFT_Pin)
        == AMR_CLIFF_ACTIVE_LEVEL;
#else
    return false;
#endif
}

static bool cliff_right_active(void)
{
#if AMR_HAS_CLIFF_RIGHT_INPUT
    return HAL_GPIO_ReadPin(CLIFF_RIGHT_GPIO_Port, CLIFF_RIGHT_Pin)
        == AMR_CLIFF_ACTIVE_LEVEL;
#else
    return false;
#endif
}

static bool handle_released(void)
{
#if AMR_HAS_HANDLE_INPUT
    return HAL_GPIO_ReadPin(HANDLE_GPIO_Port, HANDLE_Pin)
        == AMR_HANDLE_RELEASED_LEVEL;
#else
    /* A missing user-presence sensor is treated as released. */
    return true;
#endif
}

void BoardIO_ReadHardwareInputs(AmrHardwareInputs *inputs_out)
{
    uint16_t battery_mv;

    if (inputs_out == NULL) {
        return;
    }

    memset(inputs_out, 0, sizeof(*inputs_out));
    battery_mv = BoardIO_ReadBatteryVoltageMv();
    inputs_out->initialization_complete = board_ready;
    inputs_out->estop_active = estop_active();
    inputs_out->cliff_left = cliff_left_active();
    inputs_out->cliff_right = cliff_right_active();
    inputs_out->user_released = handle_released();
    inputs_out->motor_fault = MotorDriver_HasFault();
    inputs_out->critical_sensor_fault =
        (AMR_HAS_CLIFF_LEFT_INPUT == 0)
        || (AMR_HAS_CLIFF_RIGHT_INPUT == 0);
    inputs_out->low_battery_warning =
#if AMR_HAS_BATTERY_MONITOR
        battery_mv <= AMR_BATTERY_LOW_THRESHOLD_MV;
#else
        false;
#endif
    inputs_out->measured_left_velocity_mm_s =
        MotorDriver_GetLeftVelocityMmS();
    inputs_out->measured_right_velocity_mm_s =
        MotorDriver_GetRightVelocityMmS();
    inputs_out->battery_voltage_mv = battery_mv;
    inputs_out->motor_error_code = MotorDriver_GetErrorCode();
}
