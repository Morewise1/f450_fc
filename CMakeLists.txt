cmake_minimum_required(VERSION 3.20)
project(F407FcStub C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(FC_SOURCES
    App/app_main.c
    App/app_scheduler.c
    App/app_flight.c
    App/app_safety.c
    BSP/bsp_led.c
    BSP/bsp_buzzer.c
    BSP/bsp_esc_pwm.c
    BSP/bsp_battery_adc.c
    BSP/bsp_debug_uart.c
    Drivers/Radio/drv_ibus.c
    Drivers/Sensors/drv_qmi8658.c
    Drivers/Sensors/drv_bmp390.c
    Drivers/Sensors/drv_vl53l1x.c
    Estimator/est_attitude.c
    Estimator/est_altitude.c
    Control/ctl_pid.c
    Control/ctl_mixer.c
    Control/ctl_rate.c
    Control/ctl_attitude.c
    Control/ctl_altitude.c
)

add_library(f407_fc_core STATIC ${FC_SOURCES})
target_include_directories(f407_fc_core PUBLIC
    App
    BSP
    Config
    Control
    Drivers/Radio
    Drivers/Sensors
    Estimator
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(f407_fc_core PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

add_executable(fc_stub_smoke Tests/stub_smoke_test.c)
target_link_libraries(fc_stub_smoke PRIVATE f407_fc_core)

add_executable(app_scheduler_test
    App/app_scheduler.c
    Tests/app_scheduler_test.c
)
target_include_directories(app_scheduler_test PRIVATE App Config)

add_executable(app_flight_state_machine_test
    App/app_flight.c
    App/app_safety.c
    Tests/app_flight_state_machine_test.c
)
target_include_directories(app_flight_state_machine_test PRIVATE
    App
    BSP
    Config
    Control
    Drivers/Radio
    Drivers/Sensors
    Estimator
)

add_executable(qmi8658_driver_test
    Drivers/Sensors/drv_qmi8658.c
    Tests/qmi8658_driver_test.c
)
target_include_directories(qmi8658_driver_test PRIVATE
    Tests/fake_hal
    Config
    Drivers/Sensors
)
target_compile_definitions(qmi8658_driver_test PRIVATE
    FC_USE_STM32_HAL=1
    FC_IMU_BODY_X_SOURCE=1U
    FC_IMU_BODY_Y_SOURCE=0U
    FC_IMU_BODY_Z_SOURCE=2U
    FC_IMU_BODY_X_SIGN=-1.0f
    FC_IMU_BODY_Y_SIGN=1.0f
    FC_IMU_BODY_Z_SIGN=-1.0f
)

add_executable(esc_pwm_driver_test
    BSP/bsp_esc_pwm.c
    Tests/esc_pwm_driver_test.c
)
target_include_directories(esc_pwm_driver_test PRIVATE
    Tests/fake_hal
    BSP
    Config
)
target_compile_definitions(esc_pwm_driver_test PRIVATE
    FC_USE_STM32_HAL=1
    FC_ESC_M1_COUNTER_HZ=1000000U
    FC_ESC_M2_COUNTER_HZ=2000000U
    FC_ESC_M3_COUNTER_HZ=4000000U
    FC_ESC_M4_COUNTER_HZ=500000U
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(app_scheduler_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_options(app_flight_state_machine_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_options(qmi8658_driver_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_options(esc_pwm_driver_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

enable_testing()
add_test(NAME fc_stub_smoke COMMAND fc_stub_smoke)
add_test(NAME app_scheduler_test COMMAND app_scheduler_test)
add_test(NAME app_flight_state_machine_test COMMAND app_flight_state_machine_test)
add_test(NAME qmi8658_driver_test COMMAND qmi8658_driver_test)
add_test(NAME esc_pwm_driver_test COMMAND esc_pwm_driver_test)
