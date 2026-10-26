/* 电脑端验证：直接编译工程的导航、底盘、PID 源文件，仅替换硬件输入输出。 */
#include "navigation.h"
#include "chassis.h"
#include "remote_control.h"
#include "dji_motor.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

CAN_HandleTypeDef hcan1;
static uint32_t tick;
static remote_control_t remote;
static uint8_t remote_valid = 1U;
static uint8_t motor_mask = 0x0FU;
static int16_t sent[4];
static const uint8_t move_frame[] = {
    0xAA, 0x56, 0xE8, 0x03, 0x18, 0xFC, 0xF4, 0x01, 0x0C, 0xFE, 0xF4
};

uint32_t HAL_GetTick(void) { return tick; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *can)
{ (void)can; return 3U; }
void Remote_Control_Get(remote_control_t *out) { *out = remote; }
uint8_t Remote_Control_DataValid(const remote_control_t *value)
{ (void)value; return remote_valid; }
void DJI_Motor_GetFeedback(uint8_t index, dji_motor_feedback_t *motor)
{
    memset(motor, 0, sizeof(*motor));
    motor->update_tick_ms = (motor_mask & (1U << index)) ? tick : 0U;
}
HAL_StatusTypeDef DJI_Motor_SendChassisCurrent(CAN_HandleTypeDef *can,
    int16_t a, int16_t b, int16_t c, int16_t d)
{
    (void)can;
    sent[0] = a; sent[1] = b; sent[2] = c; sent[3] = d;
    return HAL_OK;
}
static void step(uint32_t now) { tick = now; Chassis_Update(); }
static void send_move(void) { Navigation_Receive(move_frame, sizeof(move_frame)); }
static void set_remote(uint32_t now, uint8_t left)
{
    remote.update_tick_ms = now;
    remote.switch_position[RC_S_LEFT] = left;
    remote.switch_position[RC_S_RIGHT] = RC_SW_DOWN;
}
static void stopped(void)
{
    unsigned i;
    assert(g_chassis_debug.enabled == 0U);
    for (i = 0; i < 4; i++)
    {
        assert(sent[i] == 0);
        assert(g_chassis_debug.motor_target_rpm[i] == 0.0f);
    }
}
static void test_parser(void)
{
    navigation_command_t cmd;
    uint8_t bad[11], joined[25];
    unsigned split;
    for (split = 0; split <= sizeof(move_frame); split++)
    {
        Navigation_Init(); tick = 0U;
        Navigation_Receive(move_frame, split);
        Navigation_Receive(move_frame + split, sizeof(move_frame) - split);
        Navigation_Get(&cmd);
        assert(cmd.valid && cmd.frame_count == 1U && cmd.update_tick_ms == 0U);
        assert(cmd.motor_rpm[0] == 1000 && cmd.motor_rpm[1] == -1000);
        assert(cmd.motor_rpm[2] == 500 && cmd.motor_rpm[3] == -500);
    }
    Navigation_Init(); tick = 20U;
    memset(joined, 0xAA, 3);
    memcpy(joined + 3, move_frame, 11);
    memcpy(joined + 14, move_frame, 11);
    Navigation_Receive(joined, sizeof(joined));
    Navigation_Get(&cmd); assert(cmd.frame_count == 2U);
    memcpy(bad, move_frame, 11); bad[10] ^= 1U;
    tick = 400U; Navigation_Receive(bad, 11);
    Navigation_Get(&cmd);
    assert(cmd.frame_count == 2U && cmd.update_tick_ms == 20U);
    assert(cmd.checksum_error_count == 1U);
    send_move(); Navigation_Get(&cmd);
    assert(cmd.frame_count == 3U && cmd.update_tick_ms == 400U);
    Navigation_Receive(move_frame, 5);
    Navigation_Invalidate(); Navigation_Receive(move_frame + 5, 6);
    Navigation_Get(&cmd); assert(!cmd.valid && cmd.frame_count == 3U);
    send_move(); Navigation_Get(&cmd); assert(cmd.valid && cmd.frame_count == 4U);
    puts("PASS USB: signed/little-endian, all split positions, concatenation, noise, checksum, disconnect");
}
static void test_control(void)
{
    uint8_t high[11] = {0xAA, 0x56, 0x30, 0x75, 0x98, 0x3A, 0xD0, 0x8A, 0, 0, 0};
    const uint8_t zero[11] = {0xAA, 0x56, 0, 0, 0, 0, 0, 0, 0, 0, 0xFC};
    unsigned i;
    Navigation_Init(); tick = 1000U; Chassis_Init(); send_move();
    step(1000U); stopped(); assert(!g_chassis_debug.navigation_armed);
    set_remote(1002U, RC_SW_MID); remote.channel[3] = 330;
    step(1002U); step(1004U);
    assert(g_chassis_debug.enabled && g_chassis_debug.control_source == CHASSIS_SOURCE_REMOTE);
    send_move();
    step(1103U); stopped(); assert(g_chassis_debug.navigation_waiting_frame);
    step(1105U); stopped();  /* 手动期间的缓存不能在断线后直接生效。 */
    send_move(); step(1107U);
    assert(g_chassis_debug.enabled && g_chassis_debug.control_source == CHASSIS_SOURCE_NAVIGATION);
    assert(g_chassis_debug.motor_target_rpm[0] == 1000.0f);
    assert(g_chassis_debug.motor_target_rpm[1] == -1000.0f);
    assert(g_chassis_debug.motor_target_rpm[2] == 500.0f);
    assert(g_chassis_debug.motor_target_rpm[3] == -500.0f);
    step(1405U); assert(g_chassis_debug.enabled); /* 恰好 300 ms 仍有效。 */
    step(1406U); stopped();
    send_move(); step(1408U); assert(sent[0] == 5050); /* 超时已清除旧积分。 */
    Navigation_Receive(zero, 11); step(1410U);
    for (i = 0; i < 4; i++) assert(sent[i] == 0);
    for (i = 0; i < 10; i++) high[10] ^= high[i];
    Navigation_Receive(high, 11); step(1412U);
    assert(fabsf(g_chassis_debug.motor_target_rpm[0] - 11459.16f) < 0.1f);
    assert(fabsf(g_chassis_debug.motor_target_rpm[1] * 2.0f - g_chassis_debug.motor_target_rpm[0]) < 0.1f);
    assert(g_chassis_debug.motor_target_rpm[2] == -g_chassis_debug.motor_target_rpm[0]);
    motor_mask = 7U; step(1414U); stopped();
    motor_mask = 15U; step(1416U); assert(g_chassis_debug.enabled);
    set_remote(1420U, RC_SW_MID); remote.channel[3] = 0;
    step(1420U); stopped(); step(1422U);
    assert(g_chassis_debug.control_source == CHASSIS_SOURCE_REMOTE);
    for (i = 0; i < 4; i++) assert(sent[i] == 0);
    remote_valid = 0U; set_remote(1424U, RC_SW_MID);
    step(1424U); stopped(); assert(g_chassis_debug.control_source == CHASSIS_SOURCE_STOP);
    remote_valid = 1U;
    for (i = RC_SW_UP; i <= RC_SW_DOWN; i++)
    {
        set_remote(1500U + i * 200U, (uint8_t)i);
        step(remote.update_tick_ms); stopped();
        step(remote.update_tick_ms + 101U); stopped();
        assert(!g_chassis_debug.navigation_armed);
    }
    set_remote(2100U, RC_SW_MID); step(2100U); step(2102U);
    step(2201U); stopped(); send_move(); step(2203U);
    assert(g_chassis_debug.enabled);
    Navigation_Invalidate(); step(2205U); stopped();
    puts("PASS control: boot inhibit, RC priority, armed loss, fresh-frame gate, 300ms expiry, PID reset, raw RPM, proportional limit, motor loss, reconnect, invalid RC, switch disarm");
}
int main(void) { test_parser(); test_control(); return 0; }
