#include "comm.h"
#include <string.h>

static FrameRx receiver;
static Frame normal;
static Frame urgent;
static uint8_t normal_ready;
static uint8_t urgent_ready;
static uint8_t key;
static uint8_t stopped;
static uint64_t boot_id;
static uint64_t session_id;
static uint32_t generation;
static uint32_t last_heart_ms;
static uint32_t progress;
static uint32_t heart_count;
static uint32_t rejected;
static uint32_t dropped;
static uint32_t faults;
static uint32_t uptime_ms;
static CommHooks executor;

uint64_t Comm_Boot(void)
{
    return boot_id;
}

uint64_t Comm_Session(void)
{
    return session_id;
}

void Comm_Bind(CommHooks hooks)
{
    executor = hooks;
}

void Comm_Init(uint64_t boot, uint32_t now_ms)
{
    memset(&receiver, 0, sizeof(receiver));
    normal_ready = urgent_ready = key = stopped = 0;
    boot_id = boot;
    session_id = 0;
    generation = 1;
    last_heart_ms = uptime_ms = now_ms;
    progress = heart_count = rejected = dropped = 0;
    faults = boot ? 0U : 1U;
    memset(&executor, 0, sizeof(executor));
}

void Comm_Fault(uint32_t reason)
{
    if (executor.stop)
    {
        executor.stop(0x100U | reason, uptime_ms);
    }
    faults |= reason;
    normal_ready = 0;
    session_id = 0;
    generation++;
    receiver.used = 0;
    receiver.dropping = 1;
}

void Comm_Tick(uint32_t now_ms, uint8_t key_pressed)
{
    uptime_ms = now_ms;
    key = key_pressed;
    if (session_id && (uint32_t)(now_ms - last_heart_ms) >= 500U)
    {
        if (executor.stop)
        {
            executor.stop(0x200U, now_ms);
        }
        normal_ready = 0;
        session_id = 0;
        generation++;
    }
}

static void Reply(const Frame *request, uint8_t result, const uint8_t *data, uint16_t size)
{
    Frame *reply = request->type == COMM_STOP ? &urgent : &normal;
    uint8_t *ready = request->type == COMM_STOP ? &urgent_ready : &normal_ready;
    if (*ready)
    {
        dropped++;
    }
    reply->type = (uint8_t)(request->type | 0x80U);
    reply->boot = boot_id;
    reply->session = request->session;
    reply->request = request->request;
    reply->ttl_ms = 0;
    reply->size = (uint16_t)(size + 1U);
    reply->data[0] = result;
    if (size)
    {
        memcpy(reply->data + 1, data, size);
    }
    *ready = 1;
    if (result != COMM_OK)
    {
        rejected++;
    }
}

static void Dispatch(const Frame *frame, uint32_t now_ms)
{
    uint8_t data[FRAME_PAYLOAD_MAX - 1U] = {0};
    uint16_t size = 0;
    uint8_t result;
    uint32_t next;
    if (frame->type & 0x80U)
    {
        return;
    }
    if (frame->ttl_ms == 0U || frame->ttl_ms > 1000U ||
        (uint32_t)(now_ms - receiver.started_ms) >= frame->ttl_ms)
    {
        Reply(frame, COMM_EXPIRED, 0, 0);
        return;
    }
    if (((frame->type == COMM_HELLO || frame->type == COMM_STATE ||
          frame->type == COMM_DIAG || frame->type == COMM_STOP) && frame->size != 0U) ||
        ((frame->type == COMM_OPEN || frame->type == COMM_HEART) && frame->size != 4U) ||
        (frame->type == COMM_HELLO && frame->session != 0U))
    {
        Reply(frame, COMM_FORMAT, 0, 0);
        return;
    }
    if (frame->type == COMM_HELLO && frame->size == 0U && frame->session == 0U)
    {
        Frame_Put(data, generation, 4);
        Frame_Put(data + 4, executor.command ? 5U : 1U, 4);
        if (executor.command)
        {
            memcpy(data + 8, "GC-BASE-0.3.51", sizeof("GC-BASE-0.3.51"));
        }
        else
        {
            memcpy(data + 8, "GC-RO-0.2.0", 12);
        }
        Reply(frame, COMM_OK, data, 24);
        return;
    }
    if (frame->type == COMM_STOP && frame->size == 0U &&
        (frame->boot == 0U || frame->boot == boot_id))
    {
        stopped = 1;
        if (executor.stop)
        {
            executor.stop(0, now_ms);
            Reply(frame, COMM_OK, 0, 0);
            return;
        }
        /* 锁存禁止标记；没有电机反馈，不能声称实车已经停稳。 */
        Reply(frame, COMM_READONLY, 0, 0);
        return;
    }
    if (!boot_id || frame->boot != boot_id)
    {
        Reply(frame, COMM_BOOT, 0, 0);
        return;
    }
    if (frame->type == COMM_OPEN)
    {
        if (frame->size != 4U || !frame->session || Frame_Get(frame->data, 4) != generation)
        {
            Reply(frame, COMM_SESSION, 0, 0);
        }
        else if (session_id && session_id != frame->session)
        {
            Reply(frame, COMM_BUSY, 0, 0);
        }
        else
        {
            if (!session_id)
            {
                session_id = frame->session;
                last_heart_ms = now_ms;
                progress = 0;
            }
            Reply(frame, COMM_OK, 0, 0);
        }
        return;
    }
    if (!session_id || frame->session != session_id)
    {
        Reply(frame, COMM_SESSION, 0, 0);
        return;
    }
    if (executor.command && (frame->type == 16 || (frame->type >= 32 && frame->type <= 38) ||
                            (frame->type >= 48 && frame->type <= 72)))
    {
        result = executor.command(frame->type, frame->data, frame->size, data, &size, now_ms);
        Reply(frame, result, data, size);
    }
    else if (frame->type == COMM_HEART && frame->size == 4U)
    {
        next = (uint32_t)Frame_Get(frame->data, 4);
        /* 重复保活不延长期限；无符号差同时允许32位进度计数回绕。 */
        if (next != progress && (uint32_t)(next - progress) < 0x80000000U)
        {
            progress = next;
            last_heart_ms = now_ms;
            heart_count++;
        }
        Reply(frame, COMM_OK, 0, 0);
    }
    else if (frame->type == COMM_STATE && frame->size == 0U)
    {
        Frame_Put(data, uptime_ms, 4);
        Frame_Put(data + 4, faults, 4);
        Frame_Put(data + 8, 1U | (session_id ? 2U : 0U) | (stopped ? 4U : 0U) | (key ? 8U : 0U), 2);
        Frame_Put(data + 10, heart_count, 4);
        Frame_Put(data + 14, receiver.good, 4);
        Frame_Put(data + 18, receiver.bad, 4);
        Frame_Put(data + 22, rejected, 4);
        Reply(frame, COMM_OK, data, 26);
    }
    else if (frame->type == COMM_DIAG && frame->size == 0U)
    {
        Frame_Put(data, dropped, 4);
        Frame_Put(data + 4, generation, 4);
        Reply(frame, COMM_OK, data, 8);
    }
    else
    {
        Reply(frame, COMM_READONLY, 0, 0);
    }
}

void Comm_Feed(uint8_t byte, uint32_t received_ms, uint32_t now_ms)
{
    Frame frame;
    Comm_Tick(now_ms, key);
    if (Frame_Push(&receiver, byte, received_ms, &frame))
    {
        Dispatch(&frame, now_ms);
    }
}

uint16_t Comm_Read(uint8_t *wire)
{
    if (urgent_ready)
    {
        urgent_ready = 0;
        return Frame_Encode(&urgent, wire);
    }
    if (normal_ready)
    {
        normal_ready = 0;
        return Frame_Encode(&normal, wire);
    }
    return 0;
}
