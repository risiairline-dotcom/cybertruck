/* 正式Base Position/Query + Bus + CAN：配置10补读四轮0x42细分，按细分换算FD脉冲，
 * 整批四轮TXOK后才回调确认，速度快照取代未发出的小步时不确认；不连接设备。 */
#define BASE_STARTUP_NO_MAIN
#include "base_startup.c"

static void WheelConfig(unsigned address)
{
    uint8_t raw[32] = {0x42, 0x21, 0x15, 0x19};
    unsigned packet, offset = 1;
    raw[8] = 16;
    raw[19] = (uint8_t)address;
    raw[31] = 0x6B;
    for (packet = 0; offset < sizeof(raw); packet++)
    {
        unsigned size = sizeof(raw) - offset;
        if (size > 7) size = 7;
        memset(&incoming, 0, sizeof(incoming));
        incoming.id = (address << 8) | packet;
        incoming.len = (uint8_t)(size + 1);
        incoming.data[0] = 0x42;
        memcpy(incoming.data + 1, raw + offset, size);
        HAL_CAN_RxFifo0MsgPendingCallback(&board_can);
        offset += size;
    }
}

/* 推进CAN：查询给回包，0x42给完整配置；记录FD首包的方向/转速/脉冲。 */
static unsigned fd_count, fd_dir[4], fd_rpm[4], fd_pulses[4];
static void Serve(unsigned ticks, int base_run)
{
    unsigned n;
    for (n = 0; n < ticks; n++)
    {
        tick++;
        if (occupied)
        {
            unsigned address = last.id >> 8, code = last.data[0], packet = last.id & 255U;
            BusFrame frame = last;
            Done();
            if (code == 0x42) WheelConfig(address);
            else if (code == 0x35 || code == 0x36 || code == 0x3A) Reply(address, code, 0);
            else if (code == 0xFD && !packet)
            {
                CHECK(fd_count < 4 && address == fd_count + 1);
                fd_dir[fd_count] = frame.data[1];
                fd_rpm[fd_count] = ((unsigned)frame.data[2] << 8) | frame.data[3];
                fd_pulses[fd_count] = ((unsigned)frame.data[5] << 24) | ((unsigned)frame.data[6] << 16) |
                    ((unsigned)frame.data[7] << 8);
                CHECK(frame.data[4] == 0); /* acc=0：低转速直接起步 */
            }
            else if (code == 0xFD)
            {
                fd_pulses[fd_count] |= frame.data[1];
                CHECK(frame.data[2] == 2 && frame.data[3] == 0); /* 模式02相对当前实时位置，不同步 */
                fd_count++;
            }
            else
                CHECK(code == 0xF6);
        }
        if (base_run)
            Base_Run();
        else
        {
            Bus_Poll();
            if (diag_busy && Bus_Status() == CAN_SENT)
            {
                BusFrame ignored;
                while (Bus_Read(&ignored)) {}
                Bus_Release();
                diag_busy = 0;
            }
        }
    }
}

int main(void)
{
    double rev[4] = {.01, -.01, .01, -.01}, large[4] = {1.5, -1.5, 1.5, -1.5};
    unsigned i, configs = 0;
    ExecIo io = {Ready, EnableWheel, Bus_Start, Send, Stop, Fault, Stopped, Rearm, Position};
    Exec_Init(&executor, io);
    Can_Refresh();
    Bus_Setup(fixture_addresses);
    tick = 1000;

    /* 配置10且缺细分：常规轮询改为逐轮补读0x42，读齐后恢复35/36/3A。 */
    executor.config.version = 10;
    CHECK(!MicroReady());
    for (i = 0; i < 400 && configs < 8; i++)
    {
        unsigned before = sends;
        Query(tick);
        Serve(1, 0);
        if (sends != before && last.data[0] == 0x42) configs++;
        if (WheelConfigReady()) break;
    }
    CHECK(WheelConfigReady() && MicroReady() && configs == 4);
    for (i = 0; i < 4; i++)
        CHECK(Bus_Wheels()[i].config.microsteps == 16 && PulsesPerRev(&Bus_Wheels()[i].config) == 3200);
    Serve(30, 0);
    executor.config.version = 0;

    /* 活动流内：非法参数拒绝；0.01圈×3200=32脉冲，方向按圈数符号。 */
    CHECK(Bus_Start());
    CHECK(!Position(large, 2, 7, 9) && !Position(rev, 0, 7, 9) && !Position(rev, 2, 0, 9) &&
          !Position(rev, 2, 7, 0) && !position_sequence);
    CHECK(Position(rev, 2, 7, 9) && position_sequence == bus_diag.speed_sequence);
    for (i = 0; i < 300 && (position_sequence || fd_count < 4); i++)
        Serve(1, 1);
    CHECK(fd_count == 4 && !position_sequence);
    for (i = 0; i < 4; i++)
        CHECK(fd_pulses[i] == 32 && fd_rpm[i] == 2 && fd_dir[i] == (i == 1 || i == 3));
    CHECK(bus_diag.position_sent_sequence && !bus_diag.speed_sent_sequence);
    for (i = 0; i < 4; i++)
        CHECK(!bus_diag.sent_rpm[i]);
    /* 位置小步后查询照常，反馈不过期。 */
    {
        unsigned before = bus_diag.query_count;
        Serve(60, 1);
        CHECK(bus_diag.query_count > before + 4 && !bus_diag.error);
    }

    /* 未发出的小步被速度快照取代：只发F6，位置票据不会被确认。 */
    {
        int16_t zero[4] = {0};
        uint32_t confirmed = bus_diag.position_sent_sequence;
        CHECK(Position(rev, 2, 8, 11));
        CHECK(Send(zero, 8, 12));
        Serve(80, 1);
        CHECK(fd_count == 4 && bus_diag.position_sent_sequence == confirmed && position_sequence);
        Stop();
        CHECK(!position_sequence);
    }
    puts("base position wheel config / FD pulses / batch ack / superseded step PASS");
    return 0;
}
