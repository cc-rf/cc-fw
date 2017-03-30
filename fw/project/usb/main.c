#include <board.h>
#include <pin_mux.h>

#include <FreeRTOS.h>
#include <task.h>

#include <stdio.h>
#include <itm.h>


#include <cc/spi.h>
#include <cc/io.h>
#include <cc/nphy.h>
#include <cc/freq.h>
#include <cc/cc1200.h>
#include <cc/isr.h>
#include <cc/tmr.h>
#include <cc/mac.h>
#include <cc/chan.h>
#include <core_cm4.h>
#include <cc/sys/kinetis/pit.h>
#include <cc/type.h>
#include <timers.h>
#include <cc/cfg.h>
#include <fsl_port.h>
#include <semphr.h>
#include <cc/nmac.h>
#include <fsl_enet.h>
#include <fsl_sim.h>
#include <virtual_com.h>
#include <malloc.h>
#include <util/uart.h>
#include <uhdcd.h>
#include <sclk.h>
#include <fsl_flash.h>

#define PFLAG_PORT          PORTB
#define PFLAG_GPIO          GPIOB
#define PFLAG_PIN           5

#define UFLAG1_ON_PORT      PORTA
#define UFLAG1_ON_GPIO      GPIOA
#define UFLAG1_ON_PIN       16

#define UFLAG1_PORT         PORTA
#define UFLAG1_GPIO         GPIOA
#define UFLAG1_PIN          17

#define UFLAG2_ON_PORT      PORTE
#define UFLAG2_ON_GPIO      GPIOE
#define UFLAG2_ON_PIN       0
#define UFLAG2_ON_MUX_ORIG  kPORT_MuxAlt2

#define UFLAG2_PORT         PORTE
#define UFLAG2_GPIO         GPIOE
#define UFLAG2_PIN          4

static void pin_flag_init(void);
static void main_task(void *param);

static void uart_relay_run(void);

static void usb_recv(size_t size, u8 *data);

//static SemaphoreHandle_t write_sem = NULL;

// For the GDB helper
bool dbgPendSVHookState = 0;


#define MAIN_TASK_STACK_SIZE    (TASK_STACK_SIZE_MEDIUM / sizeof(StackType_t))

StackType_t main_task_stack[MAIN_TASK_STACK_SIZE];
StaticTask_t main_task_static;

int main(void)
{
    BOARD_InitPins();
    LED_A_ON();
    //BOARD_BootClockRUN();
    BOARD_BootClockOCHSRUN();

    //write_sem = xSemaphoreCreateBinary();
    //xSemaphoreGive(write_sem);

    BOARD_InitDebugConsole();
    itm_init();
    itm_puts(0, "<boot>\r\n");

    /*printf("how about a fault.\r\n");
    *(int *)(0xfffefafe) = 42;
    while (1) {};*/


    itm_printf(0, "\nclocks:\n  core\t\t\t= %lu\n  bus\t\t\t= %lu\n  flexbus\t\t= %lu\n  flash\t\t\t= %lu\n  pllfllsel\t\t= %lu\n  osc0er\t\t= %lu\n  osc0erundiv\t\t= %lu\n  mcgfixedfreq\t\t= %lu\n  mcginternalref\t= %lu\n  mcgfll\t\t= %lu\n  mcgpll0\t\t= %lu\n  mcgirc48m\t\t= %lu\n  lpo\t\t\t= %lu\n\n",
       CLOCK_GetFreq(kCLOCK_CoreSysClk),
       CLOCK_GetFreq(kCLOCK_BusClk),
       CLOCK_GetFreq(kCLOCK_FlexBusClk),
       CLOCK_GetFreq(kCLOCK_FlashClk),
       CLOCK_GetFreq(kCLOCK_PllFllSelClk),
       CLOCK_GetFreq(kCLOCK_Osc0ErClk),
       CLOCK_GetFreq(kCLOCK_Osc0ErClkUndiv),
       CLOCK_GetFreq(kCLOCK_McgFixedFreqClk),
       CLOCK_GetFreq(kCLOCK_McgInternalRefClk),
       CLOCK_GetFreq(kCLOCK_McgFllClk),
       CLOCK_GetFreq(kCLOCK_McgPll0Clk),
       CLOCK_GetFreq(kCLOCK_McgIrc48MClk),
       CLOCK_GetFreq(kCLOCK_LpoClk)
    );

    LED_B_ON();

    pin_flag_init();

    LED_C_ON();

    const xTaskHandle main_task_handle = xTaskCreateStatic(
            main_task, "main", MAIN_TASK_STACK_SIZE, NULL, TASK_PRIO_DEFAULT,
            main_task_stack, &main_task_static
    );

    //LED_C_ON();

    // Theoretically this will make sure the sub-priority on all interrupt configs is zero.
    //   Not sure it's actually really needed or what it does in the long run.
    //   See comment in FreeRTOS port.c.
    //NVIC_SetPriorityGrouping(0);

    vTaskStartScheduler();
}

static bool __pflag_set, __uflag1_set, __uflag2_set;

static void pin_flag_init(void)
{
    const port_pin_config_t port_pin_config = {
            kPORT_PullDown,
            kPORT_FastSlewRate,
            kPORT_PassiveFilterDisable,
            kPORT_OpenDrainDisable,
            kPORT_LowDriveStrength,
            kPORT_MuxAsGpio,
            kPORT_LockRegister,
    };

    const port_pin_config_t port_pin_config_out = {
            kPORT_PullDisable,
            kPORT_SlowSlewRate,
            kPORT_PassiveFilterDisable,
            kPORT_OpenDrainDisable,
            kPORT_LowDriveStrength,
            kPORT_MuxAsGpio,
            kPORT_LockRegister,
    };

    const gpio_pin_config_t gpio_pin_config = {
            .pinDirection = kGPIO_DigitalInput,
            .outputLogic = 0
    };

    const gpio_pin_config_t gpio_pin_config_out = {
            .pinDirection = kGPIO_DigitalOutput,
            .outputLogic = 1
    };


    PORT_SetPinConfig(PFLAG_PORT, PFLAG_PIN, &port_pin_config);
    GPIO_PinInit(PFLAG_GPIO, PFLAG_PIN, &gpio_pin_config);
    __pflag_set = GPIO_ReadPinInput(PFLAG_GPIO, PFLAG_PIN) != 0;

    PORT_SetPinConfig(UFLAG1_ON_PORT, UFLAG1_ON_PIN, &port_pin_config_out);
    PORT_SetPinConfig(UFLAG1_PORT, UFLAG1_PIN, &port_pin_config);
    GPIO_PinInit(UFLAG1_ON_GPIO, UFLAG1_ON_PIN, &gpio_pin_config_out);
    GPIO_PinInit(UFLAG1_GPIO, UFLAG1_PIN, &gpio_pin_config);
    __uflag1_set = GPIO_ReadPinInput(UFLAG1_GPIO, UFLAG1_PIN) != 0;
    GPIO_WritePinOutput(UFLAG1_ON_GPIO, UFLAG1_ON_PIN, 0);

    PORT_SetPinConfig(UFLAG2_ON_PORT, UFLAG2_ON_PIN, &port_pin_config_out);
    PORT_SetPinConfig(UFLAG2_PORT, UFLAG2_PIN, &port_pin_config);
    GPIO_PinInit(UFLAG2_ON_GPIO, UFLAG2_ON_PIN, &gpio_pin_config_out);
    GPIO_PinInit(UFLAG2_GPIO, UFLAG2_PIN, &gpio_pin_config);
    __uflag2_set = GPIO_ReadPinInput(UFLAG2_GPIO, UFLAG2_PIN) != 0;
    GPIO_WritePinOutput(UFLAG2_ON_GPIO, UFLAG2_ON_PIN, 0);
    PORT_SetPinMux(UFLAG2_ON_PORT,  UFLAG2_ON_PIN, UFLAG2_ON_MUX_ORIG);
}

static inline bool pflag_set(void)
{
    return __pflag_set;
}

static inline bool uflag1_set(void)
{
    return __uflag1_set;
}

static inline bool uflag2_set(void)
{
    return __uflag2_set;
}

static bool boss;
static u8 cell;
static u16 addr;

static u32 recv_count = 0, recv_bytes = 0;
static u32 send_count = 0, send_bytes = 0;

static void sync_hook(u32 chan);

static void handle_rx(u16 node, u16 peer, u16 dest, u16 size, u8 data[], s8 rssi, u8 lqi);


static void main_task(void *param)
{
    (void)param;
    LED_D_ON();

    if (!vcom_init(usb_recv)) {
        itm_puts(0, "vcom: init fail\r\n");
        goto _end;
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    LED_ABCD_ALL_OFF();


    /**
     * SPI interconnect (mcuc) testing
     */

    /*if (pflag_set()) {

        extern void spi_master_init(SPI_Type *spi);
        extern void spi_master_io(size_t size, u8 *tx, u8 *rx);

        spi_master_init(SPI1);
        u8 ch[] = { 6, 'h', 'e', 'l', 'l', 'o', '\0', '\0' };
        u8 r = 0;

        //do {
            spi_master_io(1, &ch[0], &r);
        //} while (r != 0x7e);

        for (int i = 0; i < 100000; ++i) asm("nop");

        spi_master_io(ch[0], &ch[1], &ch[1]);
        itm_printf(0, "r=0x%x ch=%s\n", r, &ch[1]);

        //itm_puts(0, "\ndone.\n");
        vTaskDelay(portMAX_DELAY);

    } else {

        extern void spi_slave_init(SPI_Type *spi);
        extern void spi_slave_io(size_t size, u8 *tx, u8 *rx);

        spi_slave_init(SPI1);

        u8 len;
        u8 ch[24];

        while (1) {
            len = 0xab;
            memset(ch, 'o', 24);

            spi_slave_io(1, &len, &len);
            spi_slave_io(len, ch, ch);

            itm_printf(0, "ch/%u=%s\n", len, ch);
        }

    }*/


    if (uflag1_set()) {
        LED_A_TOGGLE();
        LED_C_TOGGLE();

    } else {
        nphy_hook_sync(sync_hook);
    }

    /*pca9685_handle_t pca = pca9685_init(1);

    if (pca) {
        printf("<pca> initialized.\r\n");
        pca9685_set(pca, PCA9685_CHAN_ALL, 0x7D0, 0x7D0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        pca9685_set(pca, PCA9685_CHAN_ALL, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        pca9685_set(pca, 0, 1000, 500);

    } else {
        printf("<pca> error: not initialized.\r\n");
    }*/

    //xTimerHandle timer = xTimerCreate(NULL, pdMS_TO_TICKS(100), pdTRUE, NULL, timer_task);

    boss = pflag_set();

    if (uflag1_set())       cell = 0xA1;
    else if (uflag2_set())  cell = 0xA2;
    else                    cell = 0xA0;

    sim_uid_t sim_uid;
    SIM_GetUniqueId(&sim_uid);

    sim_uid.L ^= sim_uid.H;
    sim_uid.ML ^= ~sim_uid.MH;

    u32 addr_quad = sim_uid.L ^ sim_uid.ML;

    addr = ((u16)addr_quad) ^ ((u16)(addr_quad>>16)); // (u16)0x4000 | (u16)(~0x4000 & ( ((u16)addr_quad) ^ ((u16)(addr_quad>>16)) ) );

    printf("\r\nCloud Chaser %08lX%08lX@%02X.%04X\r\n\r\n", sim_uid.ML, sim_uid.L, cell, addr);

    if (nmac_init(cell, addr, boss, handle_rx)) {

        if (uflag1_set()) {
            uart_relay_run();
        }

        if (uflag2_set() && boss) {
            printf("meter: auto-tx enabled\r\n");
            #define TXLEN 45

            while (1) {
                const char to_send[TXLEN] = { [ 0 ... (TXLEN-1) ] = '\xA5' };
                nmac_send(NMAC_SEND_STRM, 0x0000, TXLEN, (u8 *)to_send);
                vTaskDelay(pdMS_TO_TICKS(23));
            }
        }

        /*while (1) {
            itm_puts(0, "<itm> usb: send periodic\n");
            const char to_send[] = "Hello Peer\r\n";
            usb_write((u8 *)to_send, strlen(to_send));
            vTaskDelay(pdMS_TO_TICKS(5000));
        }*/

        //goto _end;
        while(1) vTaskDelay(portMAX_DELAY);
    }

    _end:
    vTaskDelete(NULL);
}



typedef struct __packed {
    u8 type; // == 0x2a
    u8 data[];

} uart_pkt_t;



static uart_t uart = NULL;

static void uart_relay_run(void)
{
    struct __packed {
        uart_pkt_t pkt;
        u8 input[1];

    } message = {
            .pkt.type = 0x2a
    };

    itm_puts(0, "uart: relay enabled\r\n");

    uart = uart_init(0, 115200);

    while (1) {
        uart_read(uart, (u8 *)message.input, 1);
        //itm_printf(0, "uart: send 0x%02X\r\n", message.input[0]);
        if (nmac_send(NMAC_SEND_DGRM, 0x0000, sizeof(message), (u8 *)&message)) {
            if (!uflag2_set()) {
                LED_C_TOGGLE();
                LED_D_TOGGLE();
            }
        }
    }
}

#define UART_PACKET_MAX (MAC_PKT_SIZE_MAX - sizeof(uart_pkt_t))

static void uart_relay_send(size_t size, u8 data[])
{
    const static size_t max = UART_PACKET_MAX;
    size_t pos = 0, chunk = 0;

    struct __packed {
        uart_pkt_t pkt;
        u8 data[UART_PACKET_MAX];

    } message = {
            .pkt.type = 0x2a
    };

    //itm_puts(0, "uart: relay send\r\n");


    while (size) {
        chunk = size <= max ? size : max;

        if (chunk) {
            memcpy(message.data, &data[pos], chunk);
        }

        //itm_printf(0, "uart: relay %lu byte chunk\r\n", chunk);

        if (nmac_send(NMAC_SEND_DGRM, 0x0000, sizeof(uart_pkt_t) + chunk, (u8 *)&message)) {
            if (!uflag2_set()) {
                LED_C_TOGGLE();
                LED_D_TOGGLE();
            }
        }

        size -= chunk;
        pos += chunk;
    }
}

static void sync_hook(u32 chan)
{
    if (uflag2_set()) {
        if (chan == 11 || chan == 13 || chan == 15 || chan == 17) LED_D_ON();
        else LED_D_OFF();
    } else {
        if (chan == 11) LED_A_ON();
        else LED_A_OFF();
        if (chan == 13) LED_B_ON();
        else LED_B_OFF();
        if (chan == 15) LED_C_ON();
        else LED_C_OFF();
        if (chan == 17) LED_D_ON();
        else LED_D_OFF();
    }
}


static void write_code_recv(u16 node, u16 peer, u16 dest, size_t size, u8 data[], s8 rssi, u8 lqi);

extern bool usb_attached(void);

static void handle_rx(u16 node, u16 peer, u16 dest, u16 size, u8 data[], s8 rssi, u8 lqi)
{
    ++recv_count;
    recv_bytes += size;

    //LED_A_TOGGLE();
    //LED_B_TOGGLE();

    if (uflag1_set() && size > sizeof(uart_pkt_t) && uart) {
        uart_pkt_t *const uart_pkt = (uart_pkt_t *)data;

        if (uart_pkt->type == 0x2a) {
            if (!uflag2_set()) {
                LED_A_TOGGLE();
                LED_B_TOGGLE();
            }

            size -= sizeof(uart_pkt_t);
            //itm_printf(0, "uart: rf rx %u byte(s)\r\n", size);
            uart_write(uart, uart_pkt->data, size);

            if (usb_attached()) {
                u8 *buf = malloc(size); assert(buf);
                usb_write_direct(buf, size);
            }

            return;
        }
    }

    if (uflag2_set()) {
        if (rssi >= -34) {  // 60 dB down
            LED_A_ON();
            LED_B_ON();
            LED_C_ON();
        } else if (rssi >= -44) { // 70 dB down
            LED_A_ON();
            LED_B_ON();
            LED_C_OFF();
        } else if (rssi >= -74) { // 100 dB down
            LED_A_ON();
            LED_B_OFF();
            LED_C_OFF();
        } else if (rssi >= -89) { // 115 dB down
            LED_A_OFF();
            LED_B_OFF();
            LED_C_ON();
        } else {
            LED_A_OFF();
            LED_B_OFF();
            LED_C_OFF();
        }

        if (!boss) {
            nmac_send(NMAC_SEND_STRM, 0x0000, size, data);
            return;
        }

        return;
    }

    write_code_recv(node, peer, dest, size, data, rssi, lqi);
}



size_t frame_encode(u8 code, size_t size, u8 data[], u8 **frame);


#define CODE_ID_ECHO        0
#define CODE_ID_STATUS      1
#define CODE_ID_SEND        2
#define CODE_ID_RECV        3
#define CODE_ID_RESET       9

#define RESET_MAGIC         0xD1E00D1E

typedef struct __packed {
    u32 version;
    u64 serial;
    u32 uptime;
    u16 node;
    u32 recv_count;
    u32 recv_bytes;
    u32 send_count;
    u32 send_bytes;

} code_status_t;

typedef struct __packed {
    u16 node;
    u16 peer;
    u16 dest;
    u16 size;
    s8 rssi;
    u8 lqi;
    u8 data[];

} code_recv_t;

typedef struct __packed {
    u8 type;
    u8 flag;
    u16 node;
    u16 dest;
    u16 size;
    u8 data[];

} code_send_t;

/*
typedef enum __packed {
    CODE_RLAY_CONFIG,
    CODE_RLAY_SEND,
    CODE_RLAY_RECV

} code_relay_type_t;

typedef enum __packed {
    CODE_RLAY_FLAG_NONE,
    CODE_RLAY_FLAG_UART = 1 << 2,  // Include UART in this operation
    CODE_RELAY_FLAG_DGRM = 1 << 4,
    CODE_RELAY_FLAG_STRM = 1 << 5,
    CODE_RLAY_FLAG_BCAST = 1 << 6, // Broadcast return data instead of sending to source

} code_relay_flag_t;

typedef struct __packed {
    u8 type;
    u8 flag;
    u16 node;
    u16 dest;
    u16 size;
    u8 data[];

} code_relay_t;
*/

static void write_code_status(code_status_t *code_status)
{
    u8 *frame;
    size_t size = frame_encode(CODE_ID_STATUS, sizeof(code_status_t), (u8 *)code_status, &frame);

    if (frame) {
        //itm_printf(0, "<itm> recv: frame size=%lu\n", size);
        usb_write_direct(frame, size);
        //free(frame);
    }
}



static void write_code_recv(u16 node, u16 peer, u16 dest, size_t size, u8 data[], s8 rssi, u8 lqi)
{
    //itm_printf(0, "<itm> recv: node=0x%04X peer=0x%04X dest=0x%04X size=%lu\n", node, peer, dest, size);
    code_recv_t *code_recv = malloc(sizeof(code_recv_t) + size); assert(code_recv);

    code_recv->node = node;
    code_recv->peer = peer;
    code_recv->dest = dest;
    code_recv->size = (u16)size;
    code_recv->rssi = rssi;
    code_recv->lqi = lqi;

    memcpy(code_recv->data, data, size);
    size += sizeof(code_recv_t);

    //itm_printf(0, "<itm> recv: raw frame data size=%lu\n", size);
    u8 *frame;
    size = frame_encode(CODE_ID_RECV, size, (u8 *)code_recv, &frame);
    free(code_recv);

    if (frame) {
        //itm_printf(0, "<itm> recv: frame size=%lu\n", size);
        usb_write_direct(frame, size);
        //itm_printf(0, "<itm> recv: frame size=%lu <wrote>\n", size);
        //free(frame);
    }
}

static void handle_code_reset(size_t size, u8 *data)
{
    if (size == sizeof(u32) && *(u32 *)data == RESET_MAGIC) {
        printf("<reset>\r\n");
        vTaskDelay(pdMS_TO_TICKS(2317));
        NVIC_SystemReset();
    } else {
        printf("reset: malformed magic code\r\n");
    }
}

static void handle_code_status(size_t size, u8 *data)
{
    (void)size;
    (void)data;

    sim_uid_t sim_uid;
    SIM_GetUniqueId(&sim_uid);

    sim_uid.L ^= sim_uid.H;
    sim_uid.ML ^= ~sim_uid.MH;

    code_status_t code_status = {
            .version = 1,
            .serial = (u64)sim_uid.L | ((u64)sim_uid.ML << 32),
            .uptime = SCLK_MSEC(sclk_time()),
            .node = nmac_get_addr(),
            .recv_count = recv_count,
            .recv_bytes = recv_bytes,
            .send_count = send_count,
            .send_bytes = send_bytes
    };

    write_code_status(&code_status);
}

static void handle_code_echo(size_t size, u8 *data)
{
    //itm_printf(0, "<itm> echo: \"%s\"\n", data);
    if (data[size - 1] != '\n')
        printf("(remote) %s\r\n", data);
    else
        printf("(remote) %s", data);
}

static void handle_code_send(size_t size, u8 *data)
{
    assert(size >= sizeof(code_send_t)); assert(data);

    code_send_t *const code_send = (code_send_t *)data;

    if (code_send->size != (size - sizeof(code_send_t))) {
        printf("(send) error: size mismatch: %u != %u\r\n", code_send->size, size - sizeof(code_send_t));
        return;
    }

    if (code_send->size > MAC_PKT_SIZE_MAX) {
        printf("(send) warning: truncating size from %u to %u\r\n", size, MAC_PKT_SIZE_MAX);
        code_send->size = MAC_PKT_SIZE_MAX;
    }

    if (!code_send->node || code_send->node == nmac_get_addr()) {
        const bool result = nmac_send(
                (nmac_send_t) code_send->type,
                code_send->dest,
                code_send->size,
                code_send->data
        );

        if (result) {
            ++send_count;
            send_bytes += code_send->size;

            //LED_C_TOGGLE();
            //LED_D_TOGGLE();
        }
    }

    /*itm_printf(0, "(send) type=0x%02X dest=0x%04X size=0x%02X result=%u\r\n",
           (nmac_send_t)code_send->type,
           code_send->dest,
           pkt_size,
           result
    );*/
}


#include <usr/cobs.h>

#define SERF_CODE_PROTO_M    0xE0 // 0b11100000
#define SERF_CODE_PROTO_VAL  0xA0 // 0b10100000
#define SERF_CODE_M          0x1f // 0b00011111

typedef struct __packed {
    u8  code;
    u8  data[];

} serf_t;


static void frame_recv(size_t size, u8 *data)
{
    serf_t *const frame = (serf_t *)data;

    if ((frame->code & SERF_CODE_PROTO_M) != SERF_CODE_PROTO_VAL) {
        printf("(frame) invalid proto bits: size=%u code=0x%02x\r\n", size, frame->code);
        return;
    }

    size -= sizeof(serf_t);
    frame->code &= SERF_CODE_M;

    switch (frame->code) {
        case CODE_ID_ECHO:
            return handle_code_echo(size, frame->data);

        case CODE_ID_SEND:
            return handle_code_send(size, frame->data);

        case CODE_ID_STATUS:
            return handle_code_status(size, frame->data);

        case CODE_ID_RESET:
            return handle_code_reset(size, frame->data);

        default:
            printf("(frame) unknown code: size=%u code=0x%02x\r\n", size, frame->code);
            break;
    }
}


size_t frame_encode(u8 code, size_t size, u8 data[], u8 **frame)
{
    code = (u8)(SERF_CODE_PROTO_VAL | (code & SERF_CODE_M));
    *frame = NULL;

    size_t frame_size = sizeof(serf_t) + size;

    serf_t *raw_frame = malloc(frame_size); assert(raw_frame);
    *frame = malloc(sizeof(serf_t) + cobs_encode_size_max(frame_size) + 1); assert(*frame);

    raw_frame->code = code;
    memcpy(raw_frame->data, data, size);

    frame_size = cobs_encode((u8 *)raw_frame, frame_size, *frame);
    free(raw_frame);

    if (!frame_size) {
        itm_puts(0, "(frame) error: cobs encode failed\r\n");
        free(*frame);
        return 0;
    }

    (*frame)[frame_size] = 0;

    return frame_size + 1;
}


#define USB_IN_DATA_MAX     (MAC_PKT_SIZE_MAX + 64)

static size_t usb_in_size = 0;
static u8 usb_in_data[USB_IN_DATA_MAX];
static u8 usb_in_decode_data[USB_IN_DATA_MAX+1];

static void usb_recv(size_t size, u8 *data)
{
    //itm_printf(0, "usb: (recv) size=%lu data=0x%p\r\n", size, (void *)data);

    if (!size || !data) return;

    if (uflag1_set()) {
        uart_relay_send(size, data);
        return;
    }

    size_t i, frame_size;

    for (i = 0; i < size; ++i) {
        if (!data[i]) break;
    }

    if ((usb_in_size + size) > USB_IN_DATA_MAX) {
        printf("(usb) input buffer overflow: buffered=%u in=%u overage=%u\r\n", usb_in_size, size, (usb_in_size + size) - USB_IN_DATA_MAX);
        usb_in_size = 0;
    }

    memcpy(&usb_in_data[usb_in_size], data, i);
    usb_in_size += i;

    if (i == size)
        return;

    frame_size = i;

    // max encode length: size + 1 + (size/254) + 1/*trailing zero*/
    // max decode length: size

    size_t decoded_size = cobs_decode(usb_in_data, frame_size, usb_in_decode_data);

    usb_in_size = size - (i + 1);

    if (usb_in_size) {
        memcpy(usb_in_data, &data[i+1], usb_in_size);
    }

    if (decoded_size) {
        usb_in_decode_data[decoded_size] = 0;
        frame_recv(decoded_size, usb_in_decode_data);
    }
}


/*static inline bool isInterrupt()
{
    return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0 ;
}*/


int _write(int handle, char *buffer, int size)
{
    if (buffer == 0)
        return -1;

    if ((handle != 1) && (handle != 2))
    {
        return -1;
    }

    /*BaseType_t xHigherPriorityTaskWokenAll = pdFALSE, xHigherPriorityTaskWoken;
    const bool is_interrupt = isInterrupt();

    if (!is_interrupt) xSemaphoreTake(write_sem, portMAX_DELAY);
    else {
        while (!xSemaphoreTakeFromISR(write_sem, &xHigherPriorityTaskWoken));
        xHigherPriorityTaskWokenAll |= xHigherPriorityTaskWoken;
    }*/

    itm_write(0, (const u8 *)buffer, (size_t)size);
    usb_write((u8 *)buffer, (size_t)size);

    /*if (!is_interrupt) xSemaphoreGive(write_sem);
    else {
        xSemaphoreGiveFromISR(write_sem, &xHigherPriorityTaskWoken);
        xHigherPriorityTaskWokenAll |= xHigherPriorityTaskWoken;
    };

    if (is_interrupt) portEND_SWITCHING_ISR(xHigherPriorityTaskWokenAll)*/

    return size;
}

#include "fault.c"

// begin: sbrk.c
#include <sys/types.h>
#include <errno.h>

/*!
 * @brief Function to override ARMGCC default function _sbrk
 *
 * _sbrk is called by malloc. ARMGCC default _sbrk compares "SP" register and
 * heap end, if heap end is larger than "SP", then _sbrk returns error and
 * memory allocation failed. This function changes to compare __HeapLimit with
 * heap end.
 */
caddr_t _sbrk(int incr)
{
    extern char end __asm("end");
    extern char heap_limit __asm("__HeapLimit");
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == NULL)
        heap_end = &end;

    prev_heap_end = heap_end;

    if (heap_end + incr > &heap_limit)
    {
        errno = ENOMEM;
        return (caddr_t)-1;
    }

    heap_end += incr;

    return (caddr_t)prev_heap_end;
}



void vApplicationStackOverflowHook(TaskHandle_t xTask, const char *pcTaskName)
{
    itm_printf(0, "stack overflow in task 0x%08p '%s'\r\n", xTask, pcTaskName);
}

void vApplicationMallocFailedHook(void)
{
    itm_puts(0, "malloc failed!\r\n");
    while (1) asm("nop");
}

StaticTask_t xIdleTaskTCB; // TODO: Force in RAM
StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE]; // TODO: Force in RAM

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

static StaticTask_t xTimerTaskTCB; // TODO: Force in RAM
static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH]; // TODO: Force in RAM

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
