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
#include <cc/amp.h>
#include <cc/cfg.h>
#include <fsl_port.h>
#include <semphr.h>
#include <cc/nmac.h>
#include <fsl_enet.h>
#include <fsl_sim.h>
#include <virtual_com.h>
#include <malloc.h>


#define PFLAG_PORT PORTB
#define PFLAG_GPIO GPIOB
#define PFLAG_PIN  5


static void main_task(void *param);
static void usb_recv(size_t size, u8 *data);

static SemaphoreHandle_t write_sem = NULL;

// For the GDB helper
bool dbgPendSVHookState = 0;

int main(void)
{
    BOARD_InitPins();
    LED_A_ON();

    BOARD_BootClockRUN();
    LED_B_ON();

    write_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(write_sem);

    BOARD_InitDebugConsole();
    itm_init();
    printf("<boot>\r\n");

    /*printf("how about a fault.\r\n");
    *(int *)(0xfffefafe) = 42;
    while (1) {};*/


    printf("\nclocks:\n  core\t\t\t= %lu\n  bus\t\t\t= %lu\n  flexbus\t\t= %lu\n  flash\t\t\t= %lu\n  pllfllsel\t\t= %lu\n  osc0er\t\t= %lu\n  osc0erundiv\t\t= %lu\n  mcgfixedfreq\t\t= %lu\n  mcginternalref\t= %lu\n  mcgfll\t\t= %lu\n  mcgpll0\t\t= %lu\n  mcgirc48m\t\t= %lu\n  lpo\t\t\t= %lu\n\n",
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

    // setup the rx/tx flag input pin
    const port_pin_config_t port_pin_config = {
            kPORT_PullDown,
            kPORT_FastSlewRate,
            kPORT_PassiveFilterDisable,
            kPORT_OpenDrainDisable,
            kPORT_LowDriveStrength,
            kPORT_MuxAsGpio,
            kPORT_LockRegister,
    };

    PORT_SetPinConfig(PFLAG_PORT, PFLAG_PIN, &port_pin_config);

    const gpio_pin_config_t gpio_pin_config = {
            .pinDirection = kGPIO_DigitalInput,
            .outputLogic = 0
    };

    GPIO_PinInit(PFLAG_GPIO, PFLAG_PIN, &gpio_pin_config);


    xTaskCreate(main_task, "main", TASK_STACK_SIZE_DEFAULT, NULL, TASK_PRIO_HIGHEST, NULL);

    LED_C_ON();

    // Theoretically this will make sure the sub-priority on all interrupt configs is zero.
    //   Not sure it's actually really needed or what it does in the long run.
    //   See comment in FreeRTOS port.c.
    //NVIC_SetPriorityGrouping(0);

    vTaskStartScheduler();
}

static bool pflag_set(void)
{
    return GPIO_ReadPinInput(PFLAG_GPIO, PFLAG_PIN) != 0;
}


static pit_t xsec_timer_0;
static pit_t xsec_timer;

u32 sync_timestamp(void)
{
    return pit_get_elapsed(xsec_timer);
}

static bool boss = false;
static u16 addr = 0;

static u32 recv_count = 0, recv_bytes = 0;
static u32 send_count = 0, send_bytes = 0;

#if 0
static void handle_rx(u8 flag, u8 *data, u8 size);
#else
static void handle_rx(u16 node, u16 peer, u16 dest, u16 size, u8 data[]);
#endif

static void main_task(void *param)
{
    (void)param;
    //printf("<main task>\r\n");

    if (!vcom_init(usb_recv)) {
        printf("vcom: init fail\r\n");
        goto _end;
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    //xTimerHandle timer = xTimerCreate(NULL, pdMS_TO_TICKS(100), pdTRUE, NULL, timer_task);

    #if 1

    amp_init(0);

    xsec_timer_0 = pit_alloc(&(pit_cfg_t){
            .period = pit_nsec_tick(1000000)
    });

    xsec_timer = pit_chain(xsec_timer_0, &(pit_cfg_t){
            .period = UINT32_MAX
    });

    pit_start(xsec_timer);
    pit_start(xsec_timer_0);

    #define MSG_LEN 88//38//88//48//38

    amp_ctrl(0, AMP_LNA, true);
    amp_ctrl(0, AMP_PA, true);
    amp_ctrl(0, AMP_HGM, false);

    boss = pflag_set();
    //addr = (u16)(transmitter ? 2 : 1);

    sim_uid_t sim_uid;
    SIM_GetUniqueId(&sim_uid);

    addr =  (u16)0x4000 | (u16)(~0x4000 & ( ((u16)sim_uid.L) ^ ((u16)(sim_uid.L>>16)) ) );

    printf("address: 0x%04X L=0x%08lX ML=0x%08lX MH=0x%08lX\r\n", addr, sim_uid.L, sim_uid.ML, sim_uid.MH);

    #if 0

    if (nphy_init((nphy_rx_t)handle_rx, boss)) {
        printf("nphy init successful.\r\n");


        if (!transmitter) {
            printf("mode: receive\r\n");

            u8 *buf;
            u8 len;

            //while (1) {
            //    if (nphy_recv(&buf, &len, portMAX_DELAY)) {
            //        handle_rx(0, buf, len);
            //        nphy_free_buf(buf);
            //    }
            //}

            vTaskDelay(portMAX_DELAY);

        } else {
            printf("mode: transmit\r\n");

            u8 data[MSG_LEN] = {'a'};

            u32 start_time = sync_timestamp();
            u32 sum_lengths = 0;
            u32 num_packets = 0;
            u32 time_diff;

            u8 pkt_len = MSG_LEN;

            *((u8 *)data) = 0;

            while (1) {
                //pkt.chn = (u8) chan_cur;
                //nphy_tx(0, data, (u8)(pkt_len % 3 ? pkt_len : 3));
                nphy_tx(PHY_PKT_FLAG_IMMEDIATE, data, pkt_len);
                //printf("tx/%lu: seq=%u\r\n", chan_cur, pkt.seq);
                ++(*((u8 *)data));

                sum_lengths += (u8)(pkt_len);// % 3 ? pkt_len : 3);// + 3 + /*being generous: 2 sync, 2 preamble*/4;
                num_packets++;
                time_diff = sync_timestamp() - start_time;

                if (time_diff >= 1001) {
                    printf("tx: rate = %lu bps\t\t\tpkts = %lu\r\n", (1000 * (sum_lengths * 8)) / time_diff, num_packets);
                    num_packets = 0;
                    sum_lengths = 0;
                    start_time = sync_timestamp();
                }

                //if (++pkt_len > MSG_LEN) pkt_len = 4;

                //vTaskDelay(pdMS_TO_TICKS(5));

                //const u32 pkt_time = cc_get_tx_time(0, pkt.len);

                //pkt.len = (u8)((pkt.len + 1) % (MSG_LEN + 2));
                //if (!pkt.len) pkt.len = 2;

                LED_D_TOGGLE();
                //vTaskDelay(pdMS_TO_TICKS(/*1137*/20 + (pkt.len % 2)*3 ));
                //vTaskDelay(pdMS_TO_TICKS(pkt_time*3));
            }

        }
    }

    #elif 0

    if (nmac_init(addr, boss, handle_rx)) {
        printf("nphy init successful.\r\n");

        struct {
            u32 seq;
            u32 magic;
            u8 data[MSG_LEN-8];

        } my_packet = {0};



        if (!transmitter) {
            printf("mode: receive\r\n");

            vTaskDelay(portMAX_DELAY);

        } else {
            printf("mode: transmit\r\n");

            u32 start_time = sync_timestamp();
            u32 sum_lengths = 0;
            u32 num_packets = 0;
            u32 time_diff;

            u8 pkt_len = MSG_LEN;

            while (1) {
                my_packet.seq++;
                my_packet.magic = ~my_packet.seq;
                nmac_send(NMAC_SEND_TRXN, 0, pkt_len, (u8 *)&my_packet);
                //printf("tx/%lu: seq=%u\r\n", chan_cur, pkt.seq);

                sum_lengths += pkt_len;// + 3 + /*being generous: 2 sync, 2 preamble*/4;
                num_packets++;
                time_diff = sync_timestamp() - start_time;

                if (time_diff >= 1000) {
                    printf("tx: rate = %lu bps \t\t pkts = %lu\r\n", (1000 * (sum_lengths * 8)) / time_diff, num_packets);
                    num_packets = 0;
                    sum_lengths = 0;
                    start_time = sync_timestamp();
                }

                //if (++pkt_len > MSG_LEN) pkt_len = 4;

                LED_D_TOGGLE();

                //vTaskDelay(pdMS_TO_TICKS(100));
            }

        }
    }

    #else

    if (nmac_init(addr, boss, handle_rx)) {

        /*while (1) {
            itm_puts(0, "<itm> usb: send periodic\n");
            const char to_send[] = "Hello Peer\r\n";
            usb_write((u8 *)to_send, strlen(to_send));
            vTaskDelay(pdMS_TO_TICKS(5000));
        }*/

        //goto _end;
        while(1) vTaskDelay(portMAX_DELAY);
    }

    #endif

    #endif

    _end:
    vTaskDelete(NULL);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, const char *pcTaskName)
{
    itm_printf(0, "stack overflow in task '%s'\r\n", pcTaskName);
}


/*static u32 start_time = 0;
static u32 sum_lengths = 0;
static u32 num_packets = 0;
static u32 time_diff;*/



static void write_code_recv(u16 node, u16 peer, u16 dest, size_t size, u8 data[]);

#if 0
static void handle_rx(u8 flag, u8 *data, u8 size)
#else
static void handle_rx(u16 node, u16 peer, u16 dest, u16 size, u8 data[])
#endif
{
    ++recv_count;
    recv_bytes += size;

    //itm_printf(0, "recv: total=%lu\r\n", recv_total);


    /*sum_lengths += size;
    num_packets++;
    if (!start_time) start_time = sync_timestamp();
    time_diff = sync_timestamp() - start_time;

    if (time_diff >= 1000) {
        printf("rx: rate = %lu bps \t\t pkts = %lu\r\n", (1000 * (sum_lengths * 8)) / time_diff, num_packets);
        num_packets = 0;
        sum_lengths = 0;
        start_time = 0;

    }*/

    LED_D_TOGGLE();

    write_code_recv(node, peer, dest, size, data);
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


static void write_code_status(code_status_t *code_status)
{
    u8 *frame;
    size_t size = frame_encode(CODE_ID_STATUS, sizeof(code_status_t), (u8 *)code_status, &frame);

    if (frame) {
        //itm_printf(0, "<itm> recv: frame size=%lu\n", size);
        usb_write_direct(frame, size);
        free(frame);
    }
}



static void write_code_recv(u16 node, u16 peer, u16 dest, size_t size, u8 data[])
{
    //itm_printf(0, "<itm> recv: node=0x%04X peer=0x%04X dest=0x%04X size=%lu\n", node, peer, dest, size);
    code_recv_t *code_recv = malloc(sizeof(code_recv_t) + size); assert(code_recv);

    code_recv->node = node;
    code_recv->peer = peer;
    code_recv->dest = dest;
    code_recv->size = (u16)size;

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
        free(frame);
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

    code_status_t code_status = {
            .version = 1,
            .serial = (u64)sim_uid.L | ((u64)sim_uid.ML << 32),
            .uptime = sync_timestamp(),
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
        printf("(frame) SIZE=%u CODE=0x%02x -- BAD PROTO BITS\r\n", size, frame->code);
        return;
    }

    size -= sizeof(serf_t);
    frame->code &= SERF_CODE_M;

    switch (frame->code) {
        default:
        case CODE_ID_ECHO:
            return handle_code_echo(size, frame->data);

        case CODE_ID_SEND:
            return handle_code_send(size, frame->data);

        case CODE_ID_STATUS:
            return handle_code_status(size, frame->data);

        case CODE_ID_RESET:
            return handle_code_reset(size, frame->data);
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


static size_t usb_in_size = 0;
static u8 *usb_in_data = NULL;

static void usb_recv(size_t size, u8 *data)
{
    //printf("usb: (recv) size=%u data=0x%p\r\n", size, (void *)data);

    if (!size || !data) return;

    size_t i, frame_size;

    for (i = 0; i < size; ++i) {
        if (!data[i]) break;
    }

    if (i == size) {
        //itm_puts(0, "usb: (recv) no mark found\r\n");

        usb_in_data = realloc(usb_in_data, usb_in_size + size); assert(usb_in_data);
        memcpy(&usb_in_data[usb_in_size], data, size);
        usb_in_size += size;
        return;
    }

    frame_size = i;
    //printf("usb: (recv) frame_size=%u\r\n", frame_size);

    if (usb_in_size) {
        usb_in_data = realloc(usb_in_data, usb_in_size + frame_size);
        memcpy(&usb_in_data[usb_in_size], data, frame_size);
        frame_size += usb_in_size;
        usb_in_size = frame_size;
        //printf("usb: (recv) frame_size=%u (updated)\r\n", frame_size);

    } else {
        if (usb_in_data) {
            free(usb_in_data);
            usb_in_data = NULL;
        }

        usb_in_data = data;
    }

    // max encode length: size + 1 + (size/254) + 1/*trailing zero*/
    // max decode length: size

    u8 *decoded = malloc(size + 1); assert(decoded);
    size_t decoded_size = cobs_decode(usb_in_data, frame_size, decoded);

    if (usb_in_data != data) {
        free(usb_in_data);
    }

    usb_in_data = NULL;
    usb_in_size = 0;

    if (decoded_size) {
        decoded[decoded_size] = 0;
        frame_recv(decoded_size, decoded);
        free(decoded);
    }

    if ((size - 1) > i) {

        // copy tail to usb_in_data
        //usb_in_size = size - i;
        //usb_in_data = realloc(usb_in_data, usb_in_size); assert(usb_in_data);
        //memcpy(usb_in_data, &data[i], usb_in_size);
        printf("usb: (recv) dropping %u tail byte(s): size=%u frame_size=%u usb_in_size=%u\r\n", size - i, size, frame_size, usb_in_size);
    }
}


static inline bool isInterrupt()
{
    return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0 ;
}


int _write(int handle, char *buffer, int size)
{
    if (buffer == 0)
        return -1;

    if ((handle != 1) && (handle != 2))
    {
        return -1;
    }

    BaseType_t xHigherPriorityTaskWokenAll = pdFALSE, xHigherPriorityTaskWoken;
    const bool is_interrupt = isInterrupt();

    if (!is_interrupt) xSemaphoreTake(write_sem, portMAX_DELAY);
    else {
        while (!xSemaphoreTakeFromISR(write_sem, &xHigherPriorityTaskWoken));
        xHigherPriorityTaskWokenAll |= xHigherPriorityTaskWoken;
    }

    itm_write(0, (const u8 *)buffer, (size_t)size);
    usb_write((u8 *)buffer, (size_t)size);

    if (!is_interrupt) xSemaphoreGive(write_sem);
    else {
        xSemaphoreGiveFromISR(write_sem, &xHigherPriorityTaskWoken);
        xHigherPriorityTaskWokenAll |= xHigherPriorityTaskWoken;
    };

    if (is_interrupt) portEND_SWITCHING_ISR(xHigherPriorityTaskWokenAll)

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
