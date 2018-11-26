#include <kio/flsh.h>
#include <board/clock.h>
#include <board/trace.h>

#include <fsl_flash.h>
#include <FreeRTOS.h>
#include <task.h>

#include "./info.h"

static_assert(FSL_FEATURE_SIM_FCFG_HAS_PFLSH_SWAP, "FSL_FEATURE_SIM_FCFG_HAS_PFLSH_SWAP");

#define SIZE(begin, end)    ((u32)(end) - (u32)(begin))
#define OFFSET(addr, amt)   ((u32 *)((u32)(amt) + (u8 *)(addr)))
#define SWAP(x)             OFFSET(x, 0x100000)
#define USER_FLASH_SIZE     FSL_FEATURE_FLASH_PFLASH_BLOCK_SECTOR_SIZE
#define BLOCK_SIZE          FSL_FEATURE_FLASH_PFLASH_BLOCK_SECTOR_SIZE
#define BLOCK_SIZE_32       (BLOCK_SIZE / sizeof(u32))
#define FLASH_SWAP_ADDR     (((FSL_FEATURE_FLASH_PFLASH_BLOCK_SIZE * FSL_FEATURE_FLASH_PFLASH_BLOCK_COUNT) >> 1u) - BLOCK_SIZE)


static_assert(BLOCK_SIZE >= USER_FLASH_SIZE, "user flash must fit in one block");


extern u32 __user_flash_init_base[];
extern u32 __user_flash_init_end[];
extern u32 __user_flash_base[];
extern u32 __user_flash_end[];
extern u32 __user_flash_size[];
extern u32 __user_ram_base[];
extern u32 __user_ram_end[];

extern u32 __fast_text_begin[];
extern u32 __fast_text_end[];
extern u32 __fast_code_begin[];

extern u32 __text_begin[];
extern u32 __text_end[];

extern u32 __DATA_ROM[];
extern u32 __DATA_END[];

extern u32 __VECTOR_TABLE[];
extern u32 __VECTOR_RAM[];
extern u32 __RAM_VECTOR_TABLE_SIZE_BYTES[];

extern u32 __flash_header_begin[];
extern u32 __flash_header_end[];

extern u32 __all_rom_begin[];
extern u32 __all_rom_end[];

static user_flash_t *user_flsh;
static user_flash_t *user_flsh_rom;
static user_flash_t *user_flsh_othr;
static user_flash_t user_flsh_ram __section(".user.base") = {
        .sanity = FLASH_SANITY,
        .version = FLASH_VERSION,
};

static flash_config_t flash_config;
static ftfx_config_t *ftfx_config;

const flsh_date_t __flsh_date __used = CONFIG_FLASH_TIMESTAMP;
const flsh_version_t __flsh_version __used = CONFIG_FLASH_VERSION;


static inline status_t flsh_erase(u32 begin[], u32 end[]);
static inline status_t flsh_ewrite(u32 *begin, u32 *end, u32 dest[]);
static inline status_t flsh_write(u32 *begin, u32 *end, u32 dest[]);

static status_t flsh_scpy(const char name[], u32 begin[], u32 end[]);

static status_t get_swap_state_config(ftfx_swap_state_config_t *swap_state_config)__used;

static void delay()
{
    #if DEBUG || CONFIG_DISABLE_BOARD_TRACE != 1
    for (u32 i = 0; i < 1000000; ++i) asm("");
    #endif
}

status_t flsh_init(void)
{
    status_t status = 0;

    FLASH_Init(&flash_config);

    ftfx_config = flash_config.ftfxConfig;

    user_flsh_rom = (user_flash_t *) __user_flash_init_base;
    user_flsh = (user_flash_t *) __user_flash_base;
    user_flsh_othr = (user_flash_t *) SWAP(__user_flash_base);

    wcpy(__user_flash_base, __user_flash_end, __user_ram_base);

    board_trace_f(
            "user: addr=0x%X  s=0x%08X v=0x%08X ssz=0x%02X dsz=0x%02X ",
            (u32)__user_flash_base,
            user_flsh->sanity,
            user_flsh->version,
            USER_FLASH_SIZE,
            (u32)__user_flash_end - (u32)__user_flash_base
    );

    board_trace("");
    board_trace(  "section measurements:")
    board_trace_f("  header        0x%05X-0x%05X    %u", __flash_header_begin, __flash_header_end, (u32) __flash_header_end - (u32) __flash_header_begin);
    board_trace_f("  .user <local> 0x%05X-0x%05X    %u", __user_flash_base, __user_flash_end, (u32) __user_flash_end - (u32) __user_flash_base);
    board_trace_f("  .user_rom     0x%05X-0x%05X    %u", __user_flash_init_base, __user_flash_init_end, (u32) __user_flash_init_end - (u32) __user_flash_init_base);
    board_trace_f("  .fast_code    0x%05X-0x%05X    %u", __fast_text_begin, __fast_text_end, (u32) __fast_text_end - (u32) __fast_text_begin);
    board_trace_f("  .text         0x%05X-0x%05X    %u", __text_begin, __text_end, (u32) __text_end - (u32) __text_begin);
    board_trace_f("  .data         0x%05X-0x%05X    %u", __DATA_ROM, __DATA_END, (u32) __DATA_END - (u32) __DATA_ROM);
    board_trace("");


    if (user_flsh_ram.sanity != user_flsh_rom->sanity || user_flsh_ram.version != user_flsh_rom->version) {
        board_trace_fr("update to s=0x%08X v=0x%08X ... ", user_flsh_rom->sanity, user_flsh_rom->version);

        wcpy(__user_flash_init_base, __user_flash_init_end, __user_ram_base);

        if ((status = flsh_ewrite(__user_flash_init_base, __user_flash_init_end, __user_flash_base))) {
            return status;
        } else {
            board_trace("done.");
        }
    }

    board_trace_fr(
            "othr: addr=0x%X s=0x%08X v=0x%08X ",
            user_flsh_othr, user_flsh_othr->sanity, user_flsh_othr->version
    );

    if (user_flsh_othr->sanity == user_flsh_ram.sanity) {

        board_trace("<valid>");

    } else {

        board_trace("<invalid>");

        if ((status = flsh_erase(SWAP(__flash_header_begin), SWAP(__flash_header_end)))) return status;
        if ((status = flsh_erase(SWAP(__user_flash_base),  SWAP(__user_flash_end)))) return status;
        if ((status = flsh_erase(SWAP(__all_rom_begin),  SWAP(__all_rom_end)))) return status;

        if ((status = flsh_scpy("header", __flash_header_begin, __flash_header_end))) return status;
        if ((status = flsh_scpy(".user_rom", __user_flash_init_base, __user_flash_init_end))) return status;
        if ((status = flsh_scpy(".fast_code", __fast_text_begin, __fast_text_end))) return status;
        if ((status = flsh_scpy(".text", __text_begin, __text_end))) return status;
        if ((status = flsh_scpy(".data", __DATA_ROM, __DATA_END))) return status;

        if ((status = flsh_scpy(".user <local>", __user_flash_base, __user_flash_end))) return status;
        
        delay();

        board_trace_r("swapping... ");

        if (user_flsh_othr->sanity == user_flsh_ram.sanity) {
            if ((status = FLASH_Swap(&flash_config, FLASH_SWAP_ADDR, true))) {
                board_trace_f("swap fail: s=%u", status);
            } else {
                //board_trace("success.")
                board_trace("success.\nreboot....\r\n\r\n\r\n");
                delay();
                //__BKPT(0);
                NVIC_SystemReset();
            }
        } else {
            board_trace("error: mark not set!");
            __BKPT(0);
        }
    }

    delay();
    return status;
}


static status_t flsh_scpy(const char name[], u32 begin[], u32 end[])
{
    board_trace_fr("copy %s %u ... ", name, (u32) end - (u32) begin);

    status_t status = flsh_write(begin, end, SWAP(begin));

    if (status) {
        return status;
    } else {
        board_trace("done.");
    }

    return status;
}

status_t flsh_erase(u32 begin[], u32 end[])
{
    size_t size = (u32) end - (u32) begin;

    if (size % 4) return -101;
    if (size % 16) size = size + 16 - (size % 16);

    status_t status = FLASH_Erase(&flash_config, (u32)begin, size, kFLASH_ApiEraseKey);

    if (status) {
        board_trace_f("erase fail: s=%li", status);
    }

    return status;
}

status_t flsh_ewrite(u32 *begin, u32 *end, u32 dest[])
{
    status_t status = flsh_erase(dest, OFFSET(dest, SIZE(begin, end)));

    if (!status) {
        return flsh_write(begin, end, dest);
    }

    return status;
}

status_t flsh_write(u32 *begin, u32 *end, u32 dest[])
{
    size_t size = SIZE(begin, end);

    //board_trace_fr("\nwrite %u tp 0x%08X ... ", size, dest);

    status_t status = FLASH_Program(&flash_config, (u32) dest, (u8 *) begin, size);

    if (status != kStatus_FLASH_Success) {
        board_trace_f("write fail: %li", status);
    } else {
        while (begin < end) {
            if (*begin++ != *dest++) {
                board_trace_f("verify fail: wrote=0x%X@0x%X != read=0x%X@0x%X", *begin, begin, *dest, dest);
                return -2;
            }
        }
    }


    return status;
}



status_t flsh_user_cmit(void)
{
    status_t status;

    vTaskSuspendAll();
    boot_clock_run();
    {
        status = flsh_ewrite(__user_ram_base, __user_ram_end, __user_flash_base);
    }
    boot_clock_run_hs_oc();
    (void) xTaskResumeAll();

    return status;
}


bool flsh_fota(net_t net, net_path_t path)
{
    const static size_t PART_SIZE = 0x1000;

    if (path.addr == NET_ADDR_BCST || path.addr == NET_ADDR_INVL)
        return false;

    flsh_size_t size = {
            .header = SIZE(__flash_header_begin, __flash_header_end),
            .user_rom = SIZE(__user_flash_init_base, __user_flash_init_end),
            .fast_code = SIZE(__fast_text_begin, __fast_text_end),
            .text = SIZE(__text_begin, __text_end),
            .data = SIZE(__DATA_ROM, __DATA_END)
    };

    size.total = size.header + size.user_rom + size.fast_code + size.text + size.data;

    mbuf_t mbuf = mbuf_make(size.total + sizeof(flsh_size_t), sizeof(flsh_size_t), (u8 *) &size);

    mbuf_used(&mbuf, mbuf->size);

    u8 *data = mbuf->data + sizeof(flsh_size_t);

    memcpy(data, __flash_header_begin, size.header);

    data += size.header;

    memcpy(data, __user_flash_init_base, size.user_rom);

    data += size.user_rom;

    memcpy(data, __fast_text_begin, size.fast_code);

    data += size.fast_code;

    memcpy(data, __text_begin, size.text);

    data += size.text;

    memcpy(data, __DATA_ROM, size.data);

    mbuf_t mbuf_part = mbuf_alloc(PART_SIZE, NULL);

    size_t size_part;

    do {
        size_part = MIN(mbuf->used, PART_SIZE);

        mbuf_used(&mbuf_part, size_part);
        mbuf_popf(&mbuf, size_part, mbuf_part->data);

        if (!net_mesg(net, path, &mbuf_part)) {
            board_trace("fota: tx failed!");
            break;
        }

    } while (mbuf->used);

    bool success = !mbuf->used;

    mbuf_free(&mbuf_part);
    mbuf_free(&mbuf);

    return success;
}


status_t flsh_updt_init(flsh_size_t *size)
{
    status_t status = 0;

    delay();

    vTaskSuspendAll();
    boot_clock_run();
    {
        //if (!status) status = flsh_erase(SWAP(__flash_header_begin), SWAP(__all_rom_end));

        if (!status) status = flsh_erase(SWAP(__flash_header_begin), SWAP(OFFSET(__flash_header_begin, size->header)));
        if (!status) status = flsh_erase(SWAP(__user_flash_base), SWAP(OFFSET(__user_flash_base, size->user_rom)));
        if (!status) status = flsh_erase(SWAP(__user_flash_init_base), SWAP(OFFSET(__user_flash_init_base, size->user_rom)));
        if (!status) status = flsh_erase(SWAP(__fast_text_begin), SWAP(OFFSET(__fast_text_begin, size->fast_code)));

        u32 text_end = (u32)OFFSET(__text_begin, size->text);
        u32 *data_begin = (u32 *) (text_end % 16 ? text_end + 16 - (text_end % 16) : text_end);

        if (!status) status = flsh_erase(SWAP(__text_begin), SWAP(text_end));
        if (!status) status = flsh_erase(SWAP(data_begin), SWAP(OFFSET(data_begin, size->data)));
    }
    delay();
    boot_clock_run_hs_oc();
    (void) xTaskResumeAll();

    return status;
}


status_t flsh_updt_part_1(mbuf_t header, mbuf_t user_rom)
{
    status_t status;

    delay();

    vTaskSuspendAll();
    boot_clock_run();
    {
        if (header->used == SIZE(__flash_header_begin, __flash_header_end)) {

            status = flsh_write(
                    (u32 *) header->data,
                    (u32 *) (header->data + header->used),
                    SWAP(__flash_header_begin)
            );

        } else {
            board_trace_f("header size mismatch %u != %u", header->used, (u32)__flash_header_end - (u32)__flash_header_begin);
            status = -101;
        }

        if (status) goto _done;

        if (user_rom->used >= sizeof(user_flash_t)) {

            if (user_rom->used == SIZE(__user_flash_base, __user_flash_end)) {

                user_flash_t *user_flash_rom = (user_flash_t *) user_rom->data;

                if (user_flash_rom->sanity == user_flsh->sanity && user_flash_rom->version == user_flsh->version) {

                    // compatible, copy current user data over
                    if ((status = flsh_write(__user_flash_base, __user_flash_end, SWAP(__user_flash_base))))
                        goto _done;
                } else {

                    // incompatible, init to rom values
                    if ((status = flsh_write((u32 *) user_rom->data, (u32 *) (user_rom->data + user_rom->used), SWAP(__user_flash_base))))
                        goto _done;
                }

            }

            status = flsh_write(
                    (u32 *) user_rom->data,
                    (u32 *) (user_rom->data + user_rom->used),
                    SWAP(__user_flash_init_base)
            );

        } else {
            board_trace_f("user_rom size too small %u < %u", user_rom->used, sizeof(user_flash_t));
            status = -102;
        }

    }
    _done:
    delay();
    boot_clock_run_hs_oc();
    (void) xTaskResumeAll();

    return status;
}


status_t flsh_updt_part_2(mbuf_t fast_code, mbuf_t text, mbuf_t data)
{
    status_t status;

    delay();

    vTaskSuspendAll();
    boot_clock_run();
    {
        status = flsh_write((u32 *) fast_code->data, (u32 *) (fast_code->data + fast_code->used), SWAP(__fast_text_begin));

        u32 text_end = (u32)OFFSET(__text_begin, text->used);
        u32 *data_begin = (u32 *) (text_end % 16 ? text_end + 16 - (text_end % 16) : text_end);

        if (!status)
            status = flsh_write((u32 *) text->data, (u32 *) (text->data + text->used), SWAP(__text_begin));

        if (!status)
            status = flsh_write((u32 *) data->data, (u32 *) (data->data + data->used), SWAP(data_begin));

    }
    delay();
    boot_clock_run_hs_oc();
    (void) xTaskResumeAll();

    return status;
}


status_t flsh_updt_done(u32 sanity)
{
    status_t status;

    delay();

    vTaskSuspendAll();
    boot_clock_run();
    {
        if (user_flsh_othr->sanity == sanity) {
            if ((status = FLASH_Swap(&flash_config, FLASH_SWAP_ADDR, true))) {
                board_trace_f("swap fail: s=%u", status);
            }
        } else {
            board_trace("error: mark not set!");
            status = -1001;
        }
    }
    boot_clock_run_hs_oc();
    (void) xTaskResumeAll();

    return status;
}


static status_t get_swap_state_config(ftfx_swap_state_config_t *swap_state_config)
{
    status_t status = FTFx_CMD_SwapControl(ftfx_config, FLASH_SWAP_ADDR, kFTFx_SwapControlOptionReportStatus, swap_state_config);

    if (status) {
        board_trace_f("swap: read config fail. s=%u", status);
    }

    return status;
}
