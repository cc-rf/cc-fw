#include <kio/flsh.h>
#include <board/clock.h>
#include <board/trace.h>
#include <fsl_flash.h>
#include <fsl_flash.h>
#include <FreeRTOS.h>
#include <task.h>

static_assert(FSL_FEATURE_SIM_FCFG_HAS_PFLSH_SWAP, "FSL_FEATURE_SIM_FCFG_HAS_PFLSH_SWAP");

#define SIZE(begin, end)    ((u32)(end) - (u32)(begin))
#define OFFSET(addr, amt)   ((u32 *)((amt) + (u8 *)(addr)))
#define SWAP(x)             OFFSET(x, 0x100000)
#define USER_FLASH_SIZE     FSL_FEATURE_FLASH_PFLASH_BLOCK_SECTOR_SIZE
#define BLOCK_SIZE          FSL_FEATURE_FLASH_PFLASH_BLOCK_SECTOR_SIZE
#define BLOCK_SIZE_32       (BLOCK_SIZE / sizeof(u32))
#define FLASH_SWAP_ADDR     (((FSL_FEATURE_FLASH_PFLASH_BLOCK_SIZE * FSL_FEATURE_FLASH_PFLASH_BLOCK_COUNT) >> 1u) - BLOCK_SIZE)


static_assert(BLOCK_SIZE >= USER_FLASH_SIZE, "user flash must fit in one block");


typedef struct {
    u32 sanity;
    u32 version;

    u8 data[];

} user_flash_t;


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

static status_t get_swap_state_config(ftfx_swap_state_config_t *swap_state_config)__used;

static status_t flsh_scpy(const char name[], u32 begin[], u32 end[]);

static void delay()
{
    #if DEBUG
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

    status_t status = flsh_write(begin, end, (u32 *) SWAP(begin));

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
    status_t status = FLASH_Program(&flash_config, (u32) dest, (u8 *) begin, SIZE(begin, end));

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

    taskENTER_CRITICAL();
    //portDISABLE_INTERRUPTS();

    boot_clock_run();
    itm_init();

    status = flsh_ewrite(__user_ram_base, __user_ram_end, __user_flash_base);

    boot_clock_run_hs_oc();
    itm_init();

    //portENABLE_INTERRUPTS();
    taskEXIT_CRITICAL();

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
