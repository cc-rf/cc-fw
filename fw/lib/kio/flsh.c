#include <kio/flsh.h>
#include <board/trace.h>
#include <fsl_flash.h>
#include <fsl_flash.h>
#include <FreeRTOS.h>


static_assert(FSL_FEATURE_SIM_FCFG_HAS_PFLSH_SWAP, "FSL_FEATURE_SIM_FCFG_HAS_PFLSH_SWAP");


#define OFFSET(x)           ((u32)(0x100000 + (u8 *)(x)))
#define USER_FLASH_SIZE     FSL_FEATURE_FLASH_PFLASH_BLOCK_SECTOR_SIZE
#define BLOCK_SIZE          FSL_FEATURE_FLASH_PFLASH_BLOCK_SECTOR_SIZE
#define BLOCK_SIZE_32       (BLOCK_SIZE / sizeof(u32))
#define FLASH_SWAP_ADDR     (((FSL_FEATURE_FLASH_PFLASH_BLOCK_SIZE * FSL_FEATURE_FLASH_PFLASH_BLOCK_COUNT) >> 1u) - BLOCK_SIZE)

#define FLASH_SANITY        0xB10B510B
#define FLASH_VERSION       0xDADFECE5


static_assert(BLOCK_SIZE >= USER_FLASH_SIZE, "user flash must fit in one block");


typedef struct {
    u32 sanity;
    u32 version;

    u8 data[];

} user_flash_t;


extern u32 __user_flash_base[];
extern u32 __user_flash_end[];
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


static const user_flash_t user_flash_locl __section(".user.base") = {
        .sanity = FLASH_SANITY,
        .version = FLASH_VERSION,
};

static user_flash_t *user_flash;
//static user_flash_t *user_flash_ram;
static user_flash_t *user_flash_othr;

static flash_config_t flash_config;
static ftfx_config_t *ftfx_config;

static status_t flsh_bcpy(u32 *src, u32 *end, u32 *dst);
static void print_swap_state(ftfx_swap_state_t swap_state);
static status_t get_swap_state_config(ftfx_swap_state_config_t *swap_state_config);


status_t flsh_init(void)
{
    status_t status = 0;

    FLASH_Init(&flash_config);

    ftfx_config = flash_config.ftfxConfig;

    wcpy(__user_flash_base, __user_flash_end, __user_ram_base);

    user_flash = (user_flash_t *) __user_flash_base;
    user_flash_othr = (user_flash_t *) OFFSET(__user_flash_base);

    board_trace_f(
            "user: addr=0x%X  s=0x%08X v=0x%08X ssz=0x%02X dsz=0x%02X ",
            (u32)__user_flash_base,
            user_flash->sanity,
            user_flash->version,
            USER_FLASH_SIZE,
            (u32)__user_flash_end - (u32)__user_flash_base
    );

    if (user_flash->sanity != FLASH_SANITY || user_flash->version != FLASH_VERSION) {
        board_trace_fr("updating to s=0x%08X v=0x%08X ... ", user_flash_locl.sanity, user_flash_locl.version);

        if ((status = flsh_write((u32) __user_flash_base, (u32) __user_flash_end - (u32) __user_flash_base, (u32 *) &user_flash_locl))) {
            return status;
        } else {
            board_trace("done.");
        }
    }

    board_trace_fr(
            "othr: addr=0x%X s=0x%08X v=0x%08X ",
            user_flash_othr, user_flash_othr->sanity, user_flash_othr->version
    );

    if (user_flash_othr->sanity == FLASH_SANITY) {

        board_trace("<valid>");

    } else {


        board_trace("<invalid>");
        board_trace_fr("copy flash header s=0x%X ... ", (u32)__flash_header_end - (u32)__flash_header_begin);
        
        if ((status = flsh_write(OFFSET(__flash_header_begin), (u32)__flash_header_end - (u32)__flash_header_begin, __flash_header_begin))) {
            return status;
        } else {
            board_trace("done.");
        }

        if (1) {
            board_trace_fr("copy text/data s=%u ... ",
                           (u32) __DATA_END - (u32) __user_flash_base);

            if ((status = flsh_bcpy(__user_flash_base, __DATA_END, (u32 *) OFFSET(__user_flash_base)))) {
                return status;
            } else {
                board_trace("done.");
            }

        } else {

            board_trace_fr("copy .fast_code s=%u ... ",
                           (u32) __fast_text_end - (u32) __fast_text_begin);

            if ((status = flsh_bcpy(__fast_text_begin, __fast_text_end, (u32 *) OFFSET(__fast_text_begin)))) {
                return status;
            } else {
                board_trace("done.");
            }


            board_trace_fr("copy .text s=%u ... ", (u32) __text_end - (u32) __text_begin);

            if ((status = flsh_bcpy(__text_begin, __text_end, (u32 *) OFFSET(__text_begin)))) {
                return status;
            } else {
                board_trace("done.");
            }

            board_trace_fr("copy .data s=%u ... ", (u32) __DATA_END - (u32) __DATA_ROM);

            if ((status = flsh_bcpy(__DATA_ROM, __DATA_END, (u32 *) OFFSET(__DATA_ROM)))) {
                return status;
            } else {
                board_trace("done.");
            }

            board_trace_fr("copy user s=%u ... ", USER_FLASH_SIZE);

            if ((status = flsh_bcpy(__user_flash_base, __user_flash_end, (u32 *) OFFSET(__user_flash_base)))) {
                return status;
            } else {
                board_trace("done.");
            }
        }
        
        board_trace_r("swapping... ");

        if (user_flash_othr->sanity == FLASH_SANITY) {
            if ((status = FLASH_Swap(&flash_config, FLASH_SWAP_ADDR, true))) {
                board_trace_f("swap fail: s=%u", status);
            } else {
                //board_trace("success.")
                board_trace("success.\nreboot....\r\n\r\n\r\n");
                //__BKPT(0);
                NVIC_SystemReset();
            }
        } else {
            board_trace("error: mark not set!");
        }
    }

    return status;
}


static status_t flsh_bcpy(u32 *src, u32 *end, u32 *dst)
{
    status_t status = 1;
    u32 size;
    
    while (src < end) {
        size = (u8 *) end - (u8 *) src;

        // Disable this to allow errors that prevent copying garbage.
        if (size > BLOCK_SIZE) {
            size = BLOCK_SIZE;
        } else {
            size += 16 - (size % 16);
        }

        /*board_trace_fr("copying %u: [0x%X:0x%X]->0x%X... ", size, src, (u8 *) src + size, dst)*/

        if ((status = flsh_write((u32)dst, size, src))) {
            return status;
        } else {
            //board_trace("done.");
        }
        
        src += size >> 2;
        dst += size >> 2;
    }
    
    return status;
}

static void print_swap_state(const ftfx_swap_state_t swap_state)
{
    switch (swap_state) {
        case kFTFx_SwapStateUninitialized:
        board_trace("uninit.");
            break;
        case kFTFx_SwapStateReady:
        board_trace("ready.");
            break;
        case kFTFx_SwapStateUpdate:
        board_trace("update.");
            break;
        case kFTFx_SwapStateUpdateErased:
        board_trace("update-erased.");
            break;
        case kFTFx_SwapStateComplete:
        board_trace("complete.");
            break;
        case kFTFx_SwapStateDisabled:
        board_trace("disabled.");
            break;
    }
}



static status_t get_swap_state_config(ftfx_swap_state_config_t *swap_state_config)
{
    status_t status = FTFx_CMD_SwapControl(ftfx_config, FLASH_SWAP_ADDR, kFTFx_SwapControlOptionReportStatus, swap_state_config);

    if (status) {
        board_trace_f("swap: read config fail. s=%u", status);
    }

    return status;
}


status_t flsh_erase(u32 begin, size_t size)
{
    return FLASH_Erase(&flash_config, begin, size, kFLASH_ApiEraseKey);
}


status_t flsh_write(u32 begin, size_t size, u32 *data)
{
    status_t status;

    portDISABLE_INTERRUPTS();

    status = FLASH_Erase(&flash_config, begin, size, kFLASH_ApiEraseKey);

    if (status) {
        board_trace_f("erase fail: s=%li", status);

    } else {
        status = FLASH_Program(&flash_config, begin, (u8 *) data, size);

        if (status != kStatus_FLASH_Success) {
            board_trace_f("write fail: %li", status);
        }

        //vTaskDelay(1);
    }

    portENABLE_INTERRUPTS();
    return status;
}

status_t flsh_user_cmit(void)
{
    return flsh_bcpy(__user_ram_base, __user_ram_end, __user_flash_base);
}
