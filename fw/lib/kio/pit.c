#include <kio/pit.h>
#include <board/trace.h>
#include <fsl_pit.h>
#include <FreeRTOSConfig.h>


#define PIT_COUNT               4
#define PIT_CHNL_BASE           0
#define PIT_CHNL_IRQN_BASE      PIT0_IRQn
#define PIT_CHNL(IDX)           ((pit_chnl_t)(PIT_CHNL_BASE+(IDX)))
#define PIT_CHNL_IDX(CHNL)      ((CHNL)-PIT_CHNL_BASE)
#define PIT_CHNL_IRQN(CHNL)     ((IRQn_Type)(PIT_CHNL_IRQN_BASE+PIT_CHNL_IDX(CHNL)))

#define NSEC_SEC                UINT64_C(1000000000)
#define USEC_SEC                UINT32_C(1000000)

// TODO: Check out xTimer FreeRTOS port stuff for reference

static inline void pit_setup(pit_t pit, const pit_cfg_t *cfg, bool chain);
static inline void pit_clear(pit_t pit);


static s32 pit_used = 0;
static u32 bus_freq;
static bool pit_initialized = false;

static struct pit pits[PIT_COUNT] __fast_data;


void pit_init(void)
{
    if (!pit_initialized) {
        pit_initialized = true;

        bus_freq = CLOCK_GetBusClkFreq();

        memset(pits, 0, sizeof(struct pit) * PIT_COUNT);

        for (u8 i = 0; i < PIT_COUNT; ++i) {
            pits[i].chnl = PIT_CHNL(i);
        }
    }
}


static inline void pit_auto_init(void)
{
    const pit_config_t pit_config = { false };
    PIT_Init(PIT, &pit_config);
}


static inline void pit_deinit(void)
{
    PIT_Deinit(PIT);
}


pit_t pit_alloc(const pit_cfg_t *cfg)
{
    u8 idx = 0;
    pit_t pit = NULL;

    do {
        if (!pits[idx].used) {
            pit = &pits[idx];
            break;
        }
    } while (++idx < PIT_COUNT);

    if (pit) pit_setup(pit, cfg, false);

    return pit;
}


pit_t pit_chain(pit_t pit, pit_cfg_t *cfg)
{
    board_assert(pit->used);

    if (PIT_CHNL_IDX(pit->chnl) >= (PIT_COUNT-1)) return NULL;

    pit_t const cpit = &pits[PIT_CHNL_IDX(pit->chnl)+1];

    if (cpit->used) return NULL;

    pit_setup(cpit, cfg, true);

    return cpit;
}


void pit_free(pit_t pit)
{
    board_assert(pit); board_assert(pit->used);
    __BKPT();
    pit_stop(pit);
    pit_clear(pit);
}


static inline void pit_setup(pit_t pit, const pit_cfg_t *cfg, bool chain)
{
    pit->used = true;
    if (++pit_used == 1) pit_auto_init();

    if (cfg->handler) {
        pit->handler = cfg->handler;
        pit->param = cfg->param;
        NVIC_SetPriority(PIT_CHNL_IRQN(pit->chnl), configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY); // TODO: Priority adjustment?
        EnableIRQ(PIT_CHNL_IRQN(pit->chnl));
        PIT_EnableInterrupts(PIT, pit->chnl, kPIT_TimerInterruptEnable);
    } else {
        DisableIRQ(PIT_CHNL_IRQN(pit->chnl));
        PIT_DisableInterrupts(PIT, pit->chnl, kPIT_TimerInterruptEnable);
    }

    PIT_SetTimerChainMode(PIT, pit->chnl, chain);
    PIT_SetTimerPeriod(PIT, pit->chnl, cfg->period);
    PIT_ClearStatusFlags(PIT, pit->chnl, kPIT_TimerFlag);
}


static inline void pit_clear(pit_t pit)
{
    DisableIRQ(PIT_CHNL_IRQN(pit->chnl));
    PIT_DisableInterrupts(PIT, pit->chnl, kPIT_TimerInterruptEnable);
    PIT_ClearStatusFlags(PIT, pit->chnl, kPIT_TimerFlag);
    PIT_SetTimerChainMode(PIT, pit->chnl, false);
    if (!--pit_used) pit_deinit();
    pit->handler = NULL;
    pit->param = NULL;
    pit->used = false;
}


pit_tick_t pit_nsec_tick(pit_nsec_t nsec)
{
    nsec = (nsec * bus_freq) / NSEC_SEC;
    if (nsec > UINT32_MAX) nsec = UINT32_MAX;
    return (pit_tick_t)nsec;
}


pit_nsec_t pit_tick_nsec(pit_tick_t tick)
{
    return (NSEC_SEC * tick) / bus_freq;
}


pit_prio_t pit_get_prio(pit_t pit)
{
    return (pit_prio_t)NVIC_GetPriority(PIT_CHNL_IRQN(pit->chnl));
}


void pit_set_prio(pit_t pit, pit_prio_t prio)
{
    NVIC_SetPriority(PIT_CHNL_IRQN(pit->chnl), (u32)prio);
}


void pit_set_period(pit_t pit, pit_tick_t period)
{
    PIT_SetTimerPeriod(PIT, pit->chnl, period);
}


pit_tick_t pit_get_period(pit_t pit)
{
    return PIT->CHANNEL[pit->chnl].LDVAL;
}


bool pit_started(pit_t pit)
{
    return (PIT->CHANNEL[pit->chnl].TCTRL & PIT_TCTRL_TEN_MASK) != 0;
}


void pit_start(pit_t pit)
{
    PIT_StartTimer(PIT, pit->chnl);
}


void pit_stop(pit_t pit)
{
    PIT_StopTimer(PIT, pit->chnl);
}


void pit_restart(pit_t pit)
{
    pit_stop(pit);
    pit_start(pit);
}


static u32 pit_ltt_nsec_div = 1;

void pit_ltt_init(void)
{
    board_assert(!pits[0].used && !pits[1].used);
    pit_ltt_nsec_div = bus_freq;

    pit_t pit_0 = pit_alloc(&(pit_cfg_t){
            .period = UINT32_MAX
    });

    pit_t pit_1 = pit_chain(pit_0, &(pit_cfg_t){
            .period = UINT32_MAX
    });

    pit_start(pit_1);
    pit_start(pit_0);
}


static u64 ltt_prev = UINT64_MAX;

pit_nsec_t pit_ltt_current(void)
{
    const u64 ltt = ((u64)pit_get_current(&pits[1]) << 32U) | (u64)pit_get_current(&pits[0]);

    if (ltt > ltt_prev) {
        asm("bkpt #0");
    }

    ltt_prev = ltt;

    return (NSEC_SEC * (UINT64_MAX - ltt/*PIT_GetLifetimeTimerCount(PIT)*/)) / pit_ltt_nsec_div;
}


#define PIT_CHNL_USED(CHNL)          ((CHNL) >= PIT_CHNL_BASE) && ((CHNL) < (PIT_CHNL_BASE+PIT_COUNT))

// TODO: Debug check of hander only?
#define PITN_IRQHandler(N) \
    __used __fast_isr void PIT##N##_IRQHandler(void) \
    { \
        PIT_ClearStatusFlags(PIT, (pit_chnl_t)N, kPIT_TimerFlag); \
        static const pit_t const pit = &pits[PIT_CHNL_IDX(N)]; \
        if (pit->handler) (pit->handler)(pit, pit->param); \
    }

#if PIT_CHNL_USED(0)
    PITN_IRQHandler(0)
#endif
#if PIT_CHNL_USED(1)
    PITN_IRQHandler(1)
#endif
#if PIT_CHNL_USED(2)
    PITN_IRQHandler(2)
#endif
#if PIT_CHNL_USED(3)
    PITN_IRQHandler(3)
#endif
