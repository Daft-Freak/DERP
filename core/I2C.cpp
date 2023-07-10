#include <cstdio>

#include "hardware/platform_defs.h"
#include "hardware/regs/i2c.h"

#include "I2C.h"

#include "MemoryBus.h"
#include "Logging.h"

using Logging::logf;
using LogLevel = Logging::Level;
constexpr auto logComponent = Logging::Component::I2C;

I2C::I2C(MemoryBus &mem, int index) : mem(mem), index(index)
{
}

void I2C::reset()
{
    hw.con = I2C_IC_CON_RESET;
    hw.tar = I2C_IC_TAR_RESET;
    hw.sar = I2C_IC_SAR_RESET;
    hw.data_cmd = I2C_IC_DATA_CMD_RESET;

    hw.ss_scl_hcnt = I2C_IC_SS_SCL_HCNT_RESET;
    hw.ss_scl_lcnt = I2C_IC_SS_SCL_LCNT_RESET;
    hw.fs_scl_hcnt = I2C_IC_FS_SCL_HCNT_RESET;
    hw.fs_scl_lcnt = I2C_IC_FS_SCL_LCNT_RESET;

    hw.intr_mask = I2C_IC_INTR_MASK_RESET;
    hw.raw_intr_stat = I2C_IC_RAW_INTR_STAT_RESET;

    hw.rx_tl = I2C_IC_RX_TL_RESET;
    hw.tx_tl = I2C_IC_TX_TL_RESET;

    hw.clr_intr = I2C_IC_CLR_INTR_RESET;
    hw.clr_rx_under = I2C_IC_CLR_RX_UNDER_RESET;
    hw.clr_rx_over = I2C_IC_CLR_RX_OVER_RESET;
    hw.clr_tx_over = I2C_IC_CLR_TX_OVER_RESET;
    hw.clr_rd_req = I2C_IC_CLR_RD_REQ_RESET;
    hw.clr_tx_abrt = I2C_IC_CLR_TX_ABRT_RESET;
    hw.clr_rx_done = I2C_IC_CLR_RX_DONE_RESET;
    hw.clr_activity = I2C_IC_CLR_ACTIVITY_RESET;
    hw.clr_stop_det = I2C_IC_CLR_STOP_DET_RESET;
    hw.clr_start_det = I2C_IC_CLR_START_DET_RESET;
    hw.clr_gen_call = I2C_IC_CLR_GEN_CALL_RESET;

    hw.enable = I2C_IC_ENABLE_RESET;
    hw.status = I2C_IC_STATUS_RESET;
    hw.txflr = I2C_IC_TXFLR_RESET;
    hw.rxflr = I2C_IC_RXFLR_RESET;
    hw.sda_hold = I2C_IC_SDA_HOLD_RESET;
    hw.tx_abrt_source = I2C_IC_TX_ABRT_SOURCE_RESET;
    hw.slv_data_nack_only = I2C_IC_SLV_DATA_NACK_ONLY_RESET;

    hw.dma_cr = I2C_IC_DMA_CR_RESET;
    hw.dma_tdlr = I2C_IC_DMA_TDLR_RESET;
    hw.dma_rdlr = I2C_IC_DMA_RDLR_RESET;

    hw.sda_setup = I2C_IC_SDA_SETUP_RESET;
    hw.ack_general_call = I2C_IC_ACK_GENERAL_CALL_RESET;
    hw.enable_status = I2C_IC_ENABLE_STATUS_RESET;
    hw.fs_spklen = I2C_IC_FS_SPKLEN_RESET;
    hw.clr_restart_det = I2C_IC_CLR_RESTART_DET_RESET;
}

uint32_t I2C::regRead(uint32_t addr)
{
    switch(addr)
    {
        case I2C_IC_RAW_INTR_STAT_OFFSET:
            return I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS | I2C_IC_RAW_INTR_STAT_STOP_DET_BITS;

        case I2C_IC_TX_ABRT_SOURCE_OFFSET:
            return hw.tx_abrt_source;
    }

    logf(LogLevel::NotImplemented, logComponent, "%i R %04X", index, addr);

    return 0xBADADD55;
}

void I2C::regWrite(uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    static const char *op[]{" = ", " ^= ", " |= ", " &= ~"};

    switch(addr)
    {
        case I2C_IC_TAR_OFFSET:
            updateReg(hw.tar, data, atomic);
            return;

        case I2C_IC_DATA_CMD_OFFSET:
        {
            logf(LogLevel::Info, logComponent, "%i: %02X (addr %02X)", index, data & I2C_IC_DATA_CMD_DAT_BITS, hw.tar & I2C_IC_TAR_IC_TAR_BITS);
            return;
        }

        case I2C_IC_ENABLE_OFFSET:
            updateReg(hw.enable, data, atomic);
            return;
    }

    logf(LogLevel::NotImplemented, logComponent, "%i W %04X%s%08X", index, addr, op[atomic], data);
}
