/*
* SX1276/77/78/79 - 137 MHz to 1020 MHz Low Power Long Range Transceiver Driver
* Serial Peripheral Interface using Regmap API
*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/completion.h>
#include <linux/if_arp.h>
#include <linux/wait.h>
#include <linux/moduleparam.h>
#include "lora-spi.h"

static u32 f_xosc = 32000000UL;
module_param(f_xosc, uint, S_IRUGO);
MODULE_PARM_DESC(f_xosc, "Crystal oscillator frequency");

static const char *cr_ptr[] = { "", "4/5", "4/6", "4/7", "4/8" };
static const char *bw_ptr[] = { "7.8[KHz]", "10.4[KHz]", "15.6[KHz]", "20.8[KHz]",
    "31.25[KHz]", "41.7[KHz]", "62.5[KHz]", "125[KHz]", "250[KHz]", "500[KHz]" };
/**
 * Optional callback returning true if the register
 * supports multiple write operations without incrementing
 * the register number
 */
static bool lora_rwable_noinc_reg(struct device *dev, unsigned int reg)
{
    return (reg == REG_FIFO);
}
static bool lora_rwable_reg(struct device *dev, unsigned int reg)
{
    switch (reg) {
    case REG_FIFO:
    case REG_OP_MODE:
    case REG_FR_MSB:
    case REG_FR_MID:
    case REG_FR_LSB:
    case REG_PA_CONF:
    case REG_PA_RAMP:
    case REG_OCP:
    case REG_LNA:
    case REG_FIFO_ADD_PTR:
    case REG_FIFO_TX_BASE_ADDR:
    case REG_FIFO_RX_BASE_ADDR:
    case REG_FIFO_RX_CURRENT_ADDR:
    case REG_IRQ_FLAGS_MASK:
    case REG_IRQ_FLAGS:
    case REG_RX_NB_BYTES:
    case REG_PKT_SNR_VALUE:
    case REG_PKT_RSSI_VALUE:
    case REG_MODEM_CONFIG1:
    case REG_MODEM_CONFIG2:
    case REG_MODEM_CONFIG3:
    case REG_SYMB_TIMEOUT_L:
    case REG_PREAMBLE_MSB:
    case REG_PREAMBLE_LSB:
    case REG_PAYLOAD_LENGTH:
    case REG_SYNC_WORD:
    case REG_DIO_MAPPING1:
    case REG_DIO_MAPPING2:
    case REG_VERSION:
    case REG_PA_DAC:
        return true;
    default:
        return false;
    }
}
/**
 * struct regmap_range - A register range, used for access related checks
 *                       (readable/writeable/volatile/precious checks)
 *
 * @range_min: address of first register
 * @range_max: address of last register
 */
static const struct regmap_range lora_volatile_ranges[] = {
    { range_min:REG_FIFO, range_max:REG_PA_DAC },
};
/**
 * struct regmap_access_table - A table of register ranges for access checks
 */
static const struct regmap_access_table lora_volatile_table = {
    .yes_ranges = lora_volatile_ranges,
    .n_yes_ranges = ARRAY_SIZE(lora_volatile_ranges),
};
/**
 * struct regmap_config - Configuration for the register map of a device.
 */
static struct regmap_config lora_regmap_config = {
	.reg_bits		= 8,
	.val_bits		= 8,
	.max_register		= REG_PA_DAC,
	.readable_reg           = lora_rwable_reg,
	.writeable_reg          = lora_rwable_reg,
	.writeable_noinc_reg	= lora_rwable_noinc_reg,
	.readable_noinc_reg     = lora_rwable_noinc_reg,
	.volatile_table		= &lora_volatile_table,
	.cache_type		= REGCACHE_NONE,
	.can_multi_write        = false,
	.read_flag_mask         = 0x00,
	.write_flag_mask        = 0x80,
	.zero_flag_mask         = true,
	.name			= "sx127x",
};
/* Read Register */
static __inline int lora_read_reg(struct lora_dev *lora, unsigned int reg, u8 *res)
{
    unsigned int regval;
    int ret = regmap_read(lora->regmap, reg, &regval);
    return (*res = (u8)regval, ret);
}
static __inline int lora_bulk_read(struct lora_dev *lora, unsigned int reg, u8 *buf, u8 size)
{
    return regmap_noinc_read(lora->regmap, reg, buf, size);
}
/* Write Register */
static __inline int lora_write_reg(struct lora_dev *lora, unsigned int reg, const u8 val)
{
    unsigned int regval = val;
    return regmap_write(lora->regmap, reg, regval);
}
static __inline int lora_bulk_write(struct lora_dev *lora, unsigned int reg, const u8 *buf, u8 size)
{
    return regmap_noinc_write(lora->regmap, reg, buf, size);
}
/* --------------------------- LORA IMPLEMENTATION ------------------------------- */
/* Hardware Reset */
void lora_reset(struct lora_dev *lora)
{
    if (unlikely(in_interrupt())) {
        gpiod_set_value(lora->reset_pin, LORA_PIN_LOW);
        mdelay(1000);
        gpiod_set_value(lora->reset_pin, LORA_PIN_HIGH);
        mdelay(100);
    } else {
        gpiod_set_value_cansleep(lora->reset_pin, LORA_PIN_LOW);
        msleep(1000);
        gpiod_set_value_cansleep(lora->reset_pin, LORA_PIN_HIGH);
        msleep(100);
    }
}
/* Set LoRa Operation Mode */
int lora_set_mode(struct lora_dev *lora, enum lora_mode mode)
{
    u8 cur_val, new_val;
    /* Get current Register value */
    if (unlikely(lora_read_reg(lora, REG_OP_MODE, &cur_val)))
        return -EFAULT;
    /* Write Mode */
    switch (mode) {
	case MODE_SLEEP:
	case MODE_STANDBY:
	case MODE_TRANSMIT:
	case MODE_RXCONTINUOUS:
	case MODE_RXSINGLE:
	case MODE_CAD:
		new_val = (cur_val & REGOPMODE_MASK) | mode;
		lora->mode = mode;
		break;
	default:
		return -EINVAL;
	}
    return lora_write_reg(lora, REG_OP_MODE, new_val);
}
/**
 * Set LoRa Frequency by Hz 
 * fRF = (F(xosc) * Frf)/(2^19)
 * Resolution is 61.035 Hz if F(XOSC) = 32 MHz. Default value is
 * 0x6c8000 = 434 MHz. Register values must be modified only
 * when device is in SLEEP or STAND-BY mode.
 */
int lora_set_freq(struct lora_dev *lora, u32 freq)
{
    int ret;
    u64_t fr;
    fr.val = (u64)freq;
    fr.val = div_u64((fr.val << 19), f_xosc);
    ret = lora_write_reg(lora, REG_FR_MSB, fr.byte_2);
    if (unlikely(ret))
        goto err_handle;
    ret = lora_write_reg(lora, REG_FR_MID, fr.byte_1);
    if (unlikely(ret))
        goto err_handle;
    ret = lora_write_reg(lora, REG_FR_LSB, fr.byte_0);
    if (unlikely(ret))
        goto err_handle;
    return 0;

err_handle:
    netdev_err(lora->ndev, "%s - failed!\n", __func__);
    return ret;
}
/**
 * SF rate (expressed as a base-2 logarithm)
 * 6 -> 64 chips / symbol
 * 7 -> 128 chips / symbol
 * 8 -> 256 chips / symbol
 * 9 -> 512 chips / symbol
 * 10 -> 1024 chips / symbol
 * 11 -> 2048 chips / symbol
 * 12 -> 4096 chips / symbol
 * other values reserved
 */
int lora_set_sf(struct lora_dev *lora, enum lora_sf sf)
{
    int ret;
    u8 write, read;
    sf = (enum lora_sf)(max(min((int)sf, 12), 6));
    ret = lora_read_reg(lora, REG_MODEM_CONFIG2, &read);
    if (unlikely(ret))
        goto err_handle;

    write = (u8)(((u8)sf << 4) | (read & 0x0F));
    ret = lora_write_reg(lora, REG_MODEM_CONFIG2, write);
    if (unlikely(ret))
        goto err_handle;
    return lora_set_auto_ld0(lora);

err_handle:
    netdev_err(lora->ndev, "%s - failed!\n", __func__);
    return ret;
}
/**
 * | PaSelect (1 bit) | MaxPower (3 bits) | OutPut Power (4 bits) |
 * Pout = Pmax - (15 - OutputPower) if PaSelect = 0 (RFO pins)
 * Pout = 17 - (15 - OutputPower) if PaSelect = 1 (PA_BOOST pin)
 * Select max output power: Pmax = 10.8 + 0.6 * MaxPower [dBm]
 * RegPaDAC: Enables the +20dBm option on PA_BOOST pin
 * 0x84 -> Default value
 * 0x87 -> +20dBm on PA_BOOST when OutputPower=1111
 */
int lora_set_power(struct lora_dev *lora, u8 power)
{
    int ret;
    __u8 paconfig, output_power;
    __u8 toset = min((int)power, 17);
    /* adjust over current protection */
    if (power <= 14) {
        ret = lora_set_ocp(lora, OCURRENT_14DBM);
    } else if (power <= 17) {
        ret = lora_set_ocp(lora, OCURRENT_17DBM);
    } else {
        lora->power = 20;
        ret = lora_set_ocp(lora, OCURRENT_20DBM);
    }
    if (unlikely(ret))
        return ret;
    /* Set Power */
    if (toset > 14) {
        output_power = toset - 2;
        paconfig = PA_BOOST_PIN | MAX_POWER_15 | (output_power & 0x0F);
        ret = lora_write_reg(lora, REG_PA_DAC, (power > 17) ? PA_BOOST_20DBM : PA_BOOST_DEFAULT);
    } else {
        output_power = toset;
        paconfig = RFO_PIN | MAX_POWER_15 | (output_power & 0x0F);
        ret = lora_write_reg(lora, REG_PA_DAC, PA_BOOST_DEFAULT);
    }
    return ret ? ret : lora_write_reg(lora, REG_PA_CONF, paconfig);
}
/**
 * Trimming of OCP current:
 * Imax = 45 + 5*OcpTrim [mA] if OcpTrim <= 15 (120 mA)
 * Imax = -30 + 10*OcpTrim [mA] if 15 < OcpTrim <= 27 (130 to 240 mA)
 * Imax = 240mA for higher settings
 * Default Imax = 100mA
 */
int lora_set_ocp(struct lora_dev *lora, u8 ocurrent)
{
    u8 ocp_trim = 27;
    ocurrent = max((int)ocurrent, 45);
    if (ocurrent <= 120)
        ocp_trim = (ocurrent - 45) / 5;
    else if (ocurrent <= 240)
        ocp_trim = (ocurrent + 30) / 10;
    else
        netdev_err(lora->ndev, "OCP value invalid, used default setting 240mA!\n");
    ocp_trim = (ocp_trim & 0x1F) | OCP_ENABLE_MASK;
    return lora_write_reg(lora, REG_OCP, ocp_trim);
}
/**
 * set RX Time-Out MSB
 * Enable CRC
 * Disable Continuous mode
 */
int lora_set_tomsb_set_crc_on(struct lora_dev *lora)
{
    int ret;
    u8 read, write;
    ret = lora_read_reg(lora, REG_MODEM_CONFIG2, &read);
    if (unlikely(ret))
        goto err_handle;

    read &= ~TX_CONTINOUS_MODE;
    write = read | 0x07;
    return lora_write_reg(lora, REG_MODEM_CONFIG2, write);

err_handle:
    netdev_err(lora->ndev, "%s - failed!\n", __func__);
    return ret;
}
/**
 * Mandated for when the symbol length exceeds 16ms
 */
int lora_set_lowdr_optimization(struct lora_dev *lora, u8 val)
{
    int ret;
    u8 read, write;
    ret = lora_read_reg(lora, REG_MODEM_CONFIG3, &read);
    if (unlikely(ret))
        goto err_handle;

    write = (val) ? (read | 0x08) : (read & 0xF7);
    return lora_write_reg(lora, REG_MODEM_CONFIG3, write);

err_handle:
    netdev_err(lora->ndev, "%s - failed!\n", __func__);
    return ret;
}

int lora_set_auto_ld0(struct lora_dev *lora)
{
    long calculated_bw;
    long bw[] = {78, 104, 156, 208, 313, 417, 625, 1250, 2500, 5000};
    calculated_bw = ((1 << lora->spreading_factor) * 10) / bw[lora->bandwidth];
    return lora_set_lowdr_optimization(lora, (calculated_bw > 16));
}
/**
 * LoRa Sync Word
 * Value 0x34 is reserved for LoRaWAN networks
 */
int lora_set_syncword(struct lora_dev *lora, u8 syncword)
{
    return lora_write_reg(lora, REG_SYNC_WORD, syncword);
}
/**
 * Signal bandwidth:
 * 0000 -> 7.8 kHz
 * 0001 -> 10.4 kHz
 * 0010 -> 15.6 kHz
 * 0011 -> 20.8kHz
 * 0100 -> 31.25 kHz
 * 0101 -> 41.7 kHz
 * 0110 -> 62.5 kHz
 * 0111 -> 125 kHz
 * 1000 -> 250 kHz
 * 1001 -> 500 kHz
 * other values  reserved
 */
int lora_set_bw(struct lora_dev *lora, enum lora_bw bw)
{
    int ret;
    u8 read;
    ret = lora_read_reg(lora, REG_MODEM_CONFIG1, &read);
    if (unlikely(ret))
        goto err_handle;

    read = (read & 0x0F) | ((u8)bw << 4);
    ret = lora_write_reg(lora, REG_MODEM_CONFIG1, read);
    if (unlikely(ret))
        goto err_handle;
    return lora_set_auto_ld0(lora);

err_handle:
    netdev_err(lora->ndev, "%s - failed!\n", __func__);
    return ret;
}
/**
 * Error coding rate
 * 001 -> 4/5
 * 010 -> 4/6
 * 011 -> 4/7
 * 100 -> 4/8
 * All other values -> reserved
 * In implicit header mode should be set on receiver to determine expected coding rate
 */
int lora_set_cr(struct lora_dev *lora, enum lora_cr cr)
{
    int ret;
    u8 read;
    ret = lora_read_reg(lora, REG_MODEM_CONFIG1, &read);
    if (unlikely(ret))
        goto err_handle;

    read = (read & 0xF1) | ((u8)cr << 1);
    return lora_write_reg(lora, REG_MODEM_CONFIG1, read);

err_handle:
    netdev_err(lora->ndev, "%s - failed!\n", __func__);
    return ret;
}
/** 
 * 00 -> RxDone
 * 01 -> TxDone
 * 10 -> CadDone
 */
int lora_set_dio0_mapping(struct lora_dev *lora, const u8 mask)
{
    u8 read;
    int ret;
    ret = lora_read_reg(lora, REG_DIO_MAPPING1, &read);
    if (unlikely(ret)) {
        return ret;
    } else {
    	read = (read & DIO0_MASK) | mask;
	    return lora_write_reg(lora, REG_DIO_MAPPING1, read);
    }
}
/* Check HardWare is valid by reading Version Register */
enum lora_status lora_is_valid(struct lora_dev *lora)
{
    int ret;
    u8 read;
    ret = lora_read_reg(lora, REG_VERSION, &read);
    if (unlikely(ret))
        return LORA_UNAVAILABLE;
    else if (read == LORA_VERSION)
        return LORA_OK;
    else
        return LORA_NOT_FOUND;
}
/**
 * due to the contiguous nature of the FIFO data buffer, the base addresses for Tx and Rx are fully configurable
 * across the 256 byte memory area. Each pointer can be set independently anywhere within the FIFO. To exploit the
 * maximum FIFO data buffer size in transmit or receive mode, the whole FIFO data buffer can be used in each mode by
 * setting the base addresses RegFifoTxBaseAddr and RegFifoRxBaseAddr at the bottom of the memory (0x00).
 */
int lora_set_mtu(struct lora_dev *lora, const u8 mtu)
{
    int ret;
    ret = lora_write_reg(lora, REG_FIFO_TX_BASE_ADDR, FIFO_SIZE - mtu);
    if (unlikely(ret))
        return ret;
    return lora_write_reg(lora, REG_FIFO_RX_BASE_ADDR, FIFO_SIZE - mtu);
}
/** 
 * Transmit data 
 * The LoRaTM FIFO can only be filled in Standby mode.
 * Data transmission is initiated by sending TX mode request.
 * Upon completion the TxDone interrupt is issued and the radio returns to Standby mode.
 */
int __lora_transmit(struct lora_dev *lora, const u8 *buf, u8 size, u32 timeout)
{
    int ret;
    ret = lora_set_mode(lora, MODE_STANDBY);
    if (unlikely(ret))
        return ret;
    ret = lora_write_reg(lora, REG_FIFO_ADD_PTR, FIFO_RAM_START);
    if (unlikely(ret))
        goto err_handle;
    ret = lora_write_reg(lora, REG_PAYLOAD_LENGTH, size);
    if (unlikely(ret))
        goto err_handle;
    ret = lora_bulk_write(lora, REG_FIFO, buf, size);
    if (unlikely(ret))
        goto err_handle;
    ret = lora_set_dio0_mapping(lora, DIO0_TXDONE_MAP);
    if (unlikely(ret))
        goto err_handle;
    reinit_completion(&lora->completion);
    ret = lora_set_mode(lora, MODE_TRANSMIT);
    if (unlikely(ret))
        goto err_handle;
    ret = wait_for_completion_interruptible_timeout(&lora->completion, timeout);
    ret = (ret <= 0) ? -EFAULT : 0;
    netdev_info(lora->ndev, "Transmit %s!\n", ret ? "Failed" : "Successfully");

err_handle:
    lora_start_receiving(lora);
    return ret;
}
/* Check channel is free then transmit data */
int lora_transmit(struct lora_dev *lora, const u8 *buf, u8 size, u32 timeout)
{
    int ret;
    ret = lora_cad_check(lora, timeout);
    if (unlikely(ret != CAD_NOTDETECTED)) {
        netdev_err(lora->ndev, "Channel is busy!\n");
        lora_start_receiving(lora);
        return -EBUSY;
    }
    return __lora_transmit(lora, buf, size, timeout);
}
/* Rx Mode */
int lora_start_receiving(struct lora_dev *lora)
{
    int ret;
    ret = lora_set_dio0_mapping(lora, DIO0_RXDONE_MAP);
    if (unlikely(ret))
        return ret;
    return lora_set_mode(lora, MODE_RXCONTINUOUS);
}
/* Read new Data */
u8 lora_receive(struct lora_dev *lora, u8 *buf, u8 size)
{
    int ret;
    __u8 actual_size = 0;
    __u8 rxptr, fifo_len;
    ret = lora_set_mode(lora, MODE_STANDBY);
    if (unlikely(ret))
        goto err_handle;
    ret = lora_read_reg(lora, REG_RX_NB_BYTES, &fifo_len);
    if (unlikely(ret))
        goto err_handle;
    ret = lora_read_reg(lora, REG_FIFO_RX_CURRENT_ADDR, &rxptr);
    if (unlikely(ret))
        goto err_handle;
    ret = lora_write_reg(lora, REG_FIFO_ADD_PTR, rxptr);
    if (unlikely(ret))
        goto err_handle;
    actual_size = min(size, fifo_len);
    ret = lora_bulk_read(lora, REG_FIFO, buf, actual_size);
    if (unlikely(ret))
        actual_size = 0;

err_handle:
    return actual_size;
}
/**
 * Estimation of SNR on last packet received.In two’s compliment format mutiplied by 4.
 * SNR[dB] = PacketSnr[twos complement] / 4.
 */
int lora_get_pkt_snr(struct lora_dev *lora)
{
    int8_t snr;
    if (lora_read_reg(lora, REG_PKT_SNR_VALUE, (u8 *)&snr))
        return -EIO;
    return (int)(snr / 4);
}
/**
 * RSSI of the latest packet received (dBm):
 * RSSI[dBm] = -157 + Rssi (using HF output port, SNR >= 0)
 * RSSI[dBm] = -164 + Rssi (using LF output port, SNR >= 0)
 */
int8_t lora_get_pkt_rssi(struct lora_dev *lora)
{
    u8 rssi;
    if (lora_read_reg(lora, REG_PKT_RSSI_VALUE, &rssi))
        return -EIO;
    return (int8_t)rssi - 164;
}
/**
 * The channel activity detection mode is designed to detect a LoRa preamble on the radio channel with the best possible
 * power efficiency. Once in CAD mode, the SX1276/77/78/79 will perform a very quick scan of the band to detect a LoRa
 * packet preamble.
 */
int lora_cad_check(struct lora_dev *lora, u32 timeout) 
{
    int ret;
    ret = lora_set_mode(lora, MODE_STANDBY);
    if (unlikely(ret))
        goto err_handle;
    ret = lora_set_dio0_mapping(lora, DIO0_CADDONE_MAP);
    if (unlikely(ret))
        goto err_handle;
    atomic_set(&lora->channel_activity, CAD_INIT_VALUE);
    ret = lora_set_mode(lora, MODE_CAD);
    if (unlikely(ret))
        goto err_handle;
    ret = wait_event_interruptible_timeout(lora->channel_wait, atomic_read(&lora->channel_activity) != CAD_INIT_VALUE, timeout);
    if (unlikely(ret <= 0))
        goto err_handle;
    ret = atomic_read(&lora->channel_activity);
    return ret;
    
err_handle:
    netdev_err(lora->ndev, "%s, An error occured!\n", __func__);
    return ret;
}
static irqreturn_t lora_irq(int irq, void *dev_id)
{
    return IRQ_WAKE_THREAD;
}
/* Bottomhalf LoRa Rx IRQ Handler */
static irqreturn_t lora_bh(int irq, void *dev_id)
{
    __u8 irq_flags;
    struct sk_buff *skb;
    struct lora_dev *lora = (struct lora_dev *)dev_id;
    struct lora_mailbox *msg = &lora->mailbox;

    lora_read_reg(lora, REG_IRQ_FLAGS, &irq_flags);
    lora_write_reg(lora, REG_IRQ_FLAGS, irq_flags);

    if (irq_flags & IRQ_TX_DONE) {
        complete(&lora->completion);
        return IRQ_HANDLED;
    } else if (irq_flags & IRQ_CAD_DONE) {
        atomic_set(&lora->channel_activity, (irq_flags & IRQ_CAD_DETECTED) ? CAD_DETECTED : CAD_NOTDETECTED);
        wake_up_interruptible(&lora->channel_wait);
        return IRQ_HANDLED;
    } else if (irq_flags & IRQ_RX_DONE) {
        if (irq_flags & IRQ_PAYLOAD_CRC_ERROR)
            return IRQ_HANDLED;
        else if (unlikely(mutex_lock_interruptible(&lora->msg_lock) < 0))
            return IRQ_HANDLED;
        msg->actual_len = lora_receive(lora, msg->payload, LORA_MTU);
        lora_set_mode(lora, MODE_RXCONTINUOUS);
        if (likely(msg->actual_len)) {
            skb = netdev_alloc_skb(lora->ndev, msg->actual_len);
            if (unlikely(!skb)) {
                netdev_err(lora->ndev, "No Mem for new skb!\n");
                mutex_unlock(&lora->msg_lock);
                return IRQ_HANDLED;
            }
            skb_put_data(skb, msg->payload, msg->actual_len);
            skb->dev = lora->ndev;

            netif_rx(skb);
        }
        mutex_unlock(&lora->msg_lock);
        return IRQ_HANDLED;
    } else {
        netdev_err(lora->ndev, "IRQ not found!\n");
        return IRQ_NONE;
    }
}
/* LoRa Configuration */
enum lora_status lora_init(struct lora_dev *lora)
{
    int ret;
    __u8 read;
    enum lora_status status;
    lora_reset(lora);
    status = lora_is_valid(lora);
    if (status == LORA_OK) {
	ret = lora_set_mode(lora, MODE_SLEEP);
        if (unlikely(ret))
            goto err_handle;
	/* turn on lora mode */
	ret = lora_read_reg(lora, REG_OP_MODE, &read);
        if (unlikely(ret))
            goto err_handle;
	read |= SX127X_LORA_MODE_MASK | LOW_FREQUENCY_MODE;
	ret = lora_write_reg(lora, REG_OP_MODE, read);
        if (unlikely(ret))
            goto err_handle;
	/* set frequency */
	ret = lora_set_freq(lora, lora->frequency);
        if (unlikely(ret))
            goto err_handle;
	/* set output power gain */
	ret = lora_set_power(lora, lora->power);
        if (unlikely(ret))
            goto err_handle;
	/* set LNA gain */
	ret = lora_write_reg(lora, REG_LNA, 0x23);
        if (unlikely(ret))
            goto err_handle;
	/* set spreading factor, CRC on, and Timeout Msb */
	ret = lora_set_tomsb_set_crc_on(lora);
        if (unlikely(ret))
            goto err_handle;
	ret = lora_set_sf(lora, lora->spreading_factor);
        if (unlikely(ret))
            goto err_handle;
	ret = lora_write_reg(lora, REG_SYMB_TIMEOUT_L, 0xFF);
        if (unlikely(ret))
            goto err_handle;
	/**
         * set bandwidth, coding rate and expilicit mode
         * 8 bit RegModemConfig --> | X | X | X | X | X | X | X | X |
         * bits represent       --> |   bandwidth   |     CR    |I/E|
         */
	ret = lora_read_reg(lora, REG_MODEM_CONFIG1, &read);
        if (unlikely(ret))
            goto err_handle;
	read &= ~IMPLICIT_HEADER_MODE;
	read |= (u8)((lora->bandwidth << 4) | (lora->coding_rate << 1));
	ret = lora_write_reg(lora, REG_MODEM_CONFIG1, read);
        if (unlikely(ret))
            goto err_handle;
	ret = lora_set_auto_ld0(lora);
        if (unlikely(ret))
            goto err_handle;
	/* set preamble */
	ret = lora_write_reg(lora, REG_PREAMBLE_MSB, lora->preamble >> 8);
        if (unlikely(ret))
            goto err_handle;
	ret = lora_write_reg(lora, REG_PREAMBLE_LSB, (u8)lora->preamble);
        if (unlikely(ret))
            goto err_handle;
        /* Mask unecessary IRQ */
        read = IRQ_CAD_DETECTED | IRQ_CAD_DONE | IRQ_TX_DONE | IRQ_PAYLOAD_CRC_ERROR | IRQ_RX_DONE;
        ret = lora_write_reg(lora, REG_IRQ_FLAGS_MASK, ~read);
        if (unlikely(ret))
            goto err_handle;
        ret = lora_set_mtu(lora, LORA_MTU);
        if (unlikely(ret))
            goto err_handle;
        /* Kernel Feature */
        init_waitqueue_head(&lora->channel_wait);
        mutex_init(&lora->msg_lock);
        init_completion(&lora->completion);
        skb_queue_head_init(&lora->tx_queue);
        INIT_WORK(&lora->tx_work, lora_do_transmit);
    }
err_handle:
    return status;
}
/* Free Hardware Resource */
void lora_deinit(struct lora_dev *lora)
{
    gpiod_set_value(lora->reset_pin, 0);
    gpiod_put(lora->dio0_pin);
    gpiod_put(lora->reset_pin);
}
/* Callback when create network device */
void lora_setup(struct net_device *ndev)
{
    ndev->type               = ARPHRD_NONE;
    ndev->mtu                = LORA_MTU;
    ndev->hard_header_len    = 0;
    ndev->addr_len           = 0;
    ndev->tx_queue_len       = 10;
    ndev->flags              = IFF_NOARP;
    ndev->features           = NETIF_F_HW_CSUM;
}
void lora_do_transmit(struct work_struct *work)
{
    struct sk_buff *skb;
    struct lora_dev *lora = container_of(work, struct lora_dev, tx_work);
    if (unlikely(mutex_lock_interruptible(&lora->msg_lock) < 0))
        return;
    while ((skb = skb_dequeue(&lora->tx_queue))) {
	if (skb->len <= LORA_MTU)
		lora_transmit(lora, skb->data, skb->len, msecs_to_jiffies(LORA_TIMEOUT));
	dev_kfree_skb(skb);
    }
    mutex_unlock(&lora->msg_lock);
}
/* Callback when interface is up */
static int lora_open(struct net_device *ndev)
{
    int ret;
    struct lora_dev *lora = netdev_priv(ndev);
    ret = request_threaded_irq(lora->irq, lora_irq, lora_bh, IRQF_TRIGGER_RISING | IRQF_SHARED, lora->ndev->name, lora);
    if (unlikely(ret))
	goto open_err;
    ret = lora_start_receiving(lora);
    if (unlikely(ret))
	goto rx_err;
    netif_start_queue(ndev);
    netdev_info(ndev, "%s, %d!\n", __func__, __LINE__);
    return ret;

rx_err:
    free_irq(lora->irq, lora);
open_err:
    return ret;
}
/* Callback when interface is down */
static int lora_stop(struct net_device *ndev)
{
    int ret;
    struct lora_dev *lora = netdev_priv(ndev);

    netif_stop_queue(ndev);
    flush_work(&lora->tx_work);
    ret = mutex_lock_interruptible(&lora->msg_lock);
    if (unlikely(ret < 0))
        goto stop_err;
    lora_set_mode(lora, MODE_SLEEP);
	free_irq(lora->irq, lora);
    mutex_unlock(&lora->msg_lock);
    netdev_info(ndev, "%s, %d!\n", __func__, __LINE__);
stop_err:
    return ret;
}
/* Callback when sendto socket */
static netdev_tx_t lora_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
    struct lora_dev *lora = netdev_priv(ndev);
    skb_queue_tail(&lora->tx_queue, skb);
    schedule_work(&lora->tx_work);
    return NETDEV_TX_OK;
}
/*
 * This structure defines the management hooks for network devices.
 * The following hooks can be defined; unless noted otherwise, they are
 * optional and can be filled with a null pointer.
 */
static const struct net_device_ops lora_ops = {
    ndo_open:lora_open,
    ndo_stop:lora_stop,
    ndo_start_xmit:lora_start_xmit,
};
/* SPI devices supported by this driver */
static const struct spi_device_id lora_spi_id[] = {
	{ .name = "nam", },
	{	}
};
MODULE_DEVICE_TABLE(spi, lora_spi_id);
/*
 * Struct used for matching a device
 */
static const struct of_device_id lora_of_match_id[] = {
	{ .compatible = "sx127x-lora,nam" },
	{	}
};
MODULE_DEVICE_TABLE(of, lora_of_match_id);

int lora_probe(struct spi_device *spi)
{
    int ret;
    u32 dt_res;
    struct lora_dev *lora;
    struct net_device *ndev;
    struct device_node *np;
    
    ndev = alloc_netdev(sizeof(*lora), "lora%d", NET_NAME_UNKNOWN, lora_setup);
    if (unlikely(!ndev))
        return -ENOMEM;
    lora = netdev_priv(ndev);
    lora->ndev = ndev;
    spi_set_drvdata(spi, ndev);
    SET_NETDEV_DEV(ndev, &spi->dev);
    lora->dev   = &spi->dev;
    np          = lora->dev->of_node;

    lora->reset_pin = gpiod_get(lora->dev, "reset", GPIOD_OUT_HIGH);
    if (unlikely(IS_ERR(lora->reset_pin))) {
        dev_err(lora->dev, "Cannot find RESET pin!\n");
        ret = PTR_ERR(lora->reset_pin);
        goto parse_err;
    }
    lora->dio0_pin = gpiod_get(lora->dev, "dio0", GPIOD_IN);
    if (unlikely(IS_ERR(lora->dio0_pin))) {
        dev_err(lora->dev, "Cannot find DIO0 pin!\n");
        ret = PTR_ERR(lora->dio0_pin);
        goto dio_err;
    }
    lora->irq = gpiod_to_irq(lora->dio0_pin);
    if (unlikely(lora->irq < 0)) {
        dev_err(lora->dev, "Cannot find IRQ number!\n");
        ret = lora->irq;
        goto get_irq_err;
    }
    /* Read Frequency */
    ret = of_property_read_u32(np, "frequency", &lora->frequency);
    if (unlikely(ret < 0))
        goto get_irq_err;
    dev_info(lora->dev, "Frequency: %d[Hz]\n", lora->frequency);
    /* Read SpeadingFactor */
    ret = of_property_read_u32(np, "spreading-factor", &dt_res);
    if (unlikely(ret < 0))
        goto get_irq_err;
    lora->spreading_factor = (enum lora_sf)dt_res;
    dev_info(lora->dev, "Spreading Factor: %d\n", (u8)lora->spreading_factor);
    /* Read BandWidth */
    ret = of_property_read_u32(np, "bandwidth", &dt_res);
    if (unlikely(ret < 0))
        goto get_irq_err;
    lora->bandwidth = (enum lora_bw)dt_res;
    dev_info(lora->dev, "BandWidth: %s\n", bw_ptr[(u8)lora->bandwidth]);
    /* Read Coding Rate */
    ret = of_property_read_u32(np, "coding-rate", &dt_res);
    if (unlikely(ret < 0))
        goto get_irq_err;
    lora->coding_rate = (enum lora_cr)dt_res;
    dev_info(lora->dev, "Coding Rate: %s\n", cr_ptr[(u8)lora->coding_rate]);
    /* Read Power */
    ret = of_property_read_u32(np, "power", &dt_res);
    if (unlikely(ret < 0))
        goto get_irq_err;
    lora->power = (__u8)dt_res;
    dev_info(lora->dev, "Power: %d[dBm]\n", lora->power);
    /* Read Preamble */
    ret = of_property_read_u32(np, "preamble", &dt_res);
    if (unlikely(ret < 0))
        goto get_irq_err;
    lora->preamble = (__u16)dt_res;
    dev_info(lora->dev, "Preamble: %d\n", lora->preamble);
#if !defined(USE_SOFT_SPI)
    spi->mode           = SPI_MODE_0;
    spi->bits_per_word  = 8;
    spi->max_speed_hz   = 1000000;
    ret = spi_setup(spi);
    if (unlikely(ret < 0))
        goto get_irq_err;
#endif
    lora->regmap = devm_regmap_init_spi(spi, &lora_regmap_config);
    if (IS_ERR(lora->regmap)) {
        dev_err(lora->dev, "Cannot Setup REGMAP API!\n");
        ret = PTR_ERR(lora->regmap);
        goto get_irq_err;
    } else if (lora_init(lora) != LORA_OK) {
        dev_err(lora->dev, "LoRa init failure!\n");
        ret = -ENODEV;
        goto get_irq_err;
    }
    ndev->netdev_ops    = &lora_ops;
    ndev->irq           = lora->irq;
    dev_info(lora->dev, "%s, %d!\n", __func__, __LINE__);
    return register_netdev(ndev);

get_irq_err:
    gpiod_put(lora->dio0_pin);
dio_err:
    gpiod_put(lora->reset_pin);
parse_err:
    free_netdev(ndev);
    return ret;
}
int lora_remove(struct spi_device *spi)
{
    struct net_device *ndev = spi_get_drvdata(spi);
    struct lora_dev *lora;
    if (unlikely(!ndev))
        return -ENODEV;
    lora = netdev_priv(ndev);
    unregister_netdev(ndev);
    lora_deinit(lora);
    free_netdev(ndev);
    dev_info(&spi->dev, "%s, %d!\n", __func__, __LINE__);
    return 0;
}
void lora_shutdown(struct spi_device *spi)
{
    lora_remove(spi);
}
/**
 * struct spi_driver - Host side "protocol" driver
 * @id_table: List of SPI devices supported by this driver
 * @probe: Binds this driver to the spi device.  Drivers can verify
 *	that the device is actually present, and may need to configure
 *	characteristics (such as bits_per_word) which weren't needed for
 *	the initial configuration done during system setup.
 * @remove: Unbinds this driver from the spi device.
 * @shutdown: Standard shutdown callback used during system state
 *	transitions such as powerdown/halt and kexec
 * @driver: SPI device drivers should initialize the name and owner
 *	field of this structure.
 */
static struct spi_driver lora_spi_driver = {
	.probe = lora_probe,
	.remove = lora_remove,
	.shutdown = lora_shutdown,
	.driver = {
		.name = "sx127x",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(lora_of_match_id),
	},
	.id_table = lora_spi_id,
};
/**
 * module_spi_driver() - Helper macro for registering a SPI driver
 * @__spi_driver: spi_driver struct
 *
 * Helper macro for SPI drivers which do not do anything special in module
 * init/exit. This eliminates a lot of boilerplate. Each module may only
 * use this macro once, and calling it replaces module_init() and module_exit()
 */
module_spi_driver(lora_spi_driver);
/* Driver information */
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LoRa SX1276/77/78/79 Driver");
MODULE_AUTHOR("dinhnamuet <dinhnamuet@gmail.com>");
MODULE_VERSION("1.0.0");
