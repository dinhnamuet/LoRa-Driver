#ifndef __LORA_SPI_H__
#define __LORA_SPI_H__

#define USE_SOFT_SPI
/* Constant */
#define LORA_MTU                        255U
#define LORA_TIMEOUT                    2000U
#define LORA_PIN_LOW                    0x00U
#define LORA_PIN_HIGH                   0x01U
#define LORA_VERSION                    0x12U
#define CAD_INIT_VALUE                  0x00U
#define CAD_NOTDETECTED                 0x01U
#define CAD_DETECTED                    0x02U
#define LOW_FREQUENCY_MODE              BIT(3)
/* LoRa register address */
#define REG_FIFO                        0x00
#define REG_OP_MODE                     0x01
#define REG_FR_MSB                      0x06
#define REG_FR_MID                      0x07
#define REG_FR_LSB                      0x08
#define REG_PA_CONF                     0x09
#define REG_PA_RAMP                     0x0A
#define REG_OCP                         0x0B
#define REG_LNA                         0x0C
#define REG_FIFO_ADD_PTR                0x0D
#define REG_FIFO_TX_BASE_ADDR           0x0E
#define REG_FIFO_RX_BASE_ADDR           0x0F
#define REG_FIFO_RX_CURRENT_ADDR        0x10
#define REG_IRQ_FLAGS_MASK              0x11
#define REG_IRQ_FLAGS                   0x12
#define REG_RX_NB_BYTES                 0x13
#define REG_PKT_SNR_VALUE               0x19
#define REG_PKT_RSSI_VALUE              0x1A
#define REG_MODEM_CONFIG1               0x1D
#define REG_MODEM_CONFIG2               0x1E
#define REG_MODEM_CONFIG3               0x26
#define REG_SYMB_TIMEOUT_L              0x1F
#define REG_PREAMBLE_MSB                0x20
#define REG_PREAMBLE_LSB                0x21
#define REG_PAYLOAD_LENGTH              0x22
#define REG_SYNC_WORD                   0x39
#define REG_DIO_MAPPING1                0x40
#define REG_DIO_MAPPING2                0x41
#define REG_VERSION                     0x42
#define REG_PA_DAC                      0x4D
/* Reg IRQ Flags */
#define IRQ_CAD_DETECTED                BIT(0)
#define IRQ_FHSS_CHANGE_CHANNEL         BIT(1)
#define IRQ_CAD_DONE                    BIT(2)
#define IRQ_TX_DONE                     BIT(3)
#define IRQ_VALID_HEADER                BIT(4)
#define IRQ_PAYLOAD_CRC_ERROR           BIT(5)
#define IRQ_RX_DONE                     BIT(6)
#define IRQ_RX_TIMEOUT                  BIT(7)
#define CLEAR_IRQ_FLAGS                 0xFFU
/* DIO0 INT MAPPING */
#define DIO0_MASK                       0x3F
#define DIO0_RXDONE_MAP                 0x00
#define DIO0_TXDONE_MAP                 0x40
#define DIO0_CADDONE_MAP                0x80
/* Header Mode */
#define IMPLICIT_HEADER_MODE            BIT(0)
/* SX127x Mode */
#define SX127X_FSK_OOK_MODE_MASK        0x00
#define SX127X_LORA_MODE_MASK           0x80
#define TX_CONTINOUS_MODE               BIT(3)
#define REGOPMODE_MASK                  0xF8U
/* Over Current Protection */
#define OCP_ENABLE_MASK                 BIT(5)
#define PA_BOOST_DEFAULT                0x84U
#define PA_BOOST_20DBM                  0x87U
#define OCURRENT_14DBM                  100
#define OCURRENT_17DBM                  120
#define OCURRENT_20DBM                  140
/* LoRa Specific */
#define FIFO_RX_BASE_ADDR               0x00
#define FIFO_TX_BASE_ADDR               0x80
#define FIFO_RAM_START                  0x00
#define FIFO_SIZE                       255U
/* RF */
#define RFO_PIN                         0x00    /* Output power is limited to +14 dBm */
#define PA_BOOST_PIN                    BIT(7)  /* Output power is limited to +20 dBm */
#define MAX_POWER_15                    (0x07 << 4)
/*lora operation mode */
enum lora_mode {
    MODE_SLEEP,
    MODE_STANDBY,
    FSTX,
    MODE_TRANSMIT,
    FSRX,
    MODE_RXCONTINUOUS,
    MODE_RXSINGLE,
    MODE_CAD
};
/*lora signal bandwidth */
enum lora_bw {
    BW_7_8_KHZ,
    BW_10_4_KHZ,
    BW_15_6_KHZ,
    BW_20_8_KHZ,
    BW_31_25_KHZ,
    BW_41_7_KHZ,
    BW_62_5_KHZ,
    BW_125_KHZ,
    BW_250_KHZ,
    BW_500_KHZ
};
/*lora codingrate */
enum lora_cr {
    CR_4_5 = 1,
    CR_4_6,
    CR_4_7,
    CR_4_8
};
/*lora spreadingFactor*/
enum lora_sf {
    SF_6 = 6,
    SF_7,
    SF_8,
    SF_9,
    SF_10,
    SF_11,
    SF_12
};
/*lora status */
enum lora_status {
    LORA_OK = 200,
    LORA_NOT_FOUND = 404,
    LORA_LARGE_PAYLOAD = 413,
    LORA_UNAVAILABLE = 503
};
typedef union {
    __u64 val;
    struct {
	__u8 byte_0;
	__u8 byte_1;
	__u8 byte_2;
	__u8 byte_3;
	__u8 byte_4;
	__u8 byte_5;
	__u8 byte_6;
	__u8 byte_7;
    };
    __u8 byte[sizeof(__u64)];
} u64_t;

struct lora_mailbox {
    u8 payload[LORA_MTU];
    size_t actual_len;
};

struct lora_dev {
    /* External Interrupt */
    int irq;
    /* LoRa Config */
    __u8 power;
    __u16 preamble;
    __u32 frequency;
    enum lora_status status;
    enum lora_mode mode;
    enum lora_sf spreading_factor;
    enum lora_bw bandwidth;
    enum lora_cr coding_rate;
    /* Private Data */
    atomic_t channel_activity;
    struct lora_mailbox mailbox;
    /* HardWare */
    struct gpio_desc *reset_pin;
    struct gpio_desc *dio0_pin;
    /* Kernel Feature */
    struct device *dev;
    struct net_device *ndev;
    struct work_struct tx_work;
    struct sk_buff_head tx_queue;
    struct mutex msg_lock;
    struct regmap *regmap;
    struct completion completion;
    wait_queue_head_t channel_wait;
};

void lora_reset(struct lora_dev *lora);
int lora_set_mode(struct lora_dev *lora, enum lora_mode mode);
int lora_set_freq(struct lora_dev *lora, u32 freq);
int lora_set_sf(struct lora_dev *lora, enum lora_sf sf);
int lora_set_power(struct lora_dev *lora, u8 power);
int lora_set_ocp(struct lora_dev *lora, u8 ocurrent);
int lora_set_tomsb_set_crc_on(struct lora_dev *lora);
int lora_set_lowdr_optimization(struct lora_dev *lora, u8 val);
int lora_set_auto_ld0(struct lora_dev *lora);
int lora_set_syncword(struct lora_dev *lora, u8 syncword);
int lora_set_bw(struct lora_dev *lora, enum lora_bw bw);
int lora_set_cr(struct lora_dev *lora, enum lora_cr cr);
enum lora_status lora_is_valid(struct lora_dev *lora);
int __lora_transmit(struct lora_dev *lora, const u8 *buf, u8 size, u32 timeout);
int lora_transmit(struct lora_dev *lora, const u8 *buf, u8 size, u32 timeout);
int lora_start_receiving(struct lora_dev *lora);
u8 lora_receive(struct lora_dev *lora, u8 *buf, u8 size);
int lora_get_pkt_snr(struct lora_dev *lora);
int8_t lora_get_pkt_rssi(struct lora_dev *lora);
enum lora_status lora_init(struct lora_dev *lora);
void lora_deinit(struct lora_dev *lora);
int lora_cad_check(struct lora_dev *lora, u32 timeout);
int lora_set_dio0_mapping(struct lora_dev *lora, const u8 mask);
int lora_set_mtu(struct lora_dev *lora, const u8 mtu);
void lora_do_transmit(struct work_struct *work);

#endif
