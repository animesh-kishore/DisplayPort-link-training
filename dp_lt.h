/*
 * DisplayPort link training
 *
 * Author: Animesh Kishore <animesh.kishore@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 3, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __DP_LINK_TRAINING_H__
#define __DP_LINK_TRAINING_H__

/*
 * During clock recovery, if the sink keeps the same
 * value in ADJUST_REQUEST_LANEx_x bytes while LANEx_CR_DONE
 * bits remain unset, host must try 5 times with the same
 * voltage swing. Thereafter, shift to lower bit rate.
 */
#define CR_RETRY_LIMIT 5

/*
 * Max number of tries before we give up on channel equalization
 * and move to lower bit rate.
 */
#define CE_RETRY_LIMIT 5

/*
 * If HPD drops while we are in middle of link training,
 * wait for HPD_DROP_TIMEOUT_MS for HPD to come back again.
 * If HPD comes back, restart link training, otherwise
 * move to STATE_DONE_FAIL state.
 */
#define HPD_DROP_TIMEOUT_MS 1500

enum {
	STATE_RESET, /* Reset link training state machine */
	STATE_FAST_LT, /* Fast link training */
	STATE_CLOCK_RECOVERY,
	STATE_CHANNEL_EQUALIZATION,

	STATE_DONE_FAIL, /*
			  * Marks link training failed
			  * or forced disabled
			  */

	STATE_DONE_PASS, /* Marks link training passed. */

	STATE_REDUCE_BIT_RATE, /*
				* Move to lower link bandwidth in order
				* listed in dp_link_config_priority
				*/
	STATE_COUNT,
};

/*
 * Index of these strings are in direct co-relation
 * with state enum. Change the indexes only if you
 * fully understand what you are doing.
 */
static const char * const dp_lt_state_names[] = {
	"Reset",
	"fast link training",
	"clock recovery",
	"channel equalization",
	"link training fail/disable",
	"link training pass",
	"reduce bit rate",
};

struct dp_lt_ops {
	/* return maximum number of lanes i.e. 1/2/4, as supported by host */
	u8 (*get_max_nlanes)(void *drv_data);
	
	/*
	 * return maximum link bandwidth supported by host.
	 * RBR: 0x6, HBR: 0xa, HBR2: 0x14
	 */
	u8 (*get_max_link_bw)(void *drv_data);
	
	/* return true if link training without handshake is supported */
	bool (*is_no_aux_handshake)(void *drv_data);
	
	/*
	 * return true if training pattern sequence 3 is supported.
	 * Support mandatory only for HBR2
	 */
	bool (*is_tps3_supported)(void *drv_data);
	
	/*
	 * wait period in msec before reading link status. If none provided,
	 * we dafault to 100us for clock recovery and 400us for channel
	 * equalization. Sink specific wait period can be read from
	 * dpcd offset 0xe i.e. TRAINING_AUX_RD_INTERVAL
	 */
	u8 (*aux_rd_interval)(void *drv_data);

	/* 1 for hpd asserted. 0 for hpd de-asserted. */
	u8 (*hpd_state)(void *drv_data);

	/*
	 * true if arch can support required video mode over
	 * given number of lanes and link bandwidth.
	 */
	bool (*is_link_config_possible)(void *drv_data, u8 nlanes, u8 link_bw);
	
	/*
	 * link training state machine is about to change link bandwidth
	 * and optionally lane count. Client driver should invalidate video
	 * mode, lane count and link bandwidth combo.
	 */
	void (*invalidate_link_config)(void *drv_data);
	
	/*
	 * arch specific logic to update new lane count and link bandwidth.
	 * Cannot fail, we are here after we passed for is_link_config_possible
	 */
	void (*update_link_config)(void *drv_data, u8 nlanes, u8 link_bw);

	/*
	 * we are about to start link training. Do any arch specific stuff here.
	 * e.g. some architectures do not allow changing lane count and link
	 * bandwidth while DisplayPort controller is connected to head. Such
	 * architectures can detach the head here and attach back again when link
	 * training is successfully done.
	 * This is also a good placeholder for one time ANSI 8B/10B encoding
	 * enable for host before we start link training.
	 */
	void (*pre_lt)(void *drv_data);
	
	/* Do arch specific stuff post link training completion. */
	void (*post_lt)(void *drv_data, bool is_lt_pass);

	/*
	 * arch specific pre-charge logic goes here. Host must pre-charge
	 * mainlink to common mode voltage for minimum 10us before starting
	 * link training.
	 */
	void (*precharge_lanes)(void *drv_data, size_t nlanes);
	
	/*
	 * arch specific hw configuration to read dpcd goes here.
	 * start_offset: starting dpcd offset
	 * data: read data is populated here
	 * n_bytes: number of bytes to read
	 * Returns 0 on success.
	 */
	int (*dpcd_read)(void *drv_data, u32 start_offset, u8 *data, size_t n_bytes);
	
	/*
	 * arch specific hw configuration to write dpcd goes here.
	 * start_offset: starting dpcd offset
	 * data: holds data to be written
	 * n_bytes: number of bytes to write
	 * Returns 0 on success.
	 */
	int (*dpcd_write)(void *drv_data, u32 start_offset, u8 *data, size_t n_bytes);

	/*
	 * Configure tps and scrambling on host for given number of lanes.
	 * TPS_DISABLE: enable scrambling
	 * TPS_1/2/3: disable scrambling
	 * ANSI 8B/10B encoding is required througout link training. Hence,
	 * it's preferable to enable it in pre_lt() ops
	 */
	void (*tpg)(void *drv_data, size_t tps, size_t nlanes);

	/*
	 * Configure voltage swing, pre-emphasis and post-cursor2 at host.
	 * LT state machine provides granularity to configure each lane
	 * with different level 0/1/2/3, hence 4 entries per parameter.
	 */
	void (*set_lt_param)(void *drv_data, u8 voltage_swing[4],
		u8 pre_emphasis[4], u8 post_cursor2[4], u8 nlanes);
	
	/*
	 * return max voltage swing supported by host in terms of
	 * LEVEL_0/1/2/3. If none provided, we default to LEVEL_3
	 */
	u8 (*max_supported_vs)(void *drv_data);
	
	/*
	 * return max pre-emphasis supported by host in terms of
	 * LEVEL_0/1/2/3. If none provided, we default to LEVEL_3
	 */
	u8 (*max_supported_pe)(void *drv_data);
	
	/*
	 * return max post-cursor2 supported by host in terms of
	 * LEVEL_0/1/2/3. If none provided, we default to LEVEL_3
	 */
	u8 (*max_supported_pc2)(void *drv_data);
};

enum {
	LEVEL_0,
	LEVEL_1,
	LEVEL_2,
	LEVEL_3,
};

/*
 * enums here correspond to TRAINING_PATTERN_SET[1:0]:TRAINING_PATTERN_SELECT,
 * dpcd offset 0x102.
 */
enum {
	TPS_DISABLE = 0,
	TPS_1 = 1,
	TPS_2 = 2,
	TPS_3 = 3,
};

struct dp_lt_data {
	void *drv_data; /* private driver data */
	struct dp_lt_ops *ops;
	struct mutex lock;
	struct delayed_work dwork;
	struct completion lt_complete;

	bool force_disable; /* not sticky */
	bool force_trigger; /* not sticky */
	bool no_aux_handshake;
	bool tps3_supported;
	bool lt_param_valid;
	
	u8 nlanes;
	u8 max_nlanes;
	u8 aux_rd_interval;
	u8 state;
	u8 tps;
	u8 pending_evt; /* pending link training request */
	u8 drive_current[4]; /* voltage swing */
	u8 pre_emphasis[4]; /* post cursor1 */
	u8 post_cursor2[4];
	u8 link_bw;
	u8 max_link_bw;
	u8 cr_retry;
	u8 ce_retry;
};

enum {
	LINK_BW_G1_62 = 0x6,
	LINK_BW_G2_7 = 0xa,
	LINK_BW_G5_4 = 0x14,
};

/*
 * VESA DisplayPort complaince dictates that during link training
 * reduce bit rate stage, reduction in link bw takes higer priority
 * than reduction in lane count. For a given combination of link
 * bandwidth and lane count, if link training fails, host sould retry
 * with reduced link bandwidth. If link bandwidth is already RBR,
 * host can optionally, retry link training with reduced lane count
 * and max supported link bandwidth. This should explain unintuitive
 * priority of RBR-4 and RBR-2
 */
static const u8 dp_link_config_priority[][2] = {
	/* link bandwidth, lane count */
	{LINK_BW_G5_4, 4}, /* 21.6Gbps */
	{LINK_BW_G2_7, 4}, /* 10.8Gbps */
	{LINK_BW_G1_62, 4}, /* 6.48Gbps */
	{LINK_BW_G5_4, 2}, /* 10.8Gbps */
	{LINK_BW_G2_7, 2}, /* 5.4Gbps */
	{LINK_BW_G1_62, 2}, /* 3.24Gbps */
	{LINK_BW_G5_4, 1}, /* 5.4Gbps */
	{LINK_BW_G2_7, 1}, /* 2.7Gbps */
	{LINK_BW_G1_62, 1}, /* 1.62Gbps */
};

#define DPCD_TRAINING_PATTERN_SET (0x102)
#define DPCD_TRAINING_PATTERN_SET_SCRAMBLING_DISABLE(x) ((!!(x)) << 5)

#define DPCD_MAIN_LINK_CHANNEL_CODING_SET (0x108)
#define DPCD_MAIN_LINK_CHANNEL_CODING_SET_ANSI8B10B (1)

#define DPCD_TRAINING_LANE0_SET (0x103)
#define DPCD_TRAINING_LANEX_SET_VS(x) ((x) & 0x3)
#define DPCD_TRAINING_LANEX_SET_MAX_VS_REACHED(x) ((!!(x)) << 2)
#define DPCD_TRAINING_LANEX_SET_PE(x) (((x) & 0x3) << 3)
#define DPCD_TRAINING_LANEX_SET_MAX_PE_REACHED(x) ((!!(x)) << 5)

#define DPCD_TRAINING_LANE0_1_SET2 (0x10f)
#define DPCD_TRAINING_LANEX_PC2_SET(x) ((x) & 0x3)
#define DPCD_TRAINING_LANEX_MAX_PC2_REACHED(x) ((!!(x)) << 2)
#define DPCD_TRAINING_LANEX_PLUS1_PC2_SET(x) (((x) & 0x3) << 4)
#define DPCD_TRAINING_LANEX_PLUS1_MAX_PC2_REACHED(x) ((!!(x)) << 6)

#define DPCD_ADJUST_REQUEST_LANE0_1 (0x206)
#define DPCD_ADJUST_REQUEST_VS_MASK (0x3)
#define DPCD_ADJUST_REQUEST_PE_MASK (0x3)
#define DPCD_ADJUST_REQUEST_LANEX_VS_SHIFT (0)
#define DPCD_ADJUST_REQUEST_LANEX_PE_SHIFT (2)
#define DPCD_ADJUST_REQUEST_LANEX_PLUS1_VS_SHIFT (4)
#define DPCD_ADJUST_REQUEST_LANEX_PLUS1_PE_SHIFT (6)

#define DPCD_ADJUST_REQUEST_POST_CURSOR2 (0x20c)
#define DPCD_ADJUST_REQUEST_POST_CURSOR2_SHIFT(x) (x * 2)
#define DPCD_ADJUST_REQUEST_POST_CURSOR2_MASK (0x3)

#define DPCD_TRAINING_AUX_RD_INTERVAL (0xe)

#define DPCD_MAX_LINK_RATE (0x1)
#define DPCD_MAX_LINK_RATE_MASK (0xff)

#define DPCD_MAX_LANE_COUNT (0x2)
#define DPCD_MAX_LANE_COUNT_NLANES (0x1f)
#define DPCD_MAX_LANE_COUNT_TPS3_SUPPORTED(x) (((x) >> 6) & 1)

#define DPCD_MAX_DOWNSPREAD (0x3)
#define DPCD_MAX_DOWNSPREAD_NO_AUX_HANDSHAKE(x) (((x) >> 6) & 1)

#define DPCD_LANE0_1_STATUS (0x202)
#define DPCD_LANEX_CR_DONE (0x1)
#define DPCD_LANEX_EQ_DONE (0x2)
#define DPCD_LANEX_SYM_LOCKED (0x4)
#define DPCD_LANEX_PLUS1_CR_DONE (0x10)
#define DPCD_LANEX_PLUS1_EQ_DONE (0x20)
#define DPCD_LANEX_PLUS1_SYM_LOCKED (0x40)

#define DPCD_LANE_ALIGN_STATUS_UPDATED (0x204)
#define DPCD_LANE_ALIGN_STATUS_DONE (0x1)

void dp_lt_init(struct dp_lt_data *lt_data, void *drv_data);
void dp_lt_set_pending_evt(struct dp_lt_data *lt_data);
void dp_lt_force_disable(struct dp_lt_data *lt_data);
void dp_lt_force_trigger(struct dp_lt_data *lt_data);
unsigned long dp_lt_wait_for_completion(struct dp_lt_data *lt_data,
			u8 target_state, unsigned long timeout_ms);
u8 dp_get_lt_state(struct dp_lt_data *lt_data);
bool dp_get_lt_status(struct dp_lt_data *lt_data);
void dp_lt_invalidate(struct dp_lt_data *lt_data);
#endif
