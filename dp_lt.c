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

#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/completion.h>
#include <linux/kernel.h>
#include <linux/printk.h>

#include "dp_lt.h"

static void set_lt_state(struct dp_lt_data *lt_data,
			u8 target_state, int delay_ms);
static void set_lt_tpg(struct dp_lt_data *lt_data, u8 tps);

static inline bool dp_is_max_vs(struct dp_lt_data *lt_data, u8 vs)
{
	return vs >= ((lt_data->ops->max_supported_vs) ?
		lt_data->ops->max_supported_vs(lt_data->drv_data) :
		LEVEL_3);
}

static inline bool dp_is_max_pe(struct dp_lt_data *lt_data, u8 pe)
{
	return pe >= ((lt_data->ops->max_supported_pe) ?
		lt_data->ops->max_supported_pe(lt_data->drv_data) :
		LEVEL_3);
}

static inline bool dp_is_max_pc2(struct dp_lt_data *lt_data, u8 pc2)
{
	return pc2 >= ((lt_data->ops->max_supported_pc2) ?
		lt_data->ops->max_supported_pc2(lt_data->drv_data) :
		LEVEL_3);
}

/* Wait period before reading link status. */
static inline void wait_aux_training(struct dp_lt_data *lt_data,
					bool is_clk_recovery)
{
	if (!lt_data->aux_rd_interval)
		/*
		 * 0 means use default ie. 100us for clock recovery and 400us
		 * for channel equalization. We have augmented below some extra
		 * delay to spec specified values just to be extra safe.
		 */
		is_clk_recovery ? usleep_range(150, 200) :
					usleep_range(450, 500);
	else
		msleep(lt_data->aux_rd_interval);
}

static int get_next_lower_link_config(struct dp_lt_data *lt_data,
					u8 nlanes, u8 link_bw)
{
	int priority_index;
	size_t priority_arr_size = ARRAY_SIZE(dp_link_config_priority);

	for (priority_index = 0;
		priority_index < priority_arr_size;
		priority_index++) {
		if (dp_link_config_priority[priority_index][0] ==
			link_bw &&
			dp_link_config_priority[priority_index][1] ==
			nlanes)
			break;
	}

	BUG_ON(priority_index >= priority_arr_size);

	/* already at lowest link config */
	if (priority_index == priority_arr_size - 1)
		return -ENOENT;

	for (priority_index++;
		priority_index < priority_arr_size; priority_index++) {
		if (dp_link_config_priority[priority_index][0] <=
			lt_data->max_link_bw &&
			dp_link_config_priority[priority_index][1] <=
			lt_data->max_nlanes)
				return priority_index;
	}

	/* we should never end up here */
	return -ENOENT;
}

static bool get_clock_recovery_status(struct dp_lt_data *lt_data)
{
	int cnt;
	u8 nlanes = lt_data->nlanes;
	u8 data = 0;
	int loopcnt = (nlanes == 1) ? 1 : nlanes >> 1;

	for (cnt = 0; cnt < loopcnt; cnt++) {
		lt_data->ops->dpcd_read(lt_data->drv_data,
			DPCD_LANE0_1_STATUS + cnt, &data, 1);

		if (nlanes == 1)
			return data & DPCD_LANEX_CR_DONE;
		else if (!(data & DPCD_LANEX_CR_DONE) ||
			!(data & DPCD_LANEX_PLUS1_CR_DONE))
			return false;
	}

	return true;
}

static bool get_channel_eq_status(struct dp_lt_data *lt_data)
{
	struct dp_lt_ops *ops = lt_data->ops;
	void *drv_data = lt_data->drv_data;
	int cnt;
	u8 nlanes = lt_data->nlanes;
	u8 data = 0;
	bool ce_done = true;
	int loopcnt = (nlanes == 1) ? 1 : nlanes >> 1;

	for (cnt = 0; cnt < loopcnt; cnt++) {
		ops->dpcd_read(drv_data, DPCD_LANE0_1_STATUS + cnt, &data, 1);

		if (nlanes == 1) {
			ce_done = (data & DPCD_LANEX_EQ_DONE) &&
					(data & DPCD_LANEX_SYM_LOCKED);
			break;
		} else if (!(data & DPCD_LANEX_EQ_DONE) ||
				!(data & DPCD_LANEX_SYM_LOCKED) ||
				!(data & DPCD_LANEX_PLUS1_EQ_DONE) ||
				!(data & DPCD_LANEX_PLUS1_SYM_LOCKED)) {
			ce_done = false;
			break;
		}
	}

	if (ce_done) {
		ops->dpcd_read(drv_data, DPCD_LANE_ALIGN_STATUS_UPDATED, &data, 1);
		if (!(data & DPCD_LANE_ALIGN_STATUS_DONE))
			ce_done = false;
	}

	return ce_done;
}

static bool get_lt_status(struct dp_lt_data *lt_data)
{
	bool cr_done;

	cr_done = get_clock_recovery_status(lt_data);
	if (!cr_done)
		return false;

	return get_channel_eq_status(lt_data);
}

bool dp_get_lt_status(struct dp_lt_data *lt_data)
{
	bool ret;

	BUG_ON(!lt_data);

	mutex_lock(&lt_data->lock);
	ret = get_lt_status(lt_data);
	mutex_unlock(&lt_data->lock);

	return ret;
}

/*
 * get updated voltage swing, pre-emphasis and
 * post-cursor2 settings from panel
 */
static void get_lt_new_param(struct dp_lt_data *lt_data)
{
	struct dp_lt_ops *ops = lt_data->ops;
	void *drv_data = lt_data->drv_data;
	int cnt;
	u8 data;
	u8 nlanes = lt_data->nlanes;
	u8 *vs = lt_data->drive_current;
	u8 *pe = lt_data->pre_emphasis;
	u8 *pc2 = lt_data->post_cursor2;
	bool pc2_supported = lt_data->tps3_supported;
	int loopcnt = (nlanes == 1) ? 1 : nlanes >> 1;

	for (cnt = 0; cnt < loopcnt; cnt++) {
		ops->dpcd_read(drv_data, DPCD_ADJUST_REQUEST_LANE0_1 + cnt, &data, 1);
		pe[2 * cnt] = (data >> DPCD_ADJUST_REQUEST_LANEX_PE_SHIFT) &
				DPCD_ADJUST_REQUEST_PE_MASK;
		vs[2 * cnt] = (data >> DPCD_ADJUST_REQUEST_LANEX_VS_SHIFT) &
				DPCD_ADJUST_REQUEST_VS_MASK;
		pe[1 + 2 * cnt] = (data >> DPCD_ADJUST_REQUEST_LANEX_PLUS1_PE_SHIFT) &
				DPCD_ADJUST_REQUEST_PE_MASK;
		vs[1 + 2 * cnt] = (data >> DPCD_ADJUST_REQUEST_LANEX_PLUS1_VS_SHIFT) &
				DPCD_ADJUST_REQUEST_VS_MASK;
	}

	if (pc2_supported) {
		ops->dpcd_read(drv_data, DPCD_ADJUST_REQUEST_POST_CURSOR2, &data, 1);
		for (cnt = 0; cnt < nlanes; cnt++) {
			pc2[cnt] = (data >>
				DPCD_ADJUST_REQUEST_POST_CURSOR2_SHIFT(cnt)) &
				DPCD_ADJUST_REQUEST_POST_CURSOR2_MASK;
		}
	}

	for (cnt = 0; cnt < nlanes; cnt++)
		pr_info("dp lt: new config: lane %d: "
			"vs level: %d, pe level: %d, pc2 level: %d\n",
			cnt, vs[cnt], pe[cnt], pc2_supported ? pc2[cnt] : 0);
}

/*
 * configure voltage swing, pre-emphasis and
 * post-cursor2 on host and sink
 */
static void set_lt_param(struct dp_lt_data *lt_data)
{
	struct dp_lt_ops *ops = lt_data->ops;
	void *drv_data = lt_data->drv_data;
	u8 nlanes = lt_data->nlanes;
	bool pc2_supported = lt_data->tps3_supported;
	int cnt;
	u8 val;
	u8 *vs = lt_data->drive_current;
	u8 *pe = lt_data->pre_emphasis;
	u8 *pc2 = lt_data->post_cursor2;
	u8 training_lanex_set[4] = {0, 0, 0, 0}; /*
						  * used to update all TRAINING_LANEX_SET
						  * dpcd offsets. Do not change data type,
						  * lest dpcd write would overwrite
						  * unexpected offsets.
						  */
	size_t training_lanex_set_size = sizeof(training_lanex_set);
	int loopcnt = (nlanes == 1) ? 1 : nlanes >> 1;
	
	/*
	 * All arch specific voltage swing, pre-emphasis and post-cursor2
	 * per lane configuration goes here including any corresponding
	 * HW specific calibration
	 */
	ops->set_lt_param(drv_data, vs, pe, pc2, nlanes);

	/* apply voltage swing and pre-emphasis levels to panel for each lane */
	for (cnt = 0; cnt < nlanes; cnt++) {
		bool is_max_vs = dp_is_max_vs(lt_data, vs[cnt]);
		bool is_max_pe = dp_is_max_pe(lt_data, pe[cnt]);

		val = DPCD_TRAINING_LANEX_SET_VS(vs[cnt]) |
			DPCD_TRAINING_LANEX_SET_MAX_VS_REACHED(is_max_vs) |
			DPCD_TRAINING_LANEX_SET_PE(pe[cnt]) |
			DPCD_TRAINING_LANEX_SET_MAX_PE_REACHED(is_max_pe);

		training_lanex_set[cnt] = val;
		
		pr_info("dp lt: config: lane %d: "
			"vs level: %d, pe level: %d, pc2 level: %d\n",
			cnt, vs[cnt], pe[cnt], pc2_supported ? pc2[cnt] : 0);
	}
	ops->dpcd_write(drv_data, DPCD_TRAINING_LANE0_SET,
			training_lanex_set, training_lanex_set_size);

	/* apply postcursor2 levels to panel for each lane */
	if (pc2_supported) {
		for (cnt = 0; cnt < loopcnt; cnt++) {
			bool is_max_pc2 = dp_is_max_pc2(lt_data, pc2[cnt]);
			bool is_max_pc2_next_lane = dp_is_max_pc2(lt_data, pc2[cnt + 1]);

			val = DPCD_TRAINING_LANEX_PC2_SET(pc2[cnt]) |
				DPCD_TRAINING_LANEX_MAX_PC2_REACHED(is_max_pc2) |
				DPCD_TRAINING_LANEX_PLUS1_PC2_SET(pc2[cnt + 1]) |
				DPCD_TRAINING_LANEX_PLUS1_MAX_PC2_REACHED(is_max_pc2_next_lane);
			
			ops->dpcd_write(drv_data, DPCD_TRAINING_LANE0_1_SET2 + cnt, &val, 1);
		}
	}
}

static int do_fast_lt_no_handshake(struct dp_lt_data *lt_data)
{
	BUG_ON(!lt_data->lt_param_valid);

	/* transmit link training pattern 1 for min of 500us */
	set_lt_tpg(lt_data, TPS_1);
	usleep_range(500, 600);

	/* transmit link training pattern 2/3 for min of 500us */
	if (lt_data->tps3_supported)
		set_lt_tpg(lt_data, TPS_3);
	else
		set_lt_tpg(lt_data, TPS_2);
	usleep_range(500, 600);

	return 0;
}

static void lt_data_sw_reset(struct dp_lt_data *lt_data)
{
	lt_data->lt_param_valid = false;
	lt_data->cr_retry = 0;
	lt_data->ce_retry = 0;

	/* reinit voltage swing, pre-emphasis and post-cursor2 to level 0 */
	memset(lt_data->pre_emphasis, LEVEL_0,
		sizeof(lt_data->pre_emphasis));
	memset(lt_data->drive_current, LEVEL_0,
		sizeof(lt_data->drive_current));
	memset(lt_data->post_cursor2, LEVEL_0,
		sizeof(lt_data->post_cursor2));
}

static void lt_data_reset(struct dp_lt_data *lt_data)
{
	struct dp_lt_ops *ops = lt_data->ops;
	void *drv_data = lt_data->drv_data;
	lt_data_sw_reset(lt_data);

	/* reset LT data on controller and panel only if hpd is asserted */
	if (ops->hpd_state(drv_data)) {
		/*
		 * Training pattern is disabled here. Do not HW reset
		 * lt config i.e. vs, pe, pc2. CTS mandates modifying these
		 * only when training pattern is enabled.
		 */
		ops->update_link_config(drv_data, lt_data->nlanes, lt_data->link_bw);
	}
}

static void set_lt_tpg(struct dp_lt_data *lt_data, u8 tps)
{
	struct dp_lt_ops *ops = lt_data->ops;
	void *drv_data = lt_data->drv_data;
	u8 cur_hpd = ops->hpd_state(drv_data);
	u32 offset = DPCD_TRAINING_PATTERN_SET;
	u8 val = tps;

	if (lt_data->tps == tps)
		return;

	if (cur_hpd) {
		val |= (tps == TPS_DISABLE) ?
			DPCD_TRAINING_PATTERN_SET_SCRAMBLING_DISABLE(false) :
			DPCD_TRAINING_PATTERN_SET_SCRAMBLING_DISABLE(true);
		ops->dpcd_write(drv_data, offset, &val, 1);
		
		/*
		 * ANSI 8B/10B encoding is already enabled at sink
		 * before start of LT
		 */
	}
	
	ops->tpg(drv_data, tps, lt_data->nlanes);

	lt_data->tps = tps;
}

static void lt_failed(struct dp_lt_data *lt_data)
{
	mutex_lock(&lt_data->lock);

	if (lt_data->ops->post_lt)
		lt_data->ops->post_lt(lt_data->drv_data, false);

	set_lt_tpg(lt_data, TPS_DISABLE);
	lt_data_reset(lt_data);

	mutex_unlock(&lt_data->lock);
}

static void lt_passed(struct dp_lt_data *lt_data)
{
	mutex_lock(&lt_data->lock);

	lt_data->lt_param_valid = true;
	set_lt_tpg(lt_data, TPS_DISABLE);
	
	if (lt_data->ops->post_lt)
		lt_data->ops->post_lt(lt_data->drv_data, true);

	mutex_unlock(&lt_data->lock);
}

static u8 get_aux_rd_interval(struct dp_lt_data *lt_data)
{
	struct dp_lt_ops *ops = lt_data->ops;
	void *drv_data = lt_data->drv_data;
	u8 data;
	
	if (ops->aux_rd_interval)
		return ops->aux_rd_interval(drv_data);

	ops->dpcd_read(drv_data, DPCD_TRAINING_AUX_RD_INTERVAL, &data, 1);
	
	/*
	 * TRAINING_AUX_RD_INTERVAL values correspond to
	 * <read_val> * 4 msec. 0 means default i.e. 100us for
	 * clock recovery and 400us for channel eqalization
	 */
	return data * 4;
}

static bool is_tps3_supported(struct dp_lt_data *lt_data)
{
	struct dp_lt_ops *ops = lt_data->ops;
	void *drv_data = lt_data->drv_data;
	u8 data;
	
	if (ops->is_tps3_supported)
		return ops->is_tps3_supported(drv_data);
	
	ops->dpcd_read(drv_data, DPCD_MAX_LANE_COUNT, &data, 1);
	return DPCD_MAX_LANE_COUNT_TPS3_SUPPORTED(data);
}

static bool is_no_aux_handshake(struct dp_lt_data *lt_data)
{
	struct dp_lt_ops *ops = lt_data->ops;
	void *drv_data = lt_data->drv_data;
	u8 data;
	
	if (ops->is_no_aux_handshake)
		return ops->is_no_aux_handshake(drv_data);
	
	ops->dpcd_read(drv_data, DPCD_MAX_DOWNSPREAD, &data, 1);
	return DPCD_MAX_DOWNSPREAD_NO_AUX_HANDSHAKE(data);
}

static u8 get_max_nlanes(struct dp_lt_data *lt_data)
{
	struct dp_lt_ops *ops = lt_data->ops;
	void *drv_data = lt_data->drv_data;
	u8 data;
	
	if (ops->get_max_nlanes)
		ops->get_max_nlanes(drv_data);
	
	ops->dpcd_read(drv_data, DPCD_MAX_LANE_COUNT, &data, 1);
	return data & DPCD_MAX_LANE_COUNT_NLANES;
}

static u8 get_max_link_bw(struct dp_lt_data *lt_data)
{
	struct dp_lt_ops *ops = lt_data->ops;
	void *drv_data = lt_data->drv_data;
	u8 data;
	
	if (ops->get_max_link_bw)
		ops->get_max_link_bw(drv_data);
	
	ops->dpcd_read(drv_data, DPCD_MAX_LINK_RATE, &data, 1);
	return data & DPCD_MAX_LINK_RATE_MASK;
}

static void lt_reset_state(struct dp_lt_data *lt_data)
{
	void *drv_data = lt_data->drv_data;
	struct dp_lt_ops *ops = lt_data->ops;
	u8 cur_hpd;
	u8 tgt_state;
	int timeout; /* int data type is intentional here */
	bool link_cfg_possible;
	u8 data;

	BUG_ON(!lt_data);

	cur_hpd = ops->hpd_state(drv_data);

	link_cfg_possible = ops->is_link_config_possible(
						drv_data, lt_data->nlanes,
						lt_data->link_bw);
	if (lt_data->force_disable || !cur_hpd || !link_cfg_possible) {
		/*
		 * we do not mark link training as failed for force disable
		 * state machine case since we might need last known good link
		 * training parameters for fast link training next time state
		 * machine is triggered given that the client driver did not
		 * explicitly invalidate last link training parameters.
		 */
		if (lt_data->force_disable) {
			pr_info("dp lt: link training state machine force disabled\n");
		} else if (!cur_hpd || !link_cfg_possible) {
			pr_info("dp lt: cur_hpd: %d, link config possible: %d\n",
					cur_hpd, link_cfg_possible);
			lt_failed(lt_data);
		}
		lt_data->force_disable = false;
		lt_data->force_trigger = false;
		tgt_state = STATE_DONE_FAIL;
		timeout = -1;
		goto done;
	}

	if (!lt_data->force_trigger &&
		lt_data->lt_param_valid &&
		get_lt_status(lt_data)) {
		pr_info("dp_lt: link stable, do nothing\n");
		lt_passed(lt_data);
		tgt_state = STATE_DONE_PASS;
		timeout = -1;
		goto done;
	}
	lt_data->force_trigger = false;

	/* prepare for full link training */
	lt_data->aux_rd_interval = get_aux_rd_interval(lt_data);
	lt_data->tps3_supported = is_tps3_supported(lt_data);
	lt_data->no_aux_handshake = is_no_aux_handshake(lt_data);
	lt_data->nlanes = lt_data->max_nlanes = get_max_nlanes(lt_data);
	lt_data->link_bw = lt_data->max_link_bw = get_max_link_bw(lt_data);
	lt_data_reset(lt_data);
	tgt_state = STATE_CLOCK_RECOVERY;
	timeout = 0;

	WARN_ON(lt_data->tps != TPS_DISABLE);
	
	mutex_lock(&lt_data->lock);
	
	/*
	 * one time ANSI 8B/10B encoding enable at sink before we
	 * start link training. All steps for full link training
	 * need ANSI 8B/10B set.
	 */
	data = DPCD_MAIN_LINK_CHANNEL_CODING_SET_ANSI8B10B;
	ops->dpcd_write(drv_data, DPCD_MAIN_LINK_CHANNEL_CODING_SET, &data, 1);

	if (ops->pre_lt)
		ops->pre_lt(drv_data);

	/*
	 * pre-charge main link for at
	 * least 10us before initiating
	 * link training
	 */
	ops->precharge_lanes(drv_data, lt_data->nlanes);

	mutex_unlock(&lt_data->lock);
done:
	set_lt_state(lt_data, tgt_state, timeout);
}

static void fast_lt_state(struct dp_lt_data *lt_data)
{
	struct dp_lt_ops *ops = lt_data->ops;
	void *drv_data = lt_data->drv_data;
	u8 tgt_state;
	int timeout;
	u8 cur_hpd;
	bool lt_status;
	u8 data;

	BUG_ON(!lt_data || !lt_data->no_aux_handshake);

	cur_hpd = ops->hpd_state(drv_data);
	if (!cur_hpd) {
		pr_info("lt: hpd deasserted, wait for sometime, then reset\n");

		lt_failed(lt_data);
		tgt_state = STATE_RESET;
		timeout = HPD_DROP_TIMEOUT_MS;
		goto done;
	}

	WARN_ON(lt_data->tps != TPS_DISABLE);

	mutex_lock(&lt_data->lock);
	
	/*
	 * ANSI 8B/10B encoding enable at sink before we
	 * start fast link training. 
	 */
	data = DPCD_MAIN_LINK_CHANNEL_CODING_SET_ANSI8B10B;
	ops->dpcd_write(drv_data, DPCD_MAIN_LINK_CHANNEL_CODING_SET, &data, 1);

	if (ops->pre_lt)
		ops->pre_lt(drv_data);

	set_lt_param(lt_data);

	ops->precharge_lanes(drv_data, lt_data->nlanes);

	mutex_unlock(&lt_data->lock);

	do_fast_lt_no_handshake(lt_data);
	lt_status = get_lt_status(lt_data);
	if (lt_status) {
		lt_passed(lt_data);
		tgt_state = STATE_DONE_PASS;
		timeout = -1;
	} else {
		lt_failed(lt_data);
		tgt_state = STATE_RESET;
		timeout = 0;
	}

	pr_info("dp lt: fast link training %s\n",
		tgt_state == STATE_DONE_PASS ? "pass" : "fail");
done:
	set_lt_state(lt_data, tgt_state, timeout);
}

static void lt_reduce_bit_rate_state(struct dp_lt_data *lt_data)
{
	void *drv_data = lt_data->drv_data;
	struct dp_lt_ops *ops = lt_data->ops;
	u8 new_lane_count;
	u8 new_link_bw;
	int next_link_index;
	u8 cur_hpd;

	cur_hpd = ops->hpd_state(drv_data);
	if (!cur_hpd) {
		pr_info("lt: hpd deasserted, wait for sometime, then reset\n");

		lt_failed(lt_data);
		set_lt_state(lt_data, STATE_RESET, HPD_DROP_TIMEOUT_MS);
		return;
	}

	if (ops->invalidate_link_config)
		ops->invalidate_link_config(drv_data);

	next_link_index = get_next_lower_link_config(lt_data,
						lt_data->nlanes, lt_data->link_bw);
	if (next_link_index < 0)
		goto fail;

	new_link_bw = dp_link_config_priority[next_link_index][0];
	new_lane_count = dp_link_config_priority[next_link_index][1];
	if (!ops->is_link_config_possible(drv_data, new_lane_count, new_link_bw))
		goto fail;

	ops->update_link_config(drv_data, new_lane_count, new_link_bw);

	lt_data->nlanes = new_lane_count;
	lt_data->link_bw = new_link_bw;

	pr_info("dp lt: retry CR, lanes: %d, link_bw: 0x%x\n",
		lt_data->nlanes, lt_data->link_bw);
	set_lt_state(lt_data, STATE_CLOCK_RECOVERY, 0);
	return;
fail:
	pr_info("dp lt: bit rate already lowest\n");
	lt_failed(lt_data);
	set_lt_state(lt_data, STATE_DONE_FAIL, -1);
	return;
}

static void lt_channel_equalization_state(struct dp_lt_data *lt_data)
{
	struct dp_lt_ops *ops = lt_data->ops;
	void *drv_data = lt_data->ops;
	u8 tgt_state;
	int timeout;
	u32 tps = TPS_2;
	bool cr_done = true;
	bool ce_done = true;
	u8 cur_hpd;

	cur_hpd = ops->hpd_state(drv_data);
	if (!cur_hpd) {
		pr_info("lt: hpd deasserted, wait for sometime, then reset\n");

		lt_failed(lt_data);
		tgt_state = STATE_RESET;
		timeout = HPD_DROP_TIMEOUT_MS;
		goto done;
	}

	if (lt_data->tps3_supported)
		tps = TPS_3;

	set_lt_tpg(lt_data, tps);
	wait_aux_training(lt_data, false);

	cr_done = get_clock_recovery_status(lt_data);
	if (!cr_done) {
		/*
		 * No HW reset here. CTS waits on write to
		 * reduced(where applicable) link BW dpcd offset.
		 */
		lt_data_sw_reset(lt_data);
		tgt_state = STATE_REDUCE_BIT_RATE;
		timeout = 0;
		pr_info("dp lt: CR lost\n");
		goto done;
	}

	ce_done = get_channel_eq_status(lt_data);
	if (ce_done) {
		lt_passed(lt_data);
		tgt_state = STATE_DONE_PASS;
		timeout = -1;
		pr_info("dp lt: CE done\n");
		goto done;
	}
	pr_info("dp lt: CE not done\n");

	if (++(lt_data->ce_retry) > (CE_RETRY_LIMIT + 1)) {
		pr_info("dp lt: CE retry limit %d reached\n",
				lt_data->ce_retry - 2);
		/*
		 * Just do LT SW reset here. CTS mandates that
		 * LT config should be reduced only after
		 * training pattern 1 is set. Reduce bitrate
		 * state would update new link bandwidth and optionally
		 * lane count. Proceeding clock recovery/fail/reset
		 * state would reset voltage swing, pre-emphasis
		 * and post-cursor2 after setting tps 1/0.
		 */
		lt_data_sw_reset(lt_data);
		tgt_state = STATE_REDUCE_BIT_RATE;
		timeout = 0;
		goto done;
	}

	get_lt_new_param(lt_data);
	set_lt_param(lt_data);

	tgt_state = STATE_CHANNEL_EQUALIZATION;
	timeout = 0;
	pr_info("dp lt: CE retry\n");
done:
	set_lt_state(lt_data, tgt_state, timeout);
}

static inline bool is_vs_already_max(struct dp_lt_data *lt_data,
					u8 old_vs[4], u8 new_vs[4])
{
	u8 nlanes = lt_data->nlanes;
	int cnt;

	for (cnt = 0; cnt < nlanes; cnt++) {
		if (dp_is_max_vs(lt_data, old_vs[cnt]) &&
			dp_is_max_vs(lt_data, new_vs[cnt]))
			continue;

		return false;
	}

	return true;
}

static void lt_clock_recovery_state(struct dp_lt_data *lt_data)
{
	void *drv_data = lt_data->drv_data;
	struct dp_lt_ops *ops = lt_data->ops;
	u8 tgt_state;
	int timeout;
	u8 *vs = lt_data->drive_current;
	bool cr_done;
	u8 vs_temp[4];
	u8 cur_hpd;

	BUG_ON(!lt_data);

	cur_hpd = ops->hpd_state(drv_data);
	if (!cur_hpd) {
		pr_info("lt: hpd deasserted, wait for sometime, then reset\n");

		lt_failed(lt_data);
		tgt_state = STATE_RESET;
		timeout = HPD_DROP_TIMEOUT_MS;
		goto done;
	}

	set_lt_tpg(lt_data, TPS_1);

	set_lt_param(lt_data);
	wait_aux_training(lt_data, true);
	cr_done = get_clock_recovery_status(lt_data);
	if (cr_done) {
		lt_data->cr_retry = 0;
		tgt_state = STATE_CHANNEL_EQUALIZATION;
		timeout = 0;
		pr_info("dp lt: CR done\n");
		goto done;
	}
	pr_info("dp lt: CR not done\n");

	memcpy(vs_temp, vs, sizeof(vs_temp));
	get_lt_new_param(lt_data);

	if (!memcmp(vs_temp, vs, sizeof(vs_temp))) {
		/*
		 * Reduce bit rate if voltage swing already max or
		 * CR retry limit of 5 reached.
		 */
		if (is_vs_already_max(lt_data, vs_temp, vs) ||
			(lt_data->cr_retry)++ >= (CR_RETRY_LIMIT - 1)) {
			pr_info("dp lt: CR retry limit %d %s reached\n",
				lt_data->cr_retry, is_vs_already_max(
				lt_data, vs_temp, vs) ? "for max vs" : "");
			/*
			 * Just do link training SW reset here. CTS mandates that
			 * voltage swing, pre-emphasis and post-cursor2 should be
			 * changed only after bit rate reduction. Reduce bitrate
			 * state would update new lane count and link bandwidth.
			 * Proceeding clock recovery state would reset voltage
			 * swing, pre-emphasis and post-cursor2.
			 */
			lt_data_sw_reset(lt_data);
			tgt_state = STATE_REDUCE_BIT_RATE;
			timeout = 0;
			goto done;
		}
	} else {
		lt_data->cr_retry = 1;
	}

	tgt_state = STATE_CLOCK_RECOVERY;
	timeout = 0;
	pr_info("dp lt: CR retry\n");
done:
	set_lt_state(lt_data, tgt_state, timeout);
}

/*
 * index of dispatch functions correspond to state enum.
 * Change the order only if you full understand what you are doing.
 */
typedef void (*dispatch_func_t)(struct dp_lt_data *lt_data);
static const dispatch_func_t state_machine_dispatch[] = {
	lt_reset_state,			/* STATE_RESET */
	fast_lt_state,			/* STATE_FAST_LT */
	lt_clock_recovery_state,	/* STATE_CLOCK_RECOVERY */
	lt_channel_equalization_state,	/* STATE_CHANNEL_EQUALIZATION */
	NULL,				/* STATE_DONE_FAIL */
	NULL,				/* STATE_DONE_PASS */
	lt_reduce_bit_rate_state,	/* STATE_REDUCE_BIT_RATE */
};

static void handle_lt_hpd_evt(struct dp_lt_data *lt_data, u8 cur_hpd)
{
	u8 tgt_state = STATE_RESET;
	int timeout = 0;

	/*
	 * hpd deasserted while we are still in middle
	 * of link training. Wait for HPD_DROP_TIMEOUT_MS
	 * for hpd to come up. Thereafter, reset link training
	 * state machine.
	 */
	if (!cur_hpd && !lt_data->force_disable)
		timeout = HPD_DROP_TIMEOUT_MS;

	if (lt_data->lt_param_valid &&
		lt_data->no_aux_handshake &&
		!lt_data->force_disable)
		tgt_state = STATE_FAST_LT;

	set_lt_state(lt_data, tgt_state, timeout);
}

static void lt_worker(struct work_struct *work)
{
	u8 pending_lt_evt;
	u8 cur_hpd;
	struct dp_lt_data *lt_data = container_of(to_delayed_work(work),
					struct dp_lt_data, dwork);
	void *drv_data = lt_data->drv_data;
	struct dp_lt_ops *ops = lt_data->ops;

	/*
	 * Observe and clear pending flag
	 * and latch the current HPD state.
	 */
	mutex_lock(&lt_data->lock);
	pending_lt_evt = lt_data->pending_evt;
	lt_data->pending_evt = 0;
	mutex_unlock(&lt_data->lock);
	cur_hpd = ops->hpd_state(drv_data);

	pr_info("dp lt: state %d (%s), hpd %d, pending_lt_evt %d\n",
		lt_data->state, dp_lt_state_names[lt_data->state],
		cur_hpd, pending_lt_evt);

	if (pending_lt_evt) {
		handle_lt_hpd_evt(lt_data, cur_hpd);
	} else if (lt_data->state < ARRAY_SIZE(state_machine_dispatch)) {
		dispatch_func_t func = state_machine_dispatch[lt_data->state];

		if (!func)
			pr_warn("dp lt: NULL state handler in state %d\n", lt_data->state);
		else
			func(lt_data);
	} else {
		/* should never reach here */
		pr_warn("dp lt: unexpected state scheduled %d",
			lt_data->state);
	}
}

static void sched_lt_work(struct dp_lt_data *lt_data, int delay_ms)
{
	cancel_delayed_work(&lt_data->dwork);

	if (delay_ms >= 0)
		schedule_delayed_work(&lt_data->dwork,
					msecs_to_jiffies(delay_ms));
}

static void set_lt_state(struct dp_lt_data *lt_data,
			u8 target_state, int delay_ms)
{
	mutex_lock(&lt_data->lock);

	pr_info("dp lt: switching from state %d (%s) to state %d (%s)\n",
		lt_data->state, dp_lt_state_names[lt_data->state],
		target_state, dp_lt_state_names[target_state]);

	lt_data->state = target_state;

	/* we have reached final state. notify others. */
	if (target_state == STATE_DONE_PASS ||
		target_state == STATE_DONE_FAIL)
		complete_all(&lt_data->lt_complete);

	/*
	 * If the pending_hpd_evt flag is already set, don't bother to
	 * reschedule the state machine worker. We should be able to assert
	 * that there is a worker callback already scheduled, and that it is
	 * scheduled to run immediately
	 */
	if (!lt_data->pending_evt)
		sched_lt_work(lt_data, delay_ms);

	mutex_unlock(&lt_data->lock);
}

/*
 * Request link training. State machine takes the decision to initiate
 * fast or full LT. If link is already stable we do not initiate full LT
 * again. Note, fast LT capability is mostly supported on internal eDP panels,
 * external DP panels rarely support the same. Check dp_lt_invalidate() and
 * dp_lt_force_trigger() to override above default state machine behaviour.
 */
void dp_lt_set_pending_evt(struct dp_lt_data *lt_data)
{
	mutex_lock(&lt_data->lock);

	/* always schedule work any time there is a pending lt event */
	lt_data->pending_evt = 1;
	sched_lt_work(lt_data, 0);

	mutex_unlock(&lt_data->lock);
}

/*
 * Marks previous LT configuration data as invalid.
 * Full LT is required to get new LT param data.
 * One use case is while system is being suspended
 * and we do not plan to process hotplug in suspend state.
 * Since connected panel might be changed while we are suspended
 * mark current LT config invalid so that we start from scratch
 * on resume.
 *
 * Request for LT post this call is sure to initiate full LT.
 */
void dp_lt_invalidate(struct dp_lt_data *lt_data)
{
	mutex_lock(&lt_data->lock);
	lt_data->lt_param_valid = false;
	mutex_unlock(&lt_data->lock);
}

u8 dp_get_lt_state(struct dp_lt_data *lt_data)
{
	u8 ret;

	mutex_lock(&lt_data->lock);
	ret = lt_data->state;
	mutex_unlock(&lt_data->lock);

	return ret;
}

/* block till link training has reached final state */
unsigned long dp_lt_wait_for_completion(struct dp_lt_data *lt_data,
					u8 target_state, unsigned long timeout_ms)
{
	might_sleep();

	if (target_state == dp_get_lt_state(lt_data))
		return 1;

	mutex_lock(&lt_data->lock);
	reinit_completion(&lt_data->lt_complete);
	mutex_unlock(&lt_data->lock);

	return wait_for_completion_timeout(&lt_data->lt_complete,
					msecs_to_jiffies(timeout_ms));
}

/* Force disable state machine to go to final STATE_DONE_FAIL state. */
void dp_lt_force_disable(struct dp_lt_data *lt_data)
{
	mutex_lock(&lt_data->lock);
	lt_data->force_disable = true;
	mutex_unlock(&lt_data->lock);

	dp_lt_set_pending_evt(lt_data);
}

/*
 * By default we dont restart full LT if link is already stable.
 * Use this API if you want to override this default behaviour.
 */
void dp_lt_force_trigger(struct dp_lt_data *lt_data)
{
	mutex_lock(&lt_data->lock);
	lt_data->force_trigger = true;
	mutex_unlock(&lt_data->lock);
	
	dp_lt_set_pending_evt(lt_data);
}

/* LT state machine initializer. */
void dp_lt_init(struct dp_lt_data *lt_data, void *drv_data)
{
	struct dp_lt_ops *ops;

	BUG_ON(!lt_data || !lt_data->ops);
	
	/* flag error if any of the mandatory ops is missing */
	ops = lt_data->ops;
	BUG_ON(!ops->hpd_state || !ops->is_link_config_possible ||
		!ops->update_link_config || !ops->precharge_lanes ||
		!ops->dpcd_read || !ops->dpcd_write || !ops->tpg ||
		!ops->set_lt_param);

	lt_data->drv_data = drv_data;
	lt_data->state = STATE_RESET;
	lt_data->pending_evt = 0;
	
	lt_data_sw_reset(lt_data);

	mutex_init(&lt_data->lock);
	INIT_DELAYED_WORK(&lt_data->dwork, lt_worker);
	init_completion(&lt_data->lt_complete);
}
