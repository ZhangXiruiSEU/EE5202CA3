#include "har_task.h"

#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <string.h>

#include "HARnet/network.h"

STAI_NETWORK_CONTEXT_DECLARE(har_network_ctx, STAI_NETWORK_CONTEXT_SIZE);
static STAI_ALIGNED(8) uint8_t har_activations[STAI_NETWORK_ACTIVATIONS_SIZE_BYTES];
static float har_window[HAR_WINDOW_SAMPLES][HAR_AXES];
static float *har_input;
static float *har_output;
static size_t har_count;
static size_t har_write_index;

static float accel_to_model_input(const struct sensor_value *accel)
{
	/* Match the normalization used when the HAR model was trained. */
	return (float)sensor_ms2_to_mg(accel) / HAR_INPUT_SCALE_MG;
}

int har_init(void)
{
	stai_return_code rc;
	stai_ptr activations[] = { har_activations };
	stai_ptr inputs[STAI_NETWORK_IN_NUM];
	stai_ptr outputs[STAI_NETWORK_OUT_NUM];
	stai_size n_inputs;
	stai_size n_outputs;

	/* Resolve the generated model buffers once and reuse them forever. */
	rc = stai_network_init(har_network_ctx);
	if (rc != STAI_SUCCESS) {
		printk("HAR init failed: 0x%x\n", rc);
		return -EIO;
	}

	rc = stai_network_set_activations(har_network_ctx, activations, ARRAY_SIZE(activations));
	if (rc != STAI_SUCCESS) {
		printk("HAR activations setup failed: 0x%x\n", rc);
		return -EIO;
	}

	rc = stai_network_get_inputs(har_network_ctx, inputs, &n_inputs);
	if (rc != STAI_SUCCESS || n_inputs != STAI_NETWORK_IN_NUM) {
		printk("HAR input lookup failed: 0x%x\n", rc);
		return -EIO;
	}

	rc = stai_network_get_outputs(har_network_ctx, outputs, &n_outputs);
	if (rc != STAI_SUCCESS || n_outputs != STAI_NETWORK_OUT_NUM) {
		printk("HAR output lookup failed: 0x%x\n", rc);
		return -EIO;
	}

	har_input = (float *)inputs[0];
	har_output = (float *)outputs[0];

	return 0;
}

static bool har_push_sample(struct app_context *ctx, const struct sensor_value accel[3])
{
	float sample[HAR_AXES] = {
		accel_to_model_input(&accel[0]),
		accel_to_model_input(&accel[1]),
		accel_to_model_input(&accel[2]),
	};

	memcpy(har_window[har_write_index], sample, sizeof(sample));
	har_write_index = (har_write_index + 1U) % HAR_WINDOW_SAMPLES;

	if (har_count < HAR_WINDOW_SAMPLES) {
		har_count++;
	}

	k_mutex_lock(&app_lock, K_FOREVER);
	ctx->har_sample_count = har_count;
	k_mutex_unlock(&app_lock);

	/* Inference can only start once the rolling window is completely filled. */
	return har_count == HAR_WINDOW_SAMPLES;
}

static int har_run_inference_cached(struct app_context *ctx)
{
	stai_return_code rc;
	uint32_t start_cycles;
	uint32_t end_cycles;
	uint32_t elapsed_cycles;
	uint64_t elapsed_us;

	if (har_count < HAR_WINDOW_SAMPLES) {
		k_mutex_lock(&app_lock, K_FOREVER);
		ctx->har_status = 1;
		k_mutex_unlock(&app_lock);
		return 1;
	}

	/*
	 * The ring buffer wraps continuously, so rebuild a linear input tensor
	 * in oldest-to-newest order before invoking the network.
	 */
	for (size_t i = 0; i < HAR_WINDOW_SAMPLES; i++) {
		size_t src_index = (har_write_index + i) % HAR_WINDOW_SAMPLES;

		memcpy(&har_input[i * HAR_AXES], har_window[src_index], sizeof(har_window[0]));
	}

	start_cycles = k_cycle_get_32();
	rc = stai_network_run(har_network_ctx, STAI_MODE_SYNC);
	end_cycles = k_cycle_get_32();
	if (rc != STAI_SUCCESS) {
		printk("HAR inference failed: 0x%x\n", rc);
		k_mutex_lock(&app_lock, K_FOREVER);
		ctx->har_status = -EIO;
		k_mutex_unlock(&app_lock);
		return -EIO;
	}

	elapsed_cycles = end_cycles - start_cycles;
	elapsed_us = k_cyc_to_us_floor64(elapsed_cycles);
	k_mutex_lock(&app_lock, K_FOREVER);
	ctx->har_cycles = elapsed_cycles;
	ctx->har_us = elapsed_us;
	ctx->har_scores[0] = har_output[0];
	ctx->har_scores[1] = har_output[1];
	ctx->har_scores[2] = har_output[2];
	ctx->har_status = 0;
	k_mutex_unlock(&app_lock);
	return 0;
}

void task_run_har_measure(struct app_context *ctx)
{
	struct accel_sample sample;

	while (k_msgq_get(&accel_msgq, &sample, K_NO_WAIT) == 0) {
		(void)har_push_sample(ctx, sample.accel);
	}

	(void)har_run_inference_cached(ctx);
}
