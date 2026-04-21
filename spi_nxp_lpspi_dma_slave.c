/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpspi

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(spi_lpspi, CONFIG_SPI_LOG_LEVEL);

#include <zephyr/drivers/dma.h>
#include "spi_nxp_lpspi_priv.h"

typedef enum {
	LPSPI_TRANSFER_STATE_ACTIVE,
	LPSPI_TRANSFER_STATE_DONE,
} lpspi_slave_state_t;

static uint32_t tx_nop_val;
static uint32_t dummy_buffer;

struct spi_dma_stream {
	const struct device *dma_dev;
	uint32_t channel;
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk_cfg;
};

struct spi_nxp_dma_data {
	struct spi_dma_stream dma_rx;
	struct spi_dma_stream dma_tx;
	lpspi_slave_state_t state;
};

static struct dma_block_config *lpspi_dma_common_load(struct spi_dma_stream *stream,
						      const struct device *dev,
						      const uint8_t *buf, size_t len)
{
	struct dma_block_config *blk_cfg = &stream->dma_blk_cfg;

	memset(blk_cfg, 0, sizeof(struct dma_block_config));

	blk_cfg->block_size = len;
	stream->dma_cfg.source_burst_length = 1;
	stream->dma_cfg.user_data = (void *)dev;
	stream->dma_cfg.head_block = blk_cfg;

	return blk_cfg;
}

static int lpspi_dma_tx_load(const struct device *dev, const uint8_t *buf, size_t len)
{
	LPSPI_Type *base = (LPSPI_Type *)DEVICE_MMIO_NAMED_GET(dev, reg_base);
	struct lpspi_data *data = dev->data;
	struct spi_nxp_dma_data *dma_data = (struct spi_nxp_dma_data *)data->driver_data;
	struct spi_dma_stream *stream = &dma_data->dma_tx;
	struct dma_block_config *blk_cfg = lpspi_dma_common_load(stream, dev, buf, len);

	if (buf == NULL) {
		blk_cfg->source_address = (uint32_t)&tx_nop_val;
		stream->dma_cfg.channel_direction = PERIPHERAL_TO_PERIPHERAL;
	} else {
		blk_cfg->source_address = (uint32_t)buf;
		stream->dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	}

	blk_cfg->dest_address = (uint32_t)&base->TDR;

	return dma_config(stream->dma_dev, stream->channel, &stream->dma_cfg);
}

static int lpspi_dma_rx_load(const struct device *dev, uint8_t *buf, size_t len)
{
	LPSPI_Type *base = (LPSPI_Type *)DEVICE_MMIO_NAMED_GET(dev, reg_base);
	struct lpspi_data *data = dev->data;
	struct spi_nxp_dma_data *dma_data = (struct spi_nxp_dma_data *)data->driver_data;
	struct spi_dma_stream *stream = &dma_data->dma_rx;
	struct dma_block_config *blk_cfg = lpspi_dma_common_load(stream, dev, buf, len);

	if (buf == NULL) {
		stream->dma_cfg.channel_direction = PERIPHERAL_TO_PERIPHERAL;
		blk_cfg->dest_address = (uint32_t)&dummy_buffer;
	} else {
		stream->dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		blk_cfg->dest_address = (uint32_t)buf;
	}

	blk_cfg->source_address = (uint32_t)&base->RDR;

	return dma_config(stream->dma_dev, stream->channel, &stream->dma_cfg);
}

static void lpspi_dma_callback(const struct device *dev, void *arg, uint32_t channel, int status)
{
	const struct device *spi_dev = arg;
	LPSPI_Type *base = (LPSPI_Type *)DEVICE_MMIO_NAMED_GET(spi_dev, reg_base);
	struct lpspi_data *data = (struct lpspi_data *)spi_dev->data;
	struct spi_nxp_dma_data *dma_data = (struct spi_nxp_dma_data *)data->driver_data;
	struct spi_context *ctx = &data->ctx;
	int ret;
	if (status < 0) {
		ret = status;
		goto fail;
	}
	if (dma_data->state == LPSPI_TRANSFER_STATE_DONE) {
		return;
	}
	if (channel == dma_data->dma_tx.channel) {
		spi_context_update_tx(ctx, 1, dma_data->dma_tx.dma_blk_cfg.block_size);

		if (ctx->tx_len > 0) {
			ret = lpspi_dma_tx_load(spi_dev, ctx->tx_buf, ctx->tx_len);
			if (ret < 0) {
				goto fail;
			}
			ret = dma_start(dma_data->dma_tx.dma_dev, dma_data->dma_tx.channel);
			if (ret < 0) {
				goto fail;
			}
		} else {
			base->DER &= ~LPSPI_DER_TDDE_MASK;
		}
		LOG_DBG("DMA TX block complete");
		return;
	}

	if (channel == dma_data->dma_rx.channel) {
		spi_context_update_rx(ctx, 1, dma_data->dma_rx.dma_blk_cfg.block_size);

		if (ctx->rx_len > 0) {
			ret = lpspi_dma_rx_load(spi_dev, ctx->rx_buf, ctx->rx_len);
			if (ret < 0) {
				goto fail;
			}
			ret = dma_start(dma_data->dma_rx.dma_dev, dma_data->dma_rx.channel);
			if (ret < 0) {
				goto fail;
			}
		} else {
			base->IER &= ~(LPSPI_IER_TEIE_MASK | LPSPI_IER_REIE_MASK);
			base->DER &= ~(LPSPI_DER_TDDE_MASK | LPSPI_DER_RDDE_MASK);
			dma_data->state = LPSPI_TRANSFER_STATE_DONE;
			spi_context_complete(ctx, spi_dev, 0);
		}
		LOG_DBG("DMA RX block complete");
		return;
	}

	ret = -EIO;
fail:
	LOG_ERR("DMA callback error on channel %u: %d", channel, ret);
	base->IER &= ~(LPSPI_IER_TEIE_MASK | LPSPI_IER_REIE_MASK);
	base->DER &= ~(LPSPI_DER_TDDE_MASK | LPSPI_DER_RDDE_MASK);
	dma_data->state = LPSPI_TRANSFER_STATE_DONE;
	spi_context_complete(ctx, spi_dev, ret);
}

static int transceive_dma(const struct device *dev, const struct spi_config *spi_cfg,
			  const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs,
			  bool asynchronous, spi_callback_t cb, void *userdata)
{
	const struct lpspi_config *config = dev->config;
	LPSPI_Type *base = (LPSPI_Type *)DEVICE_MMIO_NAMED_GET(dev, reg_base);
	struct lpspi_data *data = dev->data;
	struct spi_nxp_dma_data *dma_data = (struct spi_nxp_dma_data *)data->driver_data;
	struct spi_context *ctx = &data->ctx;
	int ret;

	spi_context_lock(ctx, asynchronous, cb, userdata, spi_cfg);

	if (SPI_OP_MODE_GET(spi_cfg->operation) != SPI_OP_MODE_SLAVE) {
		LOG_ERR("slave driver requires SPI_OP_MODE_SLAVE");
		ret = -EINVAL;
		goto out;
	}

	ret = lpspi_configure(dev, spi_cfg);
	if (ret) {
		goto out;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	if (spi_context_total_tx_len(ctx) != spi_context_total_rx_len(ctx)) {
		LOG_ERR("slave mode requires symmetric TX/RX totals (tx=%zu rx=%zu)",
			spi_context_total_tx_len(ctx), spi_context_total_rx_len(ctx));
		ret = -EINVAL;
		goto out;
	}
	size_t first_chunk = spi_context_max_continuous_chunk(ctx);

	if (first_chunk == 0) {
		ret = 0;
		goto out;
	}
	base->FCR = LPSPI_FCR_TXWATER(config->tx_fifo_size - 1) | LPSPI_FCR_RXWATER(0);
	dma_data->state = LPSPI_TRANSFER_STATE_ACTIVE;
	ret = lpspi_dma_rx_load(dev, ctx->rx_buf, first_chunk);
	if (ret != 0) {
		goto out;
	}
	ret = lpspi_dma_tx_load(dev, ctx->tx_buf, first_chunk);
	if (ret != 0) {
		goto out;
	}
	ret = dma_start(dma_data->dma_rx.dma_dev, dma_data->dma_rx.channel);
	if (ret != 0) {
		goto out;
	}
	ret = dma_start(dma_data->dma_tx.dma_dev, dma_data->dma_tx.channel);
	if (ret != 0) {
		goto out;
	}
	base->SR = LPSPI_SR_TEF_MASK | LPSPI_SR_REF_MASK;
	base->IER |= LPSPI_IER_TEIE_MASK | LPSPI_IER_REIE_MASK;

	base->DER |= LPSPI_DER_TDDE_MASK | LPSPI_DER_RDDE_MASK;

	ret = spi_context_wait_for_completion(ctx);
out:
	spi_context_release(ctx, ret);
	return ret;
}

static int lpspi_dma_dev_ready(const struct device *dma_dev)
{
	if (!device_is_ready(dma_dev)) {
		LOG_ERR("%s device is not ready", dma_dev->name);
		return false;
	}

	return true;
}

static int lpspi_dma_init(const struct device *dev)
{
	struct lpspi_data *data = dev->data;
	struct spi_nxp_dma_data *dma_data = (struct spi_nxp_dma_data *)data->driver_data;
	int err = 0;

	if (!lpspi_dma_dev_ready(dma_data->dma_tx.dma_dev) ||
	    !lpspi_dma_dev_ready(dma_data->dma_rx.dma_dev)) {
		return -ENODEV;
	}

	err = spi_nxp_init_common(dev);
	if (err) {
		return err;
	}

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static int spi_nxp_dma_transceive_sync(const struct device *dev, const struct spi_config *spi_cfg,
				       const struct spi_buf_set *tx_bufs,
				       const struct spi_buf_set *rx_bufs)
{
	return transceive_dma(dev, spi_cfg, tx_bufs, rx_bufs, false, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_nxp_dma_transceive_async(const struct device *dev, const struct spi_config *spi_cfg,
					const struct spi_buf_set *tx_bufs,
					const struct spi_buf_set *rx_bufs, spi_callback_t cb,
					void *userdata)
{
	return transceive_dma(dev, spi_cfg, tx_bufs, rx_bufs, true, cb, userdata);
}
#endif /* CONFIG_SPI_ASYNC */

static DEVICE_API(spi, lpspi_dma_driver_api) = {
	.transceive = spi_nxp_dma_transceive_sync,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_nxp_dma_transceive_async,
#endif
#ifdef CONFIG_SPI_RTIO
	.iodev_submit = spi_rtio_iodev_default_submit,
#endif
	.release = spi_lpspi_release,
};

static void lpspi_isr(const struct device *dev)
{
	LPSPI_Type *base = (LPSPI_Type *)DEVICE_MMIO_NAMED_GET(dev, reg_base);
	struct lpspi_data *data = dev->data;
	struct spi_nxp_dma_data *dma_data = (struct spi_nxp_dma_data *)data->driver_data;
	struct spi_context *ctx = &data->ctx;
	uint32_t sr = base->SR;
	uint32_t ier = base->IER;
	int err = 0;
	bool terminate = false;
	if ((sr & LPSPI_SR_TEF_MASK) && (ier & LPSPI_IER_TEIE_MASK)) {
		base->SR = LPSPI_SR_TEF_MASK;
		LOG_ERR("slave TX underrun (TEF)");
		err = -EIO;
		terminate = true;
	}
	if ((sr & LPSPI_SR_REF_MASK) && (ier & LPSPI_IER_REIE_MASK)) {
		base->SR = LPSPI_SR_REF_MASK;
		LOG_ERR("slave RX overflow (REF)");
		err = -EIO;
		terminate = true;
	}
	if (!terminate || dma_data->state == LPSPI_TRANSFER_STATE_DONE) {
		return;
	}
	base->IER &= ~(LPSPI_IER_TEIE_MASK | LPSPI_IER_REIE_MASK);
	base->DER &= ~(LPSPI_DER_TDDE_MASK | LPSPI_DER_RDDE_MASK);
	(void)dma_stop(dma_data->dma_tx.dma_dev, dma_data->dma_tx.channel);
	(void)dma_stop(dma_data->dma_rx.dma_dev, dma_data->dma_rx.channel);
	base->CR |= LPSPI_CR_RTF_MASK | LPSPI_CR_RRF_MASK;
	dma_data->state = LPSPI_TRANSFER_STATE_DONE;
	spi_context_complete(ctx, dev, err);
}

#define LPSPI_DMA_COMMON_CFG(n)			\
	.dma_callback = lpspi_dma_callback,	\
	.source_data_size = 1,			\
	.dest_data_size = 1,			\
	.block_count = 1

#define SPI_DMA_CHANNELS(n)                                                                        \
	IF_ENABLED(DT_INST_DMAS_HAS_NAME(n, tx),                                                   \
		(.dma_tx = {.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, tx)),            \
			    .channel = DT_INST_DMAS_CELL_BY_NAME(n, tx, mux),                      \
			    .dma_cfg = {.channel_direction = MEMORY_TO_PERIPHERAL,                 \
			    LPSPI_DMA_COMMON_CFG(n),						   \
			    .dma_slot = DT_INST_DMAS_CELL_BY_NAME(n, tx, source)}},))		   \
	IF_ENABLED(DT_INST_DMAS_HAS_NAME(n, rx),                                                   \
		(.dma_rx = {.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, rx)),            \
			    .channel = DT_INST_DMAS_CELL_BY_NAME(n, rx, mux),                      \
			    .dma_cfg = {.channel_direction = PERIPHERAL_TO_MEMORY,                 \
			    LPSPI_DMA_COMMON_CFG(n),						   \
			    .dma_slot = DT_INST_DMAS_CELL_BY_NAME(n, rx, source)}},))

#define LPSPI_DMA_INIT(n)                                                                          \
	SPI_NXP_LPSPI_COMMON_INIT(n)                                                               \
	SPI_LPSPI_CONFIG_INIT(n)                                                              \
                                                                                                   \
	static struct spi_nxp_dma_data lpspi_dma_data##n = {SPI_DMA_CHANNELS(n)};                  \
                                                                                                   \
	static struct lpspi_data lpspi_data_##n = {.driver_data = &lpspi_dma_data##n,        \
							 SPI_NXP_LPSPI_COMMON_DATA_INIT(n)};       \
                                                                                                   \
	SPI_DEVICE_DT_INST_DEFINE(n, lpspi_dma_init, NULL, &lpspi_data_##n,                  \
				  &lpspi_config_##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,     \
				  &lpspi_dma_driver_api);

#define SPI_NXP_LPSPI_DMA_INIT(n) IF_ENABLED(SPI_NXP_LPSPI_HAS_DMAS(n), (LPSPI_DMA_INIT(n)))

DT_INST_FOREACH_STATUS_OKAY(SPI_NXP_LPSPI_DMA_INIT)
