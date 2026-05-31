# SPDX-License-Identifier: GPL-2.0
CFLAGS_trace.o					:= -I$(src)

obj-$(CONFIG_TYPEC_TPS6598X)			+= tps6598x.o
tps6598x-y					:= i2c.o
tps6598x-$(CONFIG_TRACING)			+= trace.o

obj-$(CONFIG_TYPEC_TPS6598X_SPMI_APPLE)		+= tps6598x-spmi.o
tps6598x-spmi-y					:= spmi.o
tps6598x-spmi-y					+= spmi-regmap.o
tps6598x-spmi-$(CONFIG_TRACING)			+= trace.o
