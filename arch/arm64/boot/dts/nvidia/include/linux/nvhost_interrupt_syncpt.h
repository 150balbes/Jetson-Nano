#ifndef NVHOST_INTERRUPT_SYNCPT_H
#define NVHOST_INTERRUPT_SYNCPT_H

#include <linux/of.h>

struct nvhost_interrupt_syncpt;

struct nvhost_interrupt_syncpt *nvhost_interrupt_syncpt_get(
	struct device_node *np, void (*callback)(void *), void *private_data);
u32 nvhost_interrupt_syncpt_get_syncpt_index(struct nvhost_interrupt_syncpt *is);
phys_addr_t nvhost_interrupt_syncpt_get_syncpt_addr(
    struct nvhost_interrupt_syncpt *is);
void nvhost_interrupt_syncpt_free(struct nvhost_interrupt_syncpt *is);
int nvhost_interrupt_syncpt_prime(struct nvhost_interrupt_syncpt *is);

#endif
