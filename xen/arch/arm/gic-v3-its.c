/*
 * Copyright (C) 2013, 2014 ARM Limited, All Rights Reserved.
 * Author: Marc Zyngier <marc.zyngier@arm.com>
 *
 * Xen changes:
 * Vijaya Kumar K <Vijaya.Kumar@caviumnetworks.com>
 * Copyright (C) 2014, 2015 Cavium Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <xen/config.h>
#include <xen/bitops.h>
#include <xen/lib.h>
#include <xen/init.h>
#include <xen/cpu.h>
#include <xen/mm.h>
#include <xen/irq.h>
#include <xen/sched.h>
#include <xen/errno.h>
#include <xen/delay.h>
#include <xen/device_tree.h>
#include <xen/libfdt/libfdt.h>
#include <xen/xmalloc.h>
#include <xen/list.h>
#include <xen/sizes.h>
#include <xen/vmap.h>
#include <asm/p2m.h>
#include <asm/domain.h>
#include <asm/io.h>
#include <asm/device.h>
#include <asm/gic.h>
#include <asm/gic_v3_defs.h>
#include <asm/gic-its.h>

#define its_print(lvl, fmt, ...)                                      \
	printk(lvl "GIC-ITS:" fmt, ## __VA_ARGS__)

#define its_err(fmt, ...) its_print(XENLOG_ERR, fmt, ## __VA_ARGS__)

#define its_dbg(fmt, ...)                                             \
	its_print(XENLOG_DEBUG, fmt, ## __VA_ARGS__)

#define its_info(fmt, ...)                                            \
	its_print(XENLOG_INFO, fmt, ## __VA_ARGS__)

#define its_warn(fmt, ...)                                            \

//#define DEBUG_GIC_ITS

#ifdef DEBUG_GIC_ITS
# define DPRINTK(fmt, args...) printk(XENLOG_DEBUG fmt, ##args)
#else
# define DPRINTK(fmt, args...) do {} while ( 0 )
#endif

#define ITS_FLAGS_CMDQ_NEEDS_FLUSHING		(1 << 0)

#define RDIST_FLAGS_PROPBASE_NEEDS_FLUSHING	(1 << 0)

/*
 * Collection structure - just an ID, and a redistributor address to
 * ping. We use one per CPU as a bag of interrupts assigned to this
 * CPU.
 */
struct its_collection {
	u64			target_address;
	u16			col_id;
	u16			valid;
};

/*
 * The ITS structure - contains most of the infrastructure, with the
 * msi_controller, the command queue, the collections, and the list of
 * devices writing to it.
 */
struct its_node {
	spinlock_t		lock;
	struct list_head	entry;
	void __iomem		*base;
	unsigned long		phys_base;
	unsigned long		phys_size;
	struct its_cmd_block	*cmd_base;
	struct its_cmd_block	*cmd_write;
	void			*tables[GITS_BASER_NR_REGS];
	struct its_collection	*collections;
	u64			flags;
	u32			ite_size;
	u32			nr_collections;
	struct dt_device_node	*dt_node;
};

uint32_t pta_type;
/* Number of physical its nodes present */
uint32_t nr_its = 0;

#define ITS_ITT_ALIGN		SZ_256

static LIST_HEAD(its_nodes);
static DEFINE_SPINLOCK(its_lock);
static struct dt_device_node *gic_root_node;
static struct rdist_prop  *gic_rdists;

#define gic_data_rdist()		(per_cpu(rdist, smp_processor_id()))
#define gic_data_rdist_rd_base()	(per_cpu(rdist, smp_processor_id()).rbase)

/*
 * ITS command descriptors - parameters to be encoded in a command
 * block.
 */
struct its_cmd_desc {
	union {
		struct {
			struct its_collection *col;
			u32 event_id;
			u32 dev_id;
		} its_inv_cmd;

		struct {
			struct its_collection *col;
			int valid;
		} its_mapc_cmd;

		struct {
			struct its_collection *col;
			u32 id;
			u32 dev_id;
		} its_movi_cmd;

		struct {
			struct its_collection *col;
		} its_invall_cmd;
	};
};

uint32_t its_get_pta_type(void)
{
	return pta_type;
}

uint32_t its_get_nr_its(void)
{
	return nr_its;
}

struct its_node * its_get_phys_node(uint32_t dev_id)
{
	struct its_node *its;

	/* TODO: For now return ITS0 node.
	 * Need Query PCI helper function to get on which
	 * ITS node the device is attached
	 */
	list_for_each_entry(its, &its_nodes, entry) {
		return its;
	}

	return NULL;
}

static int its_search_rdist_address(struct domain *d, uint64_t ta,
				    uint32_t *col_id)
{
	int i, rg;
	paddr_t start, end;

	for (rg = 0; rg < d->arch.vgic.nr_regions; rg++) {
		i = 0;
		start = d->arch.vgic.rdist_regions[rg].base;
		end = d->arch.vgic.rdist_regions[rg].base +
			d->arch.vgic.rdist_regions[rg].size;
		while ((( start + i * d->arch.vgic.rdist_stride) < end)) {
			if ((start + i * d->arch.vgic.rdist_stride) == ta) {
				DPRINTK("ITS: Found pta 0x%lx\n", ta);
				*col_id = i;
				return 0;
			}
			i++;
		}
	}
	return 1;
}

int its_get_physical_cid(struct domain *d, uint32_t *col_id, uint64_t ta)
{
	int i;
	struct its_collection *col;

	/*
	* For Dom0, the target address info is collected
	* at boot time.
	*/
	if (is_hardware_domain(d)) {
		struct its_node *its;

		list_for_each_entry(its, &its_nodes, entry) {
			for (i = 0; i < its->nr_collections; i++) {
		                col = &its->collections[i];
				if (col->valid && col->target_address == ta) {
					DPRINTK("ITS:Match ta 0x%lx ta 0x%lx\n",
						col->target_address, ta);
					*col_id = col->col_id;
					return 0;
				}
			}
			/* All collections are mapped on every physical ITS */
			break;
		}
	}
	else
	{
		/* As per Spec, Target address is re-distributor
		 * address/cpu number.
		 * We cannot rely on collection id as it can any number.
		 * So here we should rely only on vta address to map the
		 * collection. For domU, vta != target address.
		 * So, check vta is corresponds to which GICR region and
		 * consider that vcpu id as collection id.
		 */
		if (its_get_pta_type()) {
			its_search_rdist_address(d, ta, col_id);
		}
		else
		{
			*col_id = ta;
			return 0;
		}
	}

	DPRINTK("ITS: Cannot find valid pta entry for ta 0x%lx\n", ta);
	return 1;
}

int its_get_target(uint8_t pcid, uint64_t *pta)
{
	int i;
	struct its_collection *col;
	struct its_node *its;

	list_for_each_entry(its, &its_nodes, entry) {
		for (i = 0; i < its->nr_collections; i++) {
			col = &its->collections[i];
			if (col->valid && col->col_id == pcid) {
				*pta = col->target_address;
				DPRINTK("ITS:Match pta 0x%lx vta 0x%lx\n",
					col->target_address, *pta);
				return 0;
			}
		}
		/* All collections are mapped on every physical ITS */
		break;
	}

	DPRINTK("ITS: Cannot find valid pta entry for vta 0x%lx\n",*pta);
	return 1;
}

#define ITS_CMD_QUEUE_SZ		SZ_64K
#define ITS_CMD_QUEUE_NR_ENTRIES	(ITS_CMD_QUEUE_SZ / sizeof(struct its_cmd_block))

typedef struct its_collection *(*its_cmd_builder_t)(struct its_cmd_block *,
						    struct its_cmd_desc *);

static struct its_collection *its_build_mapc_cmd(struct its_cmd_block *cmd,
						 struct its_cmd_desc *desc)
{
	its_encode_cmd(cmd, GITS_CMD_MAPC);
	its_encode_collection(cmd, desc->its_mapc_cmd.col->col_id);
	its_encode_target(cmd, desc->its_mapc_cmd.col->target_address);
	its_encode_valid(cmd, desc->its_mapc_cmd.valid);

	return desc->its_mapc_cmd.col;
}

static struct its_collection *its_build_movi_cmd(struct its_cmd_block *cmd,
						 struct its_cmd_desc *desc)
{
	its_encode_cmd(cmd, GITS_CMD_MOVI);
	its_encode_devid(cmd, desc->its_movi_cmd.dev_id);
	its_encode_event_id(cmd, desc->its_movi_cmd.id);
	its_encode_collection(cmd, desc->its_movi_cmd.col->col_id);

	return desc->its_movi_cmd.col;
}

static struct its_collection *its_build_inv_cmd(struct its_cmd_block *cmd,
						struct its_cmd_desc *desc)
{
	its_encode_cmd(cmd, GITS_CMD_INV);
	its_encode_devid(cmd, desc->its_inv_cmd.dev_id);
	its_encode_event_id(cmd, desc->its_inv_cmd.event_id);

	return desc->its_inv_cmd.col;
}

static struct its_collection *its_build_invall_cmd(struct its_cmd_block *cmd,
						   struct its_cmd_desc *desc)
{
	its_encode_cmd(cmd, GITS_CMD_INVALL);
	its_encode_collection(cmd, desc->its_mapc_cmd.col->col_id);

	return NULL;
}

static u64 its_cmd_ptr_to_offset(struct its_node *its,
				 struct its_cmd_block *ptr)
{
	return (ptr - its->cmd_base) * sizeof(*ptr);
}

static int its_queue_full(struct its_node *its)
{
	int widx;
	int ridx;

	widx = its->cmd_write - its->cmd_base;
	ridx = readl_relaxed(its->base + GITS_CREADR) / sizeof(struct its_cmd_block);

	/* This is incredibly unlikely to happen, unless the ITS locks up. */
	if (((widx + 1) % ITS_CMD_QUEUE_NR_ENTRIES) == ridx)
		return 1;

	return 0;
}

static struct its_cmd_block *its_allocate_entry(struct its_node *its)
{
	struct its_cmd_block *cmd;
	u32 count = 1000000;	/* 1s! */

	while (its_queue_full(its)) {
		count--;
		if (!count) {
			its_err("ITS queue not draining\n");
			return NULL;
		}
		cpu_relax();
		udelay(1);
	}

	cmd = its->cmd_write++;

	/* Handle queue wrapping */
	if (its->cmd_write == (its->cmd_base + ITS_CMD_QUEUE_NR_ENTRIES))
		its->cmd_write = its->cmd_base;

	return cmd;
}

static struct its_cmd_block *its_post_commands(struct its_node *its)
{
	u64 wr = its_cmd_ptr_to_offset(its, its->cmd_write);

	writel_relaxed(wr, its->base + GITS_CWRITER);

	return its->cmd_write;
}

static void its_flush_cmd(struct its_node *its, struct its_cmd_block *cmd)
{
	/*
	 * Make sure the commands written to memory are observable by
	 * the ITS.
	 */
	if (its->flags & ITS_FLAGS_CMDQ_NEEDS_FLUSHING)
		clean_and_invalidate_dcache_va_range(cmd, sizeof(*cmd));
	else
		dsb(ishst);
}

static void its_wait_for_range_completion(struct its_node *its,
					  struct its_cmd_block *from,
					  struct its_cmd_block *to)
{
	u64 rd_idx, from_idx, to_idx;
	u32 count = 1000000;	/* 1s! */

	from_idx = its_cmd_ptr_to_offset(its, from);
	to_idx = its_cmd_ptr_to_offset(its, to);

	while (1) {
		rd_idx = readl_relaxed(its->base + GITS_CREADR);
		if (rd_idx >= to_idx || rd_idx < from_idx)
			break;

		count--;
		if (!count) {
			its_err("ITS queue timeout\n");
			return;
		}
		cpu_relax();
		udelay(1);
	}
}

static void its_send_single_command(struct its_node *its,
				    its_cmd_builder_t builder,
				    struct its_cmd_desc *desc)
{
	struct its_cmd_block *cmd, *sync_cmd, *next_cmd;
	struct its_collection *sync_col;
	unsigned long flags;

	spin_lock_irqsave(&its->lock, flags);

	cmd = its_allocate_entry(its);
	if (!cmd) {		/* We're soooooo screewed... */
		its_err("ITS can't allocate, dropping command\n");
		spin_unlock_irqrestore(&its->lock, flags);
		return;
	}
	sync_col = builder(cmd, desc);
	its_flush_cmd(its, cmd);

	if (sync_col) {
		sync_cmd = its_allocate_entry(its);
		if (!sync_cmd) {
			its_err("ITS can't SYNC, skipping\n");
			goto post;
		}
		its_encode_cmd(sync_cmd, GITS_CMD_SYNC);
		its_encode_target(sync_cmd, sync_col->target_address);
		its_flush_cmd(its, sync_cmd);
	}

post:
	next_cmd = its_post_commands(its);
	spin_unlock_irqrestore(&its->lock, flags);

	its_wait_for_range_completion(its, cmd, next_cmd);
}

void its_send_inv(struct its_device *dev, u32 event_id)
{
	struct its_cmd_desc desc;

	desc.its_inv_cmd.dev_id = dev->device_id;
	desc.its_inv_cmd.event_id = event_id;

	its_send_single_command(dev->its, its_build_inv_cmd, &desc);
}

static void its_send_mapc(struct its_node *its, struct its_collection *col,
			  int valid)
{
	struct its_cmd_desc desc;

	desc.its_mapc_cmd.col = col;
	desc.its_mapc_cmd.valid = !!valid;

	its_send_single_command(its, its_build_mapc_cmd, &desc);
}

/* TODO: Remove static for the sake of compilation */
void its_send_movi(struct its_node *its, struct its_collection *col,
	           u32 dev_id, u32 id)
{
	struct its_cmd_desc desc;

	desc.its_movi_cmd.dev_id = dev_id;
	desc.its_movi_cmd.col = col;
	desc.its_movi_cmd.id = id;

	its_send_single_command(its, its_build_movi_cmd, &desc);
}

static void its_send_invall(struct its_node *its, struct its_collection *col)
{
	struct its_cmd_desc desc;

	desc.its_invall_cmd.col = col;

	its_send_single_command(its, its_build_invall_cmd, &desc);
}

/*
 * The below irqchip functions are no more required.
 * TODO: Will be implemented as separate patch
 */
#if 0
/*
 * irqchip functions - assumes MSI, mostly.
 */

static inline u32 its_get_event_id(struct irq_data *d)
{
	struct its_device *its_dev = irq_data_get_irq_chip_data(d);
	return d->hwirq - its_dev->lpi_base;
}

static void lpi_set_config(struct irq_data *d, bool enable)
{
	struct its_device *its_dev = irq_data_get_irq_chip_data(d);
	irq_hw_number_t hwirq = d->hwirq;
	u32 id = its_get_event_id(d);
	u8 *cfg = page_address(gic_rdists->prop_page) + hwirq - 8192;

	if (enable)
		*cfg |= LPI_PROP_ENABLED;
	else
		*cfg &= ~LPI_PROP_ENABLED;

	/*
	 * Make the above write visible to the redistributors.
	 * And yes, we're flushing exactly: One. Single. Byte.
	 * Humpf...
	 */
	if (gic_rdists->flags & RDIST_FLAGS_PROPBASE_NEEDS_FLUSHING)
		__flush_dcache_area(cfg, sizeof(*cfg));
	else
		dsb(ishst);
	its_send_inv(its_dev, id);
}

static void its_mask_irq(struct irq_data *d)
{
	lpi_set_config(d, false);
}

static void its_unmask_irq(struct irq_data *d)
{
	lpi_set_config(d, true);
}

static void its_eoi_irq(struct irq_data *d)
{
	gic_write_eoir(d->hwirq);
}

static int its_set_affinity(struct irq_data *d, const struct cpumask *mask_val,
			    bool force)
{
	unsigned int cpu = cpumask_any_and(mask_val, cpu_online_mask);
	struct its_device *its_dev = irq_data_get_irq_chip_data(d);
	struct its_collection *target_col;
	u32 id = its_get_event_id(d);

	if (cpu >= nr_cpu_ids)
		return -EINVAL;

	target_col = &its_dev->its->collections[cpu];
	its_send_movi(its_dev, target_col, id);
	its_dev->collection = target_col;

	return IRQ_SET_MASK_OK_DONE;
}

static void its_irq_compose_msi_msg(struct irq_data *d, struct msi_msg *msg)
{
	struct its_device *its_dev = irq_data_get_irq_chip_data(d);
	struct its_node *its;
	u64 addr;

	its = its_dev->its;
	addr = its->phys_base + GITS_TRANSLATER;

	msg->address_lo		= addr & ((1UL << 32) - 1);
	msg->address_hi		= addr >> 32;
	msg->data		= its_get_event_id(d);
}

static struct irq_chip its_irq_chip = {
	.name			= "ITS",
	.irq_mask		= its_mask_irq,
	.irq_unmask		= its_unmask_irq,
	.irq_eoi		= its_eoi_irq,
	.irq_set_affinity	= its_set_affinity,
	.irq_compose_msi_msg	= its_irq_compose_msi_msg,
};

static void its_mask_msi_irq(struct irq_data *d)
{
	pci_msi_mask_irq(d);
	irq_chip_mask_parent(d);
}

static void its_unmask_msi_irq(struct irq_data *d)
{
	pci_msi_unmask_irq(d);
	irq_chip_unmask_parent(d);
}

static struct irq_chip its_msi_irq_chip = {
	.name			= "ITS-MSI",
	.irq_unmask		= its_unmask_msi_irq,
	.irq_mask		= its_mask_msi_irq,
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_write_msi_msg	= pci_msi_domain_write_msg,
};
#endif

/*
 * How we allocate LPIs:
 *
 * The GIC has id_bits bits for interrupt identifiers. From there, we
 * must subtract 8192 which are reserved for SGIs/PPIs/SPIs. Then, as
 * we allocate LPIs by chunks of 32, we can shift the whole thing by 5
 * bits to the right.
 *
 * This gives us (((1UL << id_bits) - 8192) >> 5) possible allocations.
 */
#define IRQS_PER_CHUNK_SHIFT	5
#define IRQS_PER_CHUNK		(1 << IRQS_PER_CHUNK_SHIFT)

static unsigned long *lpi_bitmap;
static u32 lpi_chunks;
static DEFINE_SPINLOCK(lpi_lock);

static int its_lpi_to_chunk(int lpi)
{
	return (lpi - 8192) >> IRQS_PER_CHUNK_SHIFT;
}

static int its_chunk_to_lpi(int chunk)
{
	return (chunk << IRQS_PER_CHUNK_SHIFT) + 8192;
}

static int its_lpi_init(u32 id_bits)
{
	lpi_chunks = its_lpi_to_chunk(1UL << id_bits);

	lpi_bitmap = xzalloc_bytes(BITS_TO_LONGS(lpi_chunks) * sizeof(long));
	if (!lpi_bitmap) {
		lpi_chunks = 0;
		return -ENOMEM;
	}

	its_info("ITS: Allocated %d chunks for LPIs\n", (int)lpi_chunks);
	return 0;
}

unsigned long *its_lpi_alloc_chunks(int nirqs, int *base, int *nr_ids)
{
	unsigned long *bitmap = NULL;
	int chunk_id;
	int nr_chunks;
	int i;

	nr_chunks = DIV_ROUND_UP(nirqs, IRQS_PER_CHUNK);

	spin_lock(&lpi_lock);

	do {
		chunk_id = bitmap_find_next_zero_area(lpi_bitmap, lpi_chunks,
						      0, nr_chunks, 0);
		if (chunk_id < lpi_chunks)
			break;

		nr_chunks--;
	} while (nr_chunks > 0);

	if (!nr_chunks)
		goto out;

	bitmap = xzalloc_bytes(BITS_TO_LONGS(nr_chunks * IRQS_PER_CHUNK) * sizeof (long));
	if (!bitmap)
		goto out;

	for (i = 0; i < nr_chunks; i++)
		set_bit(chunk_id + i, lpi_bitmap);

	*base = its_chunk_to_lpi(chunk_id);
	*nr_ids = nr_chunks * IRQS_PER_CHUNK;

out:
	spin_unlock(&lpi_lock);

	return bitmap;
}

void its_lpi_free(unsigned long *bitmap, int base, int nr_ids)
{
	int lpi;

	spin_lock(&lpi_lock);

	for (lpi = base; lpi < (base + nr_ids); lpi += IRQS_PER_CHUNK) {
		int chunk = its_lpi_to_chunk(lpi);
		BUG_ON(chunk > lpi_chunks);
		if (test_bit(chunk, lpi_bitmap)) {
			clear_bit(chunk, lpi_bitmap);
		} else {
			its_err("Bad LPI chunk %d\n", chunk);
		}
	}

	spin_unlock(&lpi_lock);

	xfree(bitmap);
}

/*
 * We allocate 64kB for PROPBASE. That gives us at most 64K LPIs to
 * deal with (one configuration byte per interrupt). PENDBASE has to
 * be 64kB aligned (one bit per LPI, plus 8192 bits for SPI/PPI/SGI).
 */
#define LPI_PROPBASE_SZ		SZ_64K
#define LPI_PENDBASE_SZ		(LPI_PROPBASE_SZ / 8 + SZ_1K)

/*
 * This is how many bits of ID we need, including the useless ones.
 */
#define LPI_NRBITS		fls(LPI_PROPBASE_SZ + SZ_8K) - 1

#define LPI_PROP_DEFAULT_PRIO	0xa0

static int __init its_alloc_lpi_tables(void)
{
	paddr_t paddr;

	gic_rdists->prop_page = alloc_xenheap_pages(get_order_from_bytes(LPI_PROPBASE_SZ), 0);
	if (!gic_rdists->prop_page) {
		its_err("Failed to allocate PROPBASE\n");
		return -ENOMEM;
	}

	paddr = __pa(gic_rdists->prop_page);
	its_info("GIC: using LPI property table @%pa\n", &paddr);

	/* Priority 0xa0, Group-1, disabled */
	memset(gic_rdists->prop_page,
	       LPI_PROP_DEFAULT_PRIO | LPI_PROP_GROUP1,
	       LPI_PROPBASE_SZ);

	/* Make sure the GIC will observe the written configuration */
	clean_and_invalidate_dcache_va_range(gic_rdists->prop_page,
	                                     LPI_PROPBASE_SZ);

	return 0;
}

static const char *its_base_type_string[] = {
	[GITS_BASER_TYPE_DEVICE]	= "Devices",
	[GITS_BASER_TYPE_VCPU]		= "Virtual CPUs",
	[GITS_BASER_TYPE_CPU]		= "Physical CPUs",
	[GITS_BASER_TYPE_COLLECTION]	= "Interrupt Collections",
	[GITS_BASER_TYPE_RESERVED5] 	= "Reserved (5)",
	[GITS_BASER_TYPE_RESERVED6] 	= "Reserved (6)",
	[GITS_BASER_TYPE_RESERVED7] 	= "Reserved (7)",
};

static void its_free_tables(struct its_node *its)
{
	int i;

	for (i = 0; i < GITS_BASER_NR_REGS; i++) {
		if (its->tables[i]) {
			xfree(its->tables[i]);
			its->tables[i] = NULL;
		}
	}
}

static int its_alloc_tables(struct its_node *its)
{
	int err;
	int i;
	int psz = SZ_64K;
	u64 shr = GITS_BASER_InnerShareable;

	for (i = 0; i < GITS_BASER_NR_REGS; i++) {
		u64 val = readq_relaxed(its->base + GITS_BASER + i * 8);
		u64 type = GITS_BASER_TYPE(val);
		u64 entry_size = GITS_BASER_ENTRY_SIZE(val);
		int order = get_order_from_bytes(psz);
		int alloc_size;
		u64 tmp;
		void *base;

		if (type == GITS_BASER_TYPE_NONE)
			continue;

		/*
		 * Allocate as many entries as required to fit the
		 * range of device IDs that the ITS can grok... The ID
		 * space being incredibly sparse, this results in a
		 * massive waste of memory.
		 *
		 * For other tables, only allocate a single page.
		 */
		if (type == GITS_BASER_TYPE_DEVICE) {
			u64 typer = readq_relaxed(its->base + GITS_TYPER);
			u32 ids = GITS_TYPER_DEVBITS(typer);

			order = get_order_from_bytes((1UL << ids) * entry_size);
			if (order >= MAX_ORDER) {
				order = MAX_ORDER - 1;
				its_warn("Device Table too large, reduce its page order to %u\n",
					 order);
			}
		}

		alloc_size = (1 << order) * PAGE_SIZE;
		base = alloc_xenheap_pages(order, 0);
		if (!base) {
			err = -ENOMEM;
			goto out_free;
		}
		memset(base, 0, alloc_size);
		its->tables[i] = base;

retry_baser:
		val = (__pa(base) 					 |
		       (type << GITS_BASER_TYPE_SHIFT)			 |
		       ((entry_size - 1) << GITS_BASER_ENTRY_SIZE_SHIFT) |
		       GITS_BASER_WaWb					 |
		       shr						 |
		       GITS_BASER_VALID);

		switch (psz) {
		case SZ_4K:
			val |= GITS_BASER_PAGE_SIZE_4K;
			break;
		case SZ_16K:
			val |= GITS_BASER_PAGE_SIZE_16K;
			break;
		case SZ_64K:
			val |= GITS_BASER_PAGE_SIZE_64K;
			break;
		}

		val |= (alloc_size / psz) - 1;

		writeq_relaxed(val, its->base + GITS_BASER + i * 8);
		tmp = readq_relaxed(its->base + GITS_BASER + i * 8);

		if ((val ^ tmp) & GITS_BASER_SHAREABILITY_MASK) {
			/*
			 * Shareability didn't stick. Just use
			 * whatever the read reported, which is likely
			 * to be the only thing this redistributor
			 * supports.
			 */
			shr = tmp & GITS_BASER_SHAREABILITY_MASK;
			goto retry_baser;
		}

		if ((val ^ tmp) & GITS_BASER_PAGE_SIZE_MASK) {
			/*
			 * Page size didn't stick. Let's try a smaller
			 * size and retry. If we reach 4K, then
			 * something is horribly wrong...
			 */
			switch (psz) {
			case SZ_16K:
				psz = SZ_4K;
				goto retry_baser;
			case SZ_64K:
				psz = SZ_16K;
				goto retry_baser;
			}
		}

		if (val != tmp) {
			its_err("ITS: GITS_BASER%d doesn't stick: %lx %lx\n",
			       i,
			       (unsigned long) val, (unsigned long) tmp);
			err = -ENXIO;
			goto out_free;
		}

		its_info("ITS: allocated %d %s @%lx (psz %dK, shr %d)\n",
			(int)(alloc_size / entry_size),
			its_base_type_string[type],
			(unsigned long)__pa(base),
			psz / SZ_1K, (int)shr >> GITS_BASER_SHAREABILITY_SHIFT);
	}

	return 0;

out_free:
	its_free_tables(its);

	return err;
}

static int its_alloc_collections(struct its_node *its)
{
	its->collections = xzalloc_array(struct its_collection, nr_cpu_ids);
	if (!its->collections)
		return -ENOMEM;

	return 0;
}

static void its_cpu_init_lpis(void)
{
	void __iomem *rbase = gic_data_rdist_rd_base();
	void *pend_page;
	u64 val, tmp;

	/* If we didn't allocate the pending table yet, do it now */
	pend_page = gic_data_rdist().pend_page;
	if (!pend_page) {
		paddr_t paddr;
		/*
		 * The pending pages have to be at least 64kB aligned,
		 * hence the 'max(LPI_PENDBASE_SZ, SZ_64K)' below.
		 */
		pend_page = alloc_xenheap_pages(get_order_from_bytes(max(LPI_PENDBASE_SZ, SZ_64K)), 0);
		if (!pend_page) {
			its_err("Failed to allocate PENDBASE for CPU%d\n",
			       smp_processor_id());
			return;
		}
		memset(pend_page, 0, max(LPI_PENDBASE_SZ, SZ_64K));
		/* Make sure the GIC will observe the zero-ed page */
		clean_and_invalidate_dcache_va_range(pend_page, LPI_PENDBASE_SZ);

		paddr = __pa(pend_page);
		its_info("CPU%d: using LPI pending table @%pa\n",
			smp_processor_id(), &paddr);
		gic_data_rdist().pend_page = pend_page;
	}

	/* Disable LPIs */
	val = readl_relaxed(rbase + GICR_CTLR);
	val &= ~GICR_CTLR_ENABLE_LPIS;
	writel_relaxed(val, rbase + GICR_CTLR);

	/*
	 * Make sure any change to the table is observable by the GIC.
	 */
	dsb(sy);

	/* set PROPBASE */
	val = (__pa(gic_rdists->prop_page)   |
	       GICR_PROPBASER_InnerShareable |
	       GICR_PROPBASER_WaWb |
	       ((LPI_NRBITS - 1) & GICR_PROPBASER_IDBITS_MASK));

	writeq_relaxed(val, rbase + GICR_PROPBASER);
	tmp = readq_relaxed(rbase + GICR_PROPBASER);

	if ((tmp ^ val) & GICR_PROPBASER_SHAREABILITY_MASK) {
		its_info("GIC: using cache flushing for LPI property table\n");
		gic_rdists->flags |= RDIST_FLAGS_PROPBASE_NEEDS_FLUSHING;
	}

	/* set PENDBASE */
	val = (__pa(pend_page)               |
	       GICR_PROPBASER_InnerShareable |
	       GICR_PROPBASER_WaWb);

	writeq_relaxed(val, rbase + GICR_PENDBASER);

	/* Enable LPIs */
	val = readl_relaxed(rbase + GICR_CTLR);
	val |= GICR_CTLR_ENABLE_LPIS;
	writel_relaxed(val, rbase + GICR_CTLR);

	/* Make sure the GIC has seen the above */
	dsb(sy);
}

static void its_cpu_init_collection(void)
{
	struct its_node *its;
	int cpu;

	spin_lock(&its_lock);
	cpu = smp_processor_id();

	list_for_each_entry(its, &its_nodes, entry) {
		u64 target;

		/*
		 * We now have to bind each collection to its target
		 * redistributor.
		 */
		if (readq_relaxed(its->base + GITS_TYPER) & GITS_TYPER_PTA) {
			/*
			 * This ITS wants the physical address of the
			 * redistributor.
			 */
			target = gic_data_rdist().phys_base;
		} else {
			/*
			 * This ITS wants a linear CPU number.
			 */
			target = readq_relaxed(gic_data_rdist_rd_base() + GICR_TYPER);
			target = GICR_TYPER_CPU_NUMBER(target);
		}

		/* Perform collection mapping */
		its->collections[cpu].target_address = target;
		its->collections[cpu].col_id = cpu;
		its->collections[cpu].valid = 1;
		its->nr_collections++;

		its_send_mapc(its, &its->collections[cpu], 1);
		its_send_invall(its, &its->collections[cpu]);
	}

	spin_unlock(&its_lock);
}

int its_alloc_device_irq(struct its_device *dev, uint32_t *hwirq)
{
	int idx;

	idx = find_first_zero_bit(dev->lpi_map, dev->nr_lpis);
	if (idx == dev->nr_lpis)
		return -ENOSPC;

	*hwirq = dev->lpi_base + idx;
	set_bit(idx, dev->lpi_map);

	return 0;
}

static int its_send_cmd(struct vcpu *v, struct its_node *its,
			struct its_cmd_block *phys_cmd)
{
	struct its_cmd_block *cmd, *next_cmd;

	spin_lock(&its->lock);

	cmd = its_allocate_entry(its);
	if (!cmd)
		return 0;

	cmd->raw_cmd[0] = phys_cmd->raw_cmd[0];
	cmd->raw_cmd[1] = phys_cmd->raw_cmd[1];
	cmd->raw_cmd[2] = phys_cmd->raw_cmd[2];
	cmd->raw_cmd[3] = phys_cmd->raw_cmd[3];
	its_flush_cmd(its, cmd);

	next_cmd = its_post_commands(its);
	spin_unlock(&its->lock);

	its_wait_for_range_completion(its, cmd, next_cmd);

	return 1;
}

int gic_its_send_cmd(struct vcpu *v, struct its_node *its,
		     struct its_cmd_block *phys_cmd, int send_all)
{
	struct its_node *pits;
	int ret = 0;

	if (send_all) {
		list_for_each_entry(pits, &its_nodes, entry)
		ret = its_send_cmd(v, pits, phys_cmd);
	}
	else
		return its_send_cmd(v, its, phys_cmd);

	return ret;
}

static int its_force_quiescent(void __iomem *base)
{
	u32 count = 1000000;	/* 1s */
	u32 val;

	val = readl_relaxed(base + GITS_CTLR);
	if (val & GITS_CTLR_QUIESCENT)
		return 0;

	/* Disable the generation of all interrupts to this ITS */
	val &= ~GITS_CTLR_ENABLE;
	writel_relaxed(val, base + GITS_CTLR);

	/* Poll GITS_CTLR and wait until ITS becomes quiescent */
	while (1) {
		val = readl_relaxed(base + GITS_CTLR);
		if (val & GITS_CTLR_QUIESCENT)
			return 0;

		count--;
		if (!count)
			return -EBUSY;

		cpu_relax();
		udelay(1);
	}
}

static int its_probe(struct dt_device_node *node)
{
	paddr_t its_addr, its_size;
	struct its_node *its;
	void __iomem *its_base;
	u32 val;
	u64 baser, tmp;
	int err;

	err = dt_device_get_address(node, 0, &its_addr, &its_size);
	if (err) {
		its_warn("%s: no regs?\n", node->full_name);
		return -ENXIO;
	}

	its_base = ioremap_nocache(its_addr, its_size);
	if (!its_base) {
		its_warn("%s: unable to map registers\n", node->full_name);
		return -ENOMEM;
	}

	val = readl_relaxed(its_base + GITS_PIDR2) & GIC_PIDR2_ARCH_REV_MASK;
	if (val != 0x30 && val != 0x40) {
		its_warn("%s: no ITS detected, giving up\n", node->full_name);
		err = -ENODEV;
		goto out_unmap;
	}

	err = its_force_quiescent(its_base);
	if (err) {
		its_warn("%s: failed to quiesce, giving up\n",
			node->full_name);
		goto out_unmap;
	}

	its_info("ITS: %s\n", node->full_name);

	its = xzalloc(struct its_node);
	if (!its) {
		err = -ENOMEM;
		goto out_unmap;
	}

	spin_lock_init(&its->lock);
	INIT_LIST_HEAD(&its->entry);
	its->dt_node = node;
	its->base = its_base;
	its->phys_base = its_addr;
	its->phys_size = its_size;
	its->ite_size = ((readl_relaxed(its_base + GITS_TYPER) >> 4) & 0xf) + 1;

	if ( (readq_relaxed(its->base + GITS_TYPER) & GITS_TYPER_PTA) )
		pta_type = 1;
	else
		pta_type = 0;

	its->cmd_base = xzalloc_bytes(ITS_CMD_QUEUE_SZ);
	if (!its->cmd_base) {
		err = -ENOMEM;
		goto out_free_its;
	}
	its->cmd_write = its->cmd_base;

	err = its_alloc_tables(its);
	if (err)
		goto out_free_cmd;

	err = its_alloc_collections(its);
	if (err)
		goto out_free_tables;

	baser = (__pa(its->cmd_base)		|
		 GITS_CBASER_WaWb		|
		 GITS_CBASER_InnerShareable	|
		 (ITS_CMD_QUEUE_SZ / SZ_4K - 1)	|
		 GITS_CBASER_VALID);

	writeq_relaxed(baser, its->base + GITS_CBASER);
	tmp = readq_relaxed(its->base + GITS_CBASER);
	writeq_relaxed(0, its->base + GITS_CWRITER);
	writel_relaxed(GITS_CTLR_ENABLE, its->base + GITS_CTLR);

	if ((tmp ^ baser) & GITS_BASER_SHAREABILITY_MASK) {
		its_info("ITS: using cache flushing for cmd queue\n");
		its->flags |= ITS_FLAGS_CMDQ_NEEDS_FLUSHING;
	}
	spin_lock(&its_lock);
	list_add(&its->entry, &its_nodes);
	nr_its++;
	spin_unlock(&its_lock);

	return 0;
out_free_tables:
	its_free_tables(its);
out_free_cmd:
	xfree(its->cmd_base);
out_free_its:
	xfree(its);
out_unmap:
	iounmap(its_base);
	its_err("ITS: failed probing %s (%d)\n", node->full_name, err);
	return err;
}

static bool gic_rdists_supports_plpis(void)
{
	return !!(readl_relaxed(gic_data_rdist_rd_base() + GICR_TYPER) & GICR_TYPER_PLPIS);
}

int its_cpu_init(void)
{
	if (!list_empty(&its_nodes)) {
		if (!gic_rdists_supports_plpis()) {
			its_info("CPU%d: LPIs not supported\n", smp_processor_id());
			return -ENXIO;
		}
		its_cpu_init_lpis();
		its_cpu_init_collection();
	}

	return 0;
}

int its_init(struct dt_device_node *node, struct rdist_prop *rdists)
{
	struct dt_device_node *np = NULL;

	static const struct dt_device_match its_device_ids[] __initconst =
	{
		DT_MATCH_COMPATIBLE("arm,gic-v3-its"),
		{ /* sentinel */ },
	};

	while ((np = dt_find_matching_node(np, its_device_ids)))
	{
		if (!dt_find_property(np, "msi-controller", NULL))
		continue;
	}

	if (np) {
		its_probe(np);
	}

	if (list_empty(&its_nodes)) {
		its_warn("ITS: No ITS available, not enabling LPIs\n");
		return -ENXIO;
	}

	gic_rdists = rdists;
	gic_root_node = node;

	its_alloc_lpi_tables();
	its_lpi_init(rdists->id_bits);

	return 0;
}
