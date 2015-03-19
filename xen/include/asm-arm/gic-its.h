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

#ifndef __ASM_ARM_GIC_ITS_H__
#define __ASM_ARM_GIC_ITS_H__

#include <asm/gic_v3_defs.h>

struct its_node;

/* Collection ID mapping */
struct cid_mapping
{
    uint8_t nr_cid;
    /* XXX: assume one collection id per vcpu. can set to MAX_VCPUS? */
    /* Virtual Collection id */
    uint8_t vcid[32];
    /* Physical Collection id */
    uint8_t pcid[32];
    /* Virtual target address of this collection id */
    uint64_t vta[32];
};

/*
 * Per domain virtual ITS structure.
 * One per Physical ITS node available for the domain
 */
 
struct vgic_its
{
   spinlock_t lock;
   /* Emulation of BASER */
   paddr_t baser[8];
   /* Command queue base */
   paddr_t cmd_base;
   /* Command queue write pointer */
   paddr_t cmd_write;
   /* Command queue write saved pointer */
   paddr_t cmd_write_save;
   /* Command queue read pointer */
   paddr_t cmd_read;
   /* Command queue size */
   unsigned long cmd_qsize;
   /* ITS mmio physical base */
   paddr_t phys_base;
   /* ITS mmio physical size */
   unsigned long phys_size;
   /* ITS physical node */
   struct its_node *its;
   /* GICR ctrl register */
   uint32_t ctrl;
   /* Virtual to Physical Collection id mapping */
   struct cid_mapping cid_map;
};

struct vgic_lpi_conf
{
   /* LPI propbase */
   paddr_t propbase;
   /* percpu pendbase */
   paddr_t pendbase[MAX_VIRT_CPUS];
   /* Virtual LPI property table */
   void * prop_page;
};

struct vid_map
{
    uint32_t vlpi;
    uint32_t plpi;
    uint32_t id;
};

/*
 * The ITS command block, which is what the ITS actually parses.
 */
struct its_cmd_block {
    u64     raw_cmd[4];
};

/*
 * The ITS view of a device - belongs to an ITS, a collection, owns an
 * interrupt translation table, and a list of interrupts.
 */
struct its_device {
        struct list_head        entry;
        struct its_node         *its;
        struct its_collection   *collection;
        /* Virtual ITS node */
        struct vgic_its         *vits;
        paddr_t                 itt_addr;
        unsigned long           itt_size;
        unsigned long           *lpi_map;
        u32                     lpi_base;
        int                     nr_lpis;
        u32                     nr_ites;
        u32                     device_id;
        /* Spinlock for vlpi allocation */
        spinlock_t              vlpi_lock;
        /* vlpi bitmap */
        unsigned long           *vlpi_map;
        /* vlpi <=> plpi mapping */
        struct vid_map          *vlpi_entries;
};

static inline uint8_t its_decode_cmd(struct its_cmd_block *cmd)
{
    return cmd->raw_cmd[0] & 0xff;
}

static inline uint32_t its_decode_devid(struct its_cmd_block *cmd)
{
    return (cmd->raw_cmd[0] >> 32);
}

static inline uint32_t its_decode_event_id(struct its_cmd_block *cmd)
{
    return (uint32_t)cmd->raw_cmd[1];
}

static inline uint32_t its_decode_phys_id(struct its_cmd_block *cmd)
{
    return cmd->raw_cmd[1] >> 32;
}

static inline uint8_t its_decode_size(struct its_cmd_block *cmd)
{
    return (u8)(cmd->raw_cmd[1] & 0xff);
}

static inline uint64_t its_decode_itt(struct its_cmd_block *cmd)
{
    return (cmd->raw_cmd[2] & 0xffffffffff00ULL);
}

static inline int its_decode_valid(struct its_cmd_block *cmd)
{
    return cmd->raw_cmd[2] >> 63;
}

static inline uint64_t its_decode_target(struct its_cmd_block *cmd)
{
    return (cmd->raw_cmd[2] & 0xffffffff0000ULL);
}

static inline u16 its_decode_collection(struct its_cmd_block *cmd)
{
    return (u16)(cmd->raw_cmd[2] & 0xffffULL);
}

static inline void its_encode_cmd(struct its_cmd_block *cmd, u8 cmd_nr)
{
    cmd->raw_cmd[0] &= ~0xffUL;
    cmd->raw_cmd[0] |= cmd_nr;
}

static inline void its_encode_devid(struct its_cmd_block *cmd, u32 devid)
{
    cmd->raw_cmd[0] &= ~(0xffffUL << 32);
    cmd->raw_cmd[0] |= ((u64)devid) << 32;
}

static inline void its_encode_event_id(struct its_cmd_block *cmd, u32 id)
{
    cmd->raw_cmd[1] &= ~0xffffffffUL;
    cmd->raw_cmd[1] |= id;
}

static inline void its_encode_phys_id(struct its_cmd_block *cmd, u32 phys_id)
{
    cmd->raw_cmd[1] &= 0xffffffffUL;
    cmd->raw_cmd[1] |= ((u64)phys_id) << 32;
}

static inline void its_encode_size(struct its_cmd_block *cmd, u8 size)
{
    cmd->raw_cmd[1] &= ~0x1fUL;
    cmd->raw_cmd[1] |= size & 0x1f;
}

static inline void its_encode_itt(struct its_cmd_block *cmd, u64 itt_addr)
{
    cmd->raw_cmd[2] &= ~0xffffffffffffUL;
    cmd->raw_cmd[2] |= itt_addr & 0xffffffffff00UL;
}

static inline void its_encode_valid(struct its_cmd_block *cmd, int valid)
{
    cmd->raw_cmd[2] &= ~(1UL << 63);
    cmd->raw_cmd[2] |= ((u64)!!valid) << 63;
}

static inline void its_encode_target(struct its_cmd_block *cmd, u64 target_addr)
{
    cmd->raw_cmd[2] &= ~(0xffffffffUL << 16);
    cmd->raw_cmd[2] |= (target_addr & (0xffffffffUL << 16));
}

static inline void its_encode_collection(struct its_cmd_block *cmd, u16 col)
{
    cmd->raw_cmd[2] &= ~0xffffUL;
    cmd->raw_cmd[2] |= col;
}

int its_get_physical_cid(struct domain *d, uint32_t *col_id, uint64_t ta);
int its_get_target(uint8_t pcid, uint64_t *pta);
int its_alloc_device_irq(struct its_device *dev, uint32_t *plpi);
int gic_its_send_cmd(struct vcpu *v, struct its_node *its,
                     struct its_cmd_block *phys_cmd, int send_all);
int its_make_dt_node(const struct domain *d,
                     const struct dt_device_node *node, void *fdt);
void its_lpi_free(unsigned long *bitmap, int base, int nr_ids);
void its_set_affinity(struct irq_desc *d, int cpu);
void lpi_set_config(struct irq_desc *d, int enable);
unsigned long *its_lpi_alloc_chunks(int nirqs, int *base, int *nr_ids);
uint32_t its_get_pta_type(void);
uint32_t its_get_nr_its(void);
struct its_node * its_get_phys_node(uint32_t dev_id);
int vgic_its_unmap_lpi_prop(struct vcpu *v);
int vgic_its_get_pid(struct vcpu *v, uint32_t vlpi, uint32_t *plpi);
uint8_t vgic_its_get_priority(struct vcpu *v, uint32_t pid);
#endif /* __ASM_ARM_GIC_ITS_H__ */

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
