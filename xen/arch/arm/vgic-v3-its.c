/*
 * Copyright (C) 2013, 2014 ARM Limited, All Rights Reserved.
 * Author: Marc Zyngier <marc.zyngier@arm.com>
 *
 * Xen changes:
 * Vijaya Kumar K <Vijaya.Kumar@caviumnetworks.com>
 * Copyright (C) 2014 Cavium Inc.
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

#include <xen/bitops.h>
#include <xen/config.h>
#include <xen/lib.h>
#include <xen/init.h>
#include <xen/softirq.h>
#include <xen/irq.h>
#include <xen/list.h>
#include <xen/sched.h>
#include <xen/sizes.h>
#include <xen/xmalloc.h>
#include <asm/current.h>
#include <asm/device.h>
#include <asm/mmio.h>
#include <asm/io.h>
#include <asm/gic_v3_defs.h>
#include <asm/gic.h>
#include <asm/vgic.h>
#include <asm/gic-its.h>

/* GITS register definitions */
#define VITS_GITS_TYPER_HCC       (0xffU << 24)
#define VITS_GITS_TYPER_PTA_SHIFT (19)
#define VITS_GITS_DEV_BITS        (0x14U << 13)
#define VITS_GITS_ID_BITS         (0x13U << 8)
#define VITS_GITS_ITT_SIZE        (0x7U << 4)
#define VITS_GITS_DISTRIBUTED     (0x1U << 3)
#define VITS_GITS_PLPIS           (0x1U << 0)

/* GITS_PIDRn register values for ARM implementations */
#define GITS_PIDR0_VAL            (0x94)
#define GITS_PIDR1_VAL            (0xb4)
#define GITS_PIDR2_VAL            (0x3b)
#define GITS_PIDR3_VAL            (0x00)
#define GITS_PIDR4_VAL            (0x04)

//#define DEBUG_ITS

#ifdef DEBUG_ITS
# define DPRINTK(fmt, args...) printk(XENLOG_DEBUG fmt, ##args)
#else
# define DPRINTK(fmt, args...) do {} while ( 0 )
#endif

#ifdef DEBUG_ITS
static void dump_cmd(struct its_cmd_block *cmd)
{
    printk("CMD[0] = 0x%lx CMD[1] = 0x%lx CMD[2] = 0x%lx CMD[3] = 0x%lx\n",
           cmd->raw_cmd[0], cmd->raw_cmd[1], cmd->raw_cmd[2], cmd->raw_cmd[3]);
}
#endif

void vgic_its_disable_lpis(struct vcpu *v, uint32_t lpi)
{
    struct pending_irq *p;
    unsigned long flags;

    p = irq_to_pending(v, lpi);
    clear_bit(GIC_IRQ_GUEST_ENABLED, &p->status);
    gic_remove_from_queues(v, lpi);
    if ( p->desc != NULL )
    {
        spin_lock_irqsave(&p->desc->lock, flags);
        p->desc->handler->disable(p->desc);
        spin_unlock_irqrestore(&p->desc->lock, flags);
    }
}

void vgic_its_enable_lpis(struct vcpu *v, uint32_t lpi)
{
    struct pending_irq *p;
    unsigned long flags;

    p = irq_to_pending(v, lpi);
    set_bit(GIC_IRQ_GUEST_ENABLED, &p->status);

    spin_lock_irqsave(&v->arch.vgic.lock, flags);

    if ( !list_empty(&p->inflight) &&
         !test_bit(GIC_IRQ_GUEST_VISIBLE, &p->status) )
        gic_raise_guest_irq(v, p->desc->arch.virq, p->priority);

    spin_unlock_irqrestore(&v->arch.vgic.lock, flags);
    if ( p->desc != NULL )
    {
        spin_lock_irqsave(&p->desc->lock, flags);
        p->desc->handler->enable(p->desc);
        spin_unlock_irqrestore(&p->desc->lock, flags);
    }
}

static int vits_alloc_device_irq(struct its_device *dev, uint32_t id,
                                uint32_t *plpi, uint32_t vlpi, uint32_t vcol_id)
{

    int idx, i = 0;

    spin_lock(&dev->vlpi_lock);
    while ((i = find_next_bit(dev->vlpi_map, dev->nr_lpis, i)) < dev->nr_lpis )
    {
        if ( dev->vlpi_entries[i].vlpi == vlpi )
        {
             *plpi = dev->vlpi_entries[i].plpi;
             DPRINTK("Found plpi %d for device 0x%x with vlpi %d id %d\n",
                      *plpi, dev->dev_id, vlpi, dev->vlpi_entries[i].id);
             spin_unlock(&dev->vlpi_lock);
             return 0;
        }
        i++;
    }
 
    if ( its_alloc_device_irq(dev, plpi) )
        BUG_ON(1);

    idx = find_first_zero_bit(dev->vlpi_map, dev->nr_lpis);
    dev->vlpi_entries[idx].plpi = *plpi;
    dev->vlpi_entries[idx].vlpi = vlpi;
    dev->vlpi_entries[idx].id  = id;
    set_bit(idx, dev->vlpi_map);

    spin_unlock(&dev->vlpi_lock);

    DPRINTK("Allocated plpi %d for device 0x%x with vlpi %d id %d @idx %d\n",
            *plpi, dev->dev_id, vlpi, id, idx);

    return 0;
}

/* Should be called with its lock held */
static void vgic_its_unmap_id(struct vcpu *v, struct its_device *dev,
                              uint32_t id, int trash)
{
    int i = 0;

    DPRINTK("vITS: unmap id for device 0x%x id %d trash %d\n",
             dev->dev_id, id, trash);

    spin_lock(&dev->vlpi_lock);
    while ((i = find_next_bit(dev->vlpi_map, dev->nr_lpis, i)) < dev->nr_lpis )
    {
        if ( dev->vlpi_entries[i].id == id )
        {
            DPRINTK("vITS: un mapped id for device 0x%x id %d lpi %d\n",
                     dev->dev_id, dev->vlpi_entries[i].id,
                     dev->vlpi_entries[i].plpi);
            vgic_its_disable_lpis(v, dev->vlpi_entries[i].plpi);
            release_irq(dev->vlpi_entries[i].plpi, v->domain);
            dev->vlpi_entries[i].plpi = 0;
            dev->vlpi_entries[i].vlpi = 0;
            dev->vlpi_entries[i].id = 0;
            /* XXX: Clear LPI base here? */
            clear_bit(dev->vlpi_entries[i].plpi - dev->lpi_base, dev->lpi_map);
            clear_bit(i, dev->vlpi_map);
            goto out;
        }
        i++;
    }

    spin_unlock(&dev->vlpi_lock);
    dprintk(XENLOG_ERR, "vITS: id %d not found for device 0x%x to unmap\n",
           id, dev->device_id);

    return;
out:
    if ( bitmap_empty(dev->lpi_map, dev->nr_lpis) )
    {
        its_lpi_free(dev->lpi_map, dev->lpi_base, dev->nr_lpis);
        DPRINTK("vITS: Freeing lpi chunk\n");
    }
    /* XXX: Device entry is not removed on empty lpi list */
    spin_unlock(&dev->vlpi_lock);
}

static int vgic_its_check_device_id(struct vcpu *v, struct its_device *dev,
                                    uint32_t id)
{
    int i = 0;

    spin_lock(&dev->vlpi_lock);
    while ((i = find_next_bit(dev->vlpi_map, dev->nr_lpis, i)) < dev->nr_lpis )
    {
        if ( dev->vlpi_entries[i].id == id )
        {
            spin_unlock(&dev->vlpi_lock);
            return 0;
        }
        i++;
    }
    spin_unlock(&dev->vlpi_lock);

    return 1;
}

static struct its_device *vgic_its_check_device(struct vcpu *v, int dev_id)
{
    struct domain *d = v->domain;
    struct its_device *dev = NULL, *tmp;

    spin_lock(&d->arch.vits_devs.lock);
    list_for_each_entry(tmp, &d->arch.vits_devs.dev_list, entry)
    {
        if ( tmp->device_id == dev_id )
        {
            DPRINTK("vITS: Found device 0x%x\n", device_id);
            dev = tmp;
            break;
        }
    }
    spin_unlock(&d->arch.vits_devs.lock);

    return dev;
}

static int vgic_its_check_cid(struct vcpu *v,
                              struct vgic_its *vits,
                              uint8_t vcid, uint32_t *pcid)
{
    uint32_t nmap = vits->cid_map.nr_cid;
    int i;

    for ( i = 0; i < nmap; i++ )
    {
        if ( vcid == vits->cid_map.vcid[i] )
        {
            *pcid = vits->cid_map.pcid[i];
            DPRINTK("vITS: Found vcid %d for vcid %d\n", *pcid,
                     vits->cid_map.vcid[i]);
            return 0;
        }
    }

    return 1;
}

static uint64_t vgic_its_get_pta(struct vcpu *v, struct vgic_its *vits,
                                 uint64_t vta)
{
    
    uint32_t nmap = vits->cid_map.nr_cid;
    int i;
    uint8_t pcid;
    uint64_t pta;

    for ( i = 0; i < nmap; i++ )
    {
        if ( vta == vits->cid_map.vta[i] )
        {
            pcid = vits->cid_map.pcid[i];
            DPRINTK("vITS: Found vcid %d for vta 0x%lx\n", pcid,
                     vits->cid_map.vta[i]);
            if ( its_get_target(pcid, &pta) )
                BUG_ON(1);
            return pta;
        }
    }

    BUG_ON(1);
    return 1;
}

static int vgic_its_build_mapd_cmd(struct vcpu *v,
                                   struct its_cmd_block *virt_cmd,
                                   struct its_cmd_block *phys_cmd)
{
    unsigned long itt_addr;

    itt_addr = its_decode_itt(virt_cmd);
    /* Get ITT PA from ITT IPA */
    itt_addr = p2m_lookup(v->domain, itt_addr, NULL);
    its_encode_cmd(phys_cmd, GITS_CMD_MAPD);
    its_encode_devid(phys_cmd, its_decode_devid(virt_cmd));
    its_encode_size(phys_cmd, its_decode_size(virt_cmd));
    its_encode_itt(phys_cmd, itt_addr);
    its_encode_valid(phys_cmd, its_decode_valid(virt_cmd));

    DPRINTK("vITS: Build MAPD with itt_addr 0x%lx devId %d\n",itt_addr,
            its_decode_devid(virt_cmd));

    return 0;
}

static int vgic_its_build_sync_cmd(struct vcpu *v,
                                   struct vgic_its *vits,
                                   struct its_cmd_block *virt_cmd,
                                   struct its_cmd_block *phys_cmd)
{
    uint64_t pta;

    its_encode_cmd(phys_cmd, GITS_CMD_SYNC);
    pta = vgic_its_get_pta(v, vits, its_decode_target(virt_cmd));

    return 0;
}

static int vgic_its_build_mapvi_cmd(struct vcpu *v,
                                    struct vgic_its *vits,
                                    struct its_cmd_block *virt_cmd,
                                    struct its_cmd_block *phys_cmd)
{
    struct domain *d = v->domain;
    struct its_device *dev;
    uint32_t pcol_id;
    uint32_t pid;
    struct irq_desc *desc;
    uint32_t dev_id = its_decode_devid(virt_cmd);
    uint32_t id = its_decode_event_id(virt_cmd);
    uint8_t vcol_id = its_decode_collection(virt_cmd);
    uint32_t vid = its_decode_phys_id(virt_cmd);
    uint8_t cmd = its_decode_cmd(virt_cmd);

    DPRINTK("vITS: MAPVI: dev_id 0x%x vcol_id %d vid %d \n",
             dev_id, vcol_id, vid);

    /* Search if device entry exists */
    dev = vgic_its_check_device(v, dev_id);
    if ( dev == NULL )
    {
        dprintk(XENLOG_ERR, "vITS: MAPVI: Fail to find device 0x%x\n", dev_id);
        return 1;
    }

    /* Check if Collection id exists */
    if ( vgic_its_check_cid(v, vits, vcol_id, &pcol_id) )
    {
        dprintk(XENLOG_ERR, "vITS: MAPVI: with wrong Collection %d\n", vcol_id);
        return 1;
    }
    if ( vits_alloc_device_irq(dev, id, &pid, vid, vcol_id) )
    {
        dprintk(XENLOG_ERR, "vITS: MAPVI: Failed to alloc irq\n");
        return 1;
    }

    /* Allocate irq desc for this pirq */
    desc = irq_to_desc(pid);

    route_irq_to_guest(d, pid, "LPI");

     /* Assign device structure to desc data */
    desc->arch.dev = dev;
    desc->arch.virq = vid;

    its_encode_cmd(phys_cmd, GITS_CMD_MAPVI);
    its_encode_devid(phys_cmd, dev_id);

    if ( cmd == GITS_CMD_MAPI )
        its_encode_event_id(phys_cmd, vid);
    else
        its_encode_event_id(phys_cmd, its_decode_event_id(virt_cmd));

    its_encode_phys_id(phys_cmd, pid);
    its_encode_collection(phys_cmd, pcol_id);

    return 0;
}

static int vgic_its_build_movi_cmd(struct vcpu *v,
                                   struct vgic_its *vits,
                                   struct its_cmd_block *virt_cmd,
                                   struct its_cmd_block *phys_cmd)
{
    uint32_t pcol_id;
    struct its_device *dev;
    uint32_t dev_id = its_decode_devid(virt_cmd);
    uint8_t vcol_id = its_decode_collection(virt_cmd);
    uint32_t id = its_decode_event_id(virt_cmd);

    DPRINTK("vITS: MOVI: dev_id 0x%x vcol_id %d\n", dev_id, vcol_id);
    /* Search if device entry exists */
    dev = vgic_its_check_device(v, dev_id);
    if ( dev == NULL )
    {
        dprintk(XENLOG_ERR, "vITS: MOVI: Failed to find device 0x%x\n", dev_id);
        return 1;
    }

    /* Check if Collection id exists */
    if ( vgic_its_check_cid(v, vits, vcol_id, &pcol_id) )
    {
        dprintk(XENLOG_ERR, "vITS: MOVI: with wrong Collection %d\n", vcol_id);
        return 1;
    }

    if ( vgic_its_check_device_id(v, dev, id) )
    {
        dprintk(XENLOG_ERR, "vITS: MOVI: Invalid ID %d\n", id);
        return 1;
    }

    its_encode_cmd(phys_cmd, GITS_CMD_MOVI);
    its_encode_devid(phys_cmd, dev_id);
    its_encode_event_id(phys_cmd, id);
    its_encode_collection(phys_cmd, pcol_id);

    return 0;
}
   
static int vgic_its_build_discard_cmd(struct vcpu *v,
                                      struct vgic_its *vits,
                                      struct its_cmd_block *virt_cmd,
                                      struct its_cmd_block *phys_cmd)
{
    struct its_device *dev;
    uint32_t id = its_decode_event_id(virt_cmd);
    uint32_t dev_id = its_decode_devid(virt_cmd);

    DPRINTK("vITS: DISCARD: dev_id 0x%x id %d\n", dev_id, id);
    /* Search if device entry exists */
    dev = vgic_its_check_device(v, dev_id);
    if ( dev == NULL )
    {
        dprintk(XENLOG_ERR, "vITS: DISCARD: Failed to find device 0x%x\n",
                dev_id);
        return 1;
    }

    if ( vgic_its_check_device_id(v, dev, id) )
    {
        dprintk(XENLOG_ERR, "vITS: DISCARD: Invalid vID %d\n", id);
        return 1;
    }

    /* Check if PID is exists for this VID for this device and unmap it */
    vgic_its_unmap_id(v, dev, id, 0);

    /* Fetch and encode cmd */
    its_encode_cmd(phys_cmd, GITS_CMD_DISCARD);
    its_encode_devid(phys_cmd, its_decode_devid(virt_cmd));
    its_encode_event_id(phys_cmd, its_decode_event_id(virt_cmd));

    return 0;
}

static int vgic_its_build_inv_cmd(struct vcpu *v,
                                  struct vgic_its *vits,
                                  struct its_cmd_block *virt_cmd,
                                  struct its_cmd_block *phys_cmd)
{
    struct its_device *dev;
    uint32_t dev_id = its_decode_devid(virt_cmd);
    uint32_t id = its_decode_event_id(virt_cmd);

    DPRINTK("vITS: INV: dev_id 0x%x id %d\n",dev_id, id);
    /* Search if device entry exists */
    dev = vgic_its_check_device(v, dev_id);
    if ( dev == NULL )
    {
        dprintk(XENLOG_ERR, "vITS: INV: Failed to find device 0x%x\n", dev_id);
        return 1;
    }

    if ( vgic_its_check_device_id(v, dev, id) )
    {
        dprintk(XENLOG_ERR, "vITS: INV: Invalid ID %d\n", id);
        return 1;
    }

    its_encode_cmd(phys_cmd, GITS_CMD_INV);
    its_encode_devid(phys_cmd, dev_id);
    its_encode_event_id(phys_cmd, id);

    return 0;
}

static int vgic_its_build_clear_cmd(struct vcpu *v,
                                    struct vgic_its *vits,
                                    struct its_cmd_block *virt_cmd,
                                    struct its_cmd_block *phys_cmd)
{
    struct its_device *dev;
    uint32_t dev_id = its_decode_devid(virt_cmd);
    uint32_t id = its_decode_event_id(virt_cmd);

    DPRINTK("vITS: CLEAR: dev_id 0x%x id %d\n", dev_id, id);
    /* Search if device entry exists */
    dev = vgic_its_check_device(v, dev_id);
    if ( dev == NULL )
    {
        dprintk(XENLOG_ERR, "vITS: CLEAR: Fail to find device 0x%x\n", dev_id);
        return 1;
    }

    if ( vgic_its_check_device_id(v, dev, id) )
    {
        dprintk(XENLOG_ERR, "vITS: CLEAR: Invalid ID %d\n", id);
        return 1;
    }

    its_encode_cmd(phys_cmd, GITS_CMD_INV);
    its_encode_event_id(phys_cmd, id);

    return 0;
}

static int vgic_its_build_invall_cmd(struct vcpu *v,
                                     struct vgic_its *vits,
                                     struct its_cmd_block *virt_cmd,
                                     struct its_cmd_block *phys_cmd)
{
    uint32_t pcol_id;
    uint8_t vcol_id = its_decode_collection(virt_cmd);

    DPRINTK("vITS: INVALL: vCID %d\n", vcol_id);
    /* Check if Collection id exists */
    if ( vgic_its_check_cid(v, vits, vcol_id, &pcol_id) )
    {
        dprintk(XENLOG_ERR, "vITS: INVALL: Wrong Collection %d\n", vcol_id);
        return 1;
    }

    its_encode_cmd(phys_cmd, GITS_CMD_INVALL);
    its_encode_collection(phys_cmd, pcol_id);

    return 0;
}

static int vgic_its_build_int_cmd(struct vcpu *v,
                                  struct vgic_its *vits,
                                  struct its_cmd_block *virt_cmd,
                                  struct its_cmd_block *phys_cmd)
{
    uint32_t dev_id = its_decode_devid(virt_cmd);
    struct its_device *dev;
    uint32_t id = its_decode_event_id(virt_cmd);

    DPRINTK("vITS: INT: Device 0x%x id %d\n", its_decode_devid(virt_cmd), id);
    /* Search if device entry exists */
    dev = vgic_its_check_device(v, dev_id);
    if ( dev == NULL )
    {
        dprintk(XENLOG_ERR, "vITS: INT: Failed to find device 0x%x\n", dev_id);
        return 1;
    }

    if ( vgic_its_check_device_id(v, dev, id) )
    {
        dprintk(XENLOG_ERR, "vITS: INT: Invalid ID %d\n", id);
        return 1;
    }

    its_encode_cmd(phys_cmd, GITS_CMD_INT);
    its_encode_devid(phys_cmd, its_decode_devid(virt_cmd));
    its_encode_event_id(phys_cmd, its_decode_event_id(virt_cmd));

    return 0;
}

static void vgic_its_free_device(struct its_device *dev)
{
        xfree(dev);
}

static int vgic_its_add_device(struct vcpu *v, struct vgic_its *vits,
                               struct its_cmd_block *virt_cmd)
{
    struct domain *d = v->domain;
    struct its_device *dev;
    int lpi_base, nr_lpis, nr_vecs;

    /* Allocate device only if valid bit is set */
    if ( its_decode_valid(virt_cmd) )
    {
        dev = xzalloc(struct its_device);
        if ( dev == NULL )
           return ENOMEM;

        spin_lock(&d->arch.vits_devs.lock);
        dev->device_id = its_decode_devid(virt_cmd);
        dev->itt_size = its_decode_size(virt_cmd);
        dev->itt_addr = its_decode_itt(virt_cmd);
        INIT_LIST_HEAD(&dev->entry);
        /* TODO: use pci_conf_read() to read MSI vectors count */
        nr_vecs = 32;
        dev->lpi_map = its_lpi_alloc_chunks(nr_vecs, &lpi_base, &nr_lpis);
        dev->lpi_base = lpi_base;
        dev->nr_lpis = nr_lpis;
        spin_lock_init(&dev->vlpi_lock);
        dev->vlpi_entries = xzalloc_array(struct vid_map, nr_lpis);
        if ( dev->vlpi_entries == NULL )
        {
            spin_unlock(&d->arch.vits_devs.lock);
            return ENOMEM;
        }
        dev->vlpi_map = xzalloc_bytes(nr_lpis/8);
        if ( dev->vlpi_map == NULL )
        {
            spin_unlock(&d->arch.vits_devs.lock);
            return ENOMEM;
        }

        /*
         * TODO: Get ITS node of this pci device.
         * Update with proper helper function after PCI-passthrough support
         */
        dev->its = its_get_phys_node(dev->device_id);
        dev->vits = vits;
        list_add(&dev->entry, &d->arch.vits_devs.dev_list);
        spin_unlock(&d->arch.vits_devs.lock);
        DPRINTK("vITS: Added device dev_id 0x%x\n", its_decode_devid(virt_cmd));
    }
    else
    {
        spin_lock(&d->arch.vits_devs.lock);
        /* Search if device entry exists */
        dev = vgic_its_check_device(v, its_decode_devid(virt_cmd));
        if ( dev == NULL )
        {
            dprintk(XENLOG_ERR, "vITS: Failed to find device 0x%x\n",
                    dev->device_id);
            spin_unlock(&d->arch.vits_devs.lock);
            return 1;
        }

        /* Clear all lpis of this device */
        vgic_its_unmap_id(v, dev, 0, 1);

        list_del(&dev->entry);
        vgic_its_free_device(dev);
        spin_unlock(&d->arch.vits_devs.lock);
        DPRINTK("vITS: Removed device dev_id 0x%x\n", its_decode_devid(virt_cmd));
    }

    return 0;
}

static int vgic_its_process_mapc(struct vcpu *v, struct vgic_its *vits,
                                 struct its_cmd_block *virt_cmd)
{
    uint32_t pcid = 0;
    int idx;
    uint32_t nmap;
    uint8_t vcol_id;
    uint64_t vta = 0;

    nmap = vits->cid_map.nr_cid;
    vcol_id = its_decode_collection(virt_cmd);
    vta = its_decode_target(virt_cmd);

    for ( idx = 0; idx < nmap; idx++ )
    {
        if ( vcol_id == vits->cid_map.vcid[idx] )
            break;
    }
    if ( idx == nmap )
        vits->cid_map.vcid[idx] = vcol_id;

    if ( its_get_physical_cid(v->domain, &pcid, vta) )
        BUG_ON(1);
    vits->cid_map.pcid[idx] = pcid;
    vits->cid_map.vta[idx] = vta;
    vits->cid_map.nr_cid++;
    DPRINTK("vITS: MAPC: vCID %d vTA 0x%lx added @idx 0x%x \n",
             vcol_id, vta, idx);

    return 0;
}

static void vgic_its_update_read_ptr(struct vcpu *v, struct vgic_its *vits)
{
    vits->cmd_read = vits->cmd_write;
}

#ifdef DEBUG_ITS
char *cmd_str[] = {
        [GITS_CMD_MOVI]    = "MOVI",
        [GITS_CMD_INT]     = "INT",
        [GITS_CMD_CLEAR]   = "CLEAR",
        [GITS_CMD_SYNC]    = "SYNC",
        [GITS_CMD_MAPD]    = "MAPD",
        [GITS_CMD_MAPC]    = "MAPC",
        [GITS_CMD_MAPVI]   = "MAPVI",
        [GITS_CMD_MAPI]    = "MAPI",
        [GITS_CMD_INV]     = "INV",
        [GITS_CMD_INVALL]  = "INVALL",
        [GITS_CMD_MOVALL]  = "MOVALL",
        [GITS_CMD_DISCARD] = "DISCARD",
    };
#endif

#define SEND_NONE 0x0
#define SEND_CMD 0x1
#define SEND_ALL 0x2

static int vgic_its_parse_its_command(struct vcpu *v, struct vgic_its *vits,
                                      struct its_cmd_block *virt_cmd)
{
    uint8_t cmd = its_decode_cmd(virt_cmd);
    struct its_cmd_block phys_cmd;
    int ret;
    int send_flag = SEND_CMD;

#ifdef DEBUG_ITS
    DPRINTK("vITS: Received cmd %s (0x%x)\n", cmd_str[cmd], cmd);
    DPRINTK("Dump Virt cmd: ");
    dump_cmd(virt_cmd);
#endif

    memset(&phys_cmd, 0x0, sizeof(struct its_cmd_block));
    switch ( cmd )
    {
    case GITS_CMD_MAPD:
        /* create virtual device entry */
        if ( vgic_its_add_device(v, vits, virt_cmd) )
            return ENODEV;
        ret = vgic_its_build_mapd_cmd(v, virt_cmd, &phys_cmd);
        break;
    case GITS_CMD_MAPC:
        /* Physical ITS driver already mapped physical Collection */
        send_flag = SEND_NONE;
        ret =  vgic_its_process_mapc(v, vits, virt_cmd);
        break;
    case GITS_CMD_MAPI:
        /* MAPI is same as MAPVI */
    case GITS_CMD_MAPVI:
        ret = vgic_its_build_mapvi_cmd(v, vits, virt_cmd, &phys_cmd);
        break;
    case GITS_CMD_MOVI:
        ret = vgic_its_build_movi_cmd(v, vits, virt_cmd, &phys_cmd);
        break;
    case GITS_CMD_DISCARD:
        ret = vgic_its_build_discard_cmd(v, vits, virt_cmd, &phys_cmd);
        break;
    case GITS_CMD_INV:
        ret = vgic_its_build_inv_cmd(v, vits, virt_cmd, &phys_cmd);
        break;
    case GITS_CMD_INVALL:
        /* XXX: SYNC is sent on all physical ITS */
        send_flag = SEND_ALL;
        ret = vgic_its_build_invall_cmd(v, vits, virt_cmd, &phys_cmd);
        break;
    case GITS_CMD_INT:
        ret = vgic_its_build_int_cmd(v, vits, virt_cmd, &phys_cmd);
        break;
    case GITS_CMD_CLEAR:
        ret = vgic_its_build_clear_cmd(v, vits, virt_cmd, &phys_cmd);
        break;
    case GITS_CMD_SYNC:
        /* XXX: SYNC is sent on all physical ITS */
        send_flag = SEND_ALL;
        ret = vgic_its_build_sync_cmd(v, vits, virt_cmd, &phys_cmd);
        break;
        /*TODO:  GITS_CMD_MOVALL not implemented */
    default:
       dprintk(XENLOG_ERR, "vITS: Unhandled command cmd %d\n", cmd);
       return 1;
    }

#ifdef DEBUG_ITS
    DPRINTK("Dump Phys cmd: ");
    dump_cmd(&phys_cmd);
#endif

    if ( ret )
    {
       dprintk(XENLOG_ERR, "vITS: Failed to handle cmd %d\n", cmd);
       return 1;
    }

    if ( send_flag )
    {
       /* XXX: Always send on physical ITS on which device is assingned */
       if ( !gic_its_send_cmd(v,
             its_get_phys_node(its_decode_devid(&phys_cmd)),
             &phys_cmd, (send_flag & SEND_ALL)) )
       {
           dprintk(XENLOG_ERR, "vITS: Failed to push cmd %d\n", cmd);
           return 1;
       }
    }

    return 0;
}

/* Called with its lock held */
static int vgic_its_read_virt_cmd(struct vcpu *v,
                                  struct vgic_its *vits,
                                  struct its_cmd_block *virt_cmd)
{
    struct page_info * page;
    void *p;
    paddr_t paddr;
    paddr_t maddr = vits->cmd_base & 0xfffffffff000UL;
    uint64_t offset;

    /* CMD Q can be more than 1 page. Map only page that is required */
    maddr = ((vits->cmd_base & 0xfffffffff000UL) +
              vits->cmd_write_save ) & PAGE_MASK;

    paddr = p2m_lookup(v->domain, maddr, NULL);

    DPRINTK("vITS: Mapping CMD Q maddr 0x%lx paddr 0x%lx write_save 0x%lx \n",
            maddr, paddr, vits->cmd_write_save);
    page = get_page_from_paddr(v->domain, paddr, 0);
    if ( page == NULL )
    {
        dprintk(XENLOG_ERR, "vITS: Failed to get command page\n");
        return 1;
    }

    p = __map_domain_page(page);

    /* Offset within the mapped 4K page to read */
    offset = vits->cmd_write_save & 0xfff;

    memcpy(virt_cmd, p + offset, sizeof(struct its_cmd_block));

    /* No command queue is created by vits to check on Q full */
    vits->cmd_write_save += 0x20;
    if ( vits->cmd_write_save == vits->cmd_qsize )
    {
         DPRINTK("vITS: Reset write_save 0x%lx qsize 0x%lx \n",
                 vits->cmd_write_save,
                 vits->cmd_qsize);
                 vits->cmd_write_save = 0x0;
    }

    unmap_domain_page(p);
    put_page(page);

    return 0;
}

int vgic_its_process_cmd(struct vcpu *v, struct vgic_its *vits)
{
    struct its_cmd_block virt_cmd;

    /* XXX: Currently we are processing one cmd at a time */
    ASSERT(spin_is_locked(&vits->lock));

    do {
        if ( vgic_its_read_virt_cmd(v, vits, &virt_cmd) )
            goto err;
        if ( vgic_its_parse_its_command(v, vits, &virt_cmd) )
            goto err;
    } while ( vits->cmd_write != vits->cmd_write_save );

    vits->cmd_write_save = vits->cmd_write;
    DPRINTK("vITS: write_save 0x%lx write 0x%lx \n",
            vits->cmd_write_save,
            vits->cmd_write);
    /* XXX: Currently we are processing one cmd at a time */
    vgic_its_update_read_ptr(v, vits);

    dsb(ishst);

    return 1;
err:
    dprintk(XENLOG_ERR, "vITS: Failed to process guest cmd\n");
    return 0;
}

/* Search device structure and get corresponding plpi */
int vgic_its_get_pid(struct vcpu *v, uint32_t vlpi, uint32_t *plpi)
{
    struct domain *d = v->domain;
    struct its_device *dev;
    int i = 0;

    spin_lock(&d->arch.vits_devs.lock);
    list_for_each_entry( dev, &d->arch.vits_devs.dev_list, entry )
    {
        i = 0;
        while ((i = find_next_bit(dev->vlpi_map, dev->nr_lpis, i)) < dev->nr_lpis )
        {
            if ( dev->vlpi_entries[i].vlpi == vlpi )
            {
                *plpi = dev->vlpi_entries[i].plpi;
                spin_unlock(&d->arch.vits_devs.lock);
                return 0;
            }
            i++;
        }
    }
    spin_unlock(&d->arch.vits_devs.lock);

    return 1;
}

uint8_t vgic_its_get_priority(struct vcpu *v, uint32_t pid)
{
    uint8_t priority;
  
    priority =  readb_relaxed(v->domain->arch.lpi_conf->prop_page + pid);
    priority &= 0xfc;

    return priority;
}

static int vgic_v3_gits_lpi_mmio_read(struct vcpu *v, mmio_info_t *info)
{
    uint32_t offset;
    struct hsr_dabt dabt = info->dabt;
    struct cpu_user_regs *regs = guest_cpu_user_regs();
    register_t *r = select_user_reg(regs, dabt.reg);
    uint8_t cfg;

    offset = info->gpa -
             (v->domain->arch.lpi_conf->propbase & 0xfffffffff000UL);

    if ( offset < SZ_64K )
    {
        DPRINTK("vITS: LPI Table read offset 0x%x\n", offset );
        cfg = readb_relaxed(v->domain->arch.lpi_conf->prop_page + offset);
        *r = cfg;
        return 1;
    }
    else
        dprintk(XENLOG_ERR, "vITS: LPI Table read with wrong offset 0x%x\n",
                offset);

    return 0;
}

static int vgic_v3_gits_lpi_mmio_write(struct vcpu *v, mmio_info_t *info)
{
    uint32_t offset;
    uint32_t pid, vid;
    uint8_t cfg;
    bool_t enable;
    struct hsr_dabt dabt = info->dabt;
    struct cpu_user_regs *regs = guest_cpu_user_regs();
    register_t *r = select_user_reg(regs, dabt.reg);

    offset = info->gpa -
             (v->domain->arch.lpi_conf->propbase & 0xfffffffff000UL);

    vid = offset + NR_GIC_LPI;
    if ( offset < SZ_64K )
    {
        DPRINTK("vITS: LPI Table write offset 0x%x\n", offset );
        if ( vgic_its_get_pid(v, vid, &pid) )
        {
            dprintk(XENLOG_ERR, "vITS: pID not found for vid %d\n", vid);
            return 0;
        }
      
        cfg = readb_relaxed(v->domain->arch.lpi_conf->prop_page + offset);
        enable = (cfg & *r) & 0x1;

        if ( !enable )
             vgic_its_enable_lpis(v, pid);
        else
             vgic_its_disable_lpis(v, pid);

        /* Update virtual prop page */
        writeb_relaxed((*r & 0xff),
                        v->domain->arch.lpi_conf->prop_page + offset);
        
        return 1;
    }
    else
        dprintk(XENLOG_ERR, "vITS: LPI Table write with wrong offset 0x%x\n",
                offset);

    return 0; 
}

static const struct mmio_handler_ops vgic_gits_lpi_mmio_handler = {
    .read_handler  = vgic_v3_gits_lpi_mmio_read,
    .write_handler = vgic_v3_gits_lpi_mmio_write,
};

int vgic_its_unmap_lpi_prop(struct vcpu *v)
{
    paddr_t maddr;
    uint32_t lpi_size;
    int i;
    
    maddr = v->domain->arch.lpi_conf->propbase & 0xfffffffff000UL;
    lpi_size = 1UL << ((v->domain->arch.lpi_conf->propbase & 0x1f) + 1);

    DPRINTK("vITS: Unmap guest LPI conf table maddr 0x%lx lpi_size 0x%x\n", 
             maddr, lpi_size);

    if ( lpi_size < SZ_64K )
    {
        dprintk(XENLOG_ERR, "vITS: LPI Prop page < 64K\n");
        return 0;
    }

    /* XXX: As per 4.8.9 each re-distributor shares a common LPI configuration table 
     * So one set of mmio handlers to manage configuration table is enough
     */
    for ( i = 0; i < lpi_size / PAGE_SIZE; i++ )
        guest_physmap_remove_page(v->domain, paddr_to_pfn(maddr),
                                gmfn_to_mfn(v->domain, paddr_to_pfn(maddr)), 0);

    /* Register mmio handlers for this region */
    register_mmio_handler(v->domain, &vgic_gits_lpi_mmio_handler,
                          maddr, lpi_size);

    /* Allocate Virtual LPI Property table */
    v->domain->arch.lpi_conf->prop_page =
        alloc_xenheap_pages(get_order_from_bytes(lpi_size), 0);
    if ( !v->domain->arch.lpi_conf->prop_page )
    {
        dprintk(XENLOG_ERR, "vITS: Failed to allocate LPI Prop page\n");
        return 0;
    }

    memset(v->domain->arch.lpi_conf->prop_page, 0xa2, lpi_size);

    return 1;
}

struct vgic_its *its_to_vits(struct vcpu *v, paddr_t phys_base)
{
    struct vgic_its *vits = NULL;
    int i;

    /* Mask 64K offset */
    phys_base = phys_base & ~(SZ_64K - 1);
    if ( is_hardware_domain(v->domain) )
    {
        for ( i = 0; i < its_get_nr_its(); i++ )
        {
            if ( v->domain->arch.vits[i].phys_base == phys_base )
            {
                vits =  &v->domain->arch.vits[i];
                break;
            }
        }
    }
    else
        vits = &v->domain->arch.vits[0];

    return vits;
}

static inline void vits_spin_lock(struct vgic_its *vits)
{
    spin_lock(&vits->lock);
}

static inline void vits_spin_unlock(struct vgic_its *vits)
{
    spin_unlock(&vits->lock);
}

static int vgic_v3_gits_mmio_read(struct vcpu *v, mmio_info_t *info)
{
    struct vgic_its *vits;
    struct hsr_dabt dabt = info->dabt;
    struct cpu_user_regs *regs = guest_cpu_user_regs();
    register_t *r = select_user_reg(regs, dabt.reg);
    uint64_t val = 0;
    uint32_t index, gits_reg;

    vits = its_to_vits(v, info->gpa);
    if ( vits == NULL ) BUG_ON(1);

    gits_reg = info->gpa - vits->phys_base;

    if ( gits_reg >= SZ_64K )
    {
        gdprintk(XENLOG_G_WARNING, "vGITS: unknown gpa read address \
                  %"PRIpaddr"\n", info->gpa);
        return 0;
    }

    switch ( gits_reg )
    {
    case GITS_CTLR:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        return 1;
    case GITS_IIDR:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        return 1;
    case GITS_TYPER:
         /* GITS_TYPER support word read */
        vits_spin_lock(vits);
        val = ((its_get_pta_type() << VITS_GITS_TYPER_PTA_SHIFT) |
               VITS_GITS_TYPER_HCC   | VITS_GITS_DEV_BITS |
               VITS_GITS_ID_BITS     | VITS_GITS_ITT_SIZE |
               VITS_GITS_DISTRIBUTED | VITS_GITS_PLPIS);
        if ( dabt.size == DABT_DOUBLE_WORD )
            *r = val;
        else if ( dabt.size == DABT_WORD )
            *r = (u32)(val >> 32);
        else
        {
            vits_spin_unlock(vits);
            goto bad_width;
        }
        vits_spin_unlock(vits);
        return 1;
    case GITS_TYPER + 4:
        if (dabt.size != DABT_WORD ) goto bad_width;
        vits_spin_lock(vits);
        val = ((its_get_pta_type() << VITS_GITS_TYPER_PTA_SHIFT) |
               VITS_GITS_TYPER_HCC   | VITS_GITS_DEV_BITS |
               VITS_GITS_ID_BITS     | VITS_GITS_ITT_SIZE |
               VITS_GITS_DISTRIBUTED | VITS_GITS_PLPIS);
        *r = (u32)val;
        vits_spin_unlock(vits);
        return 1;
    case 0x0010 ... 0x007c:
    case 0xc000 ... 0xffcc:
        /* Implementation defined -- read ignored */
        dprintk(XENLOG_ERR,
                "vGITS: read unknown 0x000c - 0x007c r%d offset %#08x\n",
                dabt.reg, gits_reg);
        goto read_as_zero;
    case GITS_CBASER:
        vits_spin_lock(vits);
        if ( dabt.size == DABT_DOUBLE_WORD )
            *r = vits->cmd_base && 0xc7ffffffffffffffUL;
        else if ( dabt.size == DABT_WORD )
            *r = (u32)vits->cmd_base;
        else
        {
            vits_spin_unlock(vits);
            goto bad_width;
        }
        vits_spin_unlock(vits);
        return 1;
    case GITS_CBASER + 4:
         /* CBASER support word read */
        if (dabt.size != DABT_WORD ) goto bad_width;
        vits_spin_lock(vits);
        *r = (u32)(vits->cmd_base >> 32);
        vits_spin_unlock(vits);
        return 1;
    case GITS_CWRITER:
        vits_spin_lock(vits);
        if ( dabt.size == DABT_DOUBLE_WORD )
            *r = vits->cmd_write;
        else if ( dabt.size == DABT_WORD )
            *r = (u32)vits->cmd_write;
        else
        {
            vits_spin_unlock(vits);
            goto bad_width;
        }
        vits_spin_unlock(vits);
        return 1;
    case GITS_CWRITER + 4:
         /* CWRITER support word read */
        if ( dabt.size != DABT_WORD ) goto bad_width;
        vits_spin_lock(vits);
        *r = (u32)(vits->cmd_write >> 32);
        vits_spin_unlock(vits);
        return 1;
    case GITS_CREADR:
        vits_spin_lock(vits);
        if ( dabt.size == DABT_DOUBLE_WORD )
            *r = vits->cmd_read;
        else if ( dabt.size == DABT_WORD )
            *r = (u32)vits->cmd_read;
        else
        {
            vits_spin_unlock(vits);
            goto bad_width;
        }
        vits_spin_unlock(vits);
        return 1;
    case GITS_CREADR + 4:
         /* CREADR support word read */
        if ( dabt.size != DABT_WORD ) goto bad_width;
        vits_spin_lock(vits);
        *r = (u32)(vits->cmd_read >> 32);
        vits_spin_unlock(vits);
        return 1;
    case 0x0098 ... 0x009c:
    case 0x00a0 ... 0x00fc:
    case 0x0140 ... 0xbffc:
        /* Reserved -- read ignored */
        dprintk(XENLOG_ERR,
                "vGITS: read unknown 0x0098-9c or 0x00a0-fc r%d offset %#08x\n",
                dabt.reg, gits_reg);
        goto read_as_zero;
    case GITS_BASER ... GITS_BASERN:
        vits_spin_lock(vits);
        index = (gits_reg - GITS_BASER) / 8;
        if ( dabt.size == DABT_DOUBLE_WORD )
            *r = vits->baser[index];
        else if ( dabt.size == DABT_WORD )
        {
            if ( (gits_reg % 8) == 0 )
                *r = (u32)vits->baser[index];
            else
                *r = (u32)(vits->baser[index] >> 32);
        }
        else
        {
            vits_spin_unlock(vits);
            goto bad_width;
        }
        vits_spin_unlock(vits);
        return 1;
    case GITS_PIDR0:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        *r = GITS_PIDR0_VAL;
        return 1;
    case GITS_PIDR1:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        *r = GITS_PIDR1_VAL;
        return 1;
    case GITS_PIDR2:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        *r = GITS_PIDR2_VAL;
        return 1;
    case GITS_PIDR3:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        *r = GITS_PIDR3_VAL;
        return 1;
    case GITS_PIDR4:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        *r = GITS_PIDR4_VAL;
        return 1;
    case GITS_PIDR5 ... GITS_PIDR7:
        goto read_as_zero;
   default:
        dprintk(XENLOG_ERR, "vGITS: unhandled read r%d offset %#08x\n",
               dabt.reg, gits_reg);
        return 0;
    }

bad_width:
    dprintk(XENLOG_ERR, "vGITS: bad read width %d r%d offset %#08x\n",
           dabt.size, dabt.reg, gits_reg);
    domain_crash_synchronous();
    return 0;

read_as_zero:
    if ( dabt.size != DABT_WORD ) goto bad_width;
    *r = 0;
    return 1;
}

static int vgic_v3_gits_mmio_write(struct vcpu *v, mmio_info_t *info)
{
    struct vgic_its *vits;
    struct hsr_dabt dabt = info->dabt;
    struct cpu_user_regs *regs = guest_cpu_user_regs();
    register_t *r = select_user_reg(regs, dabt.reg);
    int ret;
    uint32_t index, gits_reg;
    uint64_t val;

    vits = its_to_vits(v, info->gpa);
    if ( vits == NULL ) BUG_ON(1);

    gits_reg = info->gpa - vits->phys_base;

    if ( gits_reg >= SZ_64K )
    {
        gdprintk(XENLOG_G_WARNING, "vGIC-ITS: unknown gpa write address"
                 " %"PRIpaddr"\n", info->gpa);
        return 0;
    }

    switch ( gits_reg )
    {
    case GITS_CTLR:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        vits_spin_lock(vits);
        vits->ctrl = *r;
        vits_spin_unlock(vits);
        return 1;
    case GITS_IIDR:
        /* R0 -- write ignored */
        goto write_ignore;
    case GITS_TYPER:
    case GITS_TYPER + 4:
        /* R0 -- write ignored */
        goto write_ignore;
    case 0x0010 ... 0x007c:
    case 0xc000 ... 0xffcc:
        /* Implementation defined -- write ignored */
        dprintk(XENLOG_ERR,
                "vGITS: write to unknown 0x000c - 0x007c r%d offset %#08x\n",
                dabt.reg, gits_reg);
        goto write_ignore;
    case GITS_CBASER:
        if ( dabt.size == DABT_BYTE ) goto bad_width;
        vits_spin_lock(vits);
        if ( dabt.size == DABT_DOUBLE_WORD )
            vits->cmd_base = *r;
        else
        {
            val = vits->cmd_base & 0xffffffff00000000UL;
            val = (*r) | val;
            vits->cmd_base =  val;
        }
        vits->cmd_qsize  =  SZ_4K * ((*r & 0xff) + 1);
        vits_spin_unlock(vits);
        return 1;
    case GITS_CBASER + 4:
         /* CBASER support word read */
        if (dabt.size != DABT_WORD ) goto bad_width;
        vits_spin_lock(vits);
        val = vits->cmd_base & 0xffffffffUL;
        val = ((*r & 0xffffffffUL) << 32 ) | val;
        vits->cmd_base =  val;
        /* No Need to update cmd_qsize with higher word write */
        vits_spin_unlock(vits);
        return 1;
    case GITS_CWRITER:
        if ( dabt.size == DABT_BYTE ) goto bad_width;
        vits_spin_lock(vits);
        if ( dabt.size == DABT_DOUBLE_WORD )
            vits->cmd_write = *r;
        else
        {
            val = vits->cmd_write & 0xffffffff00000000UL;
            val = (*r) | val;
            vits->cmd_write =  val;
        }
        ret = vgic_its_process_cmd(v, vits);
        vits_spin_unlock(vits);
        return ret;
    case GITS_CWRITER + 4:
        if (dabt.size != DABT_WORD ) goto bad_width;
        vits_spin_lock(vits);
        val = vits->cmd_write & 0xffffffffUL;
        val = ((*r & 0xffffffffUL) << 32) | val;
        vits->cmd_write =  val;
        ret = vgic_its_process_cmd(v, vits);
        vits_spin_unlock(vits);
        return ret;
    case GITS_CREADR:
        /* R0 -- write ignored */
        goto write_ignore;
    case 0x0098 ... 0x009c:
    case 0x00a0 ... 0x00fc:
    case 0x0140 ... 0xbffc:
        /* Reserved -- write ignored */
        dprintk(XENLOG_ERR,
                "vGITS: write to unknown 0x98-9c or 0xa0-fc r%d offset %#08x\n",
                dabt.reg, gits_reg);
        goto write_ignore;
    case GITS_BASER ... GITS_BASERN:
        /* Nothing to do with this values. Just store and emulate */
        vits_spin_lock(vits);
        index = (gits_reg - GITS_BASER) / 8;
        if ( dabt.size == DABT_DOUBLE_WORD )
            vits->baser[index] = *r;
        else if ( dabt.size == DABT_WORD )
        {
            if ( (gits_reg % 8) == 0 )
            {
                val = vits->cmd_write & 0xffffffff00000000UL;
                val = (*r) | val;
                vits->baser[index] = val;
            }
            else
            {
                val = vits->baser[index] & 0xffffffffUL;
                val = ((*r & 0xffffffffUL) << 32) | val;
                vits->baser[index] = val;
            }
        }
        else
        {
            goto bad_width;
            vits_spin_unlock(vits);
        }
        vits_spin_unlock(vits);
        return 1;
    case GITS_PIDR7 ... GITS_PIDR0:
        /* R0 -- write ignored */
        goto write_ignore;
   default:
        dprintk(XENLOG_ERR, "vGITS: unhandled write r%d offset %#08x\n",
                dabt.reg, gits_reg);
        return 0;
    }

bad_width:
    dprintk(XENLOG_ERR, "vGITS: bad write width %d r%d offset %#08x\n",
           dabt.size, dabt.reg, gits_reg);
    domain_crash_synchronous();
    return 0;

write_ignore:
    if ( dabt.size != DABT_WORD ) goto bad_width;
    *r = 0;
    return 1;
}

static const struct mmio_handler_ops vgic_gits_mmio_handler = {
    .read_handler  = vgic_v3_gits_mmio_read,
    .write_handler = vgic_v3_gits_mmio_write,
};

/*
 * Map the 64K ITS translation space in guest.
 * This is required purely for device smmu writes.
*/

static int vgic_map_translation_space(uint32_t nr_its, struct domain *d)
{
    uint64_t addr, size;
    int ret;

    addr = d->arch.vits[nr_its].phys_base + SZ_64K;
    size = SZ_64K;
    ret = map_mmio_regions(d,
                            paddr_to_pfn(addr & PAGE_MASK),
                            DIV_ROUND_UP(size, PAGE_SIZE),
                            paddr_to_pfn(addr & PAGE_MASK));

     if ( ret )
     {
          printk(XENLOG_ERR "Unable to map to dom%d access to"
                   " 0x%"PRIx64" - 0x%"PRIx64"\n",
                   d->domain_id,
                   addr & PAGE_MASK, PAGE_ALIGN(addr + size) - 1);
     }

    return ret;
}

int vgic_its_domain_init(struct domain *d)
{
    uint32_t num_its;
    int i;

    num_its =  its_get_nr_its();

    d->arch.vits = xzalloc_array(struct vgic_its, num_its);
    if ( d->arch.vits == NULL )
        return -ENOMEM;

    spin_lock_init(&d->arch.vits->lock);

    spin_lock_init(&d->arch.vits_devs.lock);
    INIT_LIST_HEAD(&d->arch.vits_devs.dev_list);

    d->arch.lpi_conf = xzalloc(struct vgic_lpi_conf);
    if ( d->arch.lpi_conf == NULL )
         return -ENOMEM;

    for ( i = 0; i < num_its; i++)
    {
         spin_lock_init(&d->arch.vits[i].lock);
         register_mmio_handler(d, &vgic_gits_mmio_handler,
                               d->arch.vits[i].phys_base,
                               SZ_64K);

        return vgic_map_translation_space(i, d);
    }

    return 0;
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
