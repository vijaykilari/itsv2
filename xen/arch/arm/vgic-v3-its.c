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

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
