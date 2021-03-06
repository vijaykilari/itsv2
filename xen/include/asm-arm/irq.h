#ifndef _ASM_HW_IRQ_H
#define _ASM_HW_IRQ_H

#include <xen/config.h>
#include <xen/radix-tree.h>
#include <xen/device_tree.h>

#define NR_VECTORS 256 /* XXX */

typedef struct {
    DECLARE_BITMAP(_bits,NR_VECTORS);
} vmask_t;

struct arch_pirq
{
};

struct arch_irq_desc {
    int eoi_cpu;
    unsigned int type;
    unsigned int virq;
    struct its_device *dev;
};

#define NR_LOCAL_IRQS	32
#define NR_IRQS		1024

#define nr_irqs NR_IRQS
#define nr_static_irqs NR_IRQS
#define arch_hwdom_irqs(domid) NR_IRQS

struct irq_desc;
struct pending_irq;
struct irqaction;

struct irq_desc *__irq_to_desc(int irq);

#define irq_to_desc(irq)    __irq_to_desc(irq)

void do_IRQ(struct cpu_user_regs *regs, unsigned int irq, int is_fiq);

#define domain_pirq_to_irq(d, pirq) (pirq)

void init_IRQ(void);
void init_secondary_IRQ(void);

int route_irq_to_guest(struct domain *d, unsigned int irq,
                       const char *devname);
void arch_move_irqs(struct vcpu *v);

/* Set IRQ type for an SPI */
int irq_set_spi_type(unsigned int spi, unsigned int type);

int irq_set_desc_data(unsigned int irq, struct its_device *d);
struct its_device *irq_get_desc_data(struct irq_desc *d);
int platform_get_irq(const struct dt_device_node *device, int index);
struct domain *irq_get_domain(struct irq_desc *desc);

void irq_set_affinity(struct irq_desc *desc, const cpumask_t *cpu_mask);

struct irq_desc *find_irq_desc(struct radix_tree_root *root_node, int irq);
struct irq_desc *insert_irq_desc(struct radix_tree_root *root_node, int irq);
struct irq_desc *delete_irq_desc(struct radix_tree_root *root_node, int irq);

struct pending_irq *insert_pending_irq_desc(struct domain *d, int irq);
struct pending_irq *find_pending_irq_desc(struct domain *d, int irq);
struct pending_irq *delete_pending_irq_desc(struct domain *d, int irq);

#endif /* _ASM_HW_IRQ_H */
/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
