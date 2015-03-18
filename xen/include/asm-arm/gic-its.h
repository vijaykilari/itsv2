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
        void                    *itt;
        unsigned long           *lpi_map;
        u32                     lpi_base;
        int                     nr_lpis;
        u32                     nr_ites;
        u32                     device_id;
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

#endif /* __ASM_ARM_GIC_ITS_H__ */

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
