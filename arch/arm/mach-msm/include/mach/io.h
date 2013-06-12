/* arch/arm/mach-msm/include/mach/io.h
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARM_ARCH_IO_H
#define __ASM_ARM_ARCH_IO_H

//QCT kernel IO read patch for checking register corruption
// #define QCT_IO_READ_ONLY_PATCH

#define IO_SPACE_LIMIT 0xffffffff

#define __arch_ioremap __msm_ioremap

#ifdef QCT_IO_READ_ONLY_PATCH
#define __arch_iounmap __msm_iounmap
#else
#define __arch_iounmap __iounmap
#endif
void __iomem *__msm_ioremap(unsigned long phys_addr, size_t size, unsigned int mtype);

#ifdef QCT_IO_READ_ONLY_PATCH
void __msm_iounmap(volatile void __iomem *io_addr);
#endif

#define __io(a)         __typesafe_io(a)
#define __mem_pci(a)    (a)

#endif
