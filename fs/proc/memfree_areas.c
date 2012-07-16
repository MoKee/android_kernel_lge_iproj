#include <linux/fs.h>
#include <linux/hugetlb.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/mmzone.h>
#include <linux/proc_fs.h>
#include <linux/quicklist.h>
#include <linux/seq_file.h>
#include <linux/swap.h>
#include <linux/vmstat.h>
#include <asm/atomic.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include "internal.h"

void __attribute__((weak)) arch_report_memfree_areas(struct seq_file *m)
{
}

static inline void show_node(struct seq_file *m, struct zone *zone)
{
	if (NUMA_BUILD)
		seq_printf(m, "Node %d ", zone_to_nid(zone));
}

#define K(x) ((x) << (PAGE_SHIFT-10))
static int memfree_areas_proc_show(struct seq_file *m, void *v)
{
	//show_free_areas(0);
	int cpu;
	struct zone *zone;
	int filter = 0;

	for_each_populated_zone(zone) {
		if (skip_free_areas_node(filter, zone_to_nid(zone)))
			continue;
		show_node(m, zone);
		seq_printf(m, "%s per-cpu:\n", zone->name);

		for_each_online_cpu(cpu) {
			struct per_cpu_pageset *pageset;

			pageset = per_cpu_ptr(zone->pageset, cpu);

			seq_printf(m, "CPU %4d: hi:%5d, btch:%4d usd:%4d\n",
			       cpu, pageset->pcp.high,
			       pageset->pcp.batch, pageset->pcp.count);
		}
	}

	seq_printf(m, "active_anon:%lu inactive_anon:%lu isolated_anon:%lu\n"
		" active_file:%lu inactive_file:%lu isolated_file:%lu\n"
		" unevictable:%lu"
		" dirty:%lu writeback:%lu unstable:%lu\n"
		" free:%lu slab_reclaimable:%lu slab_unreclaimable:%lu\n"
		" mapped:%lu shmem:%lu pagetables:%lu bounce:%lu\n",
		global_page_state(NR_ACTIVE_ANON),
		global_page_state(NR_INACTIVE_ANON),
		global_page_state(NR_ISOLATED_ANON),
		global_page_state(NR_ACTIVE_FILE),
		global_page_state(NR_INACTIVE_FILE),
		global_page_state(NR_ISOLATED_FILE),
		global_page_state(NR_UNEVICTABLE),
		global_page_state(NR_FILE_DIRTY),
		global_page_state(NR_WRITEBACK),
		global_page_state(NR_UNSTABLE_NFS),
		global_page_state(NR_FREE_PAGES),
		global_page_state(NR_SLAB_RECLAIMABLE),
		global_page_state(NR_SLAB_UNRECLAIMABLE),
		global_page_state(NR_FILE_MAPPED),
		global_page_state(NR_SHMEM),
		global_page_state(NR_PAGETABLE),
		global_page_state(NR_BOUNCE));

	for_each_populated_zone(zone) {
		int i;

		if (skip_free_areas_node(filter, zone_to_nid(zone)))
			continue;
		show_node(m, zone);
		seq_printf(m, "%s"
			" free:%lukB"
			" min:%lukB"
			" low:%lukB"
			" high:%lukB"
			" active_anon:%lukB"
			" inactive_anon:%lukB"
			" active_file:%lukB"
			" inactive_file:%lukB"
			" unevictable:%lukB"
			" isolated(anon):%lukB"
			" isolated(file):%lukB"
			" present:%lukB"
			" mlocked:%lukB"
			" dirty:%lukB"
			" writeback:%lukB"
			" mapped:%lukB"
			" shmem:%lukB"
			" slab_reclaimable:%lukB"
			" slab_unreclaimable:%lukB"
			" kernel_stack:%lukB"
			" pagetables:%lukB"
			" unstable:%lukB"
			" bounce:%lukB"
			" writeback_tmp:%lukB"
			" pages_scanned:%lu"
			" all_unreclaimable? %s"
			"\n",
			zone->name,
			K(zone_page_state(zone, NR_FREE_PAGES)),
			K(min_wmark_pages(zone)),
			K(low_wmark_pages(zone)),
			K(high_wmark_pages(zone)),
			K(zone_page_state(zone, NR_ACTIVE_ANON)),
			K(zone_page_state(zone, NR_INACTIVE_ANON)),
			K(zone_page_state(zone, NR_ACTIVE_FILE)),
			K(zone_page_state(zone, NR_INACTIVE_FILE)),
			K(zone_page_state(zone, NR_UNEVICTABLE)),
			K(zone_page_state(zone, NR_ISOLATED_ANON)),
			K(zone_page_state(zone, NR_ISOLATED_FILE)),
			K(zone->present_pages),
			K(zone_page_state(zone, NR_MLOCK)),
			K(zone_page_state(zone, NR_FILE_DIRTY)),
			K(zone_page_state(zone, NR_WRITEBACK)),
			K(zone_page_state(zone, NR_FILE_MAPPED)),
			K(zone_page_state(zone, NR_SHMEM)),
			K(zone_page_state(zone, NR_SLAB_RECLAIMABLE)),
			K(zone_page_state(zone, NR_SLAB_UNRECLAIMABLE)),
			zone_page_state(zone, NR_KERNEL_STACK) *
				THREAD_SIZE / 1024,
			K(zone_page_state(zone, NR_PAGETABLE)),
			K(zone_page_state(zone, NR_UNSTABLE_NFS)),
			K(zone_page_state(zone, NR_BOUNCE)),
			K(zone_page_state(zone, NR_WRITEBACK_TEMP)),
			zone->pages_scanned,
			(zone->all_unreclaimable ? "yes" : "no")
			);
		seq_printf(m, "lowmem_reserve[]:");
		for (i = 0; i < MAX_NR_ZONES; i++)
			seq_printf(m, " %lu", zone->lowmem_reserve[i]);
		seq_printf(m, "\n");
	}

	for_each_populated_zone(zone) {
		unsigned long nr[MAX_ORDER], flags, order, total = 0;

		if (skip_free_areas_node(filter, zone_to_nid(zone)))
			continue;
		show_node(m, zone);
		seq_printf(m, "%s: ", zone->name);

		spin_lock_irqsave(&zone->lock, flags);
		for (order = 0; order < MAX_ORDER; order++) {
			nr[order] = zone->free_area[order].nr_free;
			total += nr[order] << order;
		}
		spin_unlock_irqrestore(&zone->lock, flags);
		for (order = 0; order < MAX_ORDER; order++)
			seq_printf(m, "%lu*%lukB ", nr[order], K(1UL) << order);
		seq_printf(m, "= %lukB\n", K(total));
	}

	seq_printf(m, "%ld total pagecache pages\n", global_page_state(NR_FILE_PAGES));

	return 0;
}

static int memfree_areas_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, memfree_areas_proc_show, NULL);
}

static const struct file_operations memfree_areas_proc_fops = {
	.open		= memfree_areas_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_memfree_areas_init(void)
{
	proc_create("memfree_areas", 0, NULL, &memfree_areas_proc_fops);
	return 0;
}
module_init(proc_memfree_areas_init);
