/*
 * Copyright(c) 2015-2017 Intel Corporation.
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * BSD LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/mm.h>
#include <linux/sched/signal.h>
#include <linux/device.h>
#include <linux/module.h>

#include "hfi.h"

static unsigned long cache_size = 256;
module_param(cache_size, ulong, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(cache_size, "Send and receive side cache size limit (in MB)");

/*
 * Determine whether the caller can pin pages.
 *
 * This function should be used in the implementation of buffer caches.
 * The cache implementation should call this function prior to attempting
 * to pin buffer pages in order to determine whether they should do so.
 * The function computes cache limits based on the configured ulimit and
 * cache size. Use of this function is especially important for caches
 * which are not limited in any other way (e.g. by HW resources) and, thus,
 * could keeping caching buffers.
 *
 */
bool hfi1_can_pin_pages(struct hfi1_devdata *dd, struct mm_struct *mm,
			u32 nlocked, u32 npages)
{
	unsigned long ulimit_pages;
	unsigned long cache_limit_pages;
	unsigned int usr_ctxts;

	/*
	 * Perform RLIMIT_MEMLOCK based checks unless CAP_IPC_LOCK is present.
	 */
	if (!capable(CAP_IPC_LOCK)) {
		ulimit_pages =
			DIV_ROUND_DOWN_ULL(rlimit(RLIMIT_MEMLOCK), PAGE_SIZE);

		/*
		 * Pinning these pages would exceed this process's locked memory
		 * limit.
		 */
		if (atomic64_read(&mm->pinned_vm) + npages > ulimit_pages)
			return false;

		/*
		 * Only allow 1/4 of the user's RLIMIT_MEMLOCK to be used for HFI
		 * caches.  This fraction is then equally distributed among all
		 * existing user contexts.  Note that if RLIMIT_MEMLOCK is
		 * 'unlimited' (-1), the value of this limit will be > 2^42 pages
		 * (2^64 / 2^12 / 2^8 / 2^2).
		 *
		 * The effectiveness of this check may be reduced if I/O occurs on
		 * some user contexts before all user contexts are created.  This
		 * check assumes that this process is the only one using this
		 * context (e.g., the corresponding fd was not passed to another
		 * process for concurrent access) as there is no per-context,
		 * per-process tracking of pinned pages.  It also assumes that each
		 * user context has only one cache to limit.
		 */
		usr_ctxts = dd->num_rcv_contexts - dd->first_dyn_alloc_ctxt;
		if (nlocked + npages > (ulimit_pages / usr_ctxts / 4))
			return false;
	}

	/*
	 * Pinning these pages would exceed the size limit for this cache.
	 */
	cache_limit_pages = cache_size * (1024 * 1024) / PAGE_SIZE;
	if (nlocked + npages > cache_limit_pages)
		return false;

	return true;
}

int hfi1_acquire_user_pages(struct mm_struct *mm, unsigned long vaddr, size_t npages,
			    bool writable, struct page **pages)
{
	int ret;

	ret = get_user_pages_fast(vaddr, npages, writable, pages);
	if (ret < 0)
		return ret;

	atomic64_add(ret, &mm->pinned_vm);

	return ret;
}

void hfi1_release_user_pages(struct mm_struct *mm, struct page **p,
			     size_t npages, bool dirty)
{
	size_t i;

	for (i = 0; i < npages; i++) {
		if (dirty)
			set_page_dirty_lock(p[i]);
		put_page(p[i]);
	}

	if (mm) { /* during close after signal, mm can be NULL */
		atomic64_sub(npages, &mm->pinned_vm);
	}
}
