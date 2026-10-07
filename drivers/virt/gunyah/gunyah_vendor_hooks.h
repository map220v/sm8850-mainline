/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Android restricted vendor hooks used by the Gunyah drivers.
 *
 * The hooks live in <trace/hooks/gunyah.h> on Android GKI kernels. Mainline
 * has no vendor-hook infrastructure, so provide no-op shims there; the call
 * sites in gunyah_vcpu.c and vm_mgr.c stay identical to upstream.
 */
#ifndef _GUNYAH_VENDOR_HOOKS_H
#define _GUNYAH_VENDOR_HOOKS_H

#if IS_ENABLED(CONFIG_ANDROID_VENDOR_HOOKS)
#include <trace/hooks/gunyah.h>
#else
#include <linux/types.h>

struct gunyah_hypercall_vcpu_run_resp;
struct gunyah_vcpu;
struct gunyah_vm;

static inline void trace_android_rvh_gh_before_vcpu_run(u16 vmid, u32 vcpu_id)
{
}

static inline void trace_android_rvh_gh_after_vcpu_run(
	u16 vmid, u32 vcpu_id, int hcall_ret,
	const struct gunyah_hypercall_vcpu_run_resp *resp)
{
}

static inline void trace_android_rvh_gh_vm_release(u16 vmid,
						   struct gunyah_vm *ghvm)
{
}

static inline void trace_android_rvh_gh_vcpu_release(u16 vmid,
						     struct gunyah_vcpu *vcpu)
{
}

static inline void trace_android_rvh_gh_note_fw_parcel(struct gunyah_vm *ghvm,
							u16 ghvm_vmid,
							u32 fw_mem_handle)
{
}
#endif /* CONFIG_ANDROID_VENDOR_HOOKS */

#endif /* _GUNYAH_VENDOR_HOOKS_H */
