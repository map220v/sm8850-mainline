# Gunyah driver: mainline 7.2 port adaptations

Source: Android common kernel, tag `android17-6.18-2026-06_r1`
(30c0eaeef78d5c19f687ad8418c27b76353ab261). Driver logic is unchanged; every
edit below is an API-form or build-system adaptation.

| # | File | Upstream form | Mainline 7.2 form | Why |
|---|------|---------------|-------------------|-----|
| 1 | `drivers/virt/gunyah/rsc_mgr.h` | `#include <linux/android_kabi.h>` + `ANDROID_KABI_ENUMERATOR_IGNORE()` | header included only when present (`__has_include`), macro otherwise defined as a no-op `static_assert(1)` | Android ABI-tool annotation; no mainline counterpart |
| 2 | `drivers/virt/gunyah/gunyah_vcpu.c`, `vm_mgr.c` | `#include <trace/hooks/gunyah.h>` (restricted vendor hooks `trace_android_rvh_gh_*`) | `#include "gunyah_vendor_hooks.h"`: forwards to the Android header under `CONFIG_ANDROID_VENDOR_HOOKS`, empty inline stubs otherwise | Vendor-hook infrastructure does not exist on mainline; call sites stay identical |
| 3 | `drivers/virt/gunyah/vm_mgr.h` | open-coded loops over `set_direct_map_{invalid,default}_noflush()` | `gunyah_folio_set_direct_map(folio, valid)`, compiled out under `MODULE` | Not exported to modules on mainline arm64. Lent memory stays protected by the hypervisor stage-2 unmap; only the host-side linear-map hardening is lost when built as a module |
| 4 | `drivers/virt/gunyah/rsc_mgr.c` | `of_node_to_fwnode()` | `of_fwnode_handle()` | Helper renamed upstream |
| 5 | `drivers/virt/Kconfig`, `drivers/virt/Makefile`, `arch/arm64/Kbuild` | same three lines in the Android tree | appended unchanged | Only integration points touched in pre-existing files |

## Verification on OnePlus Pad 4 (SM8850, `7.2.0-sm8850`)

Modules: all ten load; `gunyah: Running under Gunyah hypervisor 51/v1`,
`/dev/gunyah` appears, the `gh_rm_core` auxiliary device registers and the
Qualcomm platform hooks enable. `RM rejected message 00000005. Error: -1` at
load is the optional RM log query (unimplemented by this firmware), harmless.
Boot-time loading on the device is `/etc/modules-load.d/gunyah.conf` listing
`gunyah_rsc_mgr gunyah_vcpu gunyah_irqfd gunyah_ioeventfd gunyah_qcom
gunyah_qtvm` (dependencies come from depmod).

`tools/virt/gunyah/gh_vmtest.py` (python3, no dependencies) creates a 1-vCPU
VM from /dev/gunyah with a hand-built DTB and a 16-byte guest payload;
`gh_trace.sh "<mode args>"...` wraps a run in kprobe events on the
demand-paging path.

| Mode | Result |
|------|--------|
| `off lend`  | VM_START in ~2 ms, guest PSCI SYSTEM_OFF -> `GUNYAH_VCPU_EXIT_STATUS` EXITED, 5/5 runs |
| `mmio lend` | guest store to unmapped GPA -> `GUNYAH_VCPU_EXIT_MMIO` write len 4 data 0x1234, then EXITED |
| `off` (unprotected image) | RM rejects VM_CONFIG_IMAGE with MAP_FAILED (9); see limitations |
| `shared lend [none/vmmio/shm]` | lent image + a second SHARE binding the guest writes to: never reaches the host, guest takes a data abort; see limitations |

### Guest DTB requirements learned from the resource manager

The RM validates the VMM-supplied DTB at VM_INIT and answers
`VALIDATE_FAILED (8)` without further detail. Requirements that mattered:

- `/gunyah-vm-config` with `image-name`, `os-type = "linux"`,
  `vm-type = "aarch64-guest"`, `memory { base-address; size-max }`,
  `interrupts { config = "/intc" }`, `vcpus { affinity = "proxy" }`,
  `vdevices { generate = "/hypervisor" }`.
- The RAM node must be exactly `/memory` (`device_type = "memory"`). A
  unit address such as `memory@80000000` is rejected.
- `/cpus` with `#address-cells = 1`, `#size-cells = 0`; one `cpu@N` with
  `device_type = "cpu"` and `reg` per vCPU.
- A GICv3 node (`arm,gic-v3`, `#interrupt-cells = 3`, `interrupt-controller`,
  `reg` = GICD, GICR) referenced by `interrupts/config`.
- `arm,armv8-timer` PPIs with a CPU mask matching the vCPU count
  (`0x108` for one vCPU); `0xf08` is not accepted.

### Limitations on this firmware

- Only protected VMs start: the image memparcel must be lent
  (`GH_VM_ANDROID_LEND_USER_MEM`). With `GH_VM_SET_USER_MEM_REGION` the RM
  answers `MAP_FAILED` at VM_CONFIG_IMAGE and then refuses to deallocate the
  VMID (error 20), so a VMID is lost per attempt.
- A protected VM cannot fall back to "lent image, rest of RAM shared" as
  transparent guest RAM. Two layers refuse it:
  1. The driver registers only lent bindings for demand paging in a
     protected VM (`gunyah_setup_demand_paging`), so a guest access to a
     shared binding is an unregistered RAM access and the hypervisor
     injects a data abort into the guest (observed: the next exit is the
     guest's exception-vector fetch at 0x200). Declaring the range as
     `vmmio-ranges` or as a `vdevices/shm-0` node changes nothing.
  2. With the shared bindings registered as an experiment, the RM accepts
     VM_INIT and the fault reaches the host, SCM assign and the memextent
     donate to the guest's protected-shared extent (label 3) succeed, but
     `addrspace_map` with the VMMIO flag returns Gunyah error 30 DENIED:
     the hypervisor does not let the host map shared memory into a
     protected VM.
  Shared memory for a protected guest is therefore GuestAccept only: the
  host shares an RM memparcel and the guest accepts it by handle into its
  own stage-2. That is provided by an out-of-tree module (the gunyah_share
  model), not by this tree.
- The RM log (`/sys/hypervisor/rm_log`) is unimplemented (RM error -1).
- Never fork() a VMM process that holds lent guest RAM. The lent pages are
  pinned (FOLL_LONGTERM), so dup_mmap() copies them eagerly instead of
  marking them COW; the host read of a page it no longer owns at stage-2
  raises a synchronous external abort in copy_page(), the task dies while
  holding mmap_lock and the box hangs (RCU stall in do_exit ->
  acct_collect). Observed once on the Pad 4. Multi-process VMMs must pass
  guest memory as shared memfd/dma-buf mappings, as crosvm does.
- QTVM (authenticated) launch needs a signed image and a PAS ID; untested.
