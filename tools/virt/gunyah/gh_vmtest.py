#!/usr/bin/env python3
"""Minimal Gunyah pVM smoke test for /dev/gunyah (unauthenticated VM, 1 vCPU).

usage: gh_vmtest.py [off|mmio|shared] [lend|share] [none|vmmio|shm]
  3rd arg (shared mode): how the shared region is declared in the DTB: not at all, as
          gunyah-vm-config/vmmio-ranges, or as a vdevices/shm-0 node (crosvm swiotlb style)
  off : guest issues PSCI SYSTEM_OFF via HVC               -> expect VCPU exit STATUS/EXITED
  mmio: guest stores to unmapped 0x10000000, then SYSTEM_OFF -> expect PAGE_FAULT/MMIO exit first
  shared: protected image + a second SHARE binding at RAM_BASE+4M; guest writes 0x1234 there,
          host reads it back (kernel demand-pages the shared region on the VMMIO exit, no user exit)
  lend: use GH_VM_ANDROID_LEND_USER_MEM (protected VM) instead of GH_VM_SET_USER_MEM_REGION

"""
import ctypes, fcntl, mmap, os, struct, sys, time

# ---- ioctl plumbing --------------------------------------------------------
def _IOC(d, t, nr, size): return (d << 30) | (size << 16) | (ord(t) << 8) | nr
def _IO(t, nr): return _IOC(0, t, nr, 0)
def _IOW(t, nr, size): return _IOC(1, t, nr, size)

GUNYAH_CREATE_VM            = _IO("G", 0x0)
GH_VM_SET_USER_MEM_REGION   = _IOW("G", 0x1, 32)
GUNYAH_VM_SET_DTB_CONFIG    = _IOW("G", 0x2, 16)
GUNYAH_VM_START             = _IO("G", 0x3)
GUNYAH_VM_ADD_FUNCTION      = _IOW("G", 0x4, 16)
GUNYAH_VCPU_RUN             = _IO("G", 0x5)
GUNYAH_VCPU_MMAP_SIZE       = _IO("G", 0x6)
GUNYAH_VM_SET_BOOT_CONTEXT  = _IOW("G", 0xa, 16)
GH_VM_ANDROID_LEND_USER_MEM = _IOW("A", 0x11, 32)
GH_VM_CHECK_EXTENSION       = _IO("A", 0x17)
GUNYAH_FN_VCPU = 1
MEM_RWX = 0x7
REG_SET_PC = 1 << 8

# ---- flat device tree builder ---------------------------------------------
class FDT:
    def __init__(self):
        self.struct = bytearray(); self.strings = bytearray(); self.stroff = {}
    def _str(self, s):
        if s not in self.stroff:
            self.stroff[s] = len(self.strings); self.strings += s.encode() + b"\0"
        return self.stroff[s]
    def _pad(self):
        while len(self.struct) % 4: self.struct += b"\0"
    def begin(self, name):
        self.struct += struct.pack(">I", 1) + name.encode() + b"\0"; self._pad()
    def end(self): self.struct += struct.pack(">I", 2)
    def prop(self, name, val=b""):
        self.struct += struct.pack(">III", 3, len(val), self._str(name)) + val; self._pad()
    def s(self, name, *vals): self.prop(name, b"".join(v.encode() + b"\0" for v in vals))
    def u32(self, name, *vals): self.prop(name, struct.pack(">%dI" % len(vals), *vals))
    def u64(self, name, *vals): self.prop(name, struct.pack(">%dQ" % len(vals), *vals))
    def blob(self):
        self.struct += struct.pack(">I", 9)
        hdr = 40; rsv = hdr; st = rsv + 16; strs = st + len(self.struct); total = strs + len(self.strings)
        return struct.pack(">IIIIIIIIII", 0xd00dfeed, total, st, strs, rsv, 17, 16, 0,
                           len(self.strings), len(self.struct)) + b"\0" * 16 + bytes(self.struct) + bytes(self.strings)

def build_dtb(ram_base, ram_size, gicd, gicr, shared=None, variant="none"):
    """shared = (gpa, size, label) of an extra SHARE binding, declared per variant"""
    f = FDT()
    f.begin("")
    f.u32("#address-cells", 2); f.u32("#size-cells", 2)
    f.s("compatible", "linux,dummy-virt"); f.u32("interrupt-parent", 1)
    f.begin("gunyah-vm-config")
    f.s("image-name", "iceland-smoke"); f.s("os-type", "linux"); f.s("vm-type", "aarch64-guest")
    f.begin("memory"); f.u32("#address-cells", 2); f.u32("#size-cells", 2)
    f.u64("base-address", ram_base); f.u64("size-max", 0x40000000); f.end()
    f.begin("interrupts"); f.s("config", "/intc"); f.end()
    f.begin("vcpus"); f.s("affinity", "proxy"); f.end()
    if shared and variant == "vmmio":
        f.u64("vmmio-ranges", shared[0], shared[1])
    f.begin("vdevices"); f.s("generate", "/hypervisor")
    if shared and variant == "shm":
        f.begin("shm-0"); f.s("vdevice-type", "shm"); f.prop("peer-default"); f.u64("dma_base", 0)
        f.begin("memory"); f.prop("optional"); f.u32("label", shared[2]); f.u32("#address-cells", 2); f.u64("base", shared[0]); f.end()
        f.end()
    f.end()
    f.end()
    f.begin("cpus"); f.u32("#address-cells", 1); f.u32("#size-cells", 0)
    f.begin("cpu@0"); f.s("device_type", "cpu"); f.s("compatible", "arm,armv8"); f.u32("reg", 0)
    f.s("enable-method", "psci"); f.u32("phandle", 0x100); f.end()
    f.end()
    f.begin("memory"); f.s("device_type", "memory"); f.u64("reg", ram_base, ram_size); f.end()  # RM accepts only /memory
    f.begin("intc"); f.s("compatible", "arm,gic-v3"); f.u32("#interrupt-cells", 3); f.prop("interrupt-controller")
    f.u32("#address-cells", 2); f.u32("#size-cells", 2); f.u64("reg", gicd, 0x10000, gicr, 0x20000)
    f.u32("phandle", 1); f.end()
    f.begin("psci"); f.s("compatible", "arm,psci-1.0", "arm,psci-0.2"); f.s("method", "hvc"); f.end()
    f.begin("timer"); f.s("compatible", "arm,armv8-timer")
    f.u32("interrupts", 1, 13, 0x108, 1, 14, 0x108, 1, 11, 0x108, 1, 10, 0x108); f.prop("always-on"); f.end()  # PPI, cpu mask for 1 vCPU, level-low
    f.begin("chosen"); f.s("bootargs", "console=hvc0"); f.end()
    f.end()
    return f.blob()

# ---- guest payload ---------------------------------------------------------
PSCI_OFF = [0xD2B08000,  # movz x0, #0x8400, lsl #16
            0xF2800100,  # movk x0, #0x8          -> x0 = 0x84000008 (PSCI SYSTEM_OFF)
            0xD4000002,  # hvc #0
            0x14000000]  # b .
MMIO_THEN_OFF = [0xD2A20000,  # movz x0, #0x1000, lsl #16 -> 0x10000000
                 0x52824681,  # movz w1, #0x1234
                 0xB9000001]  # str w1, [x0]
SHARED_WRITE  = [0xD2B00800,  # movz x0, #0x8040, lsl #16 -> 0x80400000 (second, shared binding)
                 0x52824681,  # movz w1, #0x1234
                 0xB9000001]  # str w1, [x0]
PAYLOADS = {"off": PSCI_OFF, "mmio": MMIO_THEN_OFF + PSCI_OFF, "shared": SHARED_WRITE + PSCI_OFF}

# ---- run -------------------------------------------------------------------
def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "off"
    lend = len(sys.argv) > 2 and sys.argv[2] == "lend"
    RAM_BASE, RAM_SIZE = 0x80000000, 0x400000
    EXTRA_SIZE = 0x400000 if mode == "shared" else 0     # second binding, always SHARE
    variant = sys.argv[3] if len(sys.argv) > 3 else "none"
    shared_decl = (RAM_BASE + RAM_SIZE, EXTRA_SIZE, 1) if EXTRA_SIZE else None
    DTB_OFF, DTB_SIZE = 0x200000, 0x10000
    GICD, GICR = 0x3FFF0000, 0x3FFD0000

    dev = os.open("/dev/gunyah", os.O_RDWR)
    vm = fcntl.ioctl(dev, GUNYAH_CREATE_VM)
    print("[1] CREATE_VM -> vm fd %d; CHECK_EXTENSION(timer_wakeup)=%d" % (vm, fcntl.ioctl(vm, GH_VM_CHECK_EXTENSION, 0)))

    mem = mmap.mmap(-1, RAM_SIZE, flags=mmap.MAP_PRIVATE | mmap.MAP_ANONYMOUS, prot=mmap.PROT_READ | mmap.PROT_WRITE)
    mem[0:RAM_SIZE] = b"\0" * RAM_SIZE                       # fault pages in
    uaddr = ctypes.addressof(ctypes.c_char.from_buffer(mem))
    code = struct.pack("<%dI" % len(PAYLOADS[mode]), *PAYLOADS[mode])
    mem[0:len(code)] = code
    dtb = build_dtb(RAM_BASE, RAM_SIZE + EXTRA_SIZE, GICD, GICR, shared_decl, variant)
    mem[DTB_OFF:DTB_OFF + len(dtb)] = dtb
    print("[2] guest RAM %d MiB @ %#x (user %#x); payload %d B; dtb %d B @ +%#x" % (RAM_SIZE >> 20, RAM_BASE, uaddr, len(code), len(dtb), DTB_OFF))

    region = struct.pack("<IIQQQ", 0, MEM_RWX, RAM_BASE, RAM_SIZE, uaddr)
    fcntl.ioctl(vm, GH_VM_ANDROID_LEND_USER_MEM if lend else GH_VM_SET_USER_MEM_REGION, region)
    print("[3] %s ok" % ("LEND_USER_MEM" if lend else "SET_USER_MEM_REGION"))
    if EXTRA_SIZE:
        extra = mmap.mmap(-1, EXTRA_SIZE, flags=mmap.MAP_PRIVATE | mmap.MAP_ANONYMOUS, prot=mmap.PROT_READ | mmap.PROT_WRITE)
        extra[0:EXTRA_SIZE] = bytes(EXTRA_SIZE)
        extra_addr = ctypes.addressof(ctypes.c_char.from_buffer(extra))
        fcntl.ioctl(vm, GH_VM_SET_USER_MEM_REGION, struct.pack("<IIQQQ", 1, MEM_RWX, RAM_BASE + RAM_SIZE, EXTRA_SIZE, extra_addr))
        print("[3b] SET_USER_MEM_REGION (shared, label 1) %d MiB @ %#x ok, dtb variant=%s" % (EXTRA_SIZE >> 20, RAM_BASE + RAM_SIZE, variant))
    fcntl.ioctl(vm, GUNYAH_VM_SET_DTB_CONFIG, struct.pack("<QQ", RAM_BASE + DTB_OFF, DTB_SIZE))
    print("[4] SET_DTB_CONFIG ok")
    fcntl.ioctl(vm, GUNYAH_VM_SET_BOOT_CONTEXT, struct.pack("<IIQ", REG_SET_PC, 0, RAM_BASE))
    print("[5] SET_BOOT_CONTEXT pc=%#x ok" % RAM_BASE)

    vcpu_arg = ctypes.c_uint32(0)
    fn = struct.pack("<IIQ", GUNYAH_FN_VCPU, 4, ctypes.addressof(vcpu_arg))
    vcpu = fcntl.ioctl(vm, GUNYAH_VM_ADD_FUNCTION, bytearray(fn), True)  # mutable buf -> returns the new fd
    run_size = fcntl.ioctl(vcpu, GUNYAH_VCPU_MMAP_SIZE)
    run = mmap.mmap(vcpu, run_size, flags=mmap.MAP_SHARED, prot=mmap.PROT_READ | mmap.PROT_WRITE)
    print("[6] ADD_FUNCTION(vcpu0) -> fd %d, run page %d B" % (vcpu, run_size))

    t0 = time.monotonic()
    fcntl.ioctl(vm, GUNYAH_VM_START)
    print("[7] VM_START ok (%.1f ms)" % ((time.monotonic() - t0) * 1000))


    for i in range(8):
        t0 = time.monotonic()
        try:
            fcntl.ioctl(vcpu, GUNYAH_VCPU_RUN)
        except OSError as e:
            print("[8.%d] VCPU_RUN -> errno %d (%s)" % (i, e.errno, e.strerror)); break
        dt = (time.monotonic() - t0) * 1000
        exit_reason, = struct.unpack_from("<I", run, 8)
        if exit_reason == 1:
            pa, data, ln, is_w, act = struct.unpack_from("<Q8sIBB", run, 16)
            print("[8.%d] exit MMIO %s pa=%#x len=%d data=%s (%.2f ms)" % (i, "write" if is_w else "read", pa, ln, data[:ln].hex(), dt))
            struct.pack_into("<B", run, 16 + 8 + 8 + 4 + 1, 0)   # resume_action = HANDLED
        elif exit_reason == 2:
            st, ty, pad, rsz, reason = struct.unpack_from("<IHHI8s", run, 16)
            names = {1: "LOAD_FAILED", 2: "EXITED", 3: "CRASHED"}
            print("[8.%d] exit STATUS %s exit_type=%d reason=%s (%.2f ms)" % (i, names.get(st, st), ty, reason[:rsz].hex(), dt)); break
        elif exit_reason == 3:
            pa, attempt, act = struct.unpack_from("<QiB", run, 16)
            print("[8.%d] exit PAGE_FAULT pa=%#x attempt=%d (%.2f ms) -> resume FAULT" % (i, pa, attempt, dt))
            struct.pack_into("<B", run, 16 + 8 + 4, 1)           # resume_action = FAULT (inject back)
        else:
            print("[8.%d] exit_reason=%d (%.2f ms)" % (i, exit_reason, dt)); break
    if EXTRA_SIZE:
        got = bytes(extra[0:4])
        print("[8x] shared binding word 0 = %s -> %s" % (got.hex(), "guest write visible on host" if got == bytes.fromhex("34120000") else "MISMATCH"))
    run.close(); os.close(vcpu); os.close(vm); os.close(dev)
    print("[9] closed")

if __name__ == "__main__":
    main()
