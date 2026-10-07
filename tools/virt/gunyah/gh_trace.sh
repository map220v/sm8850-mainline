#!/bin/bash
# Trace the Gunyah demand-paging path while running gh_vmtest.py in the given modes.
# usage: gh_trace.sh "<mode args>" ...     e.g. gh_trace.sh "shared lend none" "shared lend shm"
T=/sys/kernel/tracing
mountpoint -q $T || mount -t tracefs none $T
echo nop > $T/current_tracer; echo 1 > $T/tracing_on
echo 0 > $T/events/gh/enable 2>/dev/null
echo > $T/kprobe_events
add() { echo "$1" >> $T/kprobe_events || echo "FAILED to add: $1"; }
add 'p:gh/dp gunyah_rsc_mgr:gunyah_demand_page gpa=$arg2:x64 write=$arg3:u8'
add 'r:gh/dp_ret gunyah_rsc_mgr:gunyah_demand_page ret=$retval:s32'
add 'p:gh/pf gunyah_rsc_mgr:gunyah_vm_provide_folio gfn=$arg3:x64 share=$arg4:u8 write=$arg5:u8'
add 'r:gh/pf_ret gunyah_rsc_mgr:gunyah_vm_provide_folio ret=$retval:s32'
add 'r:gh/mmio_ret gunyah_vcpu:gunyah_handle_mmio ret=$retval:u8'
add 'r:gh/pgf_ret gunyah_vcpu:gunyah_handle_page_fault ret=$retval:u8'
add 'p:gh/res gunyah_rsc_mgr:gunyah_vm_add_resource type=+0($arg2):u32 capid=+8($arg2):x64 label=+40($arg2):u32'
echo "--- kprobe_events:"; cat $T/kprobe_events
echo 1 > $T/events/gh/enable || echo "FAILED to enable"
for args in "$@"; do
	echo "########## $args"
	echo > $T/trace
	dmesg -C
	timeout 30 python3 /root/gh_vmtest.py $args 2>&1 | grep -v -E '^\[(1|2|4|5|6|6b|7a|7b|7c|7d)\]' | head -14
	echo "--- dmesg"; dmesg | tail -6
	echo "--- trace"; grep -v '^#' $T/trace | sed -E 's/^ *[^ ]+ +\[[0-9]+\] [^ ]+ +[0-9.]+: //' | head -40
done
echo 0 > $T/events/gh/enable
echo > $T/kprobe_events
