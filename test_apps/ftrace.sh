cd /sys/kernel/debug/tracing
echo 0 > tracing_on
echo function_graph > current_tracer
echo pci_write > set_ftrace_filter
#echo pci_read >> set_ftrace_filter
#echo data_transfer >> set_ftrace_filter
echo 1 > tracing_on
