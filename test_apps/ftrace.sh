cd /sys/kernel/debug/tracing
echo 0 > tracing_on
echo funcgraph-abstime > trace_options
#echo funcgraph-proc > trace_options
echo function_graph > current_tracer
echo pci_write > set_ftrace_filter
echo pci_read >> set_ftrace_filter
echo read_thread >> set_ftrace_filter
echo write_thread >> set_ftrace_filter
echo query_ring_buff >> set_ftrace_filter
echo axi_stream_fifo_write >> set_ftrace_filter
echo axi_stream_fifo_read >> set_ftrace_filter
#echo data_transfer >> set_ftrace_filter
#echo axi_stream_fifo_init >> set_ftrace_filter
echo 1 > tracing_on
