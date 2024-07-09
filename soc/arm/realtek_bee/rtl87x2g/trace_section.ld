/* #if DT_NODE_HAS_STATUS(DT_NODELABLE(trace), okay) */
GROUP_START(TRACE)

	SECTION_PROLOGUE(_TRACE_SECTION_NAME, (NOLOAD),SUBALIGN(4))
	{
		__trace_start = .;
		*(.TRACE)
		*(".TRACE.*")
		__trace_end = .;
	} GROUP_LINK_IN(TRACE)

GROUP_END(TRACE)
/* #endif */
