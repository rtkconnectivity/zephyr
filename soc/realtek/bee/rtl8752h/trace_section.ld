/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
GROUP_START(TRACE)

	SECTION_PROLOGUE(_TRACE_SECTION_NAME, (NOLOAD),SUBALIGN(4))
	{
		__trace_start = .;
		*(.TRACE)
		*(".TRACE.*")
		__trace_end = .;
	} GROUP_LINK_IN(TRACE)

GROUP_END(TRACE)
