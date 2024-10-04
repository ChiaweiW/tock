/* Licensed under the Apache License, Version 2.0 or the MIT License. */
/* SPDX-License-Identifier: Apache-2.0 OR MIT                         */
/* Copyright Tock Contributors 2023.                                  */

/* Memory Spaces Definitions, 254KB sram */
MEMORY
{
  rom (rx): ORIGIN = 0x14b80a00, LENGTH = 0x1f600
  prog (rx) : ORIGIN = 0x14ba0000, LENGTH = 0x10000
  ram (rwx) : ORIGIN = 0x14bb0000, LENGTH = 0xf800
}

INCLUDE tock_kernel_layout.ld
