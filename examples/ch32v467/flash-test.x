SECTIONS
{
  .flash_test ORIGIN(USR_2) + LENGTH(USR_2) - 32K (NOLOAD) :
  {
    __flash_test_start = .;
    . += 32K;
    __flash_test_end = .;
  } > USR_2
}
INSERT AFTER .text;

ASSERT((__flash_test_start & 0x7FFF) == 0, "Flash test must start at a 32 KiB boundary");
ASSERT(__flash_test_start >= ORIGIN(USR_2), "Flash test must fit in non-zero-wait flash");
