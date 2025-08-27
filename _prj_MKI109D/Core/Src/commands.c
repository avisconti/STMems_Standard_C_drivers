#include "board.h"
#include <string.h>

static void print_version(void)
{
  printf("%s\n", VERSION_STRING);
}

void cmd_interpreter(void)
{
  uint8_t str_buf[CMD_MAX_SIZE];

  if (cmd_fifo_dequeue(str_buf, CMD_MAX_SIZE)) {
    if (strcmp((const char *)str_buf, "ver") == 0) {
      print_version();
    }
    /* Add application specific commands */
  }
}