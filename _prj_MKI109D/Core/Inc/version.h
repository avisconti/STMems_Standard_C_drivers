#ifndef _VERSION_H_
#define _VERSION_H_

#define MAJOR 0
#define MINOR 0
#define PATCH 0

#define FIRMWARE_ID 0 // default firmware
#define PROFI_H5_BOARD_ID 6

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define VERSION_STRING  "ProfiH5 MEMS v" TOSTRING(MAJOR) "." TOSTRING(MINOR) "." TOSTRING(PATCH) "\n"

#endif /* _VERSION_H_ */
