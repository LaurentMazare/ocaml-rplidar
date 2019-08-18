#include <assert.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <caml/mlvalues.h>
#include <caml/memory.h>
#include <caml/alloc.h>
#include <caml/fail.h>
#include <caml/bigarray.h>

#include <sys/ioctl.h>

int ml_ioctl(value ml_fd, value ml_cmd, value ml_arg) {
  CAMLparam3(ml_fd, ml_cmd, ml_arg);
  int fd = Int_val(ml_fd);
  int cmd = Int_val(ml_cmd);
  int arg = Int_val(ml_arg);
  int res = ioctl(fd, cmd, &arg);
  CAMLreturn(Val_int(res));
}


