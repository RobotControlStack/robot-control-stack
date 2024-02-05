#include <stddef.h>
#include "mujoco/mjdata.h"
#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

EXTERNC struct render_args{
  mjModel* model;
  mjData** data;
  size_t n_threads;
};
EXTERNC int render_thread(void* thrd_data);
