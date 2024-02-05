#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

EXTERNC typedef struct {
  double x;
  double y;
  double z;
} position;
EXTERNC typedef struct ik_model *IK;
EXTERNC IK ik_load(char *filename);
EXTERNC void ik_free(IK);
EXTERNC void ik_solve(IK, position, double *);
