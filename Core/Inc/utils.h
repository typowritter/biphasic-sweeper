#ifndef __UTILS_H
#define __UTILS_H

#ifdef  __cplusplus
extern "C" {
#endif

#define INLINE __attribute__((always_inline)) inline

#define min(a, b) ((a) < (b) ? (a) : (b))

#define _str(s) #s
#define str(s) _str(s)

#define lit2addr(lit) (&(uint8_t){lit})


/* bool */
typedef uint8_t bool;
#ifndef false
#define false 0
#endif
#ifndef true
#define true (!false)
#endif

#ifdef  __cplusplus
}
#endif

#endif /* __UTILS_H */
