#pragma once

#include <SDL3/SDL.h>

/* Minimal C11 threads.h compatibility layer using SDL3 threads.
   Enough for typical thrd_create/join and mutex usage. */

typedef SDL_Thread* thrd_t;
typedef SDL_Mutex*  mtx_t;

enum { thrd_success = 0, thrd_error = 1 };

static inline int thrd_create(thrd_t* thr, int (*func)(void*), void* arg) {
    SDL_Thread* t = SDL_CreateThread((SDL_ThreadFunction)func, "ensim4", arg);
    if (!t) return thrd_error;
    *thr = t;
    return thrd_success;
}

static inline int thrd_join(thrd_t thr, int* res) {
    int rc = 0;
    SDL_WaitThread(thr, &rc);
    if (res) *res = rc;
    return thrd_success;
}

/* Mutex */
static inline int mtx_init(mtx_t* m) {
    SDL_Mutex* mx = SDL_CreateMutex();
    if (!mx) return thrd_error;
    *m = mx;
    return thrd_success;
}

static inline int mtx_lock(mtx_t* m) {
    SDL_LockMutex(*m);
    return thrd_success;
}

static inline int mtx_unlock(mtx_t* m) {
    SDL_UnlockMutex(*m);
    return thrd_success;
}


static inline void mtx_destroy(mtx_t* m) {
    if (*m) SDL_DestroyMutex(*m);
    *m = NULL;
}

/* Sleep helper (optional) */
static inline void thrd_sleep_ms(unsigned ms) { SDL_Delay(ms); }
