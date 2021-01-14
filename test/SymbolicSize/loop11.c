// RUN: %clang %s -emit-llvm %O0opt -c -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out -libc=uclibc --use-loop-merge -use-optimized-merge=1 -allocate-sym-size -capacity=200 --search=dfs %t.bc 2>&1

// CHECK: KLEE: merged 111 states (complete = 1)
// CHECK: KLEE: done: unmerged completed paths = 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>

#include <klee/klee.h>

#define MAX_SIZE (10)

unsigned char buffer[MAX_SIZE] = {0,};

void inner(size_t n) {
    if (n <= MAX_SIZE) {
        for (unsigned i = 0; i < n; i++) {
            buffer[i]++;
        }
    }
}

int main(int argc, char *argv[]) {
    size_t n;
    klee_make_symbolic(&n, sizeof(n), "n");
    klee_assume(n <= MAX_SIZE);

    size_t m;
    klee_make_symbolic(&m, sizeof(m), "m");
    klee_assume(m <= MAX_SIZE);

    unsigned char *p = malloc(n);
    for (unsigned i = 0; i < n; i++) {
        p[i] = 7;
        inner(m);
    }

    return 0;
}
