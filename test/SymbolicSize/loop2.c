// RUN: %clang %s -emit-llvm %O0opt -c -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out -libc=uclibc -validate-merge -use-loop-merge -use-optimized-merge=1 -allocate-sym-size -capacity=200 --search=dfs %t.bc 2>&1 | FileCheck %s

// CHECK: KLEE: merged 11 states (complete = 1)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>

#include <klee/klee.h>

#define MAX_SIZE (10)

int main(int argc, char *argv[]) {
    size_t n;
    klee_make_symbolic(&n, sizeof(n), "n");
    klee_assume(n <= MAX_SIZE);

    unsigned char *p = malloc(n);
    int z = 0;
    for (unsigned i = 0; i < n; i++) {
        /* TODO: this loop is not detected in LLVM */
        for (unsigned j = 0; j < n; j++) {
            z++;
        }
        p[i] = z;
    }

    return 0;
}
