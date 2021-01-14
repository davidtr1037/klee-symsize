// RUN: %clang %s -emit-llvm %O0opt -c -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out -libc=uclibc --use-loop-merge -use-optimized-merge=1 -allocate-sym-size -capacity=4 --search=dfs %t.bc 2>&1 | FileCheck %s

// CHECK: KLEE: merged 2 states (complete = 0)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>

#include <klee/klee.h>


int main(int argc, char *argv[]) {
    size_t n;
    klee_make_symbolic(&n, sizeof(n), "n");

    unsigned char *p = malloc(n);
    for (unsigned i = 0; i < n; i++) {
        p[i * 2] = 7;
    }

    return 0;
}
