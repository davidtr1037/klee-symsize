// RUN: %clang %s -emit-llvm %O0opt -c -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --use-loop-merge -allocate-sym-size -capacity=200 --search=dfs %t.bc 2>&1 | FileCheck %s

// CHECK: KLEE: merged 3 states (early = 1)

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
    for (unsigned i = 0; i < n; i++) {
        p[i] = 7;
        if (i == 2) {
            abort();
        }
    }

    return 0;
}
