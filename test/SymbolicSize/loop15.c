// RUN: %clang %s -emit-llvm %O0opt -c -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out -libc=uclibc -optimize-array-values-by-tracking=1 -validate-merge -use-loop-merge -use-optimized-merge=1 -allocate-sym-size -capacity=200 --search=dfs %t.bc 2>&1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>

#include <klee/klee.h>

#define MAX_SIZE (4)

int main(int argc, char *argv[]) {
    size_t n;
    klee_make_symbolic(&n, sizeof(n), "n");
    klee_assume(n <= MAX_SIZE);

    unsigned char **p = calloc(n, sizeof(char *));
    for (unsigned i = 0; i < n; i++) {
        p[i] = malloc(100);
    }

    return 0;
}
