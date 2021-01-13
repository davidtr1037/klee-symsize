// RUN: %clang %s -emit-llvm %O0opt -c -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out -libc=uclibc --use-loop-merge -use-optimized-merge=1 -allocate-sym-size -capacity=200 --search=dfs %t.bc 2>&1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>

#include <klee/klee.h>

#define MAX_SIZE (10)
#define MIN_SIZE (3)

int main(int argc, char *argv[]) {
    size_t n;
    klee_make_symbolic(&n, sizeof(n), "n");
    klee_assume(n <= MAX_SIZE);
    klee_assume(n >= MIN_SIZE);

    unsigned char *p = malloc(n);
    klee_make_symbolic(p, n, "p");
    p[n - 1] = 0;

    for (unsigned i = 0; i < MIN_SIZE; i++) {
        switch (p[i]) {
        case 1:
            p[i] = 10;
            break;
        default:
            break;
        }
    }

    return 0;
}
