// RUN: %clang %s -emit-llvm %O0opt -c -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out -libc=uclibc --use-loop-merge -use-optimized-merge=1 -allocate-sym-size -capacity=10 --search=dfs %t.bc 2>&1 | FileCheck %s

// CHECK: KLEE: merged 10 states (early = 0)
// CHECK: KLEE: done: unmerged completed paths = 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>

#include <klee/klee.h>

void klee_dump(void *x) { }
void foo() { }

int main(int argc, char *argv[]) {
    unsigned int input_size;
    klee_make_symbolic(&input_size, sizeof(input_size), "input_size");
    unsigned char *input = malloc(input_size);
    int x;
    klee_make_symbolic(&x, sizeof(x), "x");

    int z = 0;
    for (int i = 0; i < input_size; i++) {
      z++;
      input[i] = 7;
    }

    foo();

    return 0;
}
