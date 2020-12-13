// RUN: %clang %s -emit-llvm %O0opt -c -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --use-loop-merge -allocate-sym-size -capacity=10 --search=dfs %t.bc 2>&1 | FileCheck %s

// CHECK: KLEE: merged 11 states (early = 0)
// KLEE: done: unmerged completed paths = 2

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>

#include <klee/klee.h>

void foo() {

}

int main(int argc, char *argv[]) {
    unsigned int length;
    klee_make_symbolic(&length, sizeof(length), "length");
    unsigned char *p = malloc(length);
    memset(p, 0, length);
    if (length == 1) {
        foo();
    }

    return 0;
}
