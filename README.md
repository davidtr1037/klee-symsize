# Symbolic-Size Allocations for KLEE
This is an extension of KLEE which supports symbolic size allocations (up to a user-specified bound).

## Build

The current version was tested with LLVM 7.0 (and should work with earlier versions as well).

To build our extension of KLEE:
```
mkdir <build>
cd <build>
CXXFLAGS="-fno-rtti -g" cmake \
    -DENABLE_SOLVER_STP=ON \
    -DENABLE_POSIX_RUNTIME=ON \
    -DENABLE_KLEE_UCLIBC=ON \
    -DKLEE_UCLIBC_PATH=<klee-uclibc> \
    -DLLVM_CONFIG_BINARY=<llvm_build_dir>/bin/llvm-config \
    -DLLVMCC=<llvm_build_dir>/bin/clang \
    -DLLVMCXX=<llvm_build_dir>/bin/clang++ \
    -DENABLE_UNIT_TESTS=OFF \
    -DKLEE_RUNTIME_BUILD_TYPE=Release+Asserts \
    -DENABLE_SYSTEM_TESTS=ON \
    -DENABLE_TCMALLOC=ON \
    <klee_symsize_src>
make -j4
```

## Usage
To enable symbolic-size allocations, add the following option:
```
-allocate-sym-size=1
```
To set the capacity (bound) for symbolic-size allocations:
```
-capacity=<N>
```
### Modes
This extension supports several modes:

#### Eager Forking
This mode forks at *allocation time* for each possible value of the symbolic-size.
Use the following option to enable this mode:
```
-sym-size-mode=eager
```

#### Lazy Forking
This mode forks on-demand.
Use the following option to enable this mode:
```
-sym-size-mode=lazy
```

#### Merging
In this mode, we attempt to merge loops that depend on symbolic-size expressions.
Use the following options to enable this mode (with optimizations):
```
-use-loop-merge=1 -optimize-ite-using-exec-tree=1 -optimize-array-ite-using-exec-tree=1
```
To enable without optimizations:
```
-use-loop-merge=1
```

#### Concretization
In this mode, the symbolic-size is concretized to the maximum possible value.
Use the following option to enable this mode:
```
-sym-size-mode=max
```

### POSIX runtime

#### Symbolic command-line arguments
To enabling symbolic command-line arguments with symbolic size, use the following option:
```
--model-symbolic-size
```
For example:
```
klee -libc=uclibc -posix-runtime … -allocate-sym-size=1 -capacity=<capacity> -sym-size-mode=<mode> <bc_file> --model-symbolic-size --sym-args 0 1 10
```
Note that the size of the allocated command-line strings will be bounded by both the specified capacity and the argument of `--sym-args` (in this case: 10).

#### Symbolic files
To enabling symbolic files with symbolic size, use the following option:
```
--model-symbolic-fd-size
```
For example:
```
klee -libc=uclibc -posix-runtime … -allocate-sym-size=1 -capacity=<capacity> -sym-size-mode=<mode> <bc_file> --model-symbolic-fd-size --sym-files 1 20
```
Note that the size of the allocated buffer of the symbolic file will be bounded by both the specified capacity and the argument of `--sym-files` (in this case: 20).

### Debugging
To collect statistics about symbolic-size dependent loops,
use the following option:
```
-collect-loop-stats
```
To collect statistics related to merging operations (relevant only for merging modes),
use the following option:
```
-collect-merge-stats
```
