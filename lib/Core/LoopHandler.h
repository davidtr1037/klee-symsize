#ifndef KLEE_LOOPHANDLER_H
#define KLEE_LOOPHANDLER_H

#include "klee/ADT/Ref.h"

#include "llvm/Support/CommandLine.h"

#include <map>
#include <stdint.h>
#include <vector>

namespace llvm {
class Instruction;
}

namespace klee {

extern llvm::cl::opt<bool> UseLoopMerge;

class Executor;
class ExecutionState;

class LoopHandler {

private:

  Executor *executor;

  unsigned closedStateCount;

  std::vector<ExecutionState *> openStates;

  std::map<llvm::Instruction *, std::vector<ExecutionState *>> reachedCloseMerge;

public:

  class ReferenceCounter _refCount;

  LoopHandler(Executor *_executor, ExecutionState *es);

  ~LoopHandler();

  void addClosedState(ExecutionState *es, llvm::Instruction *mp);

  void addOpenState(ExecutionState *es);

  void removeOpenState(ExecutionState *es);

  void releaseStates();

  ExecutionState *mergeStates(std::vector<ExecutionState *> &states);
};

}

#endif	/* KLEE_MERGEHANDLER_H */
