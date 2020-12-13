#ifndef KLEE_LOOPHANDLER_H
#define KLEE_LOOPHANDLER_H

#include "klee/ADT/Ref.h"
#include "klee/Expr/Constraints.h"

#include "llvm/Support/CommandLine.h"
#include "llvm/Analysis/LoopInfo.h"

#include <map>
#include <stdint.h>
#include <vector>

namespace llvm {
class Instruction;
}

namespace klee {

extern llvm::cl::opt<bool> UseLoopMerge;
extern llvm::cl::opt<bool> DebugLoopHandler;
extern llvm::cl::opt<bool> UseOptimizedMerge;

class Executor;
class ExecutionState;

class LoopHandler {

private:

  Executor *executor;

  unsigned closedStateCount;

  std::vector<ExecutionState *> openStates;

  std::map<llvm::Instruction *, std::vector<ExecutionState *>> reachedCloseMerge;

  unsigned activeStates;

  unsigned earlyTerminated;

public:

  LoopHandler(Executor *_executor, ExecutionState *es, llvm::Loop *loop);

  ~LoopHandler();

  void addClosedState(ExecutionState *es, llvm::Instruction *mp);

  void addOpenState(ExecutionState *es);

  void removeOpenState(ExecutionState *es);

  void releaseStates();

  void markEarlyTerminated(ExecutionState &state);

  unsigned getEarlyTerminated();

  class ReferenceCounter _refCount;

  llvm::Loop *loop;

  ConstraintSet initialConstraints;
};

}

#endif	/* KLEE_MERGEHANDLER_H */
