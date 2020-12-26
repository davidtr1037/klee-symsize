#ifndef KLEE_LOOPHANDLER_H
#define KLEE_LOOPHANDLER_H

#include "TimingSolver.h"

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

  unsigned closedStateCount;

  std::vector<ExecutionState *> openStates;

  std::map<llvm::Instruction *, std::vector<ExecutionState *>> mergeGroups;

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

  bool validateMerge(std::vector<ExecutionState *> &states, ExecutionState *merged);

  class ReferenceCounter _refCount;

  Executor *executor;

  TimingSolver *solver;

  llvm::Loop *loop;

  ConstraintSet initialConstraints;
};

}

#endif	/* KLEE_MERGEHANDLER_H */
