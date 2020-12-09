#include "LoopHandler.h"

#include "CoreStats.h"
#include "ExecutionState.h"
#include "Executor.h"
#include "Searcher.h"

using namespace std;
using namespace llvm;

namespace klee {

cl::OptionCategory LoopCat("Loop merging options",
                           "These options control path merging.");

cl::opt<bool> UseLoopMerge(
    "use-loop-merge", cl::init(false),
    cl::desc(""),
    cl::cat(klee::LoopCat));

cl::opt<bool> DebugLoopHandler(
    "debug-loop-handler", cl::init(false),
    cl::desc(""),
    cl::cat(klee::LoopCat));

void LoopHandler::addOpenState(ExecutionState *es){
  openStates.push_back(es);
  activeStates++;
}

void LoopHandler::removeOpenState(ExecutionState *es){
  auto it = std::find(openStates.begin(), openStates.end(), es);
  assert(it != openStates.end());
  std::swap(*it, openStates.back());
  openStates.pop_back();
}

void LoopHandler::addClosedState(ExecutionState *es,
                                 Instruction *mp) {
  ++closedStateCount;
  removeOpenState(es);

  auto closePoint = reachedCloseMerge.find(mp);
  if (closePoint == reachedCloseMerge.end()) {
    reachedCloseMerge[mp].push_back(es);
  } else {
    auto &cpv = closePoint->second;
    cpv.push_back(es);
  }

  executor->mergingSearcher->pauseState(*es);

  /* otherwise, a state sneaked out somehow */
  assert(activeStates > 0);
  activeStates--;
  if (activeStates == 0) {
    releaseStates();
  }
}

void LoopHandler::releaseStates() {
  for (auto& i: reachedCloseMerge) {
    vector<ExecutionState *> &states = i.second;
    ExecutionState *merged = ExecutionState::mergeStates(states);
    executor->mergingSearcher->continueState(*merged);
    klee_message("merged %lu states (early = %u)", states.size(), earlyTerminated);

    for (ExecutionState *es : states) {
      executor->mergingSearcher->inCloseMerge.erase(es);
    }

    for (unsigned i = 1; i < states.size(); i++) {
      ExecutionState *es = states[i];
      executor->mergingSearcher->continueState(*es);
      executor->terminateState(*es);
    }
  }
  reachedCloseMerge.clear();
}

void LoopHandler::markEarlyTerminated(ExecutionState &state) {
  earlyTerminated++;
  assert(activeStates > 0);
  activeStates--;
  if (activeStates == 0) {
    releaseStates();
  }
}

unsigned LoopHandler::getEarlyTerminated() {
  return earlyTerminated;
}

LoopHandler::LoopHandler(Executor *_executor, ExecutionState *es, Loop *loop)
    : executor(_executor), closedStateCount(0), activeStates(0), earlyTerminated(0), loop(loop) {
  assert(loop);
  addOpenState(es);
}

LoopHandler::~LoopHandler() {

}

}
