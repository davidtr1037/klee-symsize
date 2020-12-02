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
}

void LoopHandler::releaseStates() {
  for (auto& i: reachedCloseMerge) {
    vector<ExecutionState *> &states = i.second;
    ExecutionState *merged = mergeStates(states);
    executor->mergingSearcher->continueState(*merged);

    for (ExecutionState *es : states) {
      executor->mergingSearcher->inCloseMerge.erase(es);
    }
  }
  reachedCloseMerge.clear();
}

ExecutionState *LoopHandler::mergeStates(vector<ExecutionState *> &states) {
    assert(!states.empty());
    ExecutionState *merged = states[0];

    for (unsigned i = 1; i < states.size(); i++) {
        ExecutionState *es = states[i];
        merged->merge(*es);
        executor->terminateState(*es);
    }

    return merged;
}

LoopHandler::LoopHandler(Executor *_executor, ExecutionState *es, Loop *loop)
    : executor(_executor), closedStateCount(0), loop(loop) {
  assert(loop);
  addOpenState(es);
}

LoopHandler::~LoopHandler() {
  releaseStates();
}

}
