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

cl::opt<bool> UseOptimizedMerge(
    "use-optimized-merge", cl::init(false),
    cl::desc(""),
    cl::cat(klee::LoopCat));

cl::opt<bool> OptimizeGroupMerge(
    "optimize-group-merge", cl::init(false),
    cl::desc(""),
    cl::cat(klee::LoopCat));

cl::opt<bool> ValidateMerge(
    "validate-merge", cl::init(false),
    cl::desc(""),
    cl::cat(klee::LoopCat));

cl::opt<unsigned> MaxStatesToMerge(
    "max-states-to-merge", cl::init(10000),
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

  auto i = mergeGroups.find(mp);
  if (i == mergeGroups.end()) {
    mergeGroups[mp].push_back(es);
  } else {
    std::vector<ExecutionState *> &states = i->second;
    states.push_back(es);
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
  std::vector<ref<Expr>> toAdd;
  unsigned largestGroup = 0;
  if (mergeGroups.size() == 2) {
    unsigned groupId = 0;
    size_t maxCount = 0;
    for (auto &i : mergeGroups) {
      vector<ExecutionState *> &states = i.second;
      ref<Expr> e = ExecutionState::buildMergedConstraint(states);
      toAdd.push_back(e);
      if (states.size() > maxCount) {
        maxCount = states.size();
        largestGroup = groupId;
      }
      groupId++;
    }
  }

  unsigned groupId = 0;
  for (auto &i: mergeGroups) {
    vector<ExecutionState *> &states = i.second;
    vector<ExecutionState *> snapshots;
    if (ValidateMerge) {
      /* take snapshots before merging */
      for (ExecutionState *es : states) {
        snapshots.push_back(es->branch());
      }
    }

    ExecutionState *merged = nullptr;
    bool isComplete = (mergeGroups.size() == 1) && (earlyTerminated == 0);
    /* TODO: pc or prevPC? */
    klee_message("merging at %s:%u",
                 states[0]->pc->info->file.data(),
                 states[0]->pc->info->line);

    if (MaxStatesToMerge == 0 || states.size() < MaxStatesToMerge) {
      if (UseOptimizedMerge) {
        ref<Expr> e = nullptr;
        if (OptimizeGroupMerge && mergeGroups.size() > 1 && groupId == largestGroup) {
          e = ConstantExpr::create(1, Expr::Bool);
          for (unsigned k = 0; k < toAdd.size(); k++) {
            if (k != largestGroup) {
              e = AndExpr::create(e, NotExpr::create(toAdd[k]));
            }
          }
        }
        merged = ExecutionState::mergeStatesOptimized(states, isComplete, e, this);
      } else {
        merged = ExecutionState::mergeStates(states);
      }
    }
    if (!merged) {
      /* TODO: merged state might have merge side effects */
      char msg[1000];
      snprintf(msg,
               sizeof(msg),
               "unsupported merge: %s:%u",
               states[0]->prevPC->info->file.data(),
               states[0]->prevPC->info->line);
      klee_warning("%s", msg);

      for (ExecutionState *es : states) {
        executor->mergingSearcher->inCloseMerge.erase(es);
        es->suffixConstraints.clear();
        executor->mergingSearcher->continueState(*es);
      }

      /* TODO: refactor... */
      continue;
    }

    if (ValidateMerge) {
      assert(validateMerge(snapshots, merged));
      for (ExecutionState *es : snapshots) {
        delete es;
      }
    }

    executor->mergingSearcher->continueState(*merged);
    executor->collectMergeStats(*merged);
    if (mergeGroups.size() == 1) {
      klee_message("merged %lu states (complete = %u)", states.size(), isComplete);
    } else {
      klee_message("merged %lu states (complete = %u, group = %u)",
                   states.size(),
                   isComplete,
                   groupId);
    }

    for (ExecutionState *es : states) {
      executor->mergingSearcher->inCloseMerge.erase(es);
      es->suffixConstraints.clear();
    }

    for (unsigned i = 1; i < states.size(); i++) {
      ExecutionState *es = states[i];
      executor->mergingSearcher->continueState(*es);
      executor->terminateStateEarly(*es, "Merge");
      executor->interpreterHandler->decUnmergedExploredPaths();
    }

    groupId++;
  }
  mergeGroups.clear();
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
    : closedStateCount(0),
      activeStates(0),
      earlyTerminated(0),
      executor(_executor),
      solver(_executor->solver),
      loop(loop),
      tree(es->getID()),
      canUseExecTree(true) {
  assert(loop);
  addOpenState(es);
  for (ref<Expr> e : es->constraints) {
    initialConstraints.push_back(e);
  }
}

LoopHandler::~LoopHandler() {
  if (executor->haltExecution) {
    /* if execution is interrupted, may contain unreleased states */
    return;
  }

  for (auto &i: mergeGroups) {
    vector<ExecutionState *> &states = i.second;
    assert(states.empty());
  }
}

bool LoopHandler::validateMerge(std::vector<ExecutionState *> &snapshots, ExecutionState *merged) {
  ExecutionState *expected = ExecutionState::mergeStates(snapshots);
  return ExecutionState::areEquiv(executor->solver, merged, expected);
}

}
