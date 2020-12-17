#ifndef MERGE_UTILS_H
#define MERGE_UTILS_H

#include "TimingSolver.h"
#include "ExecutionState.h"

#include "klee/Expr/Expr.h"
#include "klee/Expr/ExprVisitor.h"

namespace klee {

class ITEOptimizer : public ExprVisitor {
public:

  Action visitSelect(const SelectExpr &e);

  ITEOptimizer(ExecutionState &state, unsigned offset, const ref<Expr> size, TimingSolver *solver) :
    state(state), offset(offset), size(size), solver(solver), changed(false) {

  }

  ExecutionState &state;
  unsigned offset;
  ref<Expr> size;
  TimingSolver *solver;
  bool changed;
};

}

#endif /* MERGE_UTILS_H */
