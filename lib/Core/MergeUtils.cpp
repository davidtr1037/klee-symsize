#include "MergeUtils.h"


using namespace klee;

ExprVisitor::Action ITEOptimizer::visitSelect(const SelectExpr &se) {
  ref<Expr> range = UltExpr::create(ConstantExpr::create(offset, Expr::Int64), size);
  state.constraints.push_back(range);

  Solver::Validity result;
  bool success = solver->evaluate(state.constraints, se.cond, result, state.queryMetaData);
  assert(success);

  /* TODO: a bit hacky... */
  state.constraints.pop_back();

  if (result == Solver::Unknown) {
    return Action::doChildren();
  }

  changed = true;
  if (result == Solver::True) {
    return Action::changeTo(se.trueExpr);
  } else {
    return Action::changeTo(se.falseExpr);
  }
}
