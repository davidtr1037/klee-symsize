#include "MergeUtils.h"


using namespace klee;

ExprVisitor::Action ITEOptimizer::visitSelect(const SelectExpr &se) {
  ref<Expr> range = UltExpr::create(ConstantExpr::create(offset, Expr::Int64), size);
  ref<Expr> x = AndExpr::create(range, se.cond);

  Solver::Validity result;
  bool success = solver->evaluate(state.constraints, x, result, state.queryMetaData);
  assert(success);

  //llvm::errs() << "RES " << result << "\n";
  //se.dump();

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
